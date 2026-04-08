using System;
using System.Collections.Generic;
using UnityEngine;

namespace RobotSimulation
{
	public sealed class CoordinatedTaskResolution
	{
		public bool accepted;
		public bool dockingPoseFound;
		public bool baseMoveRequired;
		public bool armReachableFromCurrentBase;
		public bool armReachabilityIsLoosePrecheck;
		public bool hasPreferredArmSolveSeed;
		public float[] preferredArmSolveSeedAnglesDeg;
		public Vector3 resolvedBaseStopWorldPosition;
		public float resolvedBaseStopYawDeg;
		public string failureReason = string.Empty;
		public string dockingSummary = string.Empty;
		public int coarseCandidateCount;
		public int fineCandidateCount;
		public int ikSolveCount;
		public int basePathCheckCount;
		public string dockingFailureCategory = string.Empty;
	}

	public sealed class CoordinatedTaskPlanner
	{
		private const float DefaultWorkspaceInnerRadiusMeters = 0.24338f;
		private const float DefaultWorkspaceOuterRadiusMeters = 0.8066f;
		private const float BasePositionToleranceMeters = 0.02f;
		private const float LoosePrecheckToleranceMeters = 0.05f;
		private const float LoosePrecheckHardSingularityThreshold = 140f;
		private const float LoosePrecheckSoftSingularityThreshold = 70f;
		private const float DockingGeometryRadiusConsistencyToleranceMeters = 0.035f;
		private const float DockingPreferredRadiusMarginMeters = 0.015f;

		private static readonly float[] SectorHalfAnglesDeg = { 25f, 45f, 70f, 180f };
		private static readonly string[] SectorLabels =
		{
			"PrimarySectorSearch",
			"ExpandedSectorSearch45",
			"ExpandedSectorSearch70",
			"FallbackFullRingSearch"
		};

		private const string FailureCategoryNone = "none";
		private const string FailureCategoryNoCollisionFreeBasePose = "NoCollisionFreeBasePose";
		private const string FailureCategoryNoWorkspaceConsistentPose = "NoWorkspaceConsistentPose";
		private const string FailureCategoryNoLooseIkPose = "NoLooseIkPose";
		private const string FailureCategoryNoPathReachablePose = "NoPathReachablePose";

		internal sealed class DockingCandidate
		{
			public Vector3 baseWorldPosition;
			public float baseYawDeg;
			public float targetAngleRad;
			public float radius;
			public float shellMargin;
			public float preferredBandPenalty;
			public float workspaceBandPenalty;
			public float travelDistance;
			public float clearance;
			public float heuristicCost;
			public Vector3 futureArmBasePosition;
			public Vector3 targetInFutureArmBase;
			public float targetDistanceInFutureArmBase;
			public Vector3 futureArmBaseEulerAngles;
			public string sectorLabel;
			public string radiusBand;
			public string evaluationStageSummary;
		}

		public sealed class DockingSearchContext
		{
			internal RobotPlanRequest request;
			internal DiffDriveTwinController diffDriveController;
			internal Arm6DOFFKController armController;
			internal PlannerPhysicsQueries physicsQueries;
			internal IReadOnlyList<Collider> obstacles;
			internal Transform baseRoot;
			internal Vector3 currentBasePosition;
			internal float currentBaseYaw;
			internal float baseRadius;
			internal float minReach;
			internal float maxReach;
			internal float preferredMin;
			internal float preferredMax;
			internal Vector3 armBaseLocalPosition;
			internal Quaternion armBaseLocalRotation;
			internal Vector3 primarySectorDirection;
			internal int sectorStage;
			internal float[] startAngles;
			internal List<DockingCandidate> coarseCandidates;
			internal List<DockingCandidate> fineCandidates;
		}

		private sealed class DockingFailureSummary
		{
			public bool anyWorkspaceShellCandidate;
			public bool anyIkCandidate;
			public bool anyCollisionFreeArmCandidate;
			public int noCollisionFreeBasePoseCount;
			public int noWorkspaceConsistentPoseCount;
			public int noLooseIkPoseCount;
			public int noPathReachablePoseCount;
			public int preferredBandConsistentCount;
			public string lastArmFailure = string.Empty;
			public string lastPathFailure = string.Empty;
			public string failureCategory = FailureCategoryNone;
		}

		private readonly BaseRrtStarPlanner _basePlanner = new BaseRrtStarPlanner();
		private readonly SceneDistanceFieldSampler _distanceFieldSampler = new SceneDistanceFieldSampler();
		private readonly ArmMotionPlanner _armMotionPlanner = new ArmMotionPlanner();
		private bool _hasCachedArmBaseLocalOffset;
		private int _cachedBaseRootId;
		private int _cachedArmBaseTransformId;
		private Vector3 _cachedArmBaseLocalPosition;
		private Quaternion _cachedArmBaseLocalRotation = Quaternion.identity;

		public int dockingAngularSamples = 24;
		public int preferredBandRadiusSamples = 4;
		public int fallbackBandRadiusSamples = 3;
		public int coarseAngularSamples = 12;
		public int coarsePreferredBandRadiusSamples = 3;
		public int coarseFallbackBandRadiusSamples = 2;
		public int coarseSeedKeepCount = 12;
		public float[] fineRadiusOffsetsMeters = { -0.06f, -0.03f, 0f, 0.03f, 0.06f };
		public float[] fineAngleOffsetsDeg = { -12f, -6f, 0f, 6f, 12f };
		public int maxFineCandidateEvaluations = 48;
		public float dockingDistanceFieldResolution = 0.35f;
		public float dockingPlanningMargin = 3.5f;
		public bool allowProvisionalDockingWhenIkSoftFails = true;
		public int provisionalDockingPathCheckBudget = 6;

		public CoordinatedTaskPlanner()
		{
			_basePlanner.settings.maxIterations = 450;
			_basePlanner.settings.stepLength = 0.55f;
			_basePlanner.settings.goalThreshold = 0.55f;
			_basePlanner.settings.rewireRadius = 1.2f;
			_basePlanner.settings.goalBias = 0.25f;
			_armMotionPlanner.settings.toleranceMeters = 0.02f;
			_armMotionPlanner.settings.maxIterations = 180;
		}

		public CoordinatedTaskResolution EvaluateCurrentBaseExecution(
			RobotPlanRequest request,
			DiffDriveTwinController diffDriveController,
			Arm6DOFFKController armController)
		{
			CoordinatedTaskResolution resolution = CreateDefaultResolution(diffDriveController);
			if (diffDriveController == null || diffDriveController.rb == null)
			{
				resolution.failureReason = L("差速底盘控制器缺失。", "DiffDrive controller is missing.");
				resolution.dockingSummary = resolution.failureReason;
				return resolution;
			}

			if (request == null || !request.requireArmMove)
			{
				resolution.accepted = true;
				resolution.dockingPoseFound = true;
				resolution.armReachableFromCurrentBase = true;
				resolution.armReachabilityIsLoosePrecheck = true;
				resolution.baseMoveRequired = false;
				resolution.dockingSummary = L(
					"本次任务不需要机械臂动作，因此当前底盘位姿可直接接受。",
					"This task does not require arm motion, so the current base pose is accepted.");
				return resolution;
			}

			if (armController == null || !armController.KinematicsReady)
			{
				resolution.failureReason = L("机械臂控制器或运动学模型尚未就绪。", "Arm controller or kinematics model is not ready.");
				resolution.dockingSummary = resolution.failureReason;
				return resolution;
			}

			if (TryEvaluateArmFeasibilityAtBasePoseLoose(
				diffDriveController.rb.transform,
				diffDriveController.rb.position,
				diffDriveController.rb.rotation,
				armController,
				request.armTargetWorldPosition,
				out float[] solvedAngles,
				out float residualMeters,
				out float singularityPenalty,
				out string currentBaseFailure,
				request != null ? request.eePositionToleranceMeters : -1f))
			{
				resolution.accepted = true;
				resolution.dockingPoseFound = true;
				resolution.armReachableFromCurrentBase = true;
				resolution.armReachabilityIsLoosePrecheck = true;
				resolution.baseMoveRequired = false;
				resolution.hasPreferredArmSolveSeed = solvedAngles != null && solvedAngles.Length >= 6;
				resolution.preferredArmSolveSeedAnglesDeg = CloneAngles(solvedAngles);
				Debug.Log($"[CoordinatedTaskPlanner] LooseReachabilityAccepted: residual={residualMeters:F4}m, singularityPenalty={singularityPenalty:F2}");
				resolution.dockingSummary = L(
					"当前底盘位姿已经支持所请求的末端世界目标，因此无需移动底盘。",
					"Current base pose already supports the requested end-effector world target, so no base move is required.");
				return resolution;
			}

			resolution.armReachableFromCurrentBase = false;
			resolution.armReachabilityIsLoosePrecheck = true;
			resolution.failureReason = string.IsNullOrEmpty(currentBaseFailure)
				? L("当前底盘位姿无法安全完成所请求的末端世界目标。", "Current base pose cannot safely execute the requested end-effector world target.")
				: currentBaseFailure;
			resolution.dockingSummary = resolution.failureReason;
			return resolution;
		}

		public bool TryPrepareAutoDockingSearch(
			RobotPlanRequest request,
			DiffDriveTwinController diffDriveController,
			Arm6DOFFKController armController,
			float baseRadius,
			PlannerPhysicsQueries physicsQueries,
			IReadOnlyList<Collider> obstacles,
			out DockingSearchContext context,
			out CoordinatedTaskResolution resolution)
		{
			context = null;
			resolution = CreateDefaultResolution(diffDriveController);
			if (diffDriveController == null || diffDriveController.rb == null)
			{
				resolution.failureReason = L("差速底盘控制器缺失。", "DiffDrive controller is missing.");
				resolution.dockingSummary = resolution.failureReason;
				return false;
			}

			if (request == null || !request.requireArmMove)
			{
				resolution.accepted = true;
				resolution.dockingPoseFound = true;
				resolution.baseMoveRequired = false;
				resolution.armReachableFromCurrentBase = true;
				resolution.dockingSummary = L(
					"本次任务没有机械臂动作，因此无需停靠位搜索。",
					"No arm task was requested, so docking search is not needed.");
				return true;
			}

			if (armController == null || !armController.KinematicsReady)
			{
				resolution.failureReason = L("机械臂控制器或运动学模型尚未就绪。", "Arm controller or kinematics model is not ready.");
				resolution.dockingSummary = resolution.failureReason;
				return false;
			}

			Transform baseRoot = diffDriveController.rb.transform;
			Vector3 currentBasePosition = diffDriveController.rb.position;
			float minReach = armController.GetMinReach() > 0f ? armController.GetMinReach() : DefaultWorkspaceInnerRadiusMeters;
			float maxReach = armController.GetMaxReach() > 0f ? armController.GetMaxReach() : DefaultWorkspaceOuterRadiusMeters;
			float preferredMin = Mathf.Clamp(maxReach * 0.4f, minReach + 0.01f, maxReach);
			float preferredMax = Mathf.Clamp(maxReach * 0.8f, preferredMin, maxReach);
			float[] startAngles = armController.CaptureMeasuredJointAngles();
			if (!TryGetCachedArmBaseLocalOffset(baseRoot, armController, out Vector3 armBaseLocalPosition, out Quaternion armBaseLocalRotation))
			{
				resolution.failureReason = L("无法解析机械臂相对底盘的安装偏移。", "Could not resolve the arm mounting offset relative to the base.");
				resolution.dockingSummary = resolution.failureReason;
				return false;
			}

			Vector3 primarySectorDirection = ComputePrimarySectorDirection(currentBasePosition, request.armTargetWorldPosition, diffDriveController.rb.rotation);
			PrepareDockingDistanceField(currentBasePosition, request.armTargetWorldPosition, obstacles, maxReach, baseRadius, armBaseLocalPosition);
			List<DockingCandidate> coarseCandidates = BuildCoarseDockingCandidates(
				baseRoot,
				armController,
				request.armTargetWorldPosition,
				currentBasePosition,
				armBaseLocalPosition,
				armBaseLocalRotation,
				minReach,
				maxReach,
				preferredMin,
				preferredMax,
				baseRadius,
				physicsQueries,
				obstacles,
				primarySectorDirection,
				out int selectedSectorStage,
				out int preferredBandConsistentCount);

			resolution.coarseCandidateCount = coarseCandidates.Count;
			if (coarseCandidates.Count == 0)
			{
				resolution.dockingFailureCategory = FailureCategoryNoCollisionFreeBasePose;
				Collider blockedCurrentPose = null;
				bool currentPoseCollisionFree = physicsQueries != null && physicsQueries.IsBasePoseCollisionFree(currentBasePosition, baseRadius, obstacles, out blockedCurrentPose);
				Debug.LogWarning($"[CoordinatedTaskPlanner] Docking coarse search found zero collision-free candidates. baseRadius={baseRadius:F3}, currentPoseCollisionFree={currentPoseCollisionFree}, blockedCurrentPose={(blockedCurrentPose != null ? blockedCurrentPose.name : "none")}, obstacleCount={(obstacles != null ? obstacles.Count : 0)}, sector={SectorLabels[Mathf.Clamp(selectedSectorStage, 0, SectorLabels.Length - 1)]}, primarySectorDirection={primarySectorDirection}");
				resolution.failureReason = L(
					"Zu5 停靠搜索环带内没有找到无碰撞的底盘停靠位。",
					"No collision-free base parking pose exists inside the Zu5 docking search ring.");
				resolution.dockingSummary = resolution.failureReason;
				return false;
			}

			List<DockingCandidate> coarseSeeds = RankAndTrimCoarseCandidates(coarseCandidates);
			List<DockingCandidate> fineCandidates = ExpandFineCandidatesAroundSeeds(
				baseRoot,
				armController,
				request.armTargetWorldPosition,
				currentBasePosition,
				armBaseLocalPosition,
				armBaseLocalRotation,
				minReach,
				maxReach,
				preferredMin,
				preferredMax,
				baseRadius,
				physicsQueries,
				obstacles,
				coarseSeeds);

			context = new DockingSearchContext
			{
				request = request,
				diffDriveController = diffDriveController,
				armController = armController,
				physicsQueries = physicsQueries,
				obstacles = obstacles,
				baseRoot = baseRoot,
				currentBasePosition = currentBasePosition,
				currentBaseYaw = diffDriveController.rb.rotation.eulerAngles.y,
				baseRadius = baseRadius,
				minReach = minReach,
				maxReach = maxReach,
				preferredMin = preferredMin,
				preferredMax = preferredMax,
				armBaseLocalPosition = armBaseLocalPosition,
				armBaseLocalRotation = armBaseLocalRotation,
				primarySectorDirection = primarySectorDirection,
				sectorStage = selectedSectorStage,
				startAngles = CloneAngles(startAngles),
				coarseCandidates = coarseCandidates,
				fineCandidates = fineCandidates
			};

			resolution.fineCandidateCount = fineCandidates.Count;
			resolution.dockingFailureCategory = FailureCategoryNone;
			resolution.dockingSummary = L(
				$"停靠位粗筛保留了 {fineCandidates.Count} 个精筛候选（原始无碰撞候选 {coarseCandidates.Count} 个）。",
				$"Docking search found {coarseCandidates.Count} coarse candidates in {SectorLabels[Mathf.Clamp(selectedSectorStage, 0, SectorLabels.Length - 1)]}; {preferredBandConsistentCount} remained inside the preferred band under the real arm-base frame, and {fineCandidates.Count} advanced to fine screening.");
			return true;
		}

		public bool TryCompleteAutoDockingSearch(DockingSearchContext context, out CoordinatedTaskResolution resolution)
		{
			resolution = CreateDefaultResolution(context != null ? context.diffDriveController : null);
			if (context == null || context.diffDriveController == null || context.diffDriveController.rb == null)
			{
				resolution.failureReason = L("停靠位搜索上下文缺失。", "Docking search context is missing.");
				resolution.dockingSummary = resolution.failureReason;
				return false;
			}

			resolution.coarseCandidateCount = context.coarseCandidates != null ? context.coarseCandidates.Count : 0;
			resolution.fineCandidateCount = context.fineCandidates != null ? context.fineCandidates.Count : 0;

			DockingFailureSummary failureSummary = new DockingFailureSummary
			{
				noCollisionFreeBasePoseCount = 0,
				noWorkspaceConsistentPoseCount = 0,
				noLooseIkPoseCount = 0,
				noPathReachablePoseCount = 0,
				preferredBandConsistentCount = 0
			};
			DockingCandidate bestCandidate = null;
			DockingCandidate bestProvisionalCandidate = null;
			float[] bestSolvedAngles = null;
			float bestCost = float.MaxValue;
			float bestProvisionalCost = float.MaxValue;
			string bestProvisionalReason = string.Empty;
			int provisionalPathChecksUsed = 0;

			List<DockingCandidate> fineCandidates = context.fineCandidates ?? new List<DockingCandidate>();
			int evaluationBudget = Mathf.Min(fineCandidates.Count, Mathf.Max(1, maxFineCandidateEvaluations));
			for (int i = 0; i < evaluationBudget; i++)
			{
				DockingCandidate candidate = fineCandidates[i];
				if (bestCandidate != null && candidate.heuristicCost >= bestCost)
				{
					break;
				}

				TryEvaluateFineCandidate(
					context,
					candidate,
					failureSummary,
					ref provisionalPathChecksUsed,
					ref bestProvisionalCandidate,
					ref bestProvisionalCost,
					ref bestProvisionalReason,
					ref bestCandidate,
					ref bestCost,
					ref bestSolvedAngles,
					ref resolution.ikSolveCount,
					ref resolution.basePathCheckCount);
			}

			resolution.dockingFailureCategory = failureSummary.failureCategory;
			if (bestCandidate == null)
			{
				if (bestProvisionalCandidate != null)
				{
					resolution.accepted = true;
					resolution.dockingPoseFound = true;
					resolution.baseMoveRequired = PlanarDistance(context.currentBasePosition, bestProvisionalCandidate.baseWorldPosition) > BasePositionToleranceMeters;
					resolution.armReachableFromCurrentBase = false;
					resolution.hasPreferredArmSolveSeed = false;
					resolution.preferredArmSolveSeedAnglesDeg = null;
					resolution.resolvedBaseStopWorldPosition = bestProvisionalCandidate.baseWorldPosition;
					resolution.resolvedBaseStopYawDeg = bestProvisionalCandidate.baseYawDeg;
					resolution.dockingFailureCategory = FailureCategoryNoLooseIkPose;
					resolution.dockingSummary = L(
						$"停靠位精筛未直接获得稳定机械臂解（{bestProvisionalReason}），将先执行保守停靠位 ({bestProvisionalCandidate.baseWorldPosition.x:F2}, {bestProvisionalCandidate.baseWorldPosition.y:F2}, {bestProvisionalCandidate.baseWorldPosition.z:F2})。粗筛 {resolution.coarseCandidateCount} 个，精筛 {resolution.fineCandidateCount} 个，IK {resolution.ikSolveCount} 次，路径检查 {resolution.basePathCheckCount} 次。",
						$"Docking fine search did not find a strict arm-ready pose ({bestProvisionalReason}), so execution will first move to a conservative docking pose ({bestProvisionalCandidate.baseWorldPosition.x:F2}, {bestProvisionalCandidate.baseWorldPosition.y:F2}, {bestProvisionalCandidate.baseWorldPosition.z:F2}) from {bestProvisionalCandidate.sectorLabel}. targetInFutureArmBase={bestProvisionalCandidate.targetInFutureArmBase}, |target|={bestProvisionalCandidate.targetDistanceInFutureArmBase:F4}. Coarse={resolution.coarseCandidateCount}, fine={resolution.fineCandidateCount}, IK={resolution.ikSolveCount}, path checks={resolution.basePathCheckCount}.");
					Debug.Log($"[CoordinatedTaskPlanner] Docking summary: sector={bestProvisionalCandidate.sectorLabel}, coarse={resolution.coarseCandidateCount}, fine={resolution.fineCandidateCount}, preferredConsistent={failureSummary.preferredBandConsistentCount}, ik={resolution.ikSolveCount}, pathChecks={resolution.basePathCheckCount}, foundProvisional=true, targetInFutureArmBase={bestProvisionalCandidate.targetInFutureArmBase}, targetDistance={bestProvisionalCandidate.targetDistanceInFutureArmBase:F4}");
					return true;
				}

				resolution.failureReason = BuildDockingFailureReasonV3(failureSummary);
				resolution.dockingSummary = resolution.failureReason;
				Debug.Log($"[CoordinatedTaskPlanner] Docking summary: sector={SectorLabels[Mathf.Clamp(context.sectorStage, 0, SectorLabels.Length - 1)]}, coarse={resolution.coarseCandidateCount}, fine={resolution.fineCandidateCount}, preferredConsistent={failureSummary.preferredBandConsistentCount}, ik={resolution.ikSolveCount}, pathChecks={resolution.basePathCheckCount}, found=false, category={resolution.dockingFailureCategory}");
				return false;
			}

			resolution.accepted = true;
			resolution.dockingPoseFound = true;
			resolution.baseMoveRequired = PlanarDistance(context.currentBasePosition, bestCandidate.baseWorldPosition) > BasePositionToleranceMeters;
			resolution.armReachableFromCurrentBase = false;
			resolution.hasPreferredArmSolveSeed = bestSolvedAngles != null && bestSolvedAngles.Length >= 6;
			resolution.preferredArmSolveSeedAnglesDeg = CloneAngles(bestSolvedAngles);
			resolution.resolvedBaseStopWorldPosition = bestCandidate.baseWorldPosition;
			resolution.resolvedBaseStopYawDeg = bestCandidate.baseYawDeg;
			resolution.dockingFailureCategory = FailureCategoryNone;
			resolution.dockingSummary = L(
				$"已解析到底盘停靠位 ({bestCandidate.baseWorldPosition.x:F2}, {bestCandidate.baseWorldPosition.y:F2}, {bestCandidate.baseWorldPosition.z:F2})，朝向 {bestCandidate.baseYawDeg:F1}°。粗筛 {resolution.coarseCandidateCount} 个，精筛 {resolution.fineCandidateCount} 个，IK {resolution.ikSolveCount} 次，路径检查 {resolution.basePathCheckCount} 次。",
				$"Resolved base stop at ({bestCandidate.baseWorldPosition.x:F2}, {bestCandidate.baseWorldPosition.y:F2}, {bestCandidate.baseWorldPosition.z:F2}) with yaw {bestCandidate.baseYawDeg:F1}deg from {bestCandidate.sectorLabel}. targetInFutureArmBase={bestCandidate.targetInFutureArmBase}, |target|={bestCandidate.targetDistanceInFutureArmBase:F4}. Coarse={resolution.coarseCandidateCount}, fine={resolution.fineCandidateCount}, IK={resolution.ikSolveCount}, path checks={resolution.basePathCheckCount}.");
			Debug.Log($"[CoordinatedTaskPlanner] Docking summary: sector={bestCandidate.sectorLabel}, coarse={resolution.coarseCandidateCount}, fine={resolution.fineCandidateCount}, preferredConsistent={failureSummary.preferredBandConsistentCount}, ik={resolution.ikSolveCount}, pathChecks={resolution.basePathCheckCount}, found=true, targetInFutureArmBase={bestCandidate.targetInFutureArmBase}, targetDistance={bestCandidate.targetDistanceInFutureArmBase:F4}");
			return true;
		}

		public bool TryFindAutoDockingPose(
			RobotPlanRequest request,
			DiffDriveTwinController diffDriveController,
			Arm6DOFFKController armController,
			float baseRadius,
			PlannerPhysicsQueries physicsQueries,
			IReadOnlyList<Collider> obstacles,
			out CoordinatedTaskResolution resolution)
		{
			if (!TryPrepareAutoDockingSearch(
				request,
				diffDriveController,
				armController,
				baseRadius,
				physicsQueries,
				obstacles,
				out DockingSearchContext context,
				out resolution))
			{
				return false;
			}

			return context == null || TryCompleteAutoDockingSearch(context, out resolution);
		}

		public bool TryResolveLegacyBaseGoal(
			RobotPlanRequest request,
			DiffDriveTwinController diffDriveController,
			Arm6DOFFKController armController,
			float baseRadius,
			PlannerPhysicsQueries physicsQueries,
			IReadOnlyList<Collider> obstacles,
			out CoordinatedTaskResolution resolution)
		{
			resolution = CreateDefaultResolution(diffDriveController);
			if (diffDriveController == null || diffDriveController.rb == null)
			{
				resolution.failureReason = L("差速底盘控制器缺失。", "DiffDrive controller is missing.");
				resolution.dockingSummary = resolution.failureReason;
				return false;
			}

			Vector3 currentBasePosition = diffDriveController.rb.position;
			float currentBaseYaw = diffDriveController.rb.rotation.eulerAngles.y;
			Vector3 requestedBasePosition = GetRequestedBasePosition(request, diffDriveController);
			bool allowBaseMove = request != null && request.requireBaseMove;
			bool hasExplicitBaseGoal = HasExplicitBaseGoal(request, diffDriveController);
			bool needArmMove = request != null && request.requireArmMove && armController != null && armController.KinematicsReady;

			if (!needArmMove)
			{
				resolution.accepted = true;
				resolution.dockingPoseFound = true;
				resolution.armReachableFromCurrentBase = true;
				resolution.baseMoveRequired = hasExplicitBaseGoal && PlanarDistance(currentBasePosition, requestedBasePosition) > BasePositionToleranceMeters;
				resolution.resolvedBaseStopWorldPosition = hasExplicitBaseGoal ? requestedBasePosition : currentBasePosition;
				resolution.resolvedBaseStopYawDeg = hasExplicitBaseGoal && request != null ? request.baseTargetYawDeg : currentBaseYaw;
				resolution.dockingSummary = hasExplicitBaseGoal
					? L("本次任务不需要机械臂动作，因此直接采用显式底盘世界目标。", "Using the explicit base world-reference target because this task does not require an arm move.")
					: L("本次任务不需要机械臂动作，因此底盘参考点保持在当前世界位置。", "No arm move is required, so the base reference point stays at its current world position.");
				return true;
			}

			if (hasExplicitBaseGoal)
			{
				resolution.resolvedBaseStopWorldPosition = requestedBasePosition;
				resolution.resolvedBaseStopYawDeg = request.baseTargetYawDeg;
				resolution.baseMoveRequired = PlanarDistance(currentBasePosition, requestedBasePosition) > BasePositionToleranceMeters;
				if (!physicsQueries.IsBasePoseCollisionFree(requestedBasePosition, baseRadius, obstacles, out Collider blockedGoal))
				{
					resolution.failureReason = L(
						$"显式底盘目标被 '{blockedGoal?.name}' 占用，因此无法从该位姿开始机械臂任务。",
						$"The explicit base target is occupied by '{blockedGoal?.name}', so the arm task cannot start from that pose.");
					resolution.dockingSummary = resolution.failureReason;
					return false;
				}

				if (!TryEvaluateArmExecutionAtBasePose(
					diffDriveController.rb.transform,
					requestedBasePosition,
					Quaternion.Euler(0f, request.baseTargetYawDeg, 0f),
					armController,
					request.armTargetWorldPosition,
					out float[] solvedAngles,
					out _,
					out string explicitBaseFailure,
					request != null ? request.eePositionToleranceMeters : -1f))
				{
					resolution.failureReason = string.IsNullOrEmpty(explicitBaseFailure)
						? L("显式底盘目标无法支持所请求的末端世界目标。", "The explicit base target cannot support the requested end-effector world target.")
						: explicitBaseFailure;
					resolution.dockingSummary = resolution.failureReason;
					return false;
				}

				resolution.accepted = true;
				resolution.dockingPoseFound = true;
				resolution.armReachableFromCurrentBase = PlanarDistance(currentBasePosition, requestedBasePosition) <= BasePositionToleranceMeters;
				resolution.hasPreferredArmSolveSeed = solvedAngles != null && solvedAngles.Length >= 6;
				resolution.preferredArmSolveSeedAnglesDeg = CloneAngles(solvedAngles);
				resolution.dockingSummary = L(
					"将先执行显式底盘世界目标，然后再执行机械臂。",
					"Using the explicit base world-reference target before arm execution.");
				return true;
			}

			CoordinatedTaskResolution currentBaseResolution = EvaluateCurrentBaseExecution(request, diffDriveController, armController);
			if (currentBaseResolution.armReachableFromCurrentBase)
			{
				resolution = currentBaseResolution;
				return true;
			}

			if (!allowBaseMove)
			{
				resolution.failureReason = L(
					"当前底盘位姿无法到达机械臂目标，并且不允许底盘移动。",
					"Arm target is not reachable from the current base pose and base motion is not allowed.");
				resolution.dockingSummary = resolution.failureReason;
				return false;
			}

			bool foundDocking = TryFindAutoDockingPose(request, diffDriveController, armController, baseRadius, physicsQueries, obstacles, out CoordinatedTaskResolution dockingResolution);
			resolution = dockingResolution;
			return foundDocking;
		}

		public bool IsArmReachableFromPose(
			Transform baseRoot,
			Vector3 baseWorldPosition,
			Quaternion baseWorldRotation,
			Arm6DOFFKController armController,
			Vector3 armTargetWorldPosition)
		{
			if (armController == null || !armController.KinematicsReady)
			{
				return false;
			}

			if (!TryGetTargetInFutureArmBase(baseRoot, baseWorldPosition, baseWorldRotation, armController, armTargetWorldPosition, out Vector3 targetInFutureBase))
			{
				return false;
			}

			return armController.IsBasePositionReachable(targetInFutureBase, _armMotionPlanner.settings.toleranceMeters);
		}

		private List<DockingCandidate> BuildCoarseDockingCandidates(
			Transform baseRoot,
			Arm6DOFFKController armController,
			Vector3 armTargetWorldPosition,
			Vector3 currentBasePosition,
			Vector3 armBaseLocalPosition,
			Quaternion armBaseLocalRotation,
			float minReach,
			float maxReach,
			float preferredMin,
			float preferredMax,
			float baseRadius,
			PlannerPhysicsQueries physicsQueries,
			IReadOnlyList<Collider> obstacles,
			Vector3 primarySectorDirection,
			out int selectedSectorStage,
			out int preferredBandConsistentCount)
		{
			selectedSectorStage = SectorHalfAnglesDeg.Length - 1;
			preferredBandConsistentCount = 0;
			List<DockingCandidate> fallback = new List<DockingCandidate>();
			for (int sectorStage = 0; sectorStage < SectorHalfAnglesDeg.Length; sectorStage++)
			{
				bool includeFallbackBand = sectorStage >= 2;
				List<DockingCandidate> stageCandidates = BuildSectorSearchCandidates(
					baseRoot,
					armController,
					armTargetWorldPosition,
					currentBasePosition,
					armBaseLocalPosition,
					armBaseLocalRotation,
					minReach,
					maxReach,
					preferredMin,
					preferredMax,
					baseRadius,
					physicsQueries,
					obstacles,
					primarySectorDirection,
					sectorStage,
					includeFallbackBand,
					true);
				int stagePreferredCount = CountPreferredBandConsistent(stageCandidates, preferredMin, preferredMax);
				if (stageCandidates.Count > 0)
				{
					selectedSectorStage = sectorStage;
					preferredBandConsistentCount = stagePreferredCount;
					return stageCandidates;
				}

				if (fallback.Count == 0 || stagePreferredCount > preferredBandConsistentCount)
				{
					fallback = stageCandidates;
					selectedSectorStage = sectorStage;
					preferredBandConsistentCount = stagePreferredCount;
				}
			}

			return fallback;
		}

		private List<DockingCandidate> BuildSectorSearchCandidates(
			Transform baseRoot,
			Arm6DOFFKController armController,
			Vector3 armTargetWorldPosition,
			Vector3 currentBasePosition,
			Vector3 armBaseLocalPosition,
			Quaternion armBaseLocalRotation,
			float minReach,
			float maxReach,
			float preferredMin,
			float preferredMax,
			float baseRadius,
			PlannerPhysicsQueries physicsQueries,
			IReadOnlyList<Collider> obstacles,
			Vector3 primarySectorDirection,
			int sectorStage,
			bool includeFallbackBand,
			bool coarseSearch)
		{
			List<float> radii = BuildDockingRadii(
				minReach,
				maxReach,
				preferredMin,
				preferredMax,
				coarseSearch ? coarsePreferredBandRadiusSamples : preferredBandRadiusSamples,
				includeFallbackBand
					? (coarseSearch ? coarseFallbackBandRadiusSamples : fallbackBandRadiusSamples)
					: 0,
				includeFallbackBand);
			float centerAngleRad = Mathf.Atan2(primarySectorDirection.z, primarySectorDirection.x);
			float sectorHalfAngleRad = Mathf.Deg2Rad * SectorHalfAnglesDeg[Mathf.Clamp(sectorStage, 0, SectorHalfAnglesDeg.Length - 1)];
			int angularSamples = ComputeSectorAngularSamples(sectorHalfAngleRad, coarseSearch ? coarseAngularSamples : dockingAngularSamples);
			List<DockingCandidate> candidates = new List<DockingCandidate>();
			for (int radiusIndex = 0; radiusIndex < radii.Count; radiusIndex++)
			{
				float radius = radii[radiusIndex];
				for (int angleIndex = 0; angleIndex < angularSamples; angleIndex++)
				{
					float angleRad = SampleSectorAngle(centerAngleRad, sectorHalfAngleRad, angleIndex, angularSamples);
					DockingCandidate candidate = CreateDockingCandidate(
						baseRoot,
						armController,
						armTargetWorldPosition,
						currentBasePosition,
						armBaseLocalPosition,
						armBaseLocalRotation,
						minReach,
						maxReach,
						preferredMin,
						preferredMax,
						baseRadius,
						sectorStage,
						SectorLabels[Mathf.Clamp(sectorStage, 0, SectorLabels.Length - 1)],
						classifyRadiusBand(radius, preferredMin, preferredMax),
						angleRad,
						radius);
					if (candidate == null)
					{
						continue;
					}

					if (!physicsQueries.IsBasePoseCollisionFree(candidate.baseWorldPosition, baseRadius, obstacles, out _))
					{
						candidate.evaluationStageSummary = "RejectedByBaseCollision";
						continue;
					}

					candidates.Add(candidate);
				}
			}

			SortDockingCandidates(candidates);
			return candidates;
		}

		private List<DockingCandidate> RankAndTrimCoarseCandidates(List<DockingCandidate> candidates)
		{
			List<DockingCandidate> ranked = new List<DockingCandidate>();
			if (candidates == null || candidates.Count == 0)
			{
				return ranked;
			}

			int keepCount = Mathf.Min(candidates.Count, Mathf.Max(1, coarseSeedKeepCount));
			for (int i = 0; i < keepCount; i++)
			{
				ranked.Add(candidates[i]);
			}

			return ranked;
		}

		private List<DockingCandidate> ExpandFineCandidatesAroundSeeds(
			Transform baseRoot,
			Arm6DOFFKController armController,
			Vector3 armTargetWorldPosition,
			Vector3 currentBasePosition,
			Vector3 armBaseLocalPosition,
			Quaternion armBaseLocalRotation,
			float minReach,
			float maxReach,
			float preferredMin,
			float preferredMax,
			float baseRadius,
			PlannerPhysicsQueries physicsQueries,
			IReadOnlyList<Collider> obstacles,
			List<DockingCandidate> coarseSeeds)
		{
			List<DockingCandidate> fineCandidates = new List<DockingCandidate>();
			if (coarseSeeds == null || coarseSeeds.Count == 0)
			{
				return fineCandidates;
			}

			float[] radiusOffsets = fineRadiusOffsetsMeters != null && fineRadiusOffsetsMeters.Length > 0
				? fineRadiusOffsetsMeters
				: new[] { 0f };
			float[] angleOffsets = fineAngleOffsetsDeg != null && fineAngleOffsetsDeg.Length > 0
				? fineAngleOffsetsDeg
				: new[] { 0f };

			for (int seedIndex = 0; seedIndex < coarseSeeds.Count; seedIndex++)
			{
				DockingCandidate seed = coarseSeeds[seedIndex];
				float seedAngleRad = seed.targetAngleRad;

				for (int radiusIndex = 0; radiusIndex < radiusOffsets.Length; radiusIndex++)
				{
						float radius = Mathf.Clamp(seed.radius + radiusOffsets[radiusIndex], minReach, maxReach);
					for (int angleIndex = 0; angleIndex < angleOffsets.Length; angleIndex++)
					{
						float angleRad = seedAngleRad + (angleOffsets[angleIndex] * Mathf.Deg2Rad);
						DockingCandidate candidate = CreateDockingCandidate(
							baseRoot,
							armController,
							armTargetWorldPosition,
							currentBasePosition,
							armBaseLocalPosition,
							armBaseLocalRotation,
							minReach,
							maxReach,
							preferredMin,
							preferredMax,
							baseRadius,
							Array.IndexOf(SectorLabels, seed.sectorLabel),
							seed.sectorLabel,
							seed.radiusBand,
							angleRad,
							radius);
						if (candidate == null
							|| IsNearDuplicateCandidate(fineCandidates, candidate)
							|| !physicsQueries.IsBasePoseCollisionFree(candidate.baseWorldPosition, baseRadius, obstacles, out _))
						{
							continue;
						}

						fineCandidates.Add(candidate);
					}
				}
			}

			SortDockingCandidates(fineCandidates);
			if (fineCandidates.Count > Mathf.Max(1, maxFineCandidateEvaluations))
			{
				fineCandidates.RemoveRange(Mathf.Max(1, maxFineCandidateEvaluations), fineCandidates.Count - Mathf.Max(1, maxFineCandidateEvaluations));
			}

			return fineCandidates;
		}

		private DockingCandidate CreateDockingCandidate(
			Transform baseRoot,
			Arm6DOFFKController armController,
			Vector3 armTargetWorldPosition,
			Vector3 currentBasePosition,
			Vector3 armBaseLocalPosition,
			Quaternion armBaseLocalRotation,
			float minReach,
			float maxReach,
			float preferredMin,
			float preferredMax,
			float baseRadius,
			int sectorStage,
			string sectorLabel,
			string radiusBand,
			float angleRad,
			float radius)
		{
			if (radius < minReach - 1e-4f || radius > maxReach + 1e-4f)
			{
				return null;
			}

			Vector3 directionToTarget = new Vector3(Mathf.Cos(angleRad), 0f, Mathf.Sin(angleRad));
			if (directionToTarget.sqrMagnitude <= 1e-6f)
			{
				return null;
			}

			directionToTarget.Normalize();
			Quaternion desiredFutureArmBaseRotation = Quaternion.LookRotation(directionToTarget, Vector3.up);
			Quaternion candidateRotation = desiredFutureArmBaseRotation * Quaternion.Inverse(armBaseLocalRotation);
			Vector3 futureArmBasePosition = armTargetWorldPosition - (directionToTarget * radius);
			Vector3 candidateBasePosition = futureArmBasePosition - (candidateRotation * armBaseLocalPosition);
			candidateBasePosition.y = currentBasePosition.y;
			Quaternion candidateBaseRotation = Quaternion.Euler(0f, candidateRotation.eulerAngles.y, 0f);
			if (!TryGetTargetInFutureArmBase(
				baseRoot,
				candidateBasePosition,
				candidateBaseRotation,
				armController,
				armTargetWorldPosition,
				out Vector3 targetInFutureArmBase))
			{
				return null;
			}

			float targetDistanceInFutureArmBase = targetInFutureArmBase.magnitude;
			if (Mathf.Abs(targetDistanceInFutureArmBase - radius) > DockingGeometryRadiusConsistencyToleranceMeters)
			{
				return null;
			}

			if (!TryGetFutureArmBasePose(
				baseRoot,
				candidateBasePosition,
				candidateBaseRotation,
				armController,
				out futureArmBasePosition,
				out Quaternion futureArmBaseRotation))
			{
				return null;
			}

			float travelDistance = PlanarDistance(currentBasePosition, candidateBasePosition);
			float shellMargin = Mathf.Max(0f, Mathf.Min(targetDistanceInFutureArmBase - minReach, maxReach - targetDistanceInFutureArmBase));
			float shellMarginPenalty = Mathf.Max(0f, 0.12f - shellMargin) * 8f;
			float clearance = Mathf.Max(0f, _distanceFieldSampler.SampleDistance(candidateBasePosition) - baseRadius);
			float preferredBandPenalty = ComputePreferredBandPenalty(targetDistanceInFutureArmBase, preferredMin, preferredMax);
			float workspaceBandPenalty = preferredBandPenalty + Mathf.Max(0f, Mathf.Abs(targetDistanceInFutureArmBase - radius) - 0.01f) * 4f;
			float heuristicCost =
				(workspaceBandPenalty * 16f) +
				shellMarginPenalty +
				(travelDistance * 0.8f) -
				Mathf.Min(clearance, 3f) * 0.2f;

			return new DockingCandidate
			{
				baseWorldPosition = candidateBasePosition,
				baseYawDeg = candidateBaseRotation.eulerAngles.y,
				targetAngleRad = angleRad,
				radius = radius,
				shellMargin = shellMargin,
				preferredBandPenalty = preferredBandPenalty,
				workspaceBandPenalty = workspaceBandPenalty,
				travelDistance = travelDistance,
				clearance = clearance,
				heuristicCost = heuristicCost,
				futureArmBasePosition = futureArmBasePosition,
				targetInFutureArmBase = targetInFutureArmBase,
				targetDistanceInFutureArmBase = targetDistanceInFutureArmBase,
				futureArmBaseEulerAngles = futureArmBaseRotation.eulerAngles,
				sectorLabel = sectorLabel,
				radiusBand = radiusBand,
				evaluationStageSummary = "AcceptedAsSeed"
			};
		}

		private static int ComputeSectorAngularSamples(float sectorHalfAngleRad, int baseSamples)
		{
			float coverageRatio = Mathf.Clamp01((sectorHalfAngleRad * 2f) / (Mathf.PI * 2f));
			return Mathf.Max(5, Mathf.RoundToInt(Mathf.Max(8, baseSamples) * Mathf.Max(coverageRatio, 0.35f)));
		}

		private static float SampleSectorAngle(float centerAngleRad, float sectorHalfAngleRad, int sampleIndex, int sampleCount)
		{
			if (sampleCount <= 1 || sectorHalfAngleRad >= Mathf.PI - 1e-4f)
			{
				return sampleCount <= 1
					? centerAngleRad
					: sampleIndex * Mathf.PI * 2f / Mathf.Max(1, sampleCount);
			}

			float t = sampleCount == 1 ? 0.5f : sampleIndex / (float)(sampleCount - 1);
			return centerAngleRad + Mathf.Lerp(-sectorHalfAngleRad, sectorHalfAngleRad, t);
		}

		private static int CountPreferredBandConsistent(List<DockingCandidate> candidates, float preferredMin, float preferredMax)
		{
			if (candidates == null)
			{
				return 0;
			}

			int count = 0;
			for (int i = 0; i < candidates.Count; i++)
			{
				DockingCandidate candidate = candidates[i];
				if (candidate != null
					&& candidate.targetDistanceInFutureArmBase >= preferredMin - DockingPreferredRadiusMarginMeters
					&& candidate.targetDistanceInFutureArmBase <= preferredMax + DockingPreferredRadiusMarginMeters)
				{
					count++;
				}
			}

			return count;
		}

		private static string classifyRadiusBand(float radius, float preferredMin, float preferredMax)
		{
			if (radius < preferredMin)
			{
				return "InnerBand";
			}

			if (radius > preferredMax)
			{
				return "OuterBand";
			}

			return "PreferredBand";
		}

		private static bool IsNearDuplicateCandidate(List<DockingCandidate> candidates, DockingCandidate candidate)
		{
			if (candidates == null || candidate == null)
			{
				return false;
			}

			for (int i = 0; i < candidates.Count; i++)
			{
				DockingCandidate existing = candidates[i];
				if (existing == null)
				{
					continue;
				}

				if (PlanarDistance(existing.baseWorldPosition, candidate.baseWorldPosition) <= 0.015f
					&& Mathf.Abs(Mathf.DeltaAngle(existing.baseYawDeg, candidate.baseYawDeg)) <= 2f)
				{
					return true;
				}
			}

			return false;
		}

		private static void SortDockingCandidates(List<DockingCandidate> candidates)
		{
			if (candidates == null)
			{
				return;
			}

			candidates.Sort((left, right) =>
			{
				int costOrder = left.heuristicCost.CompareTo(right.heuristicCost);
				if (costOrder != 0)
				{
					return costOrder;
				}

				int clearanceOrder = right.clearance.CompareTo(left.clearance);
				if (clearanceOrder != 0)
				{
					return clearanceOrder;
				}

				return left.radius.CompareTo(right.radius);
			});
		}

		private List<float> BuildDockingRadii(float minReach, float maxReach, float preferredMin, float preferredMax, int preferredSamples, int fallbackSamples, bool includeFallbackBand)
		{
			List<float> radii = new List<float>();
			AddInterpolatedRange(radii, preferredMin, preferredMax, Mathf.Max(2, preferredSamples));
			if (includeFallbackBand && fallbackSamples > 0)
			{
				AddInterpolatedRange(radii, minReach, preferredMin, Mathf.Max(1, fallbackSamples));
				AddInterpolatedRange(radii, preferredMax, maxReach, Mathf.Max(1, fallbackSamples));
			}
			return radii;
		}

		private static void AddInterpolatedRange(List<float> radii, float from, float to, int sampleCount)
		{
			if (radii == null || sampleCount <= 0)
			{
				return;
			}

			if (Mathf.Abs(to - from) <= 1e-4f)
			{
				AddUniqueRadius(radii, from);
				return;
			}

			for (int i = 0; i < sampleCount; i++)
			{
				float t = sampleCount == 1 ? 0.5f : i / (float)(sampleCount - 1);
				AddUniqueRadius(radii, Mathf.Lerp(from, to, t));
			}
		}

		private static void AddUniqueRadius(List<float> radii, float value)
		{
			for (int i = 0; i < radii.Count; i++)
			{
				if (Mathf.Abs(radii[i] - value) <= 1e-4f)
				{
					return;
				}
			}

			radii.Add(value);
		}

		private bool TryEvaluateArmFeasibilityAtBasePoseLoose(
			Transform baseRoot,
			Vector3 baseWorldPosition,
			Quaternion baseWorldRotation,
			Arm6DOFFKController armController,
			Vector3 armTargetWorldPosition,
			out float[] solvedAnglesDeg,
			out float residualMeters,
			out float singularityPenalty,
			out string failureReason,
			float targetToleranceMeters = -1f,
			float[] startAnglesOverrideDeg = null)
		{
			solvedAnglesDeg = null;
			residualMeters = float.PositiveInfinity;
			singularityPenalty = float.PositiveInfinity;
			failureReason = string.Empty;

			if (!TrySolveArmTargetAtBasePoseLoose(
				baseRoot,
				baseWorldPosition,
				baseWorldRotation,
				armController,
				armTargetWorldPosition,
				out float[] solvedAngles,
				out Vector3 targetInFutureArmBase,
				out residualMeters,
				out singularityPenalty,
				out string solveFailure,
				targetToleranceMeters,
				startAnglesOverrideDeg))
			{
				failureReason = solveFailure;
				return false;
			}

			if (singularityPenalty >= LoosePrecheckHardSingularityThreshold)
			{
				failureReason = L(
					$"前置机械臂初筛拒绝该位姿：奇异性惩罚 {singularityPenalty:F2} 已超过硬阈值。",
					$"Loose arm precheck rejected this pose because the singularity penalty {singularityPenalty:F2} exceeded the hard threshold.");
				return false;
			}

			if (EvaluatePredictedArmMotionAtBasePose(
				baseRoot,
				baseWorldPosition,
				baseWorldRotation,
				armController,
				solvedAngles,
				solvedAngles,
				out ArmCollisionGuardResult guardResult))
			{
				failureReason = string.IsNullOrEmpty(guardResult.message)
					? L("前置机械臂初筛发现目标终点姿态存在硬碰撞。", "Loose arm precheck found a hard collision at the terminal pose.")
					: guardResult.message;
				return false;
			}

			solvedAnglesDeg = CloneAngles(solvedAngles);
			return true;
		}

		private bool TrySolveArmTargetAtBasePoseLoose(
			Transform baseRoot,
			Vector3 baseWorldPosition,
			Quaternion baseWorldRotation,
			Arm6DOFFKController armController,
			Vector3 armTargetWorldPosition,
			out float[] solvedAnglesDeg,
			out Vector3 targetInFutureArmBase,
			out float residualMeters,
			out float singularityPenalty,
			out string failureReason,
			float targetToleranceMeters = -1f,
			float[] startAnglesOverrideDeg = null)
		{
			solvedAnglesDeg = null;
			targetInFutureArmBase = Vector3.zero;
			residualMeters = float.PositiveInfinity;
			singularityPenalty = float.PositiveInfinity;
			failureReason = string.Empty;

			if (armController == null || !armController.KinematicsReady)
			{
				failureReason = L("机械臂控制器或运动学模型尚未就绪。", "Arm controller or kinematics model is not ready.");
				return false;
			}

			if (!TryGetTargetInFutureArmBase(baseRoot, baseWorldPosition, baseWorldRotation, armController, armTargetWorldPosition, out targetInFutureArmBase))
			{
				failureReason = L("无法将目标点变换到未来机械臂基座坐标系。", "Could not transform the target into the future arm-base frame.");
				return false;
			}

			float solveToleranceMeters = Mathf.Max(
				targetToleranceMeters > 0f ? Mathf.Max(0.001f, targetToleranceMeters) : _armMotionPlanner.settings.toleranceMeters,
				LoosePrecheckToleranceMeters);
			float[] startAngles = startAnglesOverrideDeg != null && startAnglesOverrideDeg.Length >= 6
				? CloneAngles(startAnglesOverrideDeg)
				: armController.CaptureMeasuredJointAngles();
			if (!_armMotionPlanner.TrySolveToBasePosition(
				armController,
				targetInFutureArmBase,
				out float[] solvedAngles,
				out singularityPenalty,
				out string ikFailure,
				startAngles,
				null,
				solveToleranceMeters))
			{
				failureReason = ikFailure;
				return false;
			}

			residualMeters = Vector3.Distance(armController.ForwardPoe(solvedAngles).position, targetInFutureArmBase);
			solvedAnglesDeg = CloneAngles(solvedAngles);
			return true;
		}

		private bool TryEvaluateArmExecutionAtBasePose(
			Transform baseRoot,
			Vector3 baseWorldPosition,
			Quaternion baseWorldRotation,
			Arm6DOFFKController armController,
			Vector3 armTargetWorldPosition,
			out float[] solvedAnglesDeg,
			out float singularityPenalty,
			out string failureReason,
			float targetToleranceMeters = -1f,
			float[] startAnglesOverrideDeg = null)
		{
			solvedAnglesDeg = null;
			singularityPenalty = float.PositiveInfinity;
			failureReason = string.Empty;

			if (armController == null || !armController.KinematicsReady)
			{
				failureReason = L("机械臂控制器或运动学模型尚未就绪。", "Arm controller or kinematics model is not ready.");
				return false;
			}

			if (!TryGetTargetInFutureArmBase(baseRoot, baseWorldPosition, baseWorldRotation, armController, armTargetWorldPosition, out Vector3 targetInFutureArmBase))
			{
				failureReason = L("无法将目标点转换到未来机械臂基座坐标系。", "Could not transform the target into the future arm-base frame.");
				return false;
			}

			float solveToleranceMeters = targetToleranceMeters > 0f ? Mathf.Max(0.001f, targetToleranceMeters) : _armMotionPlanner.settings.toleranceMeters;

			float[] startAngles = startAnglesOverrideDeg != null && startAnglesOverrideDeg.Length >= 6
				? CloneAngles(startAnglesOverrideDeg)
				: armController.CaptureMeasuredJointAngles();
			if (!_armMotionPlanner.TrySolveToBasePosition(
				armController,
				targetInFutureArmBase,
				out float[] solvedAngles,
				out singularityPenalty,
				out string ikFailure,
				startAngles,
				null,
				solveToleranceMeters))
			{
				failureReason = ikFailure;
				return false;
			}

			solvedAnglesDeg = CloneAngles(solvedAngles);
			if (EvaluatePredictedArmMotionAtBasePose(baseRoot, baseWorldPosition, baseWorldRotation, armController, startAngles, solvedAngles, out ArmCollisionGuardResult guardResult))
			{
				failureReason = string.IsNullOrEmpty(guardResult.message)
					? L("机械臂碰撞预检拒绝了该底盘位姿。", "Arm collision preview rejected this base pose.")
					: guardResult.message;
				return false;
			}

			return true;
		}

		private bool EvaluatePredictedArmMotionAtBasePose(
			Transform baseRoot,
			Vector3 baseWorldPosition,
			Quaternion baseWorldRotation,
			Arm6DOFFKController armController,
			float[] startAnglesDeg,
			float[] targetAnglesDeg,
			out ArmCollisionGuardResult result)
		{
			result = new ArmCollisionGuardResult
			{
				allowed = true
			};

			ArmCollisionMonitor collisionMonitor = ArmCollisionMonitor.Instance;
			if (collisionMonitor == null)
			{
				collisionMonitor = UnityEngine.Object.FindObjectOfType<ArmCollisionMonitor>();
			}

			if (collisionMonitor == null)
			{
				return false;
			}

			float[] safeStart = startAnglesDeg != null && startAnglesDeg.Length >= 6
				? (float[])startAnglesDeg.Clone()
				: armController.CaptureMeasuredJointAngles();
			float[] safeTarget = targetAnglesDeg != null && targetAnglesDeg.Length >= 6
				? (float[])targetAnglesDeg.Clone()
				: safeStart;
			float maxDelta = 0f;
			for (int i = 0; i < 6; i++)
			{
				maxDelta = Mathf.Max(maxDelta, Mathf.Abs(safeTarget[i] - safeStart[i]));
			}

			int sampleCount = Mathf.Clamp(
				Mathf.CeilToInt(maxDelta / Mathf.Max(0.5f, collisionMonitor.previewStepDegrees)),
				1,
				Mathf.Max(1, collisionMonitor.maxPreviewSamples));
			bool hasFutureArmBasePose = TryGetFutureArmBasePose(
				baseRoot,
				baseWorldPosition,
				baseWorldRotation,
				armController,
				out Vector3 futureArmBasePosition,
				out Quaternion futureArmBaseRotation);
			float[] sampleAngles = new float[6];
			for (int sampleIndex = 1; sampleIndex <= sampleCount; sampleIndex++)
			{
				float t = sampleIndex / (float)sampleCount;
				for (int jointIndex = 0; jointIndex < 6; jointIndex++)
				{
					sampleAngles[jointIndex] = Mathf.Lerp(safeStart[jointIndex], safeTarget[jointIndex], t);
				}

				Pose[] futureLinkWorldPoses = ComputeFutureLinkWorldPoses(armController, sampleAngles, hasFutureArmBasePose, futureArmBasePosition, futureArmBaseRotation);
				if (collisionMonitor.EvaluatePredictedCollision(futureLinkWorldPoses, out result))
				{
					result.sampleIndex = sampleIndex;
					return true;
				}
			}

			return false;
		}

		private Pose[] ComputeFutureLinkWorldPoses(
			Arm6DOFFKController armController,
			float[] jointAnglesDeg,
			bool hasFutureArmBasePose,
			Vector3 futureArmBasePosition,
			Quaternion futureArmBaseRotation)
		{
			Pose[] baseLinkPoses = armController.ComputeLinkPosesBase(jointAnglesDeg);
			if (!hasFutureArmBasePose)
			{
				return baseLinkPoses;
			}

			Pose[] worldLinkPoses = new Pose[baseLinkPoses.Length];
			for (int i = 0; i < baseLinkPoses.Length; i++)
			{
				worldLinkPoses[i] = new Pose(
					futureArmBasePosition + futureArmBaseRotation * baseLinkPoses[i].position,
					futureArmBaseRotation * baseLinkPoses[i].rotation);
			}

			return worldLinkPoses;
		}

		private bool HasBasePathToCandidate(
			Vector3 startWorldPosition,
			float startYawDeg,
			Vector3 goalWorldPosition,
			float goalYawDeg,
			float baseRadius,
			PlannerPhysicsQueries physicsQueries,
			IReadOnlyList<Collider> obstacles,
			out string failureReason)
		{
			failureReason = string.Empty;
			bool startOccupied = !physicsQueries.IsBasePoseCollisionFree(startWorldPosition, baseRadius, obstacles, out Collider blockedStart);
			if (!physicsQueries.HasDirectRaycastBlock(startWorldPosition, goalWorldPosition, obstacles, out _)
				&& physicsQueries.IsSegmentCollisionFree(startWorldPosition, goalWorldPosition, baseRadius, obstacles, out _, startOccupied))
			{
				return true;
			}

			if (_basePlanner.TryPlan(
				startWorldPosition,
				startYawDeg,
				goalWorldPosition,
				goalYawDeg,
				baseRadius,
				_distanceFieldSampler,
				physicsQueries,
				obstacles,
				out _,
				out string plannerFailure))
			{
				return true;
			}

			failureReason = startOccupied
				? L(
					$"底盘路径搜索无法脱离已占用的起始位姿，附近障碍为 '{blockedStart?.name}'。",
					$"Base path search could not escape the occupied start pose near '{blockedStart?.name}'.")
				: plannerFailure;
			return false;
		}

		private void PrepareDockingDistanceField(
			Vector3 startWorldPosition,
			Vector3 armTargetWorldPosition,
			IReadOnlyList<Collider> obstacles,
			float maxReach,
			float baseRadius,
			Vector3 armBaseLocalPosition)
		{
			float searchMargin = Mathf.Max(
				2f,
				dockingPlanningMargin + maxReach + armBaseLocalPosition.magnitude + Mathf.Max(0.25f, baseRadius));
			_distanceFieldSampler.Build(
				startWorldPosition,
				armTargetWorldPosition,
				obstacles,
				Mathf.Max(0.1f, dockingDistanceFieldResolution),
				searchMargin);
		}

		private bool TryGetCachedArmBaseLocalOffset(
			Transform baseRoot,
			Arm6DOFFKController armController,
			out Vector3 armBaseLocalPosition,
			out Quaternion armBaseLocalRotation)
		{
			armBaseLocalPosition = Vector3.zero;
			armBaseLocalRotation = Quaternion.identity;
			if (baseRoot == null || armController == null || armController.BaseFrameTransform == null)
			{
				return false;
			}

			int baseRootId = baseRoot.GetInstanceID();
			int armBaseId = armController.BaseFrameTransform.GetInstanceID();
			if (_hasCachedArmBaseLocalOffset
				&& _cachedBaseRootId == baseRootId
				&& _cachedArmBaseTransformId == armBaseId)
			{
				armBaseLocalPosition = _cachedArmBaseLocalPosition;
				armBaseLocalRotation = _cachedArmBaseLocalRotation;
				return true;
			}

			if (!TryGetArmBaseLocalOffset(baseRoot, armController, out armBaseLocalPosition, out armBaseLocalRotation))
			{
				return false;
			}

			_hasCachedArmBaseLocalOffset = true;
			_cachedBaseRootId = baseRootId;
			_cachedArmBaseTransformId = armBaseId;
			_cachedArmBaseLocalPosition = armBaseLocalPosition;
			_cachedArmBaseLocalRotation = armBaseLocalRotation;
			return true;
		}

		private static bool TryGetArmBaseLocalOffset(
			Transform baseRoot,
			Arm6DOFFKController armController,
			out Vector3 armBaseLocalPosition,
			out Quaternion armBaseLocalRotation)
		{
			armBaseLocalPosition = Vector3.zero;
			armBaseLocalRotation = Quaternion.identity;
			if (baseRoot == null || armController == null || armController.BaseFrameTransform == null)
			{
				return false;
			}

			Transform armBase = armController.BaseFrameTransform;
			armBaseLocalPosition = baseRoot.InverseTransformPoint(armBase.position);
			armBaseLocalRotation = Quaternion.Inverse(baseRoot.rotation) * armBase.rotation;
			return true;
		}

		private bool TryGetFutureArmBasePose(
			Transform baseRoot,
			Vector3 baseWorldPosition,
			Quaternion baseWorldRotation,
			Arm6DOFFKController armController,
			out Vector3 futureArmBasePosition,
			out Quaternion futureArmBaseRotation)
		{
			futureArmBasePosition = baseWorldPosition;
			futureArmBaseRotation = baseWorldRotation;
			if (!TryGetCachedArmBaseLocalOffset(baseRoot, armController, out Vector3 armBaseLocalPosition, out Quaternion armBaseLocalRotation))
			{
				return false;
			}

			futureArmBasePosition = baseWorldPosition + baseWorldRotation * armBaseLocalPosition;
			futureArmBaseRotation = baseWorldRotation * armBaseLocalRotation;
			return true;
		}

		private bool TryGetTargetInFutureArmBase(
			Transform baseRoot,
			Vector3 baseWorldPosition,
			Quaternion baseWorldRotation,
			Arm6DOFFKController armController,
			Vector3 armTargetWorldPosition,
			out Vector3 targetInFutureArmBase)
		{
			targetInFutureArmBase = Vector3.zero;
			if (!TryGetFutureArmBasePose(baseRoot, baseWorldPosition, baseWorldRotation, armController, out Vector3 futureArmBasePosition, out Quaternion futureArmBaseRotation))
			{
				return false;
			}

			targetInFutureArmBase = Quaternion.Inverse(futureArmBaseRotation) * (armTargetWorldPosition - futureArmBasePosition);
			return true;
		}

		private void LogDockingCandidateGeometry(DockingCandidate candidate)
		{
			if (candidate == null)
			{
				return;
			}

			Debug.Log(
				$"[CoordinatedTaskPlanner] DockingCandidateGeometry: sectorStage={candidate.sectorLabel}, radiusBand={candidate.radiusBand}, candidateBasePosition={candidate.baseWorldPosition}, candidateYaw={candidate.baseYawDeg:F2}, futureArmBasePosition={candidate.futureArmBasePosition}, futureArmBaseRotation={candidate.futureArmBaseEulerAngles}, targetInFutureArmBase={candidate.targetInFutureArmBase}, |targetInFutureArmBase|={candidate.targetDistanceInFutureArmBase:F4}, status={candidate.evaluationStageSummary}");
		}

		private static CoordinatedTaskResolution CreateDefaultResolution(DiffDriveTwinController diffDriveController)
		{
			Vector3 basePosition = diffDriveController != null && diffDriveController.rb != null ? diffDriveController.rb.position : Vector3.zero;
			float baseYaw = diffDriveController != null && diffDriveController.rb != null ? diffDriveController.rb.rotation.eulerAngles.y : 0f;
			return new CoordinatedTaskResolution
			{
				resolvedBaseStopWorldPosition = basePosition,
				resolvedBaseStopYawDeg = baseYaw,
				dockingFailureCategory = FailureCategoryNone
			};
		}

		private static string BuildDockingFailureReason(DockingFailureSummary summary)
		{
			if (summary == null || !summary.anyWorkspaceShellCandidate)
			{
				return L(
					"所有采样到的停靠位都无法让目标点落入 Zu5 工作空间球壳。",
					"The target point is outside the Zu5 workspace shell for every sampled docking pose.");
			}

			if (!summary.anyIkCandidate)
			{
				return string.IsNullOrEmpty(summary.lastArmFailure)
					? L(
						"目标点虽然落在 Zu5 工作空间球壳内，但没有任何停靠位求解出有效机械臂关节解。",
						"The target point falls inside the Zu5 shell, but no valid arm joint solution was found for any docking pose.")
					: summary.lastArmFailure;
			}

			if (!summary.anyCollisionFreeArmCandidate)
			{
				return string.IsNullOrEmpty(summary.lastArmFailure)
					? L(
						"所有能求解机械臂目标的停靠位，都会与禁止碰撞体发生碰撞。",
						"Every docking pose that can solve the arm target would collide with the forbidden chassis or wheel set.")
					: summary.lastArmFailure;
			}

			return string.IsNullOrEmpty(summary.lastPathFailure)
				? L(
					"目标点在几何上可达，但没有任何停靠位能同时满足底盘可停、路径可达和机械臂碰撞预检。",
					"Although the target is geometrically reachable, no docking pose satisfied base parking, path reachability, and arm collision preview at the same time.")
				: summary.lastPathFailure;
		}

		private static string BuildDockingFailureReasonV3(DockingFailureSummary summary)
		{
			if (summary == null)
			{
				return "Docking search did not find a feasible candidate.";
			}

			if (summary.noCollisionFreeBasePoseCount > 0
				&& summary.noWorkspaceConsistentPoseCount == 0
				&& summary.noLooseIkPoseCount == 0
				&& summary.noPathReachablePoseCount == 0)
			{
				return "Every sampled base pose collided with the environment, so no collision-free docking pose exists.";
			}

			if (summary.noWorkspaceConsistentPoseCount > 0
				&& summary.noLooseIkPoseCount == 0
				&& summary.noPathReachablePoseCount == 0)
			{
				return "All geometrically generated candidates became inconsistent in the real arm-base frame, so the target stayed outside the workspace band.";
			}

			if (summary.noLooseIkPoseCount > 0 && summary.noPathReachablePoseCount == 0)
			{
				return string.IsNullOrEmpty(summary.lastArmFailure)
					? "Some docking poses were geometrically consistent in the real arm-base frame, but loose IK still could not find a usable arm solution."
					: summary.lastArmFailure;
			}

			if (summary.noPathReachablePoseCount > 0)
			{
				return string.IsNullOrEmpty(summary.lastPathFailure)
					? "A geometrically feasible arm-base pose existed, but the base path was not reachable or strict preview risk stayed too high."
					: summary.lastPathFailure;
			}

			return "Docking search did not find a usable candidate. Check the geometry logs for real arm-base transforms and per-candidate rejection reasons.";
		}

		/*
		private static string BuildDockingFailureReasonV2(DockingFailureSummary summary)
		{
			if (summary == null)
			{
				return L("鍋滈潬浣嶆悳绱㈡湭鎵惧埌鍙鍊欓€夈€?, "Docking search did not find a feasible candidate.");
			}

			if (summary.noCollisionFreeBasePoseCount > 0
				&& summary.noWorkspaceConsistentPoseCount == 0
				&& summary.noLooseIkPoseCount == 0
				&& summary.noPathReachablePoseCount == 0)
			{
				return L(
					"鎵€鏈夐噰鏍峰€欓€夊簳鐩樹綅濮块兘涓庨殰纰嶇墿鍙戠敓纰版挒锛屾病鏈夊彲鍋滈潬浣嶃€?,
					"Every sampled base pose collided with the environment, so no collision-free docking pose exists.");
			}

			if (summary.noWorkspaceConsistentPoseCount > 0
				&& summary.noLooseIkPoseCount == 0
				&& summary.noPathReachablePoseCount == 0)
			{
				return L(
					"鎵€鏈夐€氳繃鍑犱綍鍙嶈В鐨勫€欓€夛紝鍦ㄧ湡瀹?arm-base 鍧愭爣绯讳笅閮芥棤娉曡鐩爣鐐硅惤鍏?Zu5 宸ヤ綔鍖洪棿銆?,
					"All geometrically generated candidates became inconsistent in the real arm-base frame, so the target stayed outside the Zu5 workspace band.");
			}

			if (summary.noLooseIkPoseCount > 0 && summary.noPathReachablePoseCount == 0)
			{
				return string.IsNullOrEmpty(summary.lastArmFailure)
					? L(
						"鍋滈潬浣嶇殑鐪熷疄 arm-base 鍑犱綍宸茬粡鍙揪锛屼絾瀹芥澗 IK 浠嶇劧鏃犳硶姹傚嚭鍙敤鍏宠妭瑙ｃ€?,
						"Some docking poses were geometrically consistent in the real arm-base frame, but loose IK still could not find a usable arm solution.")
					: summary.lastArmFailure;
			}

			if (summary.noPathReachablePoseCount > 0)
			{
				return string.IsNullOrEmpty(summary.lastPathFailure)
					? L(
						"宸叉壘鍒板嚑浣曞彲琛岀殑 arm-base 鍋滈潬浣嶏紝浣嗗簳鐩樿矾寰勪粛鏃犳硶鍙揪鎴栦弗鏍奸妫€椋庨櫓杩囬珮銆?,
						"A geometrically feasible arm-base pose existed, but the base path was not reachable or strict preview risk stayed too high.")
					: summary.lastPathFailure;
			}

			return L(
				"鍋滈潬浣嶆悳绱㈡湭鎵惧埌鍙敤鍊欓€夛紝璇峰弬鑰冨嚑浣曟棩蹇楁鏌?arm-base 鍙樻崲涓庡€欓€夌瓫閫夊師鍥犮€?,
				"Docking search did not find a usable candidate. Check the geometry logs for real arm-base transforms and per-candidate rejection reasons.");
		}
		*/

		private bool TryEvaluateFineCandidate(
			DockingSearchContext context,
			DockingCandidate candidate,
			DockingFailureSummary failureSummary,
			ref int provisionalPathChecksUsed,
			ref DockingCandidate bestProvisionalCandidate,
			ref float bestProvisionalCost,
			ref string bestProvisionalReason,
			ref DockingCandidate bestCandidate,
			ref float bestCost,
			ref float[] bestSolvedAngles,
			ref int ikSolveCount,
			ref int basePathCheckCount)
		{
			Quaternion candidateRotation = Quaternion.Euler(0f, candidate.baseYawDeg, 0f);
			candidate.evaluationStageSummary = "EvaluatingFineCandidate";
			LogDockingCandidateGeometry(candidate);
			failureSummary.anyWorkspaceShellCandidate = true;
			if (candidate.targetDistanceInFutureArmBase >= context.preferredMin - DockingPreferredRadiusMarginMeters
				&& candidate.targetDistanceInFutureArmBase <= context.preferredMax + DockingPreferredRadiusMarginMeters)
			{
				failureSummary.preferredBandConsistentCount++;
			}
			ikSolveCount++;
			if (!TryEvaluateArmFeasibilityAtBasePoseLoose(
				context.baseRoot,
				candidate.baseWorldPosition,
				candidateRotation,
				context.armController,
				context.request.armTargetWorldPosition,
				out float[] solvedAngles,
				out float residualMeters,
				out float singularityPenalty,
				out string looseFailure,
				context.request != null ? context.request.eePositionToleranceMeters : -1f,
				context.startAngles))
			{
				failureSummary.lastArmFailure = looseFailure;
				failureSummary.noLooseIkPoseCount++;
				failureSummary.failureCategory = FailureCategoryNoLooseIkPose;
				candidate.evaluationStageSummary = "RejectedByLooseIK";
				LogDockingCandidateGeometry(candidate);
				return false;
			}

			failureSummary.anyIkCandidate = true;
			failureSummary.anyCollisionFreeArmCandidate = true;
			basePathCheckCount++;
			if (!HasBasePathToCandidate(
				context.currentBasePosition,
				context.currentBaseYaw,
				candidate.baseWorldPosition,
				candidate.baseYawDeg,
				context.baseRadius,
				context.physicsQueries,
				context.obstacles,
				out string pathFailure))
			{
				failureSummary.lastPathFailure = pathFailure;
				failureSummary.noPathReachablePoseCount++;
				failureSummary.failureCategory = FailureCategoryNoPathReachablePose;
				candidate.evaluationStageSummary = "RejectedByBasePath";
				LogDockingCandidateGeometry(candidate);
				return false;
			}

			float strictPreviewPenalty = 0f;
			if (EvaluatePredictedArmMotionAtBasePose(
				context.baseRoot,
				candidate.baseWorldPosition,
				candidateRotation,
				context.armController,
				context.startAngles,
				solvedAngles,
				out ArmCollisionGuardResult guardResult))
			{
				failureSummary.lastArmFailure = string.IsNullOrEmpty(guardResult.message)
					? L("机械臂碰撞预检拒绝了该停靠位。", "Arm collision preview rejected this docking pose.")
					: guardResult.message;
				failureSummary.noPathReachablePoseCount++;
				failureSummary.failureCategory = FailureCategoryNoPathReachablePose;
				strictPreviewPenalty += 8f;
			}

			if (allowProvisionalDockingWhenIkSoftFails
				&& strictPreviewPenalty > 0f
				&& provisionalPathChecksUsed < Mathf.Max(0, provisionalDockingPathCheckBudget)
				&& candidate.heuristicCost < bestProvisionalCost)
			{
				provisionalPathChecksUsed++;
				bestProvisionalCost = candidate.heuristicCost;
				bestProvisionalCandidate = candidate;
				bestProvisionalReason = string.IsNullOrEmpty(failureSummary.lastArmFailure)
					? L("宽松停靠位可达，但严格轨迹预检提示风险。", "Loose docking feasibility passed but strict trajectory preview reported risk.")
					: failureSummary.lastArmFailure;
			}

			float singularityPenaltyCost = Mathf.Clamp(singularityPenalty, 0f, LoosePrecheckHardSingularityThreshold) * 0.03f;
			if (singularityPenalty > LoosePrecheckSoftSingularityThreshold)
			{
				strictPreviewPenalty += (singularityPenalty - LoosePrecheckSoftSingularityThreshold) * 0.05f;
			}

			float armResidualScore = residualMeters * 120f;
			float workspaceBandScore = candidate.workspaceBandPenalty * 35f;
			float singularityScore = singularityPenaltyCost * 2.5f;
			float baseTravelScore = candidate.travelDistance * 1.2f;
			float pathScore = strictPreviewPenalty * 2f;
			float clearanceScore = -Mathf.Min(candidate.clearance, 2.5f) * 0.15f;
			float finalCost = armResidualScore
				+ workspaceBandScore
				+ singularityScore
				+ baseTravelScore
				+ pathScore
				+ clearanceScore
				+ candidate.heuristicCost * 0.25f;
			if (bestCandidate == null || finalCost < bestCost)
			{
				bestCost = finalCost;
				bestCandidate = candidate;
				bestSolvedAngles = CloneAngles(solvedAngles);
				candidate.evaluationStageSummary = "AcceptedAsBestDocking";
				LogDockingCandidateGeometry(candidate);
			}

			return true;
		}

		private static float ComputePreferredBandPenalty(float radius, float preferredMin, float preferredMax)
		{
			if (radius < preferredMin)
			{
				return preferredMin - radius;
			}

			if (radius > preferredMax)
			{
				return radius - preferredMax;
			}

			return 0f;
		}

		private static Vector3 ComputePrimarySectorDirection(Vector3 currentBasePosition, Vector3 armTargetWorldPosition, Quaternion currentBaseRotation)
		{
			Vector3 direction = ProjectXZ(armTargetWorldPosition - currentBasePosition);
			if (direction.sqrMagnitude <= 1e-6f)
			{
				direction = ProjectXZ(currentBaseRotation * Vector3.forward);
			}

			if (direction.sqrMagnitude <= 1e-6f)
			{
				return Vector3.forward;
			}

			return direction.normalized;
		}

		private static float PlanarDistance(Vector3 a, Vector3 b)
		{
			return Vector3.Distance(new Vector3(a.x, 0f, a.z), new Vector3(b.x, 0f, b.z));
		}

		private static Vector3 ProjectXZ(Vector3 value)
		{
			value.y = 0f;
			return value;
		}

		private static bool IsSoftArmPlanningFailure(string reason)
		{
			if (string.IsNullOrWhiteSpace(reason))
			{
				return false;
			}

			return reason.IndexOf("机械臂规划未收敛", StringComparison.OrdinalIgnoreCase) >= 0
				|| reason.IndexOf("Arm planner did not converge", StringComparison.OrdinalIgnoreCase) >= 0;
		}

		private static bool HasExplicitBaseGoal(RobotPlanRequest request, DiffDriveTwinController diffDriveController)
		{
			if (request == null || diffDriveController == null || diffDriveController.rb == null || !request.requireBaseMove)
			{
				return false;
			}

			return PlanarDistance(diffDriveController.rb.position, request.baseTargetWorldPosition) > BasePositionToleranceMeters;
		}

		private static Vector3 GetRequestedBasePosition(RobotPlanRequest request, DiffDriveTwinController diffDriveController)
		{
			if (request == null || diffDriveController == null || diffDriveController.rb == null)
			{
				return Vector3.zero;
			}

			Vector3 requested = request.baseTargetWorldPosition;
			requested.y = diffDriveController.rb.position.y;
			return requested;
		}

		private static float[] CloneAngles(float[] source)
		{
			return source != null && source.Length >= 6 ? (float[])source.Clone() : null;
		}

		private static string L(string chinese, string english)
		{
			return RobotSimulationLocalization.Text(chinese, english);
		}
	}
}
