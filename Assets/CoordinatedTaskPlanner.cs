using System;
using System.Collections.Generic;
using System.Globalization;
using UnityEngine;
using UnityEngine.AI;

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
		private const float DefaultAnnularSamplingInnerRadiusMeters = 0f;
		private const float DefaultAnnularSamplingOuterRadiusMeters = 0f;
		private const string AnnularSamplingLabel = "AnnularRandomSampling";
		private const float BasePositionToleranceMeters = 0.02f;
		private const float LoosePrecheckToleranceMeters = 0.05f;
		private const float LoosePrecheckHardSingularityThreshold = 140f;
		private const float LoosePrecheckSoftSingularityThreshold = 70f;
		private const float DockingGeometryRadiusConsistencyToleranceMeters = 0.035f;
		private const float DockingHardWorkspaceShellToleranceMeters = 0.01f;
		private const float DockingPreferredRadiusMarginMeters = 0.015f;
		private const float ArmBaseLocalOffsetWarnPositionDeltaMeters = 0.02f;
		private const float ArmBaseLocalOffsetWarnRotationDeltaDeg = 1.0f;

		private static readonly float[] SectorHalfAnglesDeg = { 25f, 45f, 70f, 180f };
		private static readonly string[] SectorLabels =
		{
			"PrimarySectorSearch",
			"ExpandedSectorSearch45",
			"ExpandedSectorSearch70",
			"FallbackFullRingSearch"
		};
		private static readonly Vector2[] ResidualDescentDirections =
		{
			new Vector2(1f, 0f),
			new Vector2(-1f, 0f),
			new Vector2(0f, 1f),
			new Vector2(0f, -1f),
			new Vector2(1f, 1f),
			new Vector2(1f, -1f),
			new Vector2(-1f, 1f),
			new Vector2(-1f, -1f)
		};

		private const string FailureCategoryNone = "none";
		private const string FailureCategoryNoCollisionFreeBasePose = "NoCollisionFreeBasePose";
		private const string FailureCategoryNoWorkspaceConsistentPose = "NoWorkspaceConsistentPose";
		private const string FailureCategoryNoLooseIkPose = "NoLooseIkPose";
		private const string FailureCategoryNoPathReachablePose = "NoPathReachablePose";
		private const string FailureCategoryNoFineCandidateAfterRefinement = "NoFineCandidateAfterRefinement";

		internal sealed class DockingCandidate
		{
			public Vector3 baseWorldPosition;
			public float baseYawDeg;
			public float targetAngleRad;
			public float targetBearingLocalDeg;
			public float radius;
			public float sampledRadiusMeters;
			public float shellMargin;
			public float preferredBandPenalty;
			public float workspaceBandPenalty;
			public float travelDistance;
			public float planarTargetDistanceMeters;
			public float ikResidualMeters;
			public float distanceBandPenaltyMeters;
			public int descentIteration;
			public string searchSource;
			public float clearance;
			public float heuristicCost;
			public float manipulabilityIndex;
			public float looseIkResidualMeters;
			public float looseIkSingularityPenalty;
			public float minJointLimitMarginDeg;
			public float jointLimitPenalty;
			public Vector3 futureArmBasePosition;
			public Vector3 targetInFutureArmBase;
			public float targetDistanceInFutureArmBase;
			public Vector3 futureArmBaseEulerAngles;
			public string sectorLabel;
			public string sampledRadiusBand;
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
			internal Vector3 baseCollisionBoxHalfExtents;
			internal float minReach;
			internal float maxReach;
			internal float preferredMin;
			internal float preferredMax;
			internal Vector3 armBaseLocalPosition;
			internal Quaternion armBaseLocalRotation;
			internal Vector3 primarySectorDirection;
			internal int sectorStage;
			internal string samplingLabel;
			internal float[] startAngles;
			internal List<DockingCandidate> coarseCandidates;
			internal List<DockingCandidate> fineCandidates;
		}

		public sealed class DockingDebugSample
		{
			public Vector3 baseWorldPosition;
			public Vector3 targetWorldPosition;
			public bool ikFailed;
			public bool strictPreviewFailed;
			public bool selected;
			public string reason = string.Empty;
		}

		private sealed class DockingCandidateBuildStats
		{
			public int sampledCount;
			public int geometryRejectedCount;
			public int navMeshRejectedCount;
			public int boxRejectedCount;
			public int collisionRejectedCount;
			public int acceptedCount;

			public string ToDebugString()
			{
				return $"sampled={sampledCount}, accepted={acceptedCount}, geometryRejected={geometryRejectedCount}, navMeshRejected={navMeshRejectedCount}, boxRejected={boxRejectedCount}, collisionRejected={collisionRejectedCount}";
			}
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

		private sealed class ResidualDescentSample
		{
			public Vector3 baseWorldPosition;
			public float baseYawDeg;
			public bool collisionBlocked;
			public bool geometryValid;
			public bool ikConverged;
			public float ikResidualMeters = float.PositiveInfinity;
			public float planarTargetDistanceMeters = float.PositiveInfinity;
			public float distanceBandPenaltyMeters = float.PositiveInfinity;
			public float moveDistanceMeters = float.PositiveInfinity;
			public string failureReason = string.Empty;
			public string searchSource = string.Empty;
			public int descentIteration = -1;
			public DockingCandidate candidate;
		}

		private readonly BaseRrtStarPlanner _basePlanner = new BaseRrtStarPlanner();
		private readonly SceneDistanceFieldSampler _distanceFieldSampler = new SceneDistanceFieldSampler();
		private readonly ArmMotionPlanner _armMotionPlanner = new ArmMotionPlanner();
		private bool _hasCachedArmBaseLocalOffset;
		private int _cachedBaseRootId;
		private int _cachedArmBaseTransformId;
		private Vector3 _cachedArmBaseLocalPosition;
		private Quaternion _cachedArmBaseLocalRotation = Quaternion.identity;
		private bool _hasDockingSamplingDebugInfo;
		private Vector3 _lastDockingTargetWorldPosition;
		private float _lastDockingSamplingInnerRadiusMeters;
		private float _lastDockingSamplingOuterRadiusMeters;
		private readonly List<DockingDebugSample> _lastDockingDebugSamples = new List<DockingDebugSample>();

		public int dockingAngularSamples = 60;
		public int preferredBandRadiusSamples = 7;
		public int fallbackBandRadiusSamples = 5;
		public int coarseAngularSamples = 72;
		public int coarsePreferredBandRadiusSamples = 7;
		public int coarseFallbackBandRadiusSamples = 4;
		public int coarseSeedKeepCount = 12;
		public float[] fineRadiusOffsetsMeters = { -0.06f, -0.03f, 0f, 0.03f, 0.06f };
		public float[] fineAngleOffsetsDeg = { -12f, -6f, 0f, 6f, 12f };
		public float[] coarseTargetBearingOffsetsDeg = { 0f, -35f, 35f, -70f, 70f };
		public float[] fineTargetBearingOffsetsDeg = { 0f, -15f, 15f };
		public int maxFineCandidateEvaluations = 48;
		public float dockingDistanceFieldResolution = 0.35f;
		public float dockingPlanningMargin = 3.5f;
		public float annularSamplingInnerRadiusMeters = DefaultAnnularSamplingInnerRadiusMeters;
		public float annularSamplingOuterRadiusMeters = DefaultAnnularSamplingOuterRadiusMeters;
		public float annularSamplingWorkspaceMarginMeters = 0.02f;
		public int annularRandomSampleCount = 128;
		public int annularAnchorSampleCount = 12;
		public int dockingDebugSampleLimit = 128;
		public float residualDescentCoarseStepMeters = 0.10f;
		public float residualDescentFineStepMeters = 0.02f;
		public float residualDescentCoarseMaxStepMeters = 0.32f;
		public float residualDescentFineMaxStepMeters = 0.08f;
		public float residualDescentDynamicStepGain = 0.60f;
		public float residualDescentAcceptResidualFloorMeters = 0.05f;
		public int residualDescentMaxCoarseIterations = 8;
		public int residualDescentMaxFineIterations = 16;
		public bool enableNavMeshPoseFilter = true;
		public float navMeshSampleMaxDistanceMeters = 1.0f;
		public int navMeshAreaMask = NavMesh.AllAreas;
		public bool enableStaticBoxCollisionFilter = true;
		public float baseCollisionBoxPaddingMeters = 0.02f;
		public float jointLimitHardMarginDeg = 8f;
		public float jointLimitSoftMarginDeg = 18f;
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

		public bool TryGetLastDockingSamplingDebugInfo(out Vector3 targetWorldPosition, out float innerRadiusMeters, out float outerRadiusMeters)
		{
			targetWorldPosition = _lastDockingTargetWorldPosition;
			innerRadiusMeters = _lastDockingSamplingInnerRadiusMeters;
			outerRadiusMeters = _lastDockingSamplingOuterRadiusMeters;
			return _hasDockingSamplingDebugInfo;
		}

		public int CopyLastDockingDebugSamples(List<DockingDebugSample> destination)
		{
			if (destination == null)
			{
				return 0;
			}

			destination.Clear();
			for (int i = 0; i < _lastDockingDebugSamples.Count; i++)
			{
				DockingDebugSample sample = _lastDockingDebugSamples[i];
				if (sample == null)
				{
					continue;
				}

				destination.Add(new DockingDebugSample
				{
					baseWorldPosition = sample.baseWorldPosition,
					targetWorldPosition = sample.targetWorldPosition,
					ikFailed = sample.ikFailed,
					strictPreviewFailed = sample.strictPreviewFailed,
					selected = sample.selected,
					reason = sample.reason
				});
			}

			return destination.Count;
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

			LogCurrentBaseFrameConsistency(
				diffDriveController.rb.transform,
				diffDriveController.rb.position,
				diffDriveController.rb.rotation,
				armController,
				request.armTargetWorldPosition,
				request != null ? request.eePositionToleranceMeters : -1f);

			if (TryEvaluateArmFeasibilityAtLiveBaseLoose(
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
				Debug.Log($"[CoordinatedTaskPlanner] LooseReachabilityAccepted(live-base): residual={residualMeters:F4}m, singularityPenalty={singularityPenalty:F2}");
				resolution.dockingSummary = L(
					"当前底盘位姿已经支持所请求的末端世界目标，因此无需移动底盘。",
					"Current base pose already supports the requested end-effector world target, so no base move is required.");
				return resolution;
			}

			bool projectedAccepted = TryEvaluateArmFeasibilityAtBasePoseLoose(
				diffDriveController.rb.transform,
				diffDriveController.rb.position,
				diffDriveController.rb.rotation,
				armController,
				request.armTargetWorldPosition,
				out _,
				out float projectedResidualMeters,
				out float projectedSingularityPenalty,
				out string projectedFailureReason,
				request != null ? request.eePositionToleranceMeters : -1f);
			if (projectedAccepted || !string.Equals(projectedFailureReason, currentBaseFailure, StringComparison.Ordinal))
			{
				Debug.LogWarning(
					$"[CoordinatedTaskPlanner] CurrentBaseReachability divergence: liveAccepted=false, projectedAccepted={projectedAccepted}, liveFailure='{currentBaseFailure}', projectedFailure='{projectedFailureReason}', projectedResidual={projectedResidualMeters:F4}m, projectedSingularity={projectedSingularityPenalty:F2}");
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
			ClearDockingDebugSamples();
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
			float currentBaseYaw = diffDriveController.rb.rotation.eulerAngles.y;
			float minReach = armController.GetMinReach() > 0f ? armController.GetMinReach() : DefaultWorkspaceInnerRadiusMeters;
			float maxReach = armController.GetMaxReach() > 0f ? armController.GetMaxReach() : DefaultWorkspaceOuterRadiusMeters;
			Vector3 liveTargetInCurrentArmBase = armController.WorldToBasePosition(request.armTargetWorldPosition);
			float liveTargetDistanceInCurrentArmBase = liveTargetInCurrentArmBase.magnitude;
			ResolvePreferredAnnularBand(minReach, maxReach, liveTargetDistanceInCurrentArmBase, out float preferredMin, out float preferredMax);
			Debug.Log($"[CoordinatedTaskPlanner] Docking search radius anchor: liveTargetInCurrentArmBase={liveTargetInCurrentArmBase}, liveRadius={liveTargetDistanceInCurrentArmBase:F4}, preferredBand=[{preferredMin:F4}, {preferredMax:F4}]");
			CacheDockingSamplingDebugInfo(request.armTargetWorldPosition, preferredMin, preferredMax);
			bool navMeshFilterActive = ShouldUseNavMeshPoseFilter(currentBasePosition);
			float[] startAngles = armController.CaptureMeasuredJointAngles();
			Vector3 baseCollisionBoxHalfExtents = physicsQueries != null
				? physicsQueries.EstimateBaseCollisionBoxHalfExtents(diffDriveController)
				: new Vector3(baseRadius, 0.35f, baseRadius);
			baseCollisionBoxHalfExtents.x += Mathf.Max(0f, baseCollisionBoxPaddingMeters);
			baseCollisionBoxHalfExtents.z += Mathf.Max(0f, baseCollisionBoxPaddingMeters);
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
				request.eePositionToleranceMeters,
				liveTargetDistanceInCurrentArmBase,
				baseRadius,
				currentBaseYaw,
				baseCollisionBoxHalfExtents,
				physicsQueries,
				obstacles,
				primarySectorDirection,
				navMeshFilterActive,
				out DockingCandidateBuildStats coarseBuildStats,
				out string coarseSamplingLabel,
				out int selectedSectorStage,
				out int preferredBandConsistentCount);

			resolution.coarseCandidateCount = coarseCandidates.Count;
			if (coarseCandidates.Count == 0)
			{
				resolution.dockingFailureCategory = FailureCategoryNoCollisionFreeBasePose;
				Collider blockedCurrentPose = null;
				bool currentPoseCollisionFree = physicsQueries != null && physicsQueries.IsBasePoseCollisionFree(currentBasePosition, baseRadius, obstacles, out blockedCurrentPose);
				Debug.LogWarning($"[CoordinatedTaskPlanner] Docking coarse search found zero collision-free candidates. baseRadius={baseRadius:F3}, baseFootprintHalfExtents={baseCollisionBoxHalfExtents}, currentPoseCollisionFree={currentPoseCollisionFree}, blockedCurrentPose={(blockedCurrentPose != null ? blockedCurrentPose.name : "none")}, obstacleCount={(obstacles != null ? obstacles.Count : 0)}, sampling={coarseSamplingLabel}, primaryDirection={primarySectorDirection}, navMeshFilterActive={navMeshFilterActive}, staticBoxFilterActive={enableStaticBoxCollisionFilter}, stats={(coarseBuildStats != null ? coarseBuildStats.ToDebugString() : "none")}");
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
				baseCollisionBoxHalfExtents,
				physicsQueries,
				obstacles,
				navMeshFilterActive,
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
				baseCollisionBoxHalfExtents = baseCollisionBoxHalfExtents,
				minReach = minReach,
				maxReach = maxReach,
				preferredMin = preferredMin,
				preferredMax = preferredMax,
				armBaseLocalPosition = armBaseLocalPosition,
				armBaseLocalRotation = armBaseLocalRotation,
				primarySectorDirection = primarySectorDirection,
				sectorStage = selectedSectorStage,
				samplingLabel = coarseSamplingLabel,
				startAngles = CloneAngles(startAngles),
				coarseCandidates = coarseCandidates,
				fineCandidates = fineCandidates
			};

			resolution.fineCandidateCount = fineCandidates.Count;
			resolution.dockingFailureCategory = FailureCategoryNone;
			resolution.dockingSummary = L(
				$"停靠位粗筛保留了 {fineCandidates.Count} 个精筛候选（原始无碰撞候选 {coarseCandidates.Count} 个）。",
				$"Docking search found {coarseCandidates.Count} coarse candidates in {coarseSamplingLabel}; {preferredBandConsistentCount} remained inside the preferred band under the real arm-base frame, and {fineCandidates.Count} advanced to fine screening.");
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
			if (fineCandidates.Count <= 0)
			{
				resolution.dockingFailureCategory = FailureCategoryNoFineCandidateAfterRefinement;
				resolution.failureReason = L(
					$"停靠位粗筛已找到 {resolution.coarseCandidateCount} 个候选，但精筛局部细化后没有保留下任何可评估候选。",
					$"Docking coarse search found {resolution.coarseCandidateCount} candidates, but fine-stage refinement did not preserve any evaluable candidate.");
				resolution.dockingSummary = resolution.failureReason;
				Debug.LogWarning($"[CoordinatedTaskPlanner] Docking fine refinement produced zero candidates. sampling={context.samplingLabel ?? AnnularSamplingLabel}, coarse={resolution.coarseCandidateCount}, fine={resolution.fineCandidateCount}");
				return false;
			}

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
					Debug.Log($"[CoordinatedTaskPlanner] Docking summary: sampling={bestProvisionalCandidate.sectorLabel}, coarse={resolution.coarseCandidateCount}, fine={resolution.fineCandidateCount}, preferredConsistent={failureSummary.preferredBandConsistentCount}, ik={resolution.ikSolveCount}, pathChecks={resolution.basePathCheckCount}, foundProvisional=true, targetInFutureArmBase={bestProvisionalCandidate.targetInFutureArmBase}, targetDistance={bestProvisionalCandidate.targetDistanceInFutureArmBase:F4}");
					return true;
				}

				resolution.failureReason = BuildDockingFailureReasonV3(failureSummary);
				resolution.dockingSummary = resolution.failureReason;
				Debug.Log($"[CoordinatedTaskPlanner] Docking summary: sampling={context.samplingLabel ?? AnnularSamplingLabel}, coarse={resolution.coarseCandidateCount}, fine={resolution.fineCandidateCount}, preferredConsistent={failureSummary.preferredBandConsistentCount}, ik={resolution.ikSolveCount}, pathChecks={resolution.basePathCheckCount}, found=false, category={resolution.dockingFailureCategory}");
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
			Debug.Log($"[CoordinatedTaskPlanner] Docking summary: sampling={bestCandidate.sectorLabel}, coarse={resolution.coarseCandidateCount}, fine={resolution.fineCandidateCount}, preferredConsistent={failureSummary.preferredBandConsistentCount}, ik={resolution.ikSolveCount}, pathChecks={resolution.basePathCheckCount}, found=true, targetInFutureArmBase={bestCandidate.targetInFutureArmBase}, targetDistance={bestCandidate.targetDistanceInFutureArmBase:F4}");
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
			float requestedEeToleranceMeters,
			float referenceRadiusMeters,
			float baseRadius,
			float currentBaseYaw,
			Vector3 baseCollisionBoxHalfExtents,
			PlannerPhysicsQueries physicsQueries,
			IReadOnlyList<Collider> obstacles,
			Vector3 primarySectorDirection,
			bool navMeshFilterActive,
			out DockingCandidateBuildStats buildStats,
			out string samplingLabel,
			out int selectedSectorStage,
			out int preferredBandConsistentCount)
		{
			buildStats = new DockingCandidateBuildStats();
			samplingLabel = "MidpointResidualDescent";
			selectedSectorStage = 0;
			preferredBandConsistentCount = 0;
			List<DockingCandidate> residualCandidates = BuildResidualDescentDockingCandidates(
				baseRoot,
				armController,
				armTargetWorldPosition,
				currentBasePosition,
				minReach,
				maxReach,
				preferredMin,
				preferredMax,
				requestedEeToleranceMeters,
				baseRadius,
				obstacles,
				buildStats,
				currentBaseYaw,
				out bool hitAcceptResidual,
				out bool foundImprovement);
			preferredBandConsistentCount = CountPreferredBandConsistent(residualCandidates, preferredMin, preferredMax);
			if (residualCandidates.Count > 0 && (hitAcceptResidual || foundImprovement))
			{
				return residualCandidates;
			}

			Debug.LogWarning(
				$"[CoordinatedTaskPlanner] MidpointResidualDescent exhausted without a strong improvement. residualCandidates={residualCandidates.Count}, hitAcceptResidual={hitAcceptResidual}, foundImprovement={foundImprovement}. Falling back to sector search.");
			samplingLabel = "SectorFallback";
			for (int sectorIndex = 0; sectorIndex < SectorLabels.Length; sectorIndex++)
			{
				List<DockingCandidate> sectorCandidates = BuildSectorSearchCandidates(
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
					referenceRadiusMeters,
					baseRadius,
					baseCollisionBoxHalfExtents,
					physicsQueries,
					obstacles,
					primarySectorDirection,
					sectorIndex,
					includeFallbackBand: true,
					navMeshFilterActive,
					buildStats,
					coarseSearch: true);
				if (sectorCandidates.Count <= 0)
				{
					continue;
				}

				for (int candidateIndex = 0; candidateIndex < sectorCandidates.Count; candidateIndex++)
				{
					DockingCandidate candidate = sectorCandidates[candidateIndex];
					if (candidate == null)
					{
						continue;
					}

					candidate.searchSource = "SectorFallback";
					candidate.descentIteration = -1;
				}

				for (int residualIndex = 0; residualIndex < residualCandidates.Count; residualIndex++)
				{
					DockingCandidate residualCandidate = residualCandidates[residualIndex];
					if (residualCandidate == null || IsNearDuplicateCandidate(sectorCandidates, residualCandidate))
					{
						continue;
					}

					sectorCandidates.Add(residualCandidate);
				}

				SortDockingCandidates(sectorCandidates);
				selectedSectorStage = sectorIndex;
				preferredBandConsistentCount = CountPreferredBandConsistent(sectorCandidates, preferredMin, preferredMax);
				return sectorCandidates;
			}

			if (residualCandidates.Count > 0)
			{
				samplingLabel = "MidpointResidualDescentNoImprovement";
				return residualCandidates;
			}

			return new List<DockingCandidate>();
		}

		private List<DockingCandidate> BuildResidualDescentDockingCandidates(
			Transform baseRoot,
			Arm6DOFFKController armController,
			Vector3 armTargetWorldPosition,
			Vector3 currentBasePosition,
			float minReach,
			float maxReach,
			float preferredMin,
			float preferredMax,
			float requestedEeToleranceMeters,
			float baseRadius,
			IReadOnlyList<Collider> obstacles,
			DockingCandidateBuildStats buildStats,
			float currentBaseYaw,
			out bool hitAcceptResidual,
			out bool foundImprovement)
		{
			List<DockingCandidate> candidates = new List<DockingCandidate>();
			hitAcceptResidual = false;
			foundImprovement = false;

			HashSet<Collider> obstacleLookup = BuildObstacleLookup(obstacles);
			float acceptResidualMeters = Mathf.Max(
				requestedEeToleranceMeters > 0f ? Mathf.Max(0.001f, requestedEeToleranceMeters) : _armMotionPlanner.settings.toleranceMeters,
				residualDescentAcceptResidualFloorMeters);
			Vector3 midpoint = new Vector3(
				(currentBasePosition.x + armTargetWorldPosition.x) * 0.5f,
				currentBasePosition.y,
				(currentBasePosition.z + armTargetWorldPosition.z) * 0.5f);
			ResidualDescentSample currentBaseSample = EvaluateResidualDescentSample(
				baseRoot,
				armController,
				armTargetWorldPosition,
				currentBasePosition,
				minReach,
				maxReach,
				preferredMin,
				preferredMax,
				requestedEeToleranceMeters,
				baseRadius,
				obstacleLookup,
				buildStats,
				currentBasePosition,
				currentBaseYaw,
				"CurrentBaseBaseline",
				0);
			ResidualDescentSample midpointSample = EvaluateResidualDescentSample(
				baseRoot,
				armController,
				armTargetWorldPosition,
				currentBasePosition,
				minReach,
				maxReach,
				preferredMin,
				preferredMax,
				requestedEeToleranceMeters,
				baseRadius,
				obstacleLookup,
				buildStats,
				midpoint,
				currentBaseYaw,
				"MidpointResidualDescent",
				0);

			LogResidualDescentSample("CurrentBaseBaseline", currentBaseSample);
			LogResidualDescentSample("MidpointResidualDescent", midpointSample);

			TryAddResidualDescentCandidate(candidates, currentBaseSample, false, buildStats);
			TryAddResidualDescentCandidate(candidates, midpointSample, false, buildStats);

			ResidualDescentSample globalBest = CompareResidualDescentSamples(currentBaseSample, midpointSample) <= 0
				? currentBaseSample
				: midpointSample;
			ResidualDescentSample activeSample = midpointSample;

			hitAcceptResidual =
				HasResidualDescentAccepted(currentBaseSample, acceptResidualMeters) ||
				HasResidualDescentAccepted(midpointSample, acceptResidualMeters);
			if (hitAcceptResidual)
			{
				TryAddResidualDescentCandidate(candidates, globalBest, true, buildStats);
				SortResidualDescentCandidates(candidates);
				return candidates;
			}

			for (int coarseIteration = 0; coarseIteration < Mathf.Max(1, residualDescentMaxCoarseIterations) && !hitAcceptResidual; coarseIteration++)
			{
				float coarseStepMeters = ComputeResidualDescentStepMeters(activeSample, acceptResidualMeters, fineSearch: false);
				ResidualDescentSample coarseImprovement = FindBestResidualDescentNeighbor(
					baseRoot,
					armController,
					armTargetWorldPosition,
					currentBasePosition,
					minReach,
					maxReach,
					preferredMin,
					preferredMax,
					requestedEeToleranceMeters,
					baseRadius,
					obstacleLookup,
					buildStats,
					activeSample,
					coarseStepMeters,
					coarseIteration + 1);
				if (coarseImprovement == null)
				{
					break;
				}

				foundImprovement = true;
				activeSample = coarseImprovement;
				globalBest = CompareResidualDescentSamples(activeSample, globalBest) < 0 ? activeSample : globalBest;
				LogAcceptedResidualNeighbor(activeSample, coarseStepMeters);
				TryAddResidualDescentCandidate(candidates, activeSample, false, buildStats);
				hitAcceptResidual = HasResidualDescentAccepted(activeSample, acceptResidualMeters);
			}

			if (!hitAcceptResidual)
			{
				for (int iteration = 0; iteration < Mathf.Max(1, residualDescentMaxFineIterations); iteration++)
				{
					float fineStepMeters = ComputeResidualDescentStepMeters(activeSample, acceptResidualMeters, fineSearch: true);
					ResidualDescentSample fineImprovement = FindBestResidualDescentNeighbor(
						baseRoot,
						armController,
						armTargetWorldPosition,
						currentBasePosition,
						minReach,
						maxReach,
						preferredMin,
						preferredMax,
						requestedEeToleranceMeters,
						baseRadius,
						obstacleLookup,
						buildStats,
						activeSample,
						fineStepMeters,
						activeSample.descentIteration + 1);
					if (fineImprovement == null)
					{
						break;
					}

					foundImprovement = true;
					activeSample = fineImprovement;
					globalBest = CompareResidualDescentSamples(activeSample, globalBest) < 0 ? activeSample : globalBest;
					LogAcceptedResidualNeighbor(activeSample, fineStepMeters);
					TryAddResidualDescentCandidate(candidates, activeSample, false, buildStats);
					if (HasResidualDescentAccepted(activeSample, acceptResidualMeters))
					{
						hitAcceptResidual = true;
						break;
					}
				}
			}

			TryAddResidualDescentCandidate(candidates, globalBest, true, buildStats);
			SortResidualDescentCandidates(candidates);
			return candidates;
		}

		private ResidualDescentSample FindBestResidualDescentNeighbor(
			Transform baseRoot,
			Arm6DOFFKController armController,
			Vector3 armTargetWorldPosition,
			Vector3 currentBasePosition,
			float minReach,
			float maxReach,
			float preferredMin,
			float preferredMax,
			float requestedEeToleranceMeters,
			float baseRadius,
			HashSet<Collider> obstacleLookup,
			DockingCandidateBuildStats buildStats,
			ResidualDescentSample activeSample,
			float stepSizeMeters,
			int nextDescentIteration)
		{
			if (activeSample == null)
			{
				return null;
			}

			ResidualDescentSample bestNeighbor = null;
			for (int directionIndex = 0; directionIndex < ResidualDescentDirections.Length; directionIndex++)
			{
				Vector2 direction = ResidualDescentDirections[directionIndex];
				Vector3 candidateBasePosition = activeSample.baseWorldPosition + new Vector3(
					direction.x * stepSizeMeters,
					0f,
					direction.y * stepSizeMeters);
				ResidualDescentSample neighbor = EvaluateResidualDescentSample(
					baseRoot,
					armController,
					armTargetWorldPosition,
					currentBasePosition,
					minReach,
					maxReach,
					preferredMin,
					preferredMax,
					requestedEeToleranceMeters,
					baseRadius,
					obstacleLookup,
					buildStats,
					candidateBasePosition,
					activeSample.baseYawDeg,
					"MidpointResidualDescent",
					nextDescentIteration);
				if (CompareResidualDescentSamples(neighbor, activeSample) >= 0)
				{
					continue;
				}

				if (bestNeighbor == null || CompareResidualDescentSamples(neighbor, bestNeighbor) < 0)
				{
					bestNeighbor = neighbor;
				}
			}

			return bestNeighbor;
		}

		private ResidualDescentSample EvaluateResidualDescentSample(
			Transform baseRoot,
			Arm6DOFFKController armController,
			Vector3 armTargetWorldPosition,
			Vector3 currentBasePosition,
			float minReach,
			float maxReach,
			float preferredMin,
			float preferredMax,
			float requestedEeToleranceMeters,
			float baseRadius,
			HashSet<Collider> obstacleLookup,
			DockingCandidateBuildStats buildStats,
			Vector3 candidateBasePosition,
			float fallbackYawDeg,
			string searchSource,
			int descentIteration)
		{
			ResidualDescentSample sample = new ResidualDescentSample
			{
				baseWorldPosition = new Vector3(candidateBasePosition.x, currentBasePosition.y, candidateBasePosition.z),
				searchSource = searchSource ?? string.Empty,
				descentIteration = descentIteration
			};
			sample.baseYawDeg = ComputeFacingTargetYawDeg(armTargetWorldPosition, sample.baseWorldPosition, fallbackYawDeg);
			sample.planarTargetDistanceMeters = PlanarDistance(sample.baseWorldPosition, armTargetWorldPosition);
			sample.distanceBandPenaltyMeters = ComputePlanarDistanceBandPenalty(sample.planarTargetDistanceMeters);
			sample.moveDistanceMeters = PlanarDistance(currentBasePosition, sample.baseWorldPosition);

			if (buildStats != null)
			{
				buildStats.sampledCount++;
			}

			if (IsResidualDescentCollisionBlocked(sample.baseWorldPosition, baseRadius, obstacleLookup, out Collider blockingCollider))
			{
				sample.collisionBlocked = true;
				sample.failureReason = blockingCollider != null ? $"OverlapSphere:{blockingCollider.name}" : "OverlapSphere";
				if (buildStats != null)
				{
					buildStats.collisionRejectedCount++;
				}

				return sample;
			}

			bool solved = TrySolveArmTargetAtBasePoseLoose(
				baseRoot,
				sample.baseWorldPosition,
				Quaternion.Euler(0f, sample.baseYawDeg, 0f),
				armController,
				armTargetWorldPosition,
				out float[] solvedAnglesDeg,
				out _,
				out float residualMeters,
				out float singularityPenalty,
				out string failureReason,
				requestedEeToleranceMeters);
			sample.ikConverged = solved;
			sample.ikResidualMeters = SanitizeResidualMetric(residualMeters);
			sample.failureReason = failureReason ?? string.Empty;

			DockingCandidate candidate = CreateDockingCandidateFromBasePose(
				baseRoot,
				armController,
				armTargetWorldPosition,
				currentBasePosition,
				minReach,
				maxReach,
				preferredMin,
				preferredMax,
				baseRadius,
				0,
				searchSource,
				classifyRadiusBand(sample.planarTargetDistanceMeters, preferredMin, preferredMax),
				sample.baseWorldPosition,
				sample.baseYawDeg,
				sample.planarTargetDistanceMeters,
				ComputeAngleFromDirection(ProjectXZ(armTargetWorldPosition - sample.baseWorldPosition)));
			if (candidate == null)
			{
				if (buildStats != null)
				{
					buildStats.geometryRejectedCount++;
				}

				return sample;
			}

			sample.geometryValid = true;
			candidate.searchSource = sample.searchSource;
			candidate.descentIteration = sample.descentIteration;
			candidate.planarTargetDistanceMeters = sample.planarTargetDistanceMeters;
			candidate.ikResidualMeters = sample.ikResidualMeters;
			candidate.distanceBandPenaltyMeters = sample.distanceBandPenaltyMeters;
			candidate.evaluationStageSummary = "AcceptedAsSeed";
			PopulateCandidateIkDiagnostics(candidate, armController, solvedAnglesDeg, sample.ikResidualMeters, singularityPenalty);
			sample.candidate = candidate;
			return sample;
		}

		private bool TryAddResidualDescentCandidate(
			List<DockingCandidate> candidates,
			ResidualDescentSample sample,
			bool markAsBest,
			DockingCandidateBuildStats buildStats)
		{
			DockingCandidate candidate = sample != null ? sample.candidate : null;
			if (candidate == null || IsNearDuplicateCandidate(candidates, candidate))
			{
				return false;
			}

			candidate.searchSource = string.IsNullOrEmpty(sample.searchSource) ? "MidpointResidualDescent" : sample.searchSource;
			candidate.descentIteration = sample.descentIteration;
			candidate.planarTargetDistanceMeters = sample.planarTargetDistanceMeters;
			candidate.ikResidualMeters = sample.ikResidualMeters;
			candidate.distanceBandPenaltyMeters = sample.distanceBandPenaltyMeters;
			candidate.evaluationStageSummary = markAsBest ? "AcceptedAsBestDocking" : "AcceptedAsSeed";
			candidates.Add(candidate);
			if (buildStats != null)
			{
				buildStats.acceptedCount++;
			}

			return true;
		}

		private static void LogResidualDescentSample(string label, ResidualDescentSample sample)
		{
			if (sample == null)
			{
				return;
			}

			Debug.Log(
				$"[CoordinatedTaskPlanner] {label}: candidateBasePosition={sample.baseWorldPosition}, yaw={sample.baseYawDeg:F1}, residual={sample.ikResidualMeters:F4}m, planarTargetDistance={sample.planarTargetDistanceMeters:F4}m, distanceBandPenalty={sample.distanceBandPenaltyMeters:F4}m, collisionBlocked={sample.collisionBlocked}, failure='{sample.failureReason}'");
		}

		private static void LogAcceptedResidualNeighbor(ResidualDescentSample sample, float stepSizeMeters)
		{
			if (sample == null)
			{
				return;
			}

			Debug.Log(
				$"[CoordinatedTaskPlanner] MidpointResidualDescent accepted neighbor: candidateBasePosition={sample.baseWorldPosition}, stepSizeMeters={stepSizeMeters:F3}, ikResidualMeters={sample.ikResidualMeters:F4}, planarTargetDistanceMeters={sample.planarTargetDistanceMeters:F4}, descentIteration={sample.descentIteration}");
		}

		private static bool HasResidualDescentAccepted(ResidualDescentSample sample, float acceptResidualMeters)
		{
			return sample != null
				&& !sample.collisionBlocked
				&& sample.candidate != null
				&& sample.ikResidualMeters < Mathf.Max(0.001f, acceptResidualMeters);
		}

		private float ComputeResidualDescentStepMeters(
			ResidualDescentSample activeSample,
			float acceptResidualMeters,
			bool fineSearch)
		{
			float minStepMeters = fineSearch
				? Mathf.Max(0.005f, residualDescentFineStepMeters)
				: Mathf.Max(0.01f, residualDescentCoarseStepMeters);
			float maxStepMeters = fineSearch
				? Mathf.Max(minStepMeters, residualDescentFineMaxStepMeters)
				: Mathf.Max(minStepMeters, residualDescentCoarseMaxStepMeters);
			if (activeSample == null)
			{
				return minStepMeters;
			}

			float ikSignal = IsFiniteValue(activeSample.ikResidualMeters)
				? Mathf.Max(0f, activeSample.ikResidualMeters - acceptResidualMeters)
				: Mathf.Max(acceptResidualMeters * 2f, activeSample.distanceBandPenaltyMeters);
			float distanceSignal = Mathf.Max(0f, activeSample.distanceBandPenaltyMeters);
			float residualSignal = Mathf.Max(ikSignal, distanceSignal);
			if (activeSample.collisionBlocked)
			{
				residualSignal = Mathf.Max(residualSignal, minStepMeters * 2f);
			}

			float adaptiveStepMeters = minStepMeters + (residualSignal * Mathf.Max(0.01f, residualDescentDynamicStepGain));
			return Mathf.Clamp(adaptiveStepMeters, minStepMeters, maxStepMeters);
		}

		private static int CompareResidualDescentSamples(ResidualDescentSample left, ResidualDescentSample right)
		{
			if (ReferenceEquals(left, right))
			{
				return 0;
			}

			if (left == null)
			{
				return 1;
			}

			if (right == null)
			{
				return -1;
			}

			int collisionOrder = left.collisionBlocked.CompareTo(right.collisionBlocked);
			if (collisionOrder != 0)
			{
				return collisionOrder;
			}

			int ikOrder = CompareResidualMetric(left.ikResidualMeters, right.ikResidualMeters);
			if (ikOrder != 0)
			{
				return ikOrder;
			}

			int distanceOrder = CompareResidualMetric(left.distanceBandPenaltyMeters, right.distanceBandPenaltyMeters);
			if (distanceOrder != 0)
			{
				return distanceOrder;
			}

			int moveOrder = CompareResidualMetric(left.moveDistanceMeters, right.moveDistanceMeters);
			if (moveOrder != 0)
			{
				return moveOrder;
			}

			return CompareResidualMetric(left.planarTargetDistanceMeters, right.planarTargetDistanceMeters);
		}

		private static int CompareResidualMetric(float left, float right)
		{
			bool leftFinite = IsFiniteValue(left);
			bool rightFinite = IsFiniteValue(right);
			if (leftFinite != rightFinite)
			{
				return leftFinite ? -1 : 1;
			}

			if (!leftFinite && !rightFinite)
			{
				return 0;
			}

			return left.CompareTo(right);
		}

		private static void SortResidualDescentCandidates(List<DockingCandidate> candidates)
		{
			if (candidates == null)
			{
				return;
			}

			candidates.Sort((left, right) =>
			{
				int ikOrder = CompareResidualMetric(left != null ? left.ikResidualMeters : float.PositiveInfinity, right != null ? right.ikResidualMeters : float.PositiveInfinity);
				if (ikOrder != 0)
				{
					return ikOrder;
				}

				int distanceOrder = CompareResidualMetric(left != null ? left.distanceBandPenaltyMeters : float.PositiveInfinity, right != null ? right.distanceBandPenaltyMeters : float.PositiveInfinity);
				if (distanceOrder != 0)
				{
					return distanceOrder;
				}

				int moveOrder = CompareResidualMetric(left != null ? left.travelDistance : float.PositiveInfinity, right != null ? right.travelDistance : float.PositiveInfinity);
				if (moveOrder != 0)
				{
					return moveOrder;
				}

				return CompareResidualMetric(left != null ? left.heuristicCost : float.PositiveInfinity, right != null ? right.heuristicCost : float.PositiveInfinity);
			});
		}

		private static float SanitizeResidualMetric(float value)
		{
			if (float.IsNaN(value))
			{
				return float.PositiveInfinity;
			}

			return value;
		}

		private static bool IsFiniteValue(float value)
		{
			return !float.IsNaN(value) && !float.IsInfinity(value);
		}

		private static HashSet<Collider> BuildObstacleLookup(IReadOnlyList<Collider> obstacles)
		{
			if (obstacles == null || obstacles.Count == 0)
			{
				return null;
			}

			HashSet<Collider> obstacleLookup = new HashSet<Collider>();
			for (int i = 0; i < obstacles.Count; i++)
			{
				Collider obstacle = obstacles[i];
				if (obstacle != null)
				{
					obstacleLookup.Add(obstacle);
				}
			}

			return obstacleLookup.Count > 0 ? obstacleLookup : null;
		}

		private static bool IsResidualDescentCollisionBlocked(Vector3 candidateBasePosition, float baseRadius, HashSet<Collider> obstacleLookup, out Collider blockingCollider)
		{
			blockingCollider = null;
			if (obstacleLookup == null || obstacleLookup.Count == 0)
			{
				return false;
			}

			Collider[] overlaps = Physics.OverlapSphere(
				candidateBasePosition,
				Mathf.Max(0.01f, baseRadius),
				~0,
				QueryTriggerInteraction.Ignore);
			for (int i = 0; i < overlaps.Length; i++)
			{
				Collider overlap = overlaps[i];
				if (overlap == null || !obstacleLookup.Contains(overlap))
				{
					continue;
				}

				blockingCollider = overlap;
				return true;
			}

			return false;
		}

		private static float ComputeFacingTargetYawDeg(Vector3 armTargetWorldPosition, Vector3 baseWorldPosition, float fallbackYawDeg)
		{
			Vector3 toTarget = ProjectXZ(armTargetWorldPosition - baseWorldPosition);
			if (toTarget.sqrMagnitude <= 1e-6f)
			{
				return fallbackYawDeg;
			}

			return Mathf.Atan2(toTarget.x, toTarget.z) * Mathf.Rad2Deg;
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
			float referenceRadiusMeters,
			float baseRadius,
			Vector3 baseCollisionBoxHalfExtents,
			PlannerPhysicsQueries physicsQueries,
			IReadOnlyList<Collider> obstacles,
			Vector3 primarySectorDirection,
			int sectorStage,
			bool includeFallbackBand,
			bool navMeshFilterActive,
			DockingCandidateBuildStats buildStats,
			bool coarseSearch)
		{
			List<float> radii = BuildDockingRadii(
				minReach,
				maxReach,
				preferredMin,
				preferredMax,
				referenceRadiusMeters,
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
					float[] targetBearingOffsets = coarseTargetBearingOffsetsDeg != null && coarseTargetBearingOffsetsDeg.Length > 0
						? coarseTargetBearingOffsetsDeg
						: new[] { 0f };
					float angleRad = SampleSectorAngle(centerAngleRad, sectorHalfAngleRad, angleIndex, angularSamples);
					for (int bearingIndex = 0; bearingIndex < targetBearingOffsets.Length; bearingIndex++)
					{
						if (buildStats != null)
						{
							buildStats.sampledCount++;
						}

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
							radius,
							targetBearingOffsets[bearingIndex]);
						if (candidate == null)
						{
							if (buildStats != null)
							{
								buildStats.geometryRejectedCount++;
							}

							continue;
						}

						DockingCandidate unguidedCandidate = candidate;
						candidate = GuideDockingCandidateWithNavMesh(
							baseRoot,
							armController,
							armTargetWorldPosition,
							currentBasePosition,
							armBaseLocalRotation,
							minReach,
							maxReach,
							preferredMin,
							preferredMax,
							baseRadius,
							navMeshFilterActive,
							candidate);
						if (candidate == null)
						{
							RecordDockingDebugSample(unguidedCandidate, armTargetWorldPosition, false, false, false, "RejectedByNavMesh");
							if (buildStats != null)
							{
								buildStats.navMeshRejectedCount++;
							}

							continue;
						}

						candidate.searchSource = "SectorFallback";
						candidate.descentIteration = -1;
						if (TryAcceptDockingCandidate(candidate, baseRadius, baseCollisionBoxHalfExtents, physicsQueries, obstacles, navMeshFilterActive))
						{
							RecordDockingDebugSample(candidate, armTargetWorldPosition, false, false, false, candidate.evaluationStageSummary);
							if (buildStats != null)
							{
								buildStats.acceptedCount++;
							}

							candidates.Add(candidate);
							continue;
						}

						if (buildStats == null)
						{
							continue;
						}

						switch (candidate.evaluationStageSummary)
						{
							case "RejectedByNavMesh":
								buildStats.navMeshRejectedCount++;
								break;
							case "RejectedByBaseFootprintBox":
							case "RejectedByBaseCheckBox":
							case "RejectedByBaseOverlapBox":
								buildStats.boxRejectedCount++;
								break;
							case "RejectedByBaseCollision":
								buildStats.collisionRejectedCount++;
								break;
							default:
								buildStats.geometryRejectedCount++;
								break;
						}
						RecordDockingDebugSample(candidate, armTargetWorldPosition, false, false, false, candidate.evaluationStageSummary);
					}
				}
			}

			SortDockingCandidates(candidates);
			return candidates;
		}

		private List<DockingCandidate> BuildRandomAnnularCandidates(
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
			Vector3 baseCollisionBoxHalfExtents,
			PlannerPhysicsQueries physicsQueries,
			IReadOnlyList<Collider> obstacles,
			Vector3 primaryDirection,
			bool navMeshFilterActive,
			DockingCandidateBuildStats buildStats,
			int sampleCount)
		{
			List<DockingCandidate> candidates = new List<DockingCandidate>();
			System.Random random = CreateDeterministicSampler(currentBasePosition, armTargetWorldPosition);
			float anchorAngleRad = ComputeAngleFromDirection(primaryDirection);
			int anchorCount = Mathf.Clamp(annularAnchorSampleCount, 0, Mathf.Max(0, sampleCount));
			for (int sampleIndex = 0; sampleIndex < sampleCount; sampleIndex++)
			{
				buildStats.sampledCount++;
				float angleRad;
				float radius;
				if (sampleIndex < anchorCount && anchorCount > 0)
				{
					float anchorOffset = sampleIndex * (Mathf.PI * 2f / Mathf.Max(1, anchorCount));
					angleRad = anchorAngleRad + anchorOffset;
					float anchorT = anchorCount <= 1 ? 0.5f : sampleIndex / (float)(anchorCount - 1);
					radius = Mathf.Lerp(preferredMin, preferredMax, anchorT);
				}
				else
				{
					angleRad = Mathf.Lerp(0f, Mathf.PI * 2f, (float)random.NextDouble());
					radius = SampleAnnularRadius(preferredMin, preferredMax, random);
				}

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
					0,
					AnnularSamplingLabel,
					classifyRadiusBand(radius, preferredMin, preferredMax),
					angleRad,
					radius,
					0f);
				if (candidate == null)
				{
					buildStats.geometryRejectedCount++;
					continue;
				}

				DockingCandidate unguidedCandidate = candidate;
				candidate = GuideDockingCandidateWithNavMesh(
					baseRoot,
					armController,
					armTargetWorldPosition,
					currentBasePosition,
					armBaseLocalRotation,
					minReach,
					maxReach,
					preferredMin,
					preferredMax,
					baseRadius,
					navMeshFilterActive,
					candidate);
				if (candidate == null)
				{
					buildStats.navMeshRejectedCount++;
					RecordDockingDebugSample(unguidedCandidate, armTargetWorldPosition, false, false, false, "RejectedByNavMesh");
					continue;
				}

				candidate.searchSource = AnnularSamplingLabel;
				candidate.descentIteration = -1;
				if (IsNearDuplicateCandidate(candidates, candidate))
				{
					buildStats.geometryRejectedCount++;
					continue;
				}

				if (TryAcceptDockingCandidate(candidate, baseRadius, baseCollisionBoxHalfExtents, physicsQueries, obstacles, navMeshFilterActive))
				{
					buildStats.acceptedCount++;
					RecordDockingDebugSample(candidate, armTargetWorldPosition, false, false, false, candidate.evaluationStageSummary);
					candidates.Add(candidate);
					continue;
				}

				switch (candidate.evaluationStageSummary)
				{
					case "RejectedByNavMesh":
						buildStats.navMeshRejectedCount++;
						break;
					case "RejectedByBaseFootprintBox":
					case "RejectedByBaseCheckBox":
					case "RejectedByBaseOverlapBox":
						buildStats.boxRejectedCount++;
						break;
					case "RejectedByBaseCollision":
						buildStats.collisionRejectedCount++;
						break;
					default:
						buildStats.geometryRejectedCount++;
						break;
				}
				RecordDockingDebugSample(candidate, armTargetWorldPosition, false, false, false, candidate.evaluationStageSummary);
			}

			SortDockingCandidates(candidates);
			return candidates;
		}

		private static System.Random CreateDeterministicSampler(Vector3 currentBasePosition, Vector3 targetWorldPosition)
		{
			int xHash = Mathf.RoundToInt(currentBasePosition.x * 1000f);
			int zHash = Mathf.RoundToInt(currentBasePosition.z * 1000f);
			int targetXHash = Mathf.RoundToInt(targetWorldPosition.x * 1000f);
			int targetZHash = Mathf.RoundToInt(targetWorldPosition.z * 1000f);
			int seed = 17;
			seed = (seed * 31) + xHash;
			seed = (seed * 31) + zHash;
			seed = (seed * 31) + targetXHash;
			seed = (seed * 31) + targetZHash;
			return new System.Random(seed);
		}

		private static float ComputeAngleFromDirection(Vector3 direction)
		{
			Vector3 planar = ProjectXZ(direction);
			if (planar.sqrMagnitude <= 1e-6f)
			{
				return 0f;
			}

			return Mathf.Atan2(planar.z, planar.x);
		}

		private static float SampleAnnularRadius(float innerRadius, float outerRadius, System.Random random)
		{
			float innerSquared = innerRadius * innerRadius;
			float outerSquared = outerRadius * outerRadius;
			float t = (float)random.NextDouble();
			return Mathf.Sqrt(Mathf.Lerp(innerSquared, outerSquared, t));
		}

		private static float SampleWeightedDockingRadius(float minReach, float maxReach, float preferredMin, float preferredMax, System.Random random)
		{
			float bandRoll = (float)random.NextDouble();
			if (preferredMax > preferredMin + 1e-4f && (bandRoll < 0.65f || maxReach <= preferredMax + 1e-4f))
			{
				return SampleAnnularRadius(preferredMin, preferredMax, random);
			}

			if (preferredMin > minReach + 1e-4f && bandRoll < 0.82f)
			{
				return SampleAnnularRadius(minReach, preferredMin, random);
			}

			if (preferredMax < maxReach - 1e-4f)
			{
				return SampleAnnularRadius(preferredMax, maxReach, random);
			}

			return SampleAnnularRadius(minReach, maxReach, random);
		}

		private float ComputeMinJointLimitMarginDeg(Arm6DOFFKController armController, float[] jointAnglesDeg)
		{
			if (armController == null || jointAnglesDeg == null || jointAnglesDeg.Length < 6)
			{
				return float.NegativeInfinity;
			}

			float minMargin = float.PositiveInfinity;
			for (int jointIndex = 0; jointIndex < 6; jointIndex++)
			{
				Vector2 limits = armController.GetJointLimits(jointIndex);
				float margin = Mathf.Min(jointAnglesDeg[jointIndex] - limits.x, limits.y - jointAnglesDeg[jointIndex]);
				minMargin = Mathf.Min(minMargin, margin);
			}

			return minMargin;
		}

		private float ComputeJointLimitPenalty(float minJointLimitMarginDeg)
		{
			if (float.IsInfinity(minJointLimitMarginDeg) || float.IsNaN(minJointLimitMarginDeg))
			{
				return 100f;
			}

			if (minJointLimitMarginDeg >= jointLimitSoftMarginDeg)
			{
				return 0f;
			}

			return Mathf.Max(0f, jointLimitSoftMarginDeg - minJointLimitMarginDeg);
		}

		private void PopulateCandidateIkDiagnostics(
			DockingCandidate candidate,
			Arm6DOFFKController armController,
			float[] jointAnglesDeg,
			float residualMeters,
			float singularityPenalty)
		{
			if (candidate == null)
			{
				return;
			}

			candidate.looseIkResidualMeters = residualMeters;
			candidate.looseIkSingularityPenalty = singularityPenalty;
			if (armController == null || jointAnglesDeg == null || jointAnglesDeg.Length < 6)
			{
				candidate.manipulabilityIndex = float.NaN;
				candidate.minJointLimitMarginDeg = float.NaN;
				candidate.jointLimitPenalty = float.NaN;
				return;
			}

			candidate.manipulabilityIndex = Mathf.Max(0f, armController.ComputeManipulabilityIndex(jointAnglesDeg));
			candidate.minJointLimitMarginDeg = ComputeMinJointLimitMarginDeg(armController, jointAnglesDeg);
			candidate.jointLimitPenalty = ComputeJointLimitPenalty(candidate.minJointLimitMarginDeg);
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
			Vector3 baseCollisionBoxHalfExtents,
			PlannerPhysicsQueries physicsQueries,
			IReadOnlyList<Collider> obstacles,
			bool navMeshFilterActive,
			List<DockingCandidate> coarseSeeds)
		{
			List<DockingCandidate> fineCandidates = new List<DockingCandidate>();
			if (coarseSeeds == null || coarseSeeds.Count == 0)
			{
				return fineCandidates;
			}

			for (int seedIndex = 0; seedIndex < coarseSeeds.Count; seedIndex++)
			{
				DockingCandidate seed = coarseSeeds[seedIndex];
				if (seed == null
					|| IsNearDuplicateCandidate(fineCandidates, seed)
					|| !TryAcceptDockingCandidate(seed, baseRadius, baseCollisionBoxHalfExtents, physicsQueries, obstacles, navMeshFilterActive))
				{
					continue;
				}

				seed.evaluationStageSummary = "AcceptedAsFineSeed";
				fineCandidates.Add(seed);
			}

			float[] radiusOffsets = fineRadiusOffsetsMeters != null && fineRadiusOffsetsMeters.Length > 0
				? fineRadiusOffsetsMeters
				: new[] { 0f };
			float[] angleOffsets = fineAngleOffsetsDeg != null && fineAngleOffsetsDeg.Length > 0
				? fineAngleOffsetsDeg
				: new[] { 0f };
			float[] bearingOffsets = fineTargetBearingOffsetsDeg != null && fineTargetBearingOffsetsDeg.Length > 0
				? fineTargetBearingOffsetsDeg
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
						for (int bearingIndex = 0; bearingIndex < bearingOffsets.Length; bearingIndex++)
						{
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
								classifyRadiusBand(radius, preferredMin, preferredMax),
								angleRad,
								radius,
								seed.targetBearingLocalDeg + bearingOffsets[bearingIndex]);
							if (candidate == null)
							{
								continue;
							}

							candidate = GuideDockingCandidateWithNavMesh(
								baseRoot,
								armController,
								armTargetWorldPosition,
								currentBasePosition,
								armBaseLocalRotation,
								minReach,
								maxReach,
								preferredMin,
								preferredMax,
								baseRadius,
								navMeshFilterActive,
								candidate);
							if (candidate == null
								|| IsNearDuplicateCandidate(fineCandidates, candidate)
								|| !TryAcceptDockingCandidate(candidate, baseRadius, baseCollisionBoxHalfExtents, physicsQueries, obstacles, navMeshFilterActive))
							{
								continue;
							}

							fineCandidates.Add(candidate);
						}
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
			float radius,
			float targetBearingLocalDeg)
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
			Quaternion desiredFutureArmBaseRotation =
				Quaternion.LookRotation(directionToTarget, Vector3.up)
				* Quaternion.Inverse(Quaternion.Euler(0f, targetBearingLocalDeg, 0f));
			Quaternion candidateRotation = desiredFutureArmBaseRotation * Quaternion.Inverse(armBaseLocalRotation);
			Vector3 futureArmBasePosition = armTargetWorldPosition - (directionToTarget * radius);
			Vector3 candidateBasePosition = futureArmBasePosition - (candidateRotation * armBaseLocalPosition);
			candidateBasePosition.y = currentBasePosition.y;
			return CreateDockingCandidateFromBasePose(
				baseRoot,
				armController,
				armTargetWorldPosition,
				currentBasePosition,
				minReach,
				maxReach,
				preferredMin,
				preferredMax,
				baseRadius,
				sectorStage,
				sectorLabel,
				radiusBand,
				candidateBasePosition,
				candidateRotation.eulerAngles.y,
				radius,
				angleRad);
		}

		private DockingCandidate CreateDockingCandidateFromBasePose(
			Transform baseRoot,
			Arm6DOFFKController armController,
			Vector3 armTargetWorldPosition,
			Vector3 currentBasePosition,
			float minReach,
			float maxReach,
			float preferredMin,
			float preferredMax,
			float baseRadius,
			int sectorStage,
			string sectorLabel,
			string radiusBand,
			Vector3 candidateBasePosition,
			float candidateBaseYawDeg,
			float requestedRadius,
			float fallbackAngleRad)
		{
			Quaternion candidateBaseRotation = Quaternion.Euler(0f, candidateBaseYawDeg, 0f);
			candidateBasePosition.y = currentBasePosition.y;
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
			if (targetDistanceInFutureArmBase < minReach - DockingGeometryRadiusConsistencyToleranceMeters
				|| targetDistanceInFutureArmBase > maxReach + DockingGeometryRadiusConsistencyToleranceMeters)
			{
				return null;
			}

			if (!TryGetFutureArmBasePose(
				baseRoot,
				candidateBasePosition,
				candidateBaseRotation,
				armController,
				out Vector3 futureArmBasePosition,
				out Quaternion futureArmBaseRotation))
			{
				return null;
			}

			Vector3 actualDirection = ProjectXZ(armTargetWorldPosition - futureArmBasePosition);
			float targetAngleRad = actualDirection.sqrMagnitude > 1e-6f
				? Mathf.Atan2(actualDirection.z, actualDirection.x)
				: fallbackAngleRad;
			float travelDistance = PlanarDistance(currentBasePosition, candidateBasePosition);
			float planarTargetDistance = PlanarDistance(candidateBasePosition, armTargetWorldPosition);
			float shellMargin = Mathf.Max(0f, Mathf.Min(targetDistanceInFutureArmBase - minReach, maxReach - targetDistanceInFutureArmBase));
			float shellMarginPenalty = Mathf.Max(0f, 0.12f - shellMargin) * 8f;
			float clearance = Mathf.Max(0f, _distanceFieldSampler.SampleDistance(candidateBasePosition) - baseRadius);
			float preferredBandPenalty = ComputePreferredBandPenalty(targetDistanceInFutureArmBase, preferredMin, preferredMax);
			float workspaceBandPenalty = preferredBandPenalty + Mathf.Max(0f, Mathf.Abs(targetDistanceInFutureArmBase - requestedRadius) - 0.01f) * 4f;
			float sectorStagePenalty = Mathf.Max(0, sectorStage) * 0.35f;
			float heuristicCost =
				(workspaceBandPenalty * 16f) +
				shellMarginPenalty +
				sectorStagePenalty +
				(travelDistance * 0.8f) -
				Mathf.Min(clearance, 3f) * 0.2f;

			return new DockingCandidate
			{
				baseWorldPosition = candidateBasePosition,
				baseYawDeg = candidateBaseRotation.eulerAngles.y,
				targetAngleRad = targetAngleRad,
				targetBearingLocalDeg = Mathf.Atan2(targetInFutureArmBase.x, targetInFutureArmBase.z) * Mathf.Rad2Deg,
				radius = targetDistanceInFutureArmBase,
				sampledRadiusMeters = requestedRadius,
				shellMargin = shellMargin,
				preferredBandPenalty = preferredBandPenalty,
				workspaceBandPenalty = workspaceBandPenalty,
				travelDistance = travelDistance,
				planarTargetDistanceMeters = planarTargetDistance,
				ikResidualMeters = float.PositiveInfinity,
				distanceBandPenaltyMeters = ComputePlanarDistanceBandPenalty(planarTargetDistance),
				descentIteration = -1,
				searchSource = string.Empty,
				clearance = clearance,
				heuristicCost = heuristicCost,
				manipulabilityIndex = float.NaN,
				looseIkResidualMeters = float.PositiveInfinity,
				looseIkSingularityPenalty = float.NaN,
				minJointLimitMarginDeg = float.NaN,
				jointLimitPenalty = float.NaN,
				futureArmBasePosition = futureArmBasePosition,
				targetInFutureArmBase = targetInFutureArmBase,
				targetDistanceInFutureArmBase = targetDistanceInFutureArmBase,
				futureArmBaseEulerAngles = futureArmBaseRotation.eulerAngles,
				sectorLabel = sectorLabel,
				sampledRadiusBand = string.IsNullOrEmpty(radiusBand) ? classifyRadiusBand(requestedRadius, preferredMin, preferredMax) : radiusBand,
				radiusBand = classifyRadiusBand(targetDistanceInFutureArmBase, preferredMin, preferredMax),
				evaluationStageSummary = "AcceptedAsSeed"
			};
		}

		private DockingCandidate GuideDockingCandidateWithNavMesh(
			Transform baseRoot,
			Arm6DOFFKController armController,
			Vector3 armTargetWorldPosition,
			Vector3 currentBasePosition,
			Quaternion armBaseLocalRotation,
			float minReach,
			float maxReach,
			float preferredMin,
			float preferredMax,
			float baseRadius,
			bool navMeshFilterActive,
			DockingCandidate candidate)
		{
			if (candidate == null || !navMeshFilterActive)
			{
				return candidate;
			}

			if (!NavMesh.SamplePosition(
				candidate.baseWorldPosition,
				out NavMeshHit hit,
				Mathf.Max(0.1f, navMeshSampleMaxDistanceMeters),
				navMeshAreaMask))
			{
				candidate.evaluationStageSummary = "RejectedByNavMesh";
				return null;
			}

			Vector3 snappedBasePosition = hit.position;
			snappedBasePosition.y = currentBasePosition.y;
			if (PlanarDistance(snappedBasePosition, candidate.baseWorldPosition) <= 0.01f)
			{
				return candidate;
			}

			Vector3 directionToTarget = ProjectXZ(armTargetWorldPosition - snappedBasePosition);
			float baseYawDeg;
			if (directionToTarget.sqrMagnitude <= 1e-6f)
			{
				baseYawDeg = candidate.baseYawDeg;
			}
			else
			{
				Quaternion desiredFutureArmBaseRotation = Quaternion.LookRotation(directionToTarget.normalized, Vector3.up);
				Quaternion candidateRotation = desiredFutureArmBaseRotation * Quaternion.Inverse(armBaseLocalRotation);
				baseYawDeg = candidateRotation.eulerAngles.y;
			}

			DockingCandidate snappedCandidate = CreateDockingCandidateFromBasePose(
				baseRoot,
				armController,
				armTargetWorldPosition,
				currentBasePosition,
				minReach,
				maxReach,
				preferredMin,
				preferredMax,
				baseRadius,
				Array.IndexOf(SectorLabels, candidate.sectorLabel),
				candidate.sectorLabel,
				candidate.sampledRadiusBand,
				snappedBasePosition,
				baseYawDeg,
				candidate.sampledRadiusMeters,
				candidate.targetAngleRad);
			if (snappedCandidate == null)
			{
				candidate.evaluationStageSummary = "RejectedByNavMeshSnapGeometry";
				return null;
			}

			snappedCandidate.evaluationStageSummary = "GuidedByNavMesh";
			return snappedCandidate;
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

		private List<float> BuildDockingRadii(float minReach, float maxReach, float preferredMin, float preferredMax, float referenceRadiusMeters, int preferredSamples, int fallbackSamples, bool includeFallbackBand)
		{
			List<float> radii = new List<float>();
			AddInterpolatedRange(radii, preferredMin, preferredMax, Mathf.Max(2, preferredSamples));
			if (referenceRadiusMeters >= minReach - 1e-4f && referenceRadiusMeters <= maxReach + 1e-4f)
			{
				float clampedReference = Mathf.Clamp(referenceRadiusMeters, minReach, maxReach);
				AddUniqueRadius(radii, clampedReference);
				AddUniqueRadius(radii, Mathf.Clamp(clampedReference - 0.03f, minReach, maxReach));
				AddUniqueRadius(radii, Mathf.Clamp(clampedReference + 0.03f, minReach, maxReach));
			}
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

		private void ResolvePreferredAnnularBand(float minReach, float maxReach, float referenceRadiusMeters, out float preferredMin, out float preferredMax)
		{
			float workspaceMargin = Mathf.Max(0.005f, annularSamplingWorkspaceMarginMeters);
			float workspaceSpan = Mathf.Max(0.05f, maxReach - minReach);
			float lowerBound = minReach + workspaceMargin;
			float upperBound = Mathf.Max(lowerBound + 0.01f, maxReach - workspaceMargin);
			float clampedReference = Mathf.Clamp(referenceRadiusMeters, lowerBound, upperBound);
			bool hasUsableReference = referenceRadiusMeters >= minReach - 1e-4f && referenceRadiusMeters <= maxReach + 1e-4f;
			float comfortCenter = hasUsableReference
				? clampedReference
				: Mathf.Lerp(minReach, maxReach, 0.62f);
			float comfortHalfWidth = hasUsableReference
				? Mathf.Clamp(workspaceSpan * 0.08f, 0.025f, 0.07f)
				: Mathf.Clamp(workspaceSpan * 0.12f, 0.03f, 0.08f);
			float desiredMin = comfortCenter - comfortHalfWidth;
			float desiredMax = comfortCenter + comfortHalfWidth;
			float configuredMin = annularSamplingInnerRadiusMeters > 0f ? annularSamplingInnerRadiusMeters : desiredMin;
			float configuredMax = annularSamplingOuterRadiusMeters > 0f ? annularSamplingOuterRadiusMeters : desiredMax;
			float safeMin = Mathf.Clamp(configuredMin, lowerBound, upperBound);
			float safeMax = Mathf.Clamp(configuredMax, safeMin + 0.01f, upperBound);
			if (safeMax <= safeMin)
			{
				safeMax = Mathf.Min(upperBound, safeMin + 0.05f);
			}

			preferredMin = Mathf.Clamp(safeMin, lowerBound, upperBound);
			preferredMax = Mathf.Clamp(safeMax, preferredMin + 0.01f, upperBound);
		}

		private void CacheDockingSamplingDebugInfo(Vector3 targetWorldPosition, float innerRadiusMeters, float outerRadiusMeters)
		{
			_hasDockingSamplingDebugInfo = true;
			_lastDockingTargetWorldPosition = targetWorldPosition;
			_lastDockingSamplingInnerRadiusMeters = Mathf.Max(0f, innerRadiusMeters);
			_lastDockingSamplingOuterRadiusMeters = Mathf.Max(_lastDockingSamplingInnerRadiusMeters, outerRadiusMeters);
		}

		private void ClearDockingDebugSamples()
		{
			_lastDockingDebugSamples.Clear();
		}

		private void RecordDockingDebugSample(
			DockingCandidate candidate,
			Vector3 targetWorldPosition,
			bool ikFailed,
			bool strictPreviewFailed,
			bool selected,
			string reason)
		{
			if (candidate == null)
			{
				return;
			}

			if (selected)
			{
				for (int i = _lastDockingDebugSamples.Count - 1; i >= 0; i--)
				{
					if (_lastDockingDebugSamples[i] != null && _lastDockingDebugSamples[i].selected)
					{
						_lastDockingDebugSamples.RemoveAt(i);
					}
				}
			}

			if (_lastDockingDebugSamples.Count >= Mathf.Max(8, dockingDebugSampleLimit))
			{
				_lastDockingDebugSamples.RemoveAt(0);
			}

			_lastDockingDebugSamples.Add(new DockingDebugSample
			{
				baseWorldPosition = candidate.baseWorldPosition,
				targetWorldPosition = targetWorldPosition,
				ikFailed = ikFailed,
				strictPreviewFailed = strictPreviewFailed,
				selected = selected,
				reason = reason ?? string.Empty
			});
		}

		private bool ShouldUseNavMeshPoseFilter(Vector3 currentBasePosition)
		{
			if (!enableNavMeshPoseFilter)
			{
				return false;
			}

			float probeDistance = Mathf.Max(0.25f, navMeshSampleMaxDistanceMeters * 2f);
			if (NavMesh.SamplePosition(currentBasePosition, out _, probeDistance, navMeshAreaMask))
			{
				return true;
			}

			NavMeshTriangulation triangulation = NavMesh.CalculateTriangulation();
			return triangulation.vertices != null && triangulation.vertices.Length > 0;
		}

		private bool TryAcceptDockingCandidate(
			DockingCandidate candidate,
			float baseRadius,
			Vector3 baseCollisionBoxHalfExtents,
			PlannerPhysicsQueries physicsQueries,
			IReadOnlyList<Collider> obstacles,
			bool navMeshFilterActive)
		{
			if (candidate == null || physicsQueries == null)
			{
				return false;
			}

			bool hasFootprintBox = enableStaticBoxCollisionFilter
				&& baseCollisionBoxHalfExtents.x > 0.05f
				&& baseCollisionBoxHalfExtents.y > 0.05f
				&& baseCollisionBoxHalfExtents.z > 0.05f;
			if (hasFootprintBox
				&& !physicsQueries.IsBasePoseCheckBoxCollisionFree(
					candidate.baseWorldPosition,
					Quaternion.Euler(0f, candidate.baseYawDeg, 0f),
					baseCollisionBoxHalfExtents,
					obstacles,
					out _))
			{
				candidate.evaluationStageSummary = "RejectedByBaseCheckBox";
				return false;
			}

			bool circleCollisionFree = physicsQueries.IsBasePoseCollisionFree(candidate.baseWorldPosition, baseRadius, obstacles, out _);
			if (!circleCollisionFree && !hasFootprintBox)
			{
				candidate.evaluationStageSummary = "RejectedByBaseCollision";
				return false;
			}

			if (!circleCollisionFree && hasFootprintBox)
			{
				candidate.evaluationStageSummary = navMeshFilterActive
					? "AcceptedByFootprintBoxAfterGuidedNavMesh"
					: "AcceptedByFootprintBox";
				return true;
			}

			candidate.evaluationStageSummary = navMeshFilterActive
				? "AcceptedAfterGuidedNavMeshAndFootprintFilters"
				: "AcceptedAfterFootprintFilters";
			return true;
		}

		private bool TryEvaluateArmFeasibilityAtLiveBaseLoose(
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

			if (!TrySolveArmTargetAtLiveBaseLoose(
				armController,
				armTargetWorldPosition,
				out float[] solvedAngles,
				out Vector3 targetInLiveArmBase,
				out residualMeters,
				out singularityPenalty,
				out string solveFailure,
				targetToleranceMeters,
				startAnglesOverrideDeg))
			{
				solvedAnglesDeg = CloneAngles(solvedAngles);
				failureReason = solveFailure;
				return false;
			}

			if (singularityPenalty >= LoosePrecheckHardSingularityThreshold)
			{
				solvedAnglesDeg = CloneAngles(solvedAngles);
				failureReason = L(
					$"前置机械臂初筛拒绝该位姿：奇异性惩罚 {singularityPenalty:F2} 已超过硬阈值。",
					$"Loose arm precheck rejected this pose because the singularity penalty {singularityPenalty:F2} exceeded the hard threshold.");
				return false;
			}

			float minJointLimitMarginDeg = ComputeMinJointLimitMarginDeg(armController, solvedAngles);
			float hardLimitViolationToleranceDeg = -Mathf.Max(0.5f, jointLimitHardMarginDeg * 0.1f);
			if (minJointLimitMarginDeg < hardLimitViolationToleranceDeg)
			{
				solvedAnglesDeg = CloneAngles(solvedAngles);
				failureReason = L(
					$"前置机械臂初筛拒绝该位姿：关节解超出了允许限位，最小余量 {minJointLimitMarginDeg:F1}°。",
					$"Loose arm precheck rejected this pose because the solved joint configuration exceeded the allowed joint limits. Minimum margin={minJointLimitMarginDeg:F1}deg.");
				return false;
			}

			if (armController.EvaluateMotionCollision(solvedAngles, solvedAngles, out ArmCollisionGuardResult guardResult))
			{
				solvedAnglesDeg = CloneAngles(solvedAngles);
				failureReason = string.IsNullOrEmpty(guardResult.message)
					? L("前置机械臂初筛发现目标终点姿态存在硬碰撞。", "Loose arm precheck found a hard collision at the terminal pose.")
					: guardResult.message;
				Debug.LogWarning($"[CoordinatedTaskPlanner] Live-base loose precheck terminal collision. targetInLiveArmBase={targetInLiveArmBase}");
				return false;
			}

			solvedAnglesDeg = CloneAngles(solvedAngles);
			return true;
		}

		private bool TrySolveArmTargetAtLiveBaseLoose(
			Arm6DOFFKController armController,
			Vector3 armTargetWorldPosition,
			out float[] solvedAnglesDeg,
			out Vector3 targetInLiveArmBase,
			out float residualMeters,
			out float singularityPenalty,
			out string failureReason,
			float targetToleranceMeters = -1f,
			float[] startAnglesOverrideDeg = null)
		{
			solvedAnglesDeg = null;
			targetInLiveArmBase = Vector3.zero;
			residualMeters = float.PositiveInfinity;
			singularityPenalty = float.PositiveInfinity;
			failureReason = string.Empty;

			if (armController == null || !armController.KinematicsReady)
			{
				failureReason = L("机械臂控制器或运动学模型尚未就绪。", "Arm controller or kinematics model is not ready.");
				return false;
			}

			targetInLiveArmBase = armController.WorldToBasePosition(armTargetWorldPosition);
			float solveToleranceMeters = Mathf.Max(
				targetToleranceMeters > 0f ? Mathf.Max(0.001f, targetToleranceMeters) : _armMotionPlanner.settings.toleranceMeters,
				LoosePrecheckToleranceMeters);
			float[] startAngles = startAnglesOverrideDeg != null && startAnglesOverrideDeg.Length >= 6
				? CloneAngles(startAnglesOverrideDeg)
				: armController.CaptureMeasuredJointAngles();
			if (!_armMotionPlanner.TrySolveToBasePosition(
				armController,
				targetInLiveArmBase,
				out float[] solvedAngles,
				out float bestResidualMeters,
				out singularityPenalty,
				out string ikFailure,
				startAngles,
				null,
				solveToleranceMeters))
			{
				residualMeters = bestResidualMeters;
				solvedAnglesDeg = CloneAngles(solvedAngles);
				failureReason = ikFailure;
				return false;
			}

			residualMeters = Vector3.Distance(armController.ForwardPoe(solvedAngles).position, targetInLiveArmBase);
			solvedAnglesDeg = CloneAngles(solvedAngles);
			return true;
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
				solvedAnglesDeg = CloneAngles(solvedAngles);
				failureReason = solveFailure;
				return false;
			}

			if (singularityPenalty >= LoosePrecheckHardSingularityThreshold)
			{
				solvedAnglesDeg = CloneAngles(solvedAngles);
				failureReason = L(
					$"前置机械臂初筛拒绝该位姿：奇异性惩罚 {singularityPenalty:F2} 已超过硬阈值。",
					$"Loose arm precheck rejected this pose because the singularity penalty {singularityPenalty:F2} exceeded the hard threshold.");
				return false;
			}

			float minJointLimitMarginDeg = ComputeMinJointLimitMarginDeg(armController, solvedAngles);
			float hardLimitViolationToleranceDeg = -Mathf.Max(0.5f, jointLimitHardMarginDeg * 0.1f);
			if (minJointLimitMarginDeg < hardLimitViolationToleranceDeg)
			{
				solvedAnglesDeg = CloneAngles(solvedAngles);
				failureReason = L(
					$"前置机械臂初筛拒绝该位姿：关节解超出了允许限位，最小余量 {minJointLimitMarginDeg:F1}°。",
					$"Loose arm precheck rejected this pose because the solved joint configuration exceeded the allowed joint limits. Minimum margin={minJointLimitMarginDeg:F1}deg.");
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
				solvedAnglesDeg = CloneAngles(solvedAngles);
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
				out float bestResidualMeters,
				out singularityPenalty,
				out string ikFailure,
				startAngles,
				null,
				solveToleranceMeters))
			{
				residualMeters = bestResidualMeters;
				solvedAnglesDeg = CloneAngles(solvedAngles);
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
			if (!TryGetArmBaseLocalOffset(baseRoot, armController, out armBaseLocalPosition, out armBaseLocalRotation))
			{
				return false;
			}

			if (_hasCachedArmBaseLocalOffset
				&& _cachedBaseRootId == baseRootId
				&& _cachedArmBaseTransformId == armBaseId)
			{
				float localPositionDelta = Vector3.Distance(_cachedArmBaseLocalPosition, armBaseLocalPosition);
				float localRotationDelta = Quaternion.Angle(_cachedArmBaseLocalRotation, armBaseLocalRotation);
				if (localPositionDelta > ArmBaseLocalOffsetWarnPositionDeltaMeters
					|| localRotationDelta > ArmBaseLocalOffsetWarnRotationDeltaDeg)
				{
					Debug.LogWarning(
						$"[CoordinatedTaskPlanner] Arm-base local offset drift detected. cachedPosition={_cachedArmBaseLocalPosition}, livePosition={armBaseLocalPosition}, cachedRotation={_cachedArmBaseLocalRotation.eulerAngles}, liveRotation={armBaseLocalRotation.eulerAngles}, deltaPos={localPositionDelta:F4}m, deltaRot={localRotationDelta:F2}deg");
				}
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

		private void LogCurrentBaseFrameConsistency(
			Transform baseRoot,
			Vector3 baseWorldPosition,
			Quaternion baseWorldRotation,
			Arm6DOFFKController armController,
			Vector3 armTargetWorldPosition,
			float targetToleranceMeters = -1f)
		{
			if (baseRoot == null || armController == null || !armController.KinematicsReady)
			{
				return;
			}

			Vector3 liveTargetInArmBase = armController.WorldToBasePosition(armTargetWorldPosition);
			if (!TryGetTargetInFutureArmBase(
				baseRoot,
				baseWorldPosition,
				baseWorldRotation,
				armController,
				armTargetWorldPosition,
				out Vector3 projectedTargetInArmBase))
			{
				return;
			}

			float deltaMeters = Vector3.Distance(liveTargetInArmBase, projectedTargetInArmBase);
			float reportThreshold = Mathf.Max(0.002f, targetToleranceMeters > 0f ? targetToleranceMeters * 0.25f : 0.005f);
			if (deltaMeters <= reportThreshold)
			{
				return;
			}

			Transform liveArmBase = armController.BaseFrameTransform;
			Debug.LogWarning(
				$"[CoordinatedTaskPlanner] CurrentBaseFrameConsistency: delta={deltaMeters:F4}m, liveTargetInArmBase={liveTargetInArmBase}, projectedTargetInArmBase={projectedTargetInArmBase}, liveArmBasePosition={(liveArmBase != null ? liveArmBase.position : Vector3.zero)}, liveArmBaseRotation={(liveArmBase != null ? liveArmBase.rotation.eulerAngles : Vector3.zero)}, baseRootPosition={baseWorldPosition}, baseRootRotation={baseWorldRotation.eulerAngles}");
		}

		private void LogDockingCandidateGeometry(DockingCandidate candidate)
		{
			if (candidate == null)
			{
				return;
			}

			Debug.Log(
				$"[CoordinatedTaskPlanner] DockingCandidateGeometry: samplingStage={candidate.sectorLabel}, sampledRadius={FormatDiagnosticFloat(candidate.sampledRadiusMeters, "F4")}, sampledRadiusBand={candidate.sampledRadiusBand}, actualRadiusBand={candidate.radiusBand}, candidateBasePosition={candidate.baseWorldPosition}, candidateYaw={candidate.baseYawDeg:F2}, futureArmBasePosition={candidate.futureArmBasePosition}, futureArmBaseRotation={candidate.futureArmBaseEulerAngles}, targetInFutureArmBase={candidate.targetInFutureArmBase}, |targetInFutureArmBase|={candidate.targetDistanceInFutureArmBase:F4}, targetBearingLocalDeg={candidate.targetBearingLocalDeg:F1}, manipulability={FormatDiagnosticFloat(candidate.manipulabilityIndex, "F5")}, looseResidual={FormatDiagnosticFloat(candidate.looseIkResidualMeters, "F4")}, looseSingularityPenalty={FormatDiagnosticFloat(candidate.looseIkSingularityPenalty, "F2")}, minJointLimitMargin={FormatDiagnosticFloat(candidate.minJointLimitMarginDeg, "F1")}, jointLimitPenalty={FormatDiagnosticFloat(candidate.jointLimitPenalty, "F2")}, status={candidate.evaluationStageSummary}");
		}

		private static string FormatDiagnosticFloat(float value, string format)
		{
			if (float.IsNaN(value) || float.IsInfinity(value))
			{
				return "n/a";
			}

			return value.ToString(format, CultureInfo.InvariantCulture);
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
			if (candidate.targetDistanceInFutureArmBase < context.minReach - DockingHardWorkspaceShellToleranceMeters
				|| candidate.targetDistanceInFutureArmBase > context.maxReach + DockingHardWorkspaceShellToleranceMeters)
			{
				failureSummary.noWorkspaceConsistentPoseCount++;
				failureSummary.failureCategory = FailureCategoryNoWorkspaceConsistentPose;
				candidate.evaluationStageSummary = "RejectedByWorkspaceShell";
				RecordDockingDebugSample(candidate, context.request.armTargetWorldPosition, false, false, false, candidate.evaluationStageSummary);
				LogDockingCandidateGeometry(candidate);
				return false;
			}

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
				PopulateCandidateIkDiagnostics(candidate, context.armController, solvedAngles, residualMeters, singularityPenalty);
				failureSummary.lastArmFailure = looseFailure;
				failureSummary.noLooseIkPoseCount++;
				failureSummary.failureCategory = FailureCategoryNoLooseIkPose;
				candidate.evaluationStageSummary = "RejectedByLooseIK";
				RecordDockingDebugSample(candidate, context.request.armTargetWorldPosition, true, false, false, looseFailure);
				LogDockingCandidateGeometry(candidate);
				return false;
			}

			failureSummary.anyIkCandidate = true;
			failureSummary.anyCollisionFreeArmCandidate = true;
			PopulateCandidateIkDiagnostics(candidate, context.armController, solvedAngles, residualMeters, singularityPenalty);
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
				RecordDockingDebugSample(candidate, context.request.armTargetWorldPosition, false, false, false, pathFailure);
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
				RecordDockingDebugSample(candidate, context.request.armTargetWorldPosition, false, true, false, failureSummary.lastArmFailure);
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
			float manipulabilityReward = Mathf.Min(candidate.manipulabilityIndex, 0.08f) * 90f;
			float finalCost = armResidualScore
				+ workspaceBandScore
				+ singularityScore
				+ candidate.jointLimitPenalty * 1.5f
				+ baseTravelScore
				+ pathScore
				+ clearanceScore
				+ candidate.heuristicCost * 0.25f
				- manipulabilityReward;
			if (bestCandidate == null || finalCost < bestCost)
			{
				bestCost = finalCost;
				bestCandidate = candidate;
				bestSolvedAngles = CloneAngles(solvedAngles);
				candidate.evaluationStageSummary = "AcceptedAsBestDocking";
				RecordDockingDebugSample(candidate, context.request.armTargetWorldPosition, false, strictPreviewPenalty > 0f, true, candidate.evaluationStageSummary);
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

		private static float ComputePlanarDistanceBandPenalty(float planarTargetDistanceMeters)
		{
			const float preferredMinDistance = 0.45f;
			const float preferredMaxDistance = 0.65f;
			if (planarTargetDistanceMeters < preferredMinDistance)
			{
				return preferredMinDistance - planarTargetDistanceMeters;
			}

			if (planarTargetDistanceMeters > preferredMaxDistance)
			{
				return planarTargetDistanceMeters - preferredMaxDistance;
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
