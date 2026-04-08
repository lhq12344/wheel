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
		public bool hasPreferredArmSolveSeed;
		public float[] preferredArmSolveSeedAnglesDeg;
		public Vector3 resolvedBaseStopWorldPosition;
		public float resolvedBaseStopYawDeg;
		public string failureReason = string.Empty;
		public string dockingSummary = string.Empty;
	}

	public sealed class CoordinatedTaskPlanner
	{
		private const float DefaultWorkspaceInnerRadiusMeters = 0.24338f;
		private const float DefaultWorkspaceOuterRadiusMeters = 0.8066f;
		private const float BasePositionToleranceMeters = 0.02f;

		private sealed class DockingCandidate
		{
			public Vector3 baseWorldPosition;
			public float baseYawDeg;
			public float radius;
			public float shellMargin;
			public float preferredBandPenalty;
			public float travelDistance;
			public float clearance;
			public float heuristicCost;
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
				resolution.baseMoveRequired = false;
				resolution.dockingSummary = L(
					"本次任务不需要机械臂动作，因此接受当前底盘位姿。",
					"This task does not require arm motion, so the current base pose is accepted.");
				return resolution;
			}

			if (armController == null || !armController.KinematicsReady)
			{
				resolution.failureReason = L("机械臂控制器或运动学模型尚未就绪。", "Arm controller or kinematics model is not ready.");
				resolution.dockingSummary = resolution.failureReason;
				return resolution;
			}

			if (TryEvaluateArmExecutionAtBasePose(
				diffDriveController.rb.transform,
				diffDriveController.rb.position,
				diffDriveController.rb.rotation,
				armController,
				request.armTargetWorldPosition,
				out float[] solvedAngles,
				out _,
				out string currentBaseFailure,
				request != null ? request.eePositionToleranceMeters : -1f))
			{
				resolution.accepted = true;
				resolution.dockingPoseFound = true;
				resolution.armReachableFromCurrentBase = true;
				resolution.baseMoveRequired = false;
				resolution.hasPreferredArmSolveSeed = solvedAngles != null && solvedAngles.Length >= 6;
				resolution.preferredArmSolveSeedAnglesDeg = CloneAngles(solvedAngles);
				resolution.dockingSummary = L(
					"当前底盘位姿已经支持所请求的末端世界目标，因此无需移动底盘。",
					"Current base pose already supports the requested end-effector world target, so no base move is required.");
				return resolution;
			}

			resolution.armReachableFromCurrentBase = false;
			resolution.failureReason = string.IsNullOrEmpty(currentBaseFailure)
				? L("当前底盘位姿无法安全完成所请求的末端世界目标。", "Current base pose cannot safely execute the requested end-effector world target.")
				: currentBaseFailure;
			resolution.dockingSummary = resolution.failureReason;
			return resolution;
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
					"本次任务没有机械臂动作，因此无需搜索停靠位。",
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
			float preferredMin = Mathf.Clamp(maxReach * 0.4f, minReach + 0.01f, maxReach);
			float preferredMax = Mathf.Clamp(maxReach * 0.8f, preferredMin, maxReach);
			float[] startAngles = armController.CaptureMeasuredJointAngles();
			if (!TryGetCachedArmBaseLocalOffset(baseRoot, armController, out Vector3 armBaseLocalPosition, out _))
			{
				resolution.failureReason = L("无法解析机械臂安装位相对底盘的局部偏移。", "Could not resolve the arm mounting offset relative to the base.");
				resolution.dockingSummary = resolution.failureReason;
				return false;
			}

			PrepareDockingDistanceField(currentBasePosition, request.armTargetWorldPosition, obstacles, maxReach, baseRadius, armBaseLocalPosition);
			List<DockingCandidate> candidates = BuildDockingCandidates(
				request.armTargetWorldPosition,
				currentBasePosition,
				armBaseLocalPosition,
				minReach,
				maxReach,
				preferredMin,
				preferredMax,
				baseRadius,
				physicsQueries,
				obstacles);

			if (candidates.Count == 0)
			{
				resolution.failureReason = L(
					"在 Zu5 停靠搜索环带内没有找到无碰撞的底盘停靠位。",
					"No collision-free base parking pose exists inside the Zu5 docking search ring.");
				resolution.dockingSummary = resolution.failureReason;
				return false;
			}

			bool anyWorkspaceShellCandidate = false;
			bool anyIkCandidate = false;
			bool anyCollisionFreeArmCandidate = false;
			string lastArmFailure = string.Empty;
			string lastPathFailure = string.Empty;
			DockingCandidate bestCandidate = null;
			DockingCandidate bestProvisionalCandidate = null;
			float[] bestSolvedAngles = null;
			float bestCost = float.MaxValue;
			float bestProvisionalCost = float.MaxValue;
			string bestProvisionalReason = string.Empty;
			int provisionalPathChecksUsed = 0;

			for (int i = 0; i < candidates.Count; i++)
			{
				DockingCandidate candidate = candidates[i];
				if (bestCandidate != null && candidate.heuristicCost >= bestCost)
				{
					break;
				}

				Quaternion candidateRotation = Quaternion.Euler(0f, candidate.baseYawDeg, 0f);
				if (!TryGetTargetInFutureArmBase(
					baseRoot,
					candidate.baseWorldPosition,
					candidateRotation,
					armController,
					request.armTargetWorldPosition,
					out Vector3 targetInFutureArmBase))
				{
					continue;
				}

				if (!armController.IsBasePositionReachable(targetInFutureArmBase, request != null ? Mathf.Max(0.001f, request.eePositionToleranceMeters) : _armMotionPlanner.settings.toleranceMeters))
				{
					continue;
				}

				anyWorkspaceShellCandidate = true;
				if (!_armMotionPlanner.TrySolveToBasePosition(
					armController,
					targetInFutureArmBase,
					out float[] solvedAngles,
					out float singularityPenalty,
					out string ikFailure,
					startAngles,
					null,
					request != null ? request.eePositionToleranceMeters : -1f))
				{
					lastArmFailure = ikFailure;
					if (allowProvisionalDockingWhenIkSoftFails
						&& IsSoftArmPlanningFailure(ikFailure)
						&& provisionalPathChecksUsed < Mathf.Max(0, provisionalDockingPathCheckBudget))
					{
						provisionalPathChecksUsed++;
						if (HasBasePathToCandidate(
							currentBasePosition,
							currentBaseYaw,
							candidate.baseWorldPosition,
							candidate.baseYawDeg,
							baseRadius,
							physicsQueries,
							obstacles,
							out string provisionalPathFailure))
						{
							if (candidate.heuristicCost < bestProvisionalCost)
							{
								bestProvisionalCost = candidate.heuristicCost;
								bestProvisionalCandidate = candidate;
								bestProvisionalReason = ikFailure;
							}
						}
						else if (!string.IsNullOrEmpty(provisionalPathFailure))
						{
							lastPathFailure = provisionalPathFailure;
						}
					}

					continue;
				}

				anyIkCandidate = true;
				if (EvaluatePredictedArmMotionAtBasePose(
					baseRoot,
					candidate.baseWorldPosition,
					candidateRotation,
					armController,
					startAngles,
					solvedAngles,
					out ArmCollisionGuardResult guardResult))
				{
					lastArmFailure = string.IsNullOrEmpty(guardResult.message)
						? L("机械臂碰撞预检拒绝了该停靠位。", "Arm collision preview rejected this docking pose.")
						: guardResult.message;
					continue;
				}

				anyCollisionFreeArmCandidate = true;
				if (!HasBasePathToCandidate(
					currentBasePosition,
					currentBaseYaw,
					candidate.baseWorldPosition,
					candidate.baseYawDeg,
					baseRadius,
					physicsQueries,
					obstacles,
					out string pathFailure))
				{
					lastPathFailure = pathFailure;
					continue;
				}

				float finalCost = candidate.heuristicCost + (Mathf.Clamp(singularityPenalty, 0f, 50f) * 0.05f);
				if (finalCost < bestCost)
				{
					bestCost = finalCost;
					bestCandidate = candidate;
					bestSolvedAngles = CloneAngles(solvedAngles);
				}
			}

			if (bestCandidate == null)
			{
				if (bestProvisionalCandidate != null)
				{
					resolution.accepted = true;
					resolution.dockingPoseFound = true;
					resolution.baseMoveRequired = PlanarDistance(currentBasePosition, bestProvisionalCandidate.baseWorldPosition) > BasePositionToleranceMeters;
					resolution.armReachableFromCurrentBase = false;
					resolution.hasPreferredArmSolveSeed = false;
					resolution.preferredArmSolveSeedAnglesDeg = null;
					resolution.resolvedBaseStopWorldPosition = bestProvisionalCandidate.baseWorldPosition;
					resolution.resolvedBaseStopYawDeg = bestProvisionalCandidate.baseYawDeg;
					resolution.dockingSummary = L(
						$"停靠位搜索阶段未获得直接收敛的机械臂解（{bestProvisionalReason}），将先执行保守停靠位 ({bestProvisionalCandidate.baseWorldPosition.x:F2}, {bestProvisionalCandidate.baseWorldPosition.y:F2}, {bestProvisionalCandidate.baseWorldPosition.z:F2})，再在机械臂规划阶段继续求解。",
						$"Docking search did not find a directly converged arm IK solution ({bestProvisionalReason}), so execution will first move to a conservative docking pose ({bestProvisionalCandidate.baseWorldPosition.x:F2}, {bestProvisionalCandidate.baseWorldPosition.y:F2}, {bestProvisionalCandidate.baseWorldPosition.z:F2}) and continue solving during the arm-planning stage.");
					return true;
				}

				resolution.failureReason = BuildDockingFailureReason(
					anyWorkspaceShellCandidate,
					anyIkCandidate,
					anyCollisionFreeArmCandidate,
					lastArmFailure,
					lastPathFailure);
				resolution.dockingSummary = resolution.failureReason;
				return false;
			}

			resolution.accepted = true;
			resolution.dockingPoseFound = true;
			resolution.baseMoveRequired = PlanarDistance(currentBasePosition, bestCandidate.baseWorldPosition) > BasePositionToleranceMeters;
			resolution.armReachableFromCurrentBase = false;
			resolution.hasPreferredArmSolveSeed = bestSolvedAngles != null && bestSolvedAngles.Length >= 6;
			resolution.preferredArmSolveSeedAnglesDeg = CloneAngles(bestSolvedAngles);
			resolution.resolvedBaseStopWorldPosition = bestCandidate.baseWorldPosition;
			resolution.resolvedBaseStopYawDeg = bestCandidate.baseYawDeg;
			resolution.dockingSummary = L(
				$"已解析到底盘停靠位 ({bestCandidate.baseWorldPosition.x:F2}, {bestCandidate.baseWorldPosition.y:F2}, {bestCandidate.baseWorldPosition.z:F2})，朝向 {bestCandidate.baseYawDeg:F1}°，机械臂可在该位姿下安全到达目标世界点。",
				$"Resolved base stop at ({bestCandidate.baseWorldPosition.x:F2}, {bestCandidate.baseWorldPosition.y:F2}, {bestCandidate.baseWorldPosition.z:F2}) with yaw {bestCandidate.baseYawDeg:F1}deg so the arm can safely reach the requested world target.");
			return true;
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

		private List<DockingCandidate> BuildDockingCandidates(
			Vector3 armTargetWorldPosition,
			Vector3 currentBasePosition,
			Vector3 armBaseLocalPosition,
			float minReach,
			float maxReach,
			float preferredMin,
			float preferredMax,
			float baseRadius,
			PlannerPhysicsQueries physicsQueries,
			IReadOnlyList<Collider> obstacles)
		{
			List<DockingCandidate> candidates = new List<DockingCandidate>();
			List<float> radii = BuildDockingRadii(minReach, maxReach, preferredMin, preferredMax);

			int angularSamples = Mathf.Max(8, dockingAngularSamples);
			for (int radiusIndex = 0; radiusIndex < radii.Count; radiusIndex++)
			{
				float radius = radii[radiusIndex];
				for (int angleIndex = 0; angleIndex < angularSamples; angleIndex++)
				{
					float angleRad = angleIndex * Mathf.PI * 2f / angularSamples;
					Vector3 directionToTarget = new Vector3(Mathf.Cos(angleRad), 0f, Mathf.Sin(angleRad));
					float candidateYaw = Mathf.Atan2(directionToTarget.x, directionToTarget.z) * Mathf.Rad2Deg;
					Quaternion candidateRotation = Quaternion.Euler(0f, candidateYaw, 0f);
					Vector3 rotatedArmBaseOffset = candidateRotation * armBaseLocalPosition;
					Vector3 candidateArmBasePosition = armTargetWorldPosition - (directionToTarget * radius);
					candidateArmBasePosition.y = currentBasePosition.y + rotatedArmBaseOffset.y;
					Vector3 candidateBasePosition = candidateArmBasePosition - rotatedArmBaseOffset;
					candidateBasePosition.y = currentBasePosition.y;

					if (!physicsQueries.IsBasePoseCollisionFree(candidateBasePosition, baseRadius, obstacles, out _))
					{
						continue;
					}

					float travelDistance = PlanarDistance(currentBasePosition, candidateBasePosition);
					float shellMargin = Mathf.Max(0f, Mathf.Min(radius - minReach, maxReach - radius));
					float shellMarginPenalty = Mathf.Max(0f, 0.12f - shellMargin) * 8f;
					float clearance = Mathf.Max(0f, _distanceFieldSampler.SampleDistance(candidateBasePosition) - baseRadius);
					float preferredBandPenalty = ComputePreferredBandPenalty(radius, preferredMin, preferredMax);
					float heuristicCost =
						(preferredBandPenalty * 10f) +
						shellMarginPenalty +
						travelDistance -
						Mathf.Min(clearance, 3f) * 0.25f;

					candidates.Add(new DockingCandidate
					{
						baseWorldPosition = candidateBasePosition,
						baseYawDeg = candidateYaw,
						radius = radius,
						shellMargin = shellMargin,
						preferredBandPenalty = preferredBandPenalty,
						travelDistance = travelDistance,
						clearance = clearance,
						heuristicCost = heuristicCost
					});
				}
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
			return candidates;
		}

		private List<float> BuildDockingRadii(float minReach, float maxReach, float preferredMin, float preferredMax)
		{
			List<float> radii = new List<float>();
			AddInterpolatedRange(radii, preferredMin, preferredMax, Mathf.Max(2, preferredBandRadiusSamples));
			AddInterpolatedRange(radii, minReach, preferredMin, Mathf.Max(1, fallbackBandRadiusSamples));
			AddInterpolatedRange(radii, preferredMax, maxReach, Mathf.Max(1, fallbackBandRadiusSamples));
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

		private bool TryEvaluateArmExecutionAtBasePose(
			Transform baseRoot,
			Vector3 baseWorldPosition,
			Quaternion baseWorldRotation,
			Arm6DOFFKController armController,
			Vector3 armTargetWorldPosition,
			out float[] solvedAnglesDeg,
			out float singularityPenalty,
			out string failureReason,
			float targetToleranceMeters = -1f)
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
			if (!armController.IsBasePositionReachable(targetInFutureArmBase, solveToleranceMeters))
			{
				failureReason = L("该底盘位姿下，目标点落在 Zu5 工作空间球壳之外。", "Target point is outside the Zu5 workspace shell for this base pose.");
				return false;
			}

			float[] startAngles = armController.CaptureMeasuredJointAngles();
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
				collisionMonitor = Object.FindObjectOfType<ArmCollisionMonitor>();
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

		private static CoordinatedTaskResolution CreateDefaultResolution(DiffDriveTwinController diffDriveController)
		{
			Vector3 basePosition = diffDriveController != null && diffDriveController.rb != null ? diffDriveController.rb.position : Vector3.zero;
			float baseYaw = diffDriveController != null && diffDriveController.rb != null ? diffDriveController.rb.rotation.eulerAngles.y : 0f;
			return new CoordinatedTaskResolution
			{
				resolvedBaseStopWorldPosition = basePosition,
				resolvedBaseStopYawDeg = baseYaw
			};
		}

		private static string BuildDockingFailureReason(
			bool anyWorkspaceShellCandidate,
			bool anyIkCandidate,
			bool anyCollisionFreeArmCandidate,
			string lastArmFailure,
			string lastPathFailure)
		{
			if (!anyWorkspaceShellCandidate)
			{
				return L(
					"所有采样到的停靠位都无法让目标点落入 Zu5 工作空间球壳。",
					"The target point is outside the Zu5 workspace shell for every sampled docking pose.");
			}

			if (!anyIkCandidate)
			{
				return string.IsNullOrEmpty(lastArmFailure)
					? L(
						"目标点虽然落在 Zu5 工作空间球壳内，但没有任何停靠位求解出有效机械臂关节解。",
						"The target point falls inside the Zu5 shell, but no valid arm joint solution was found for any docking pose.")
					: lastArmFailure;
			}

			if (!anyCollisionFreeArmCandidate)
			{
				return string.IsNullOrEmpty(lastArmFailure)
					? L(
						"所有能够求解机械臂目标的停靠位，都会与禁碰车体或轮组发生碰撞。",
						"Every docking pose that can solve the arm target would collide with the forbidden chassis or wheel set.")
					: lastArmFailure;
			}

			return string.IsNullOrEmpty(lastPathFailure)
				? L(
					"目标点在几何上可达，但没有任何停靠位能同时满足底盘可停、路径可达和机械臂碰撞预检。",
					"Although the target is geometrically reachable, no docking pose satisfied base parking, path reachability, and arm collision preview at the same time.")
				: lastPathFailure;
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

		private static float PlanarDistance(Vector3 a, Vector3 b)
		{
			return Vector3.Distance(new Vector3(a.x, 0f, a.z), new Vector3(b.x, 0f, b.z));
		}

		private static bool IsSoftArmPlanningFailure(string reason)
		{
			if (string.IsNullOrWhiteSpace(reason))
			{
				return false;
			}

			return reason.IndexOf("机械臂规划未收敛", System.StringComparison.OrdinalIgnoreCase) >= 0
				|| reason.IndexOf("Arm planner did not converge", System.StringComparison.OrdinalIgnoreCase) >= 0;
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
