using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace RobotSimulation
{
	public class RobotTrajectoryPlanner : MonoBehaviour
	{
		[Header("References")]
		[SerializeField] private RobotSimulationManager manager;

		[Header("Planner Tuning")]
		[SerializeField] private float distanceFieldResolution = 0.4f;
		[SerializeField] private float basePlanningMargin = 4f;
		[SerializeField] private float shadowSimulationSpeedMultiplier = 4f;
		[SerializeField] private int maxLocalReplans = 3;

		[Header("Status")]
		[SerializeField] private RobotPlanningStage currentStage;
		[SerializeField] private bool isPlanning;
		[SerializeField] private string lastSummary = "规划器空闲 / Planner idle.";
		[SerializeField] private ShadowValidationResult lastShadowValidationResult = new ShadowValidationResult();

		private readonly PlannerPhysicsQueries _physicsQueries = new PlannerPhysicsQueries();
		private readonly SceneDistanceFieldSampler _distanceFieldSampler = new SceneDistanceFieldSampler();
		private readonly BaseRrtStarPlanner _basePlanner = new BaseRrtStarPlanner();
		private readonly BasePathFollower _basePathFollower = new BasePathFollower();
		private readonly ArmMotionPlanner _armMotionPlanner = new ArmMotionPlanner();
		private readonly ShadowSimulationGate _shadowGate = new ShadowSimulationGate();
		private readonly LocalReplanner _localReplanner = new LocalReplanner();
		private readonly CoordinatedTaskPlanner _coordinatedTaskPlanner = new CoordinatedTaskPlanner();

		private const float BasePositionToleranceMeters = 0.02f;
		private const float BaseStopAcceptanceMeters = 0.025f;
		private const float BaseSettlingNearGoalAcceptanceMeters = 0.035f;
		private const float BaseLinearSpeedToleranceMetersPerSecond = 0.03f;
		private const float BaseAngularSpeedToleranceRadPerSecond = 0.05f;
		private const int BaseStableFixedFramesRequired = 3;
		private const float ArmWorldTargetToleranceMeters = 0.03f;

		private ExecutionSafetyFilter _safetyFilter;
		private Coroutine _planningRoutine;
		private bool _stopRequested;
		private bool _planOnly;

		public RobotPlanningStage CurrentStage => currentStage;
		public bool IsPlanning => isPlanning;
		public string LastSummary => lastSummary;
		public ShadowValidationResult LastShadowValidationResult => lastShadowValidationResult;
		public float DistanceFieldResolution
		{
			get => distanceFieldResolution;
			set => distanceFieldResolution = Mathf.Max(0.1f, value);
		}
		public float BaseSamplingStep
		{
			get => _basePlanner.settings.stepLength;
			set => _basePlanner.settings.stepLength = Mathf.Max(0.1f, value);
		}
		public float ArmTrajectorySampleDensity
		{
			get => _armMotionPlanner.settings.sampleSpacingDeg;
			set => _armMotionPlanner.settings.sampleSpacingDeg = Mathf.Max(1f, value);
		}
		public float ShadowSimulationSpeedMultiplier
		{
			get => shadowSimulationSpeedMultiplier;
			set => shadowSimulationSpeedMultiplier = Mathf.Max(1f, value);
		}

		private void Awake()
		{
			EnsurePlannerInternals();
		}

		public void Configure(RobotSimulationManager robotManager)
		{
			manager = robotManager;
			EnsurePlannerInternals();
		}

		public Coroutine PlanAndExecute(RobotPlanRequest request, bool executePlan, Action<RobotPlanResult> onComplete = null)
		{
			EnsurePlannerInternals();

			if (!Application.isPlaying)
			{
				RobotPlanResult failed = BuildImmediateFailure(L("RobotTrajectoryPlanner 只能在 Play Mode 下运行。", "RobotTrajectoryPlanner can only run in Play Mode."));
				lastSummary = failed.summary;
				onComplete?.Invoke(failed);
				return null;
			}

			if (!gameObject.activeInHierarchy || !enabled || !isActiveAndEnabled)
			{
				RobotPlanResult failed = BuildImmediateFailure(L("RobotTrajectoryPlanner 当前未激活或被禁用，无法启动规划协程。", "RobotTrajectoryPlanner is inactive or disabled, so the planning coroutine cannot start."));
				lastSummary = failed.summary;
				Debug.LogError($"[RobotTrajectoryPlanner] {failed.summary}");
				onComplete?.Invoke(failed);
				return null;
			}

			if (_planningRoutine != null)
			{
				StopCoroutine(_planningRoutine);
				_planningRoutine = null;
			}

			_planOnly = !executePlan;
			_stopRequested = false;
			lastSummary = _planOnly
				? L("开始规划（仅规划）。", "Planning started (plan only).")
				: L("开始规划。", "Planning started.");
			Debug.Log($"[RobotTrajectoryPlanner] {lastSummary}");
			_planningRoutine = StartCoroutine(PlanAndExecuteSafeCoroutine(request ?? new RobotPlanRequest(), onComplete));
			return _planningRoutine;
		}

		public void StopPlanning(bool emergencyStop)
		{
			_stopRequested = true;
			if (manager != null)
			{
				if (emergencyStop)
				{
					manager.EmergencyStop();
				}
				else
				{
					manager.StopArmMove(false);
					manager.ClearTarget();
				}
			}

			isPlanning = false;
		}

		private IEnumerator PlanAndExecuteCoroutine(RobotPlanRequest request, Action<RobotPlanResult> onComplete)
		{
			EnsurePlannerInternals();
			float basePlanningBudgetSeconds = Mathf.Max(1f, request.planningTimeoutSeconds);
			float baseExecutionBudgetSeconds = Mathf.Max(10f, basePlanningBudgetSeconds * 3f);
			float armPlanningBudgetSeconds = Mathf.Max(1f, request.planningTimeoutSeconds);
			float armExecutionBudgetSeconds = Mathf.Max(10f, armPlanningBudgetSeconds * 3f);
			float eeToleranceMeters = GetRequestedEeTolerance(request);
			RobotPlanResult result = new RobotPlanResult
			{
				accepted = true,
				success = false
			};
			List<string> summaryParts = new List<string>();

			isPlanning = true;
			currentStage = RobotPlanningStage.None;
			lastSummary = _planOnly
				? L("开始规划（仅规划）。", "Planning started (plan only).")
				: L("开始规划。", "Planning started.");
			lastShadowValidationResult = new ShadowValidationResult();

			EnsureManager();
			if (manager == null || manager.diffDriveController == null || manager.diffDriveController.rb == null)
			{
				CompleteFailure(result, RobotPlanningStage.Failed, L("规划器找不到底盘控制器。", "Planner cannot find the base controller."), onComplete);
				yield break;
			}

			manager.EnsurePlanningSupportComponents();
			Transform baseIgnoreRoot = manager.diffDriveController.rb.transform.root;
			Transform armIgnoreRoot = null;
			if (manager.armBinder != null && manager.armBinder.armRoot != null)
			{
				armIgnoreRoot = manager.armBinder.armRoot.transform;
			}
			else if (manager.arm6DOFFKController != null)
			{
				armIgnoreRoot = manager.arm6DOFFKController.BaseFrameTransform;
			}

			List<Collider> obstacles = _physicsQueries.CollectObstacleColliders(baseIgnoreRoot, armIgnoreRoot);
			RemoveRobotOwnedObstacles(obstacles, baseIgnoreRoot, armIgnoreRoot);
			float baseRadius = _physicsQueries.EstimateBaseRadius(manager.diffDriveController);
			Vector3 currentBasePosition = manager.diffDriveController.rb.position;
			float currentBaseYaw = manager.diffDriveController.rb.rotation.eulerAngles.y;
			Vector3 resolvedBaseGoal = currentBasePosition;
			float resolvedBaseYaw = currentBaseYaw;
			result.resolvedBaseStopWorldPosition = currentBasePosition;
			result.resolvedBaseStopYawDeg = currentBaseYaw;

			if (request.requireArmMove)
			{
				currentStage = RobotPlanningStage.TargetCheck;
				yield return null;
				if (IsArmTargetWithinTolerance(request.armTargetWorldPosition, eeToleranceMeters))
				{
					result.dockingPoseFound = true;
					result.baseMoveRequired = false;
					result.armReachableFromCurrentBase = true;
					result.dockingSummary = L(
						"当前末端世界点已经落在请求容差内，因此无需移动底盘。",
						"Current end-effector world point is already within the requested tolerance, so no base move is required.");
					AppendSummary(summaryParts, result.dockingSummary);
					CompleteSuccess(result, summaryParts, onComplete);
					yield break;
				}
			}

			CoordinatedTaskResolution resolution;
			if (request.autoResolveBaseDockingPose)
			{
				currentStage = RobotPlanningStage.CurrentBaseReachabilityCheck;
				yield return null;
				CoordinatedTaskResolution currentBaseResolution = _coordinatedTaskPlanner.EvaluateCurrentBaseExecution(
					request,
					manager.diffDriveController,
					manager.arm6DOFFKController);
				result.armReachableFromCurrentBase = currentBaseResolution.armReachableFromCurrentBase;
				if (currentBaseResolution.armReachableFromCurrentBase)
				{
					resolution = currentBaseResolution;
				}
				else
				{
					if (!request.requireBaseMove)
					{
						CompleteFailure(
							result,
							RobotPlanningStage.CurrentBaseReachabilityCheck,
							currentBaseResolution.failureReason,
							onComplete);
						yield break;
					}

					currentStage = RobotPlanningStage.DockingSearch;
					yield return null;
					if (!_coordinatedTaskPlanner.TryFindAutoDockingPose(
						request,
						manager.diffDriveController,
						manager.arm6DOFFKController,
						baseRadius,
						_physicsQueries,
						obstacles,
						out resolution))
					{
						CompleteFailure(result, RobotPlanningStage.DockingSearch, resolution.failureReason, onComplete);
						yield break;
					}
				}
			}
			else
			{
				currentStage = RobotPlanningStage.BasePlanning;
				yield return null;
				if (!_coordinatedTaskPlanner.TryResolveLegacyBaseGoal(
					request,
					manager.diffDriveController,
					manager.arm6DOFFKController,
					baseRadius,
					_physicsQueries,
					obstacles,
					out resolution))
				{
					CompleteFailure(result, RobotPlanningStage.BasePlanning, resolution.failureReason, onComplete);
					yield break;
				}
			}

			result.dockingPoseFound = resolution.dockingPoseFound;
			result.baseMoveRequired = resolution.baseMoveRequired;
			result.armReachableFromCurrentBase = resolution.armReachableFromCurrentBase;
			result.resolvedBaseStopWorldPosition = resolution.resolvedBaseStopWorldPosition;
			result.resolvedBaseStopYawDeg = resolution.resolvedBaseStopYawDeg;
			result.dockingSummary = resolution.dockingSummary;
			resolvedBaseGoal = resolution.resolvedBaseStopWorldPosition;
			resolvedBaseYaw = resolution.resolvedBaseStopYawDeg;
			resolvedBaseGoal.y = currentBasePosition.y;
			AppendSummary(summaryParts, resolution.dockingSummary);
			float[] preferredArmSolveSeed = resolution.hasPreferredArmSolveSeed
				? CloneAngles(resolution.preferredArmSolveSeedAnglesDeg)
				: null;

			bool shouldMoveBase =
				request.requireBaseMove &&
				result.baseMoveRequired &&
				Vector3.Distance(ProjectXZ(currentBasePosition), ProjectXZ(resolvedBaseGoal)) > BasePositionToleranceMeters;
			if (shouldMoveBase)
			{
				float basePlanningDeadline = Time.realtimeSinceStartup + basePlanningBudgetSeconds;
				if (Time.realtimeSinceStartup > basePlanningDeadline)
				{
					CompleteFailure(result, RobotPlanningStage.BasePlanning, L("底盘规划开始前就已经超时。", "Planning timed out before base planning finished."), onComplete);
					yield break;
				}

				currentStage = RobotPlanningStage.BasePlanning;
				Debug.Log($"[RobotTrajectoryPlanner] Base planning: start={currentBasePosition}, goal={resolvedBaseGoal}, yaw={resolvedBaseYaw:F1}, radius={baseRadius:F3}");
				_distanceFieldSampler.Build(currentBasePosition, resolvedBaseGoal, obstacles, distanceFieldResolution, basePlanningMargin);
				if (!_basePlanner.TryPlan(currentBasePosition, currentBaseYaw, resolvedBaseGoal, resolvedBaseYaw, baseRadius, _distanceFieldSampler, _physicsQueries, obstacles, out List<Vector3> baseWaypoints, out string baseFailure))
				{
					CompleteFailure(result, RobotPlanningStage.BasePlanning, baseFailure, onComplete);
					yield break;
				}

				result.baseWaypoints = baseWaypoints;
				AppendSummary(summaryParts, L(
					$"底盘已在统一世界坐标系下规划到 ({resolvedBaseGoal.x:F2}, {resolvedBaseGoal.y:F2}, {resolvedBaseGoal.z:F2})。",
					$"Base planned toward ({resolvedBaseGoal.x:F2}, {resolvedBaseGoal.y:F2}, {resolvedBaseGoal.z:F2}) in the shared world frame."));

				currentStage = RobotPlanningStage.BaseShadowValidation;
				Debug.Log($"[RobotTrajectoryPlanner] Base shadow validation: waypoints={baseWaypoints.Count}");
				lastShadowValidationResult = _shadowGate.ValidateBasePath(baseWaypoints, baseRadius, _physicsQueries, _distanceFieldSampler, obstacles);
				if (!lastShadowValidationResult.passed)
				{
					CompleteFailure(result, RobotPlanningStage.BaseShadowValidation, lastShadowValidationResult.message, onComplete);
					yield break;
				}

				result.shadowValidationPassed = true;
				if (!_planOnly)
				{
					currentStage = RobotPlanningStage.BaseExecution;
					Debug.Log("[RobotTrajectoryPlanner] Base execution started.");
					float baseExecutionDeadline = Time.realtimeSinceStartup + baseExecutionBudgetSeconds;
					yield return ExecuteBasePath(result, request, resolvedBaseGoal, resolvedBaseYaw, baseRadius, obstacles, baseExecutionDeadline);
					if (!string.IsNullOrEmpty(result.failureReason))
					{
						CompleteFailure(result, result.failedAtStage, result.failureReason, onComplete);
						yield break;
					}
				}
			}
			else
			{
				AppendSummary(summaryParts,
					result.armReachableFromCurrentBase
						? L("跳过底盘移动，因为当前底盘位姿已经支持所请求的末端世界目标。", "Skipping base motion because the current base pose already supports the requested end-effector world target.")
						: L("跳过底盘移动，因为解析出的底盘停靠位已经与当前底盘参考点一致。", "Skipping base motion because the resolved base stop already matches the current base reference point."));
				Debug.Log("[RobotTrajectoryPlanner] Base move skipped.");
			}

			if (!_planOnly && shouldMoveBase)
			{
				currentStage = RobotPlanningStage.BaseSettling;
				float baseSettlingDeadline = Time.realtimeSinceStartup + Mathf.Max(2f, baseExecutionBudgetSeconds * 0.5f);
				yield return WaitForBaseSettled(result, resolvedBaseGoal, baseSettlingDeadline);
				if (!string.IsNullOrEmpty(result.failureReason))
				{
					CompleteFailure(result, result.failedAtStage, result.failureReason, onComplete);
					yield break;
				}

				manager.diffDriveController.CompletePointGoal(resolvedBaseGoal);
				manager.SyncArmToCurrentBasePoseImmediate();
				manager.arm6DOFFKController?.RefreshRuntimeState();
				yield return new WaitForFixedUpdate();
				AppendSummary(summaryParts, L("底盘已稳定停稳，随后开始机械臂阶段。", "Base settled cleanly before the arm stage started."));
			}

			if (request.requireArmMove)
			{
				if (manager.arm6DOFFKController == null)
				{
					CompleteFailure(result, RobotPlanningStage.ArmPlanning, L("机械臂控制器缺失。", "Arm controller is missing."), onComplete);
					yield break;
				}

				if (_planOnly && shouldMoveBase)
				{
					AppendSummary(summaryParts, L("已解析到底盘停靠位；机械臂轨迹规划会在底盘到达该停靠位后再执行。", "Resolved a base docking pose. Arm trajectory planning is deferred until the base reaches that stop pose."));
				}
				else if (IsArmTargetWithinTolerance(request.armTargetWorldPosition, eeToleranceMeters))
				{
					AppendSummary(summaryParts, L("跳过机械臂移动，因为当前末端世界点已经落在目标容差内。", "Skipping arm motion because the current end-effector world point is already within the requested target tolerance."));
				}
				else
				{
					float armPlanningDeadline = Time.realtimeSinceStartup + armPlanningBudgetSeconds;
					if (Time.realtimeSinceStartup > armPlanningDeadline)
					{
						CompleteFailure(result, RobotPlanningStage.ArmPlanning, L("机械臂规划开始前就已经超时。", "Planning timed out before arm planning finished."), onComplete);
						yield break;
					}

					currentStage = RobotPlanningStage.ArmPlanning;
					Vector3 armStart = manager.arm6DOFFKController.EndEffectorWorldPosition;
					Debug.Log($"[RobotTrajectoryPlanner] Arm planning: start={armStart}, goal={request.armTargetWorldPosition}");
					_distanceFieldSampler.Build(armStart, request.armTargetWorldPosition, obstacles, Mathf.Max(0.2f, distanceFieldResolution), 2.5f);
					if (!_armMotionPlanner.TryPlanToWorldPosition(
						manager.arm6DOFFKController,
						request.armTargetWorldPosition,
						_distanceFieldSampler,
						out List<RobotPlanJointSample> armSamples,
						out string armFailure,
						preferredArmSolveSeed))
					{
						CompleteFailure(result, RobotPlanningStage.ArmPlanning, armFailure, onComplete);
						yield break;
					}

					result.armTrajectorySamples = armSamples;
					currentStage = RobotPlanningStage.ArmShadowValidation;
					Debug.Log($"[RobotTrajectoryPlanner] Arm shadow validation: samples={armSamples.Count}");
					lastShadowValidationResult = _shadowGate.ValidateArmTrajectory(manager.arm6DOFFKController, armSamples);
					if (!lastShadowValidationResult.passed)
					{
						CompleteFailure(result, RobotPlanningStage.ArmShadowValidation, lastShadowValidationResult.message, onComplete);
						yield break;
					}
					if (lastShadowValidationResult.singularityRisk && !string.IsNullOrEmpty(lastShadowValidationResult.message))
					{
						Debug.LogWarning($"[RobotTrajectoryPlanner] Arm shadow validation warning: {lastShadowValidationResult.message}");
						AppendSummary(summaryParts, lastShadowValidationResult.message);
					}

					AppendSummary(summaryParts, _planOnly
						? L("机械臂轨迹已规划到所请求的末端世界目标。", "Arm trajectory planned toward the requested end-effector world target.")
						: L("机械臂轨迹已验证，可执行到所请求的末端世界目标。", "Arm trajectory validated for the requested end-effector world target."));

					if (!_planOnly)
					{
						currentStage = RobotPlanningStage.ArmExecution;
						Debug.Log("[RobotTrajectoryPlanner] Arm execution started.");
						float armExecutionDeadline = Time.realtimeSinceStartup + armExecutionBudgetSeconds;
						yield return ExecuteArmTrajectory(result, request, obstacles, armExecutionDeadline);
						if (!string.IsNullOrEmpty(result.failureReason))
						{
							CompleteFailure(result, result.failedAtStage, result.failureReason, onComplete);
							yield break;
						}

						AppendSummary(summaryParts, L("机械臂执行完成。", "Arm execution finished at the requested end-effector world target."));
					}
				}
			}

			CompleteSuccess(result, summaryParts, onComplete);
		}

		private IEnumerator PlanAndExecuteSafeCoroutine(RobotPlanRequest request, Action<RobotPlanResult> onComplete)
		{
			IEnumerator inner = null;
			try
			{
				inner = PlanAndExecuteCoroutine(request, onComplete);
			}
			catch (Exception ex)
			{
				HandlePlanningException(ex, onComplete);
				yield break;
			}

			while (true)
			{
				object current = null;
				bool movedNext;
				try
				{
					movedNext = inner.MoveNext();
					if (movedNext)
					{
						current = inner.Current;
					}
				}
				catch (Exception ex)
				{
					HandlePlanningException(ex, onComplete);
					yield break;
				}

				if (!movedNext)
				{
					yield break;
				}

				yield return current;
			}
		}

		private IEnumerator ExecuteBasePath(RobotPlanResult result, RobotPlanRequest request, Vector3 finalBaseGoal, float finalBaseYaw, float baseRadius, List<Collider> obstacles, float deadline)
		{
			EnsurePlannerInternals();
			if (manager == null || manager.diffDriveController == null || manager.diffDriveController.rb == null)
			{
				result.failedAtStage = RobotPlanningStage.BaseExecution;
				result.failureReason = L("底盘执行中止：差速底盘控制器缺失。", "Base execution aborted because the diff-drive controller is missing.");
				yield break;
			}

			int replanCount = result.replanCount;
			List<Vector3> currentWaypoints = result.baseWaypoints != null ? new List<Vector3>(result.baseWaypoints) : new List<Vector3>();
			while (true)
			{
				if (_stopRequested)
				{
					result.failedAtStage = RobotPlanningStage.BaseExecution;
					result.failureReason = L("规划已停止。", "Planning was stopped.");
					yield break;
				}

				if (Time.realtimeSinceStartup > deadline)
				{
					result.failedAtStage = RobotPlanningStage.BaseExecution;
					result.failureReason = L("底盘执行阶段超时。", "Planning timed out during base execution.");
					yield break;
				}

				Vector3 currentPosition = manager.diffDriveController.rb.position;
				if (!ValidateBasePathSegments(currentWaypoints, baseRadius, obstacles, out ShadowValidationResult segmentValidation))
				{
					lastShadowValidationResult = segmentValidation;
					if (request.allowReplan && replanCount < maxLocalReplans)
					{
						replanCount++;
						result.replanCount = replanCount;
						currentStage = RobotPlanningStage.LocalReplan;
						_distanceFieldSampler.Build(currentPosition, finalBaseGoal, obstacles, distanceFieldResolution, basePlanningMargin);
						if (_localReplanner.TryReplanBase(_basePlanner, currentPosition, manager.diffDriveController.rb.rotation.eulerAngles.y, finalBaseGoal, finalBaseYaw, baseRadius, _distanceFieldSampler, _physicsQueries, obstacles, out List<Vector3> replannedWaypoints, out string replanFailure))
						{
							currentWaypoints = replannedWaypoints;
							result.baseWaypoints = replannedWaypoints;
							currentStage = RobotPlanningStage.BaseExecution;
							continue;
						}

						result.failedAtStage = RobotPlanningStage.LocalReplan;
						result.failureReason = replanFailure;
						yield break;
					}

					result.failedAtStage = RobotPlanningStage.BaseExecution;
					result.failureReason = segmentValidation.message;
					yield break;
				}

				bool done = false;
				bool success = false;
				string message = string.Empty;
				yield return _basePathFollower.FollowWaypoints(
					manager.diffDriveController,
					currentWaypoints,
					finalBaseYaw,
					false,
					() => _stopRequested || Time.realtimeSinceStartup > deadline,
					(ok, text) =>
					{
						success = ok;
						message = text;
						done = true;
					});

				while (!done)
				{
					yield return null;
				}

				if (!success)
				{
					result.failedAtStage = RobotPlanningStage.BaseExecution;
					result.failureReason = message;
					yield break;
				}

				yield break;
			}
		}

		private IEnumerator ExecuteArmTrajectory(RobotPlanResult result, RobotPlanRequest request, List<Collider> obstacles, float deadline)
		{
			EnsurePlannerInternals();
			int replanCount = result.replanCount;
			List<RobotPlanJointSample> currentSamples = result.armTrajectorySamples != null ? new List<RobotPlanJointSample>(result.armTrajectorySamples) : new List<RobotPlanJointSample>();
			float[] previousAngles = manager.arm6DOFFKController.CaptureMeasuredJointAngles();
			float trajectoryStartTime = Time.realtimeSinceStartup;

			for (int sampleIndex = 0; sampleIndex < currentSamples.Count; sampleIndex++)
			{
				if (_stopRequested)
				{
					result.failedAtStage = RobotPlanningStage.ArmExecution;
					result.failureReason = L("规划已停止。", "Planning was stopped.");
					yield break;
				}

				if (Time.realtimeSinceStartup > deadline)
				{
					result.failedAtStage = RobotPlanningStage.ArmExecution;
					result.failureReason = L("机械臂执行阶段超时。", "Planning timed out during arm execution.");
					yield break;
				}

				RobotPlanJointSample sample = currentSamples[sampleIndex];
				if (!_safetyFilter.ValidateArmSegment(manager.arm6DOFFKController, previousAngles, sample.jointAnglesDeg, out ShadowValidationResult segmentValidation))
				{
					lastShadowValidationResult = segmentValidation;
					if (request.allowReplan && replanCount < maxLocalReplans)
					{
						replanCount++;
						result.replanCount = replanCount;
						currentStage = RobotPlanningStage.LocalReplan;
						_distanceFieldSampler.Build(manager.arm6DOFFKController.EndEffectorWorldPosition, request.armTargetWorldPosition, obstacles, Mathf.Max(0.2f, distanceFieldResolution), 2.5f);
						if (_localReplanner.TryReplanArm(_armMotionPlanner, manager.arm6DOFFKController, request.armTargetWorldPosition, _distanceFieldSampler, out List<RobotPlanJointSample> replannedSamples, out string replanFailure))
						{
							currentSamples = replannedSamples;
							result.armTrajectorySamples = replannedSamples;
							previousAngles = manager.arm6DOFFKController.CaptureMeasuredJointAngles();
							sampleIndex = -1;
							currentStage = RobotPlanningStage.ArmExecution;
							continue;
						}

						result.failedAtStage = RobotPlanningStage.LocalReplan;
						result.failureReason = replanFailure;
						yield break;
					}

					result.failedAtStage = RobotPlanningStage.ArmExecution;
					result.failureReason = segmentValidation.message;
					yield break;
				}

				manager.arm6DOFFKController.ApplyAllJointTargetsRaw(sample.jointAnglesDeg);
				float targetCommandTime = trajectoryStartTime + Mathf.Max(0.02f, sample.timeSeconds);
				while (Time.realtimeSinceStartup < targetCommandTime)
				{
					if (_stopRequested)
					{
						result.failedAtStage = RobotPlanningStage.ArmExecution;
						result.failureReason = L("规划已停止。", "Planning was stopped.");
						yield break;
					}

					if (Time.realtimeSinceStartup > deadline)
					{
						result.failedAtStage = RobotPlanningStage.ArmExecution;
						result.failureReason = L("机械臂执行阶段超时。", "Planning timed out during arm execution.");
						yield break;
					}

					if (manager.armCollisionMonitor != null && manager.armCollisionMonitor.EvaluateCollisionState())
					{
						result.failedAtStage = RobotPlanningStage.ArmExecution;
						result.failureReason = string.IsNullOrEmpty(manager.armCollisionMonitor.ActiveCollisionMessage)
							? L("机械臂在轨迹执行过程中发生碰撞。", "Arm collided during trajectory execution.")
							: manager.armCollisionMonitor.ActiveCollisionMessage;
						yield break;
					}

					yield return new WaitForFixedUpdate();
				}

				previousAngles = (float[])sample.jointAnglesDeg.Clone();
			}

			if (currentSamples.Count > 0)
			{
				yield return WaitForArmTrajectorySettled(result, request, currentSamples[currentSamples.Count - 1].jointAnglesDeg, deadline);
				if (!string.IsNullOrEmpty(result.failureReason))
				{
					yield break;
				}
			}

			manager.arm6DOFFKController.HoldCurrentPose();
		}

		private IEnumerator WaitForArmTrajectorySettled(RobotPlanResult result, RobotPlanRequest request, float[] finalTargetAnglesDeg, float deadline)
		{
			const float finalJointToleranceDeg = 2.0f;
			const int requiredStableFrames = 3;
			int stableFrames = 0;
			float finalPositionToleranceMeters = GetRequestedEeTolerance(request);

			while (Time.realtimeSinceStartup <= deadline)
			{
				if (_stopRequested)
				{
					result.failedAtStage = RobotPlanningStage.ArmExecution;
					result.failureReason = L("规划已停止。", "Planning was stopped.");
					yield break;
				}

				if (manager.armCollisionMonitor != null && manager.armCollisionMonitor.EvaluateCollisionState())
				{
					result.failedAtStage = RobotPlanningStage.ArmExecution;
					result.failureReason = string.IsNullOrEmpty(manager.armCollisionMonitor.ActiveCollisionMessage)
						? L("机械臂在轨迹执行过程中发生碰撞。", "Arm collided during trajectory execution.")
						: manager.armCollisionMonitor.ActiveCollisionMessage;
					yield break;
				}

				bool jointsSettled = manager.arm6DOFFKController.AreJointAnglesNear(finalTargetAnglesDeg, finalJointToleranceDeg);
				float eeError = Vector3.Distance(manager.arm6DOFFKController.EndEffectorWorldPosition, request.armTargetWorldPosition);
				if (jointsSettled && eeError <= finalPositionToleranceMeters)
				{
					stableFrames++;
					if (stableFrames >= requiredStableFrames)
					{
						yield break;
					}
				}
				else
				{
					stableFrames = 0;
				}

				yield return new WaitForFixedUpdate();
			}

			float residualDeg = manager.arm6DOFFKController.GetMaxJointAngleError(finalTargetAnglesDeg);
			float residualMeters = Vector3.Distance(manager.arm6DOFFKController.EndEffectorWorldPosition, request.armTargetWorldPosition);
			result.failedAtStage = RobotPlanningStage.ArmExecution;
			result.failureReason = L(
				$"机械臂轨迹未能平滑稳定到位，最终关节误差={residualDeg:F2}°，末端误差={residualMeters:F3}m。",
				$"Arm trajectory did not settle smoothly enough. Final joint error={residualDeg:F2}deg, EE error={residualMeters:F3}m.");
		}

		private bool ValidateBasePathSegments(List<Vector3> waypoints, float baseRadius, List<Collider> obstacles, out ShadowValidationResult validation)
		{
			validation = new ShadowValidationResult();
			if (manager == null || manager.diffDriveController == null || manager.diffDriveController.rb == null)
			{
				validation.passed = false;
				validation.message = L("底盘控制器缺失。", "Base controller is missing.");
				return false;
			}

			Vector3 segmentStart = manager.diffDriveController.rb.position;
			if (waypoints == null)
			{
				return true;
			}

			for (int i = 0; i < waypoints.Count; i++)
			{
				Vector3 segmentEnd = waypoints[i];
				if (Vector3.Distance(ProjectXZ(segmentStart), ProjectXZ(segmentEnd)) <= 0.01f)
				{
					segmentStart = segmentEnd;
					continue;
				}

				if (!_safetyFilter.ValidateBaseSegment(segmentStart, segmentEnd, baseRadius, _physicsQueries, _distanceFieldSampler, obstacles, out validation))
				{
					return false;
				}

				segmentStart = segmentEnd;
			}

			return true;
		}

		private IEnumerator WaitForBaseSettled(RobotPlanResult result, Vector3 goalWorldPosition, float deadline)
		{
			int stableFrames = 0;
			while (Time.realtimeSinceStartup <= deadline)
			{
				if (_stopRequested)
				{
					result.failedAtStage = RobotPlanningStage.BaseSettling;
					result.failureReason = L("等待底盘停稳时规划被停止。", "Planning was stopped while waiting for the base to settle.");
					yield break;
				}

				if (manager == null || manager.diffDriveController == null || manager.diffDriveController.rb == null)
				{
					result.failedAtStage = RobotPlanningStage.BaseSettling;
					result.failureReason = L("底盘停稳阶段中止：差速底盘控制器缺失。", "Base settling aborted because the diff-drive controller is missing.");
					yield break;
				}

				DiffDriveTwinController controller = manager.diffDriveController;
				float positionError = Vector3.Distance(ProjectXZ(controller.rb.position), ProjectXZ(goalWorldPosition));
				if (positionError <= BaseSettlingNearGoalAcceptanceMeters)
				{
					controller.CompletePointGoal(goalWorldPosition);
				}

				float linearSpeed = controller.CurrentPlanarSpeedMeasured;
				float angularSpeed = controller.CurrentYawRateMeasured;
				bool strictSettled = positionError <= BaseStopAcceptanceMeters
					&& linearSpeed <= BaseLinearSpeedToleranceMetersPerSecond
					&& angularSpeed <= BaseAngularSpeedToleranceRadPerSecond;
				bool nearGoalSettled = positionError <= BaseSettlingNearGoalAcceptanceMeters
					&& linearSpeed <= BaseLinearSpeedToleranceMetersPerSecond
					&& angularSpeed <= BaseAngularSpeedToleranceRadPerSecond;
				if (strictSettled || nearGoalSettled)
				{
					stableFrames++;
					if (stableFrames >= BaseStableFixedFramesRequired)
					{
						yield break;
					}
				}
				else
				{
					stableFrames = 0;
				}

				yield return new WaitForFixedUpdate();
			}

			result.failedAtStage = RobotPlanningStage.BaseSettling;
			if (manager != null && manager.diffDriveController != null && manager.diffDriveController.rb != null)
			{
				float finalPositionError = Vector3.Distance(ProjectXZ(manager.diffDriveController.rb.position), ProjectXZ(goalWorldPosition));
				float finalLinearSpeed = manager.diffDriveController.CurrentPlanarSpeedMeasured;
				float finalAngularSpeed = manager.diffDriveController.CurrentYawRateMeasured;
				result.failureReason = L(
					$"等待底盘在请求的世界参考点稳定停稳时超时。误差={finalPositionError:F3}m，线速度={finalLinearSpeed:F3}m/s，角速度={finalAngularSpeed:F3}rad/s。",
					$"Timed out while waiting for the base to settle at the requested world-reference point. Error={finalPositionError:F3}m, linear speed={finalLinearSpeed:F3}m/s, angular speed={finalAngularSpeed:F3}rad/s.");
			}
			else
			{
				result.failureReason = L("等待底盘在请求的世界参考点稳定停稳时超时。", "Timed out while waiting for the base to settle at the requested world-reference point.");
			}
		}

		private bool IsArmTargetWithinTolerance(Vector3 armTargetWorldPosition, float toleranceMeters)
		{
			return manager != null
				&& manager.arm6DOFFKController != null
				&& Vector3.Distance(manager.arm6DOFFKController.EndEffectorWorldPosition, armTargetWorldPosition) <= Mathf.Max(0.001f, toleranceMeters);
		}

		private static void AppendSummary(List<string> parts, string message)
		{
			if (parts == null || string.IsNullOrWhiteSpace(message))
			{
				return;
			}

			parts.Add(message.Trim());
		}

		private static string BuildSuccessSummary(List<string> parts, bool planOnly)
		{
			if (parts == null || parts.Count == 0)
			{
				return planOnly
					? L("规划成功完成。", "Planning completed successfully.")
					: L("规划与执行成功完成。", "Planning and execution completed successfully.");
			}

			return $"{string.Join(" ", parts)} {L(planOnly ? "规划成功完成。" : "规划与执行成功完成。", planOnly ? "Planning completed successfully." : "Planning and execution completed successfully.")}";
		}

		private void CompleteSuccess(RobotPlanResult result, List<string> summaryParts, Action<RobotPlanResult> onComplete)
		{
			currentStage = RobotPlanningStage.Completed;
			result.success = true;
			result.failedAtStage = RobotPlanningStage.None;
			result.failureReason = string.Empty;
			result.summary = BuildSuccessSummary(summaryParts, _planOnly);
			lastSummary = result.summary;
			isPlanning = false;
			_planningRoutine = null;
			onComplete?.Invoke(result);
		}

		private void CompleteFailure(RobotPlanResult result, RobotPlanningStage stage, string reason, Action<RobotPlanResult> onComplete)
		{
			currentStage = RobotPlanningStage.Failed;
			result.success = false;
			result.failedAtStage = stage;
			result.failureReason = reason;
			if (string.IsNullOrEmpty(result.dockingSummary))
			{
				result.dockingSummary = reason;
			}
			result.summary = reason;
			lastSummary = reason;
			Debug.LogWarning($"[RobotTrajectoryPlanner] Failed at {RobotSimulationLocalization.PlanningStage(stage)}: {reason}");
			isPlanning = false;
			_planningRoutine = null;
			onComplete?.Invoke(result);
		}

		private void RemoveRobotOwnedObstacles(List<Collider> obstacles, Transform baseIgnoreRoot, Transform armIgnoreRoot)
		{
			if (obstacles == null || obstacles.Count == 0)
			{
				return;
			}

			int removedCount = 0;
			for (int i = obstacles.Count - 1; i >= 0; i--)
			{
				Collider collider = obstacles[i];
				if (collider == null)
				{
					obstacles.RemoveAt(i);
					removedCount++;
					continue;
				}

				if ((baseIgnoreRoot != null && collider.transform.IsChildOf(baseIgnoreRoot))
					|| (armIgnoreRoot != null && collider.transform.IsChildOf(armIgnoreRoot))
					|| IsArmLinkCollider(collider))
				{
					obstacles.RemoveAt(i);
					removedCount++;
				}
			}

			if (removedCount > 0)
			{
				Debug.Log($"[RobotTrajectoryPlanner] Removed {removedCount} robot-owned colliders from the obstacle set before planning.");
			}
		}

		private static bool IsArmLinkCollider(Collider collider)
		{
			if (collider == null)
			{
				return false;
			}

			ArticulationBody ownerBody = collider.GetComponentInParent<ArticulationBody>();
			if (ownerBody == null)
			{
				return false;
			}

			string ownerName = ownerBody.name;
			if (!string.IsNullOrEmpty(ownerName) && ownerName.StartsWith("Link_", StringComparison.OrdinalIgnoreCase))
			{
				return true;
			}

			string colliderName = collider.transform.name;
			return !string.IsNullOrEmpty(colliderName) && colliderName.StartsWith("Link_", StringComparison.OrdinalIgnoreCase);
		}

		private void HandlePlanningException(Exception exception, Action<RobotPlanResult> onComplete)
		{
			RobotPlanResult failed = BuildImmediateFailure(L($"规划器异常：{exception.Message}", $"Planner exception: {exception.Message}"));
			currentStage = RobotPlanningStage.Failed;
			isPlanning = false;
			_planningRoutine = null;
			lastSummary = failed.summary;
			lastShadowValidationResult = new ShadowValidationResult
			{
				passed = false,
				message = failed.summary
			};
			Debug.LogError($"[RobotTrajectoryPlanner] {exception}");
			onComplete?.Invoke(failed);
		}

		private void EnsurePlannerInternals()
		{
			if (_safetyFilter == null)
			{
				_safetyFilter = new ExecutionSafetyFilter(_shadowGate);
			}
		}

		private static float[] CloneAngles(float[] source)
		{
			return source != null && source.Length >= 6 ? (float[])source.Clone() : null;
		}

		private static RobotPlanResult BuildImmediateFailure(string message)
		{
			return new RobotPlanResult
			{
				accepted = false,
				success = false,
				failedAtStage = RobotPlanningStage.Failed,
				failureReason = message,
				summary = message
			};
		}

		private void EnsureManager()
		{
			if (manager == null)
			{
				manager = GetComponent<RobotSimulationManager>();
			}

			if (manager == null)
			{
				manager = FindObjectOfType<RobotSimulationManager>();
			}
		}

		private static float GetRequestedEeTolerance(RobotPlanRequest request)
		{
			if (request == null)
			{
				return ArmWorldTargetToleranceMeters;
			}

			return Mathf.Max(0.001f, request.eePositionToleranceMeters > 0f ? request.eePositionToleranceMeters : ArmWorldTargetToleranceMeters);
		}

		private static Vector3 ProjectXZ(Vector3 value)
		{
			return new Vector3(value.x, 0f, value.z);
		}

		private static string L(string chinese, string english)
		{
			return RobotSimulationLocalization.Text(chinese, english);
		}
	}
}
