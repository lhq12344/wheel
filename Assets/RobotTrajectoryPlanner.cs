using System;
using System.Collections;
using System.Collections.Generic;
using System.Threading;
using System.Threading.Tasks;
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
		[SerializeField] private bool drawDockingSamplingGizmos = true;

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
		private readonly List<CoordinatedTaskPlanner.DockingDebugSample> _dockingDebugGizmoSamples = new List<CoordinatedTaskPlanner.DockingDebugSample>();
		private readonly SafetyGateTimelineBuffer _safetyGateTimelineBuffer = new SafetyGateTimelineBuffer();

		private const float BasePositionToleranceMeters = 0.02f;
		private const float BaseStopAcceptanceMeters = 0.025f;
		private const float BaseSettlingNearGoalAcceptanceMeters = 0.04f;
		private const float BaseSettlingToleranceEpsilonMeters = 0.003f;
		private const float BaseLinearSpeedToleranceMetersPerSecond = 0.03f;
		private const float BaseAngularSpeedToleranceRadPerSecond = 0.05f;
		private const int BaseStableFixedFramesRequired = 3;
		private const float ArmWorldTargetToleranceMeters = 0.10f;
		private const float ArmPlanningFallbackToleranceMeters = 0.10f;
		private const float ArmPlanningFallbackToleranceSlackMeters = 0.012f;
		private const float ArmPlanningDockingRetryResidualThresholdMeters = 0.08f;
		private const float ArmExecutionDurationPaddingSeconds = 4f;
		private const float ArmExecutionMinSegmentSeconds = 0.02f;
		private const float ArmExecutionSpeedSafetyFactor = 1.2f;
		private const float ArmExecutionCorrectionGraceSeconds = 2.5f;
		private const float ArmExecutionCorrectionMaxResidualMeters = 0.18f;
		private const float ArmExecutionCorrectionMaxJointErrorDeg = 65f;
		private const float ArmExecutionCorrectionToleranceMeters = 0.05f;
		private const int MaxArmPlanningDockingRetries = 1;
		private const int MaxPostSettleCorrectionAttempts = 1;
		private const int SafetyGateBlockDecelFrames = 2;
		private const float SafetyGateMinLookaheadFloorSeconds = 0.02f;
		private const float SafetyGateRecoveryBudgetFloorSeconds = 4f;
		private const float SafetyGateRecoveryBudgetCeilingSeconds = 12f;

		private sealed class ArmStagePreparationState
		{
			public Vector3 ResolvedBaseGoal;
			public float ResolvedBaseYaw;
			public float[] PreferredArmSolveSeed;
		}

		private ExecutionSafetyFilter _safetyFilter;
		private readonly IMirrorStateProvider _mirrorStateProvider = new UnityMirrorStateProvider();
		private Coroutine _planningRoutine;
		private bool _stopRequested;
		private bool _planOnly;
		private long _safetyGateSequenceCounter;
		private bool _loggedDynamicBaseLock;
		private bool _startupCommandTraceLogged;
		private bool _startupTraceSeedInitialized;
		private long _startupTraceSequenceSeed;
		private Vector3 _startupTraceBaseStartWorldPosition;
		private float _startupTraceBaseTargetYawDeg;
		private readonly List<Vector3> _startupTraceBaseWaypoints = new List<Vector3>();
		private long _lastBaseQueueWaitSequenceId;
		private long _lastArmQueueWaitSequenceId;
		private CancellationTokenSource _offlineComputeCts;
		private readonly SemaphoreSlim _offlineComputeSemaphore = new SemaphoreSlim(2, 2);

		private struct ArmTimingSnapshot
		{
			public float[] startAnglesDeg;
			public float[] maxJointSpeedDegPerSecond;
			public List<RobotPlanJointSample> samples;
		}

		private struct ArmExecutionTimingEstimate
		{
			public float executionTimeScale;
			public float scaledExecutionSeconds;
		}

		private struct StartupPreviewBuildResult
		{
			public int totalCount;
			public bool hasBaseCommands;
			public bool hasArmCommands;
			public List<SafetyGateStartupPreviewItem> previewItems;
		}

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

		private void OnDrawGizmos()
		{
			if (!drawDockingSamplingGizmos
				|| !_coordinatedTaskPlanner.TryGetLastDockingSamplingDebugInfo(out Vector3 targetWorldPosition, out float innerRadiusMeters, out float outerRadiusMeters))
			{
				return;
			}

			Gizmos.color = new Color(0.12f, 0.82f, 0.22f, 0.9f);
			Gizmos.DrawWireSphere(targetWorldPosition, innerRadiusMeters);
			Gizmos.color = new Color(0.12f, 0.82f, 0.22f, 0.6f);
			Gizmos.DrawWireSphere(targetWorldPosition, outerRadiusMeters);
			Gizmos.color = Color.white;
			Gizmos.DrawSphere(targetWorldPosition, 0.035f);

			if (_coordinatedTaskPlanner.CopyLastDockingDebugSamples(_dockingDebugGizmoSamples) <= 0)
			{
				return;
			}

			for (int i = 0; i < _dockingDebugGizmoSamples.Count; i++)
			{
				CoordinatedTaskPlanner.DockingDebugSample sample = _dockingDebugGizmoSamples[i];
				if (sample == null)
				{
					continue;
				}

				Vector3 ray = sample.targetWorldPosition - sample.baseWorldPosition;
				if (sample.selected)
				{
					Gizmos.color = new Color(0.12f, 0.82f, 0.22f, 0.95f);
					Gizmos.DrawRay(sample.baseWorldPosition, ray);
					DrawGizmoCross(sample.baseWorldPosition, 0.05f);
					continue;
				}

				if (sample.ikFailed)
				{
					Gizmos.color = new Color(0.92f, 0.15f, 0.15f, 0.9f);
					Gizmos.DrawRay(sample.baseWorldPosition, ray);
					DrawGizmoCross(sample.baseWorldPosition, 0.07f);
					continue;
				}

				if (sample.strictPreviewFailed)
				{
					Gizmos.color = new Color(0.96f, 0.52f, 0.1f, 0.8f);
					Gizmos.DrawRay(sample.baseWorldPosition, ray);
					DrawGizmoCross(sample.baseWorldPosition, 0.04f);
					continue;
				}

				Gizmos.color = new Color(0.18f, 0.58f, 0.96f, 0.55f);
				Gizmos.DrawRay(sample.baseWorldPosition, ray);
				DrawGizmoCross(sample.baseWorldPosition, 0.03f);
			}
		}

		private static void DrawGizmoCross(Vector3 center, float halfSize)
		{
			Vector3 a = new Vector3(-halfSize, 0f, -halfSize);
			Vector3 b = new Vector3(halfSize, 0f, halfSize);
			Vector3 c = new Vector3(-halfSize, 0f, halfSize);
			Vector3 d = new Vector3(halfSize, 0f, -halfSize);
			Gizmos.DrawLine(center + a, center + b);
			Gizmos.DrawLine(center + c, center + d);
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
			ResetSafetyGateTimelineState();
			StartOfflineComputeSession();
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
			_safetyGateTimelineBuffer.Clear();
			CancelOfflineComputeSession();
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
			ReturnShadowToMirror();
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
			MirrorSnapshot initialMirror = _mirrorStateProvider.Capture(manager.diffDriveController, manager.arm6DOFFKController);
			if (manager.shadowRobotVisualizer != null)
			{
				manager.shadowRobotVisualizer.ResetToMirrorSnapshot(initialMirror, initialMirror.armJointAnglesDeg);
			}
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

			Transform shadowBaseIgnoreRoot = manager.GetShadowBaseTwinRoot();
			List<Collider> obstacles = _physicsQueries.CollectObstacleColliders(baseIgnoreRoot, armIgnoreRoot, shadowBaseIgnoreRoot);
			RemoveRobotOwnedObstacles(obstacles, baseIgnoreRoot, armIgnoreRoot);
			float baseRadius = _physicsQueries.EstimateBaseRadius(manager.diffDriveController);
			Vector3 currentBasePosition = manager.diffDriveController.rb.position;
			float currentBaseYaw = manager.diffDriveController.rb.rotation.eulerAngles.y;
			ArmStagePreparationState armStageState = new ArmStagePreparationState
			{
				ResolvedBaseGoal = currentBasePosition,
				ResolvedBaseYaw = currentBaseYaw,
				PreferredArmSolveSeed = null
			};
			result.resolvedBaseStopWorldPosition = currentBasePosition;
			result.resolvedBaseStopYawDeg = currentBaseYaw;
			result.effectiveEePositionToleranceMeters = eeToleranceMeters;

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
				result.armReachabilityIsLoosePrecheck = currentBaseResolution.armReachabilityIsLoosePrecheck;
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
					if (!_coordinatedTaskPlanner.TryPrepareAutoDockingSearch(
						request,
						manager.diffDriveController,
						manager.arm6DOFFKController,
						baseRadius,
						_physicsQueries,
						obstacles,
						out CoordinatedTaskPlanner.DockingSearchContext dockingContext,
						out resolution))
					{
						CompleteFailure(result, RobotPlanningStage.DockingSearch, resolution.failureReason, onComplete);
						yield break;
					}

					if (dockingContext != null)
					{
						yield return null;
						if (!_coordinatedTaskPlanner.TryCompleteAutoDockingSearch(dockingContext, out resolution))
						{
							CompleteFailure(result, RobotPlanningStage.DockingSearch, resolution.failureReason, onComplete);
							yield break;
						}
					}

					if (resolution != null && !resolution.accepted && !resolution.dockingPoseFound && !string.IsNullOrEmpty(resolution.failureReason))
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
			result.armReachabilityIsLoosePrecheck = resolution.armReachabilityIsLoosePrecheck;
			result.dockingSummary = resolution.dockingSummary;
			result.coarseCandidateCount = resolution.coarseCandidateCount;
			result.fineCandidateCount = resolution.fineCandidateCount;
			result.ikSolveCount = resolution.ikSolveCount;
			result.basePathCheckCount = resolution.basePathCheckCount;
			result.dockingFailureCategory = resolution.dockingFailureCategory;
			armStageState.ResolvedBaseGoal = resolution.resolvedBaseStopWorldPosition;
			armStageState.ResolvedBaseYaw = resolution.resolvedBaseStopYawDeg;
			armStageState.ResolvedBaseGoal.y = currentBasePosition.y;
			armStageState.PreferredArmSolveSeed = resolution.hasPreferredArmSolveSeed
				? CloneAngles(resolution.preferredArmSolveSeedAnglesDeg)
				: null;
			result.resolvedBaseStopWorldPosition = armStageState.ResolvedBaseGoal;
			result.resolvedBaseStopYawDeg = armStageState.ResolvedBaseYaw;
			AppendSummary(summaryParts, resolution.dockingSummary);

			bool shouldMoveBase =
				request.requireBaseMove &&
				result.baseMoveRequired &&
				Vector3.Distance(ProjectXZ(currentBasePosition), ProjectXZ(armStageState.ResolvedBaseGoal)) > BasePositionToleranceMeters;
			if (shouldMoveBase)
			{
				float basePlanningDeadline = Time.realtimeSinceStartup + basePlanningBudgetSeconds;
				if (Time.realtimeSinceStartup > basePlanningDeadline)
				{
					CompleteFailure(result, RobotPlanningStage.BasePlanning, L("底盘规划开始前就已经超时。", "Planning timed out before base planning finished."), onComplete);
					yield break;
				}

				currentStage = RobotPlanningStage.BasePlanning;
				Debug.Log($"[RobotTrajectoryPlanner] Base planning: start={currentBasePosition}, goal={armStageState.ResolvedBaseGoal}, yaw={armStageState.ResolvedBaseYaw:F1}, radius={baseRadius:F3}");
				_distanceFieldSampler.Build(currentBasePosition, armStageState.ResolvedBaseGoal, obstacles, distanceFieldResolution, basePlanningMargin);
				if (!_basePlanner.TryPlan(currentBasePosition, currentBaseYaw, armStageState.ResolvedBaseGoal, armStageState.ResolvedBaseYaw, baseRadius, _distanceFieldSampler, _physicsQueries, obstacles, out List<Vector3> baseWaypoints, out string baseFailure))
				{
					CompleteFailure(result, RobotPlanningStage.BasePlanning, baseFailure, onComplete);
					yield break;
				}

				result.baseWaypoints = baseWaypoints;
				AppendSummary(summaryParts, L(
					$"底盘已在统一世界坐标系下规划到 ({armStageState.ResolvedBaseGoal.x:F2}, {armStageState.ResolvedBaseGoal.y:F2}, {armStageState.ResolvedBaseGoal.z:F2})。",
					$"Base planned toward ({armStageState.ResolvedBaseGoal.x:F2}, {armStageState.ResolvedBaseGoal.y:F2}, {armStageState.ResolvedBaseGoal.z:F2}) in the shared world frame."));

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
					EnterShadowBasePreview();
					Debug.Log("[RobotTrajectoryPlanner] Base execution started.");
					float baseExecutionDeadline = Time.realtimeSinceStartup + ComputeBaseExecutionBudgetSeconds(
						manager.diffDriveController,
						manager.diffDriveController.rb.position,
						baseWaypoints,
						baseExecutionBudgetSeconds);
					yield return ExecuteBasePath(result, request, armStageState.ResolvedBaseGoal, armStageState.ResolvedBaseYaw, baseRadius, obstacles, baseExecutionDeadline);
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
				yield return WaitForBaseSettled(result, armStageState.ResolvedBaseGoal, baseSettlingDeadline);
				if (!string.IsNullOrEmpty(result.failureReason))
				{
					CompleteFailure(result, result.failedAtStage, result.failureReason, onComplete);
					yield break;
				}

				manager.CompleteShadowBaseSession(resetShadowToLivePose: false);
				manager.SyncArmToCurrentBasePoseImmediate();
				manager.arm6DOFFKController?.RefreshRuntimeState();
				yield return new WaitForFixedUpdate();
				EnterShadowArmPreviewFromLiveBase();
				AppendSummary(summaryParts, L("底盘已稳定停稳，随后开始机械臂阶段。", "Base settled cleanly before the arm stage started."));

				yield return EnsureSettledBaseSupportsArmTarget(
					request,
					result,
					summaryParts,
					obstacles,
					baseRadius,
					baseExecutionBudgetSeconds,
					armStageState);
				if (!string.IsNullOrEmpty(result.failureReason))
				{
					CompleteFailure(result, result.failedAtStage, result.failureReason, onComplete);
					yield break;
				}
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
					int armPlanningDockingRetryCount = 0;
					while (true)
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
						bool usedFallbackTolerance = false;
						float planningToleranceMeters = eeToleranceMeters;
						if (!_armMotionPlanner.TryPlanToWorldPosition(
							manager.arm6DOFFKController,
							request.armTargetWorldPosition,
							_distanceFieldSampler,
							out List<RobotPlanJointSample> armSamples,
							out string armFailure,
							armStageState.PreferredArmSolveSeed,
							planningToleranceMeters))
						{
							if (IsSoftArmPlanningFailure(armFailure)
								&& TryGetRelaxedArmPlanningTolerance(eeToleranceMeters, out float relaxedToleranceMeters)
								&& _armMotionPlanner.TryPlanToWorldPosition(
									manager.arm6DOFFKController,
									request.armTargetWorldPosition,
									_distanceFieldSampler,
									out armSamples,
									out armFailure,
									armStageState.PreferredArmSolveSeed,
									relaxedToleranceMeters))
							{
								usedFallbackTolerance = true;
								planningToleranceMeters = relaxedToleranceMeters;
								AppendSummary(summaryParts, L(
									$"机械臂在严格容差 {eeToleranceMeters:F3}m 下未直接收敛，已自动切换到回退容差 {relaxedToleranceMeters:F3}m 继续规划。",
									$"Arm planning did not converge under the strict tolerance {eeToleranceMeters:F3}m, so planning automatically retried with fallback tolerance {relaxedToleranceMeters:F3}m."));
							}
							else if (!_planOnly
								&& request.autoResolveBaseDockingPose
								&& request.allowReplan
								&& armPlanningDockingRetryCount < MaxArmPlanningDockingRetries
								&& IsLargeResidualArmPlanningFailure(armFailure))
							{
								float baseExecutionDeadline = Time.realtimeSinceStartup + Mathf.Max(4f, baseExecutionBudgetSeconds);
								if (TryExtractArmPlanningResidualMeters(armFailure, out float residualMeters))
								{
									AppendSummary(summaryParts, L(
										$"机械臂规划残差达到 {residualMeters:F3}m，判断当前停靠位不理想，开始重新搜索停靠位并重试。",
										$"Arm planning residual reached {residualMeters:F3}m, so the current docking pose is being retried with a new docking search."));
								}
								else
								{
									AppendSummary(summaryParts, L(
										"机械臂规划未收敛，开始重新搜索停靠位并重试一次。",
										"Arm planning did not converge, so docking search will retry once."));
								}

								armPlanningDockingRetryCount++;
								yield return RetryDockingSearchAndMoveBaseIfNeeded(
									request,
									result,
									summaryParts,
									obstacles,
									baseRadius,
									baseExecutionDeadline,
									armStageState);
								if (!string.IsNullOrEmpty(result.failureReason))
								{
									CompleteFailure(result, result.failedAtStage, result.failureReason, onComplete);
									yield break;
								}

								manager.SyncArmToCurrentBasePoseImmediate();
								manager.arm6DOFFKController?.RefreshRuntimeState();
								yield return new WaitForFixedUpdate();
								continue;
							}
							else
							{
								CompleteFailure(result, RobotPlanningStage.ArmPlanning, armFailure, onComplete);
								yield break;
							}
						}

						float plannedFinalResidualMeters = ComputeArmTrajectoryFinalWorldResidualMeters(
							manager.arm6DOFFKController,
							armSamples,
							request.armTargetWorldPosition);
						Debug.Log(
							$"[RobotTrajectoryPlanner] Arm plan final EE residual={plannedFinalResidualMeters:F3}m, requestedTolerance={eeToleranceMeters:F3}m, effectiveTolerance={planningToleranceMeters:F3}m");
						if (plannedFinalResidualMeters > eeToleranceMeters + 1e-4f)
						{
							if (!_planOnly
								&& request.autoResolveBaseDockingPose
								&& request.allowReplan
								&& armPlanningDockingRetryCount < MaxArmPlanningDockingRetries)
							{
								AppendSummary(summaryParts, L(
									$"机械臂规划虽然在执行容差 {planningToleranceMeters:F3}m 内找到解，但最终末端残差 {plannedFinalResidualMeters:F3}m 仍超过请求容差 {eeToleranceMeters:F3}m，开始重新搜索停靠位并重试。",
									$"Arm planning found a solution within the execution tolerance {planningToleranceMeters:F3}m, but the final end-effector residual {plannedFinalResidualMeters:F3}m still exceeded the requested tolerance {eeToleranceMeters:F3}m, so docking search will retry."));
								armPlanningDockingRetryCount++;
								float baseExecutionDeadline = Time.realtimeSinceStartup + Mathf.Max(4f, baseExecutionBudgetSeconds);
								yield return RetryDockingSearchAndMoveBaseIfNeeded(
									request,
									result,
									summaryParts,
									obstacles,
									baseRadius,
									baseExecutionDeadline,
									armStageState);
								if (!string.IsNullOrEmpty(result.failureReason))
								{
									CompleteFailure(result, result.failedAtStage, result.failureReason, onComplete);
									yield break;
								}

								manager.SyncArmToCurrentBasePoseImmediate();
								manager.arm6DOFFKController?.RefreshRuntimeState();
								yield return new WaitForFixedUpdate();
								continue;
							}

							CompleteFailure(
								result,
								RobotPlanningStage.ArmPlanning,
								L(
									$"机械臂规划最终末端残差为 {plannedFinalResidualMeters:F3}m，超过请求容差 {eeToleranceMeters:F3}m。",
									$"Arm plan final end-effector residual {plannedFinalResidualMeters:F3}m exceeded the requested tolerance {eeToleranceMeters:F3}m."),
								onComplete);
							yield break;
						}

						result.armTrajectorySamples = armSamples;
						result.effectiveEePositionToleranceMeters = planningToleranceMeters;
						currentStage = RobotPlanningStage.ArmShadowValidation;
						Debug.Log($"[RobotTrajectoryPlanner] Arm shadow validation: samples={armSamples.Count}");
						lastShadowValidationResult = _shadowGate.ValidateArmTrajectory(manager.arm6DOFFKController, armSamples);
						if (!lastShadowValidationResult.passed)
						{
							if (!_planOnly
								&& request.autoResolveBaseDockingPose
								&& request.allowReplan
								&& armPlanningDockingRetryCount < MaxArmPlanningDockingRetries
								&& lastShadowValidationResult.singularityRisk)
							{
								AppendSummary(summaryParts, L(
									$"机械臂影子验证发现轨迹靠近奇异区（{lastShadowValidationResult.message}），开始重新搜索停靠位并重试。",
									$"Arm shadow validation detected a singular trajectory ({lastShadowValidationResult.message}), so docking search will retry."));
								armPlanningDockingRetryCount++;
								float baseExecutionDeadline = Time.realtimeSinceStartup + Mathf.Max(4f, baseExecutionBudgetSeconds);
								yield return RetryDockingSearchAndMoveBaseIfNeeded(
									request,
									result,
									summaryParts,
									obstacles,
									baseRadius,
									baseExecutionDeadline,
									armStageState);
								if (!string.IsNullOrEmpty(result.failureReason))
								{
									CompleteFailure(result, result.failedAtStage, result.failureReason, onComplete);
									yield break;
								}

								manager.SyncArmToCurrentBasePoseImmediate();
								manager.arm6DOFFKController?.RefreshRuntimeState();
								yield return new WaitForFixedUpdate();
								continue;
							}

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
						if (usedFallbackTolerance)
						{
							AppendSummary(summaryParts, L(
								$"本次机械臂规划使用了 {planningToleranceMeters:F3}m 的回退末端容差。",
								$"This arm plan used a fallback end-effector tolerance of {planningToleranceMeters:F3}m."));
						}

						if (!_planOnly)
						{
							currentStage = RobotPlanningStage.ArmExecution;
							EnterShadowArmPreviewFromLiveBase();
							float armExecutionTimeScale = 1f;
							float scaledExecutionSeconds = Mathf.Max(ArmExecutionMinSegmentSeconds, ComputeArmScaledExecutionSeconds(armSamples, 1f));
							yield return ComputeArmExecutionTimingEstimateAsync(
								manager.arm6DOFFKController,
								armSamples,
								estimate =>
								{
									armExecutionTimeScale = Mathf.Clamp(estimate.executionTimeScale, 1f, 8f);
									scaledExecutionSeconds = Mathf.Max(ArmExecutionMinSegmentSeconds, estimate.scaledExecutionSeconds);
								});
							float armExecutionDeadline = Time.realtimeSinceStartup + Mathf.Max(
								armExecutionBudgetSeconds,
								scaledExecutionSeconds + ArmExecutionDurationPaddingSeconds);
							Debug.Log($"[RobotTrajectoryPlanner] Arm execution started. timeScale={armExecutionTimeScale:F2}, scaledDuration={scaledExecutionSeconds:F2}s, budget={armExecutionBudgetSeconds:F2}s");
							yield return ExecuteArmTrajectory(result, request, obstacles, armExecutionDeadline, armExecutionTimeScale);
							if (!string.IsNullOrEmpty(result.failureReason))
							{
								CompleteFailure(result, result.failedAtStage, result.failureReason, onComplete);
								yield break;
							}

							AppendSummary(summaryParts, L("机械臂执行完成。", "Arm execution finished at the requested end-effector world target."));
						}

						break;
					}
				}
			}

			CompleteSuccess(result, summaryParts, onComplete);
		}

		private IEnumerator EnsureSettledBaseSupportsArmTarget(
			RobotPlanRequest request,
			RobotPlanResult result,
			List<string> summaryParts,
			List<Collider> obstacles,
			float baseRadius,
			float baseExecutionBudgetSeconds,
			ArmStagePreparationState state)
		{
			if (_planOnly || request == null || !request.requireArmMove)
			{
				yield break;
			}

			currentStage = RobotPlanningStage.CurrentBaseReachabilityCheck;
			yield return null;

			CoordinatedTaskResolution evaluation = _coordinatedTaskPlanner.EvaluateCurrentBaseExecution(
				request,
				manager.diffDriveController,
				manager.arm6DOFFKController);
			result.armReachableFromCurrentBase = evaluation.armReachableFromCurrentBase;
			result.armReachabilityIsLoosePrecheck = evaluation.armReachabilityIsLoosePrecheck;
			if (evaluation.armReachableFromCurrentBase)
			{
				if (evaluation.hasPreferredArmSolveSeed)
				{
					state.PreferredArmSolveSeed = CloneAngles(evaluation.preferredArmSolveSeedAnglesDeg);
				}

				yield break;
			}

			string lastFailureReason = string.IsNullOrEmpty(evaluation.failureReason)
				? L("底盘实际停稳后，当前真实位姿仍然无法安全支持机械臂目标。", "After the base settled, the real base pose still cannot safely support the arm target.")
				: evaluation.failureReason;
			if (IsSoftArmPlanningFailure(lastFailureReason))
			{
				AppendSummary(summaryParts, L(
					"底盘停稳后的快速可达性检查未能直接收敛，转交正式机械臂规划阶段继续求解。",
					"Post-settle reachability check did not converge directly, so control is being handed off to the full arm-planning stage."));
				yield break;
			}

			bool attemptedFallbackDockingSearch = false;
			if (request.allowReplan && request.autoResolveBaseDockingPose)
			{
				attemptedFallbackDockingSearch = true;
				AppendSummary(summaryParts, L(
					"当前真实底盘位姿无法支持机械臂目标，开始重新搜索新的可执行停靠位。",
					"The current real base pose cannot support the arm target, so a new executable docking pose search is starting."));

				currentStage = RobotPlanningStage.DockingSearch;
				yield return null;

				if (_coordinatedTaskPlanner.TryPrepareAutoDockingSearch(
					request,
					manager.diffDriveController,
					manager.arm6DOFFKController,
					baseRadius,
					_physicsQueries,
					obstacles,
					out CoordinatedTaskPlanner.DockingSearchContext fallbackDockingContext,
					out CoordinatedTaskResolution fallbackResolution))
				{
					if (fallbackDockingContext != null)
					{
						yield return null;
						if (_coordinatedTaskPlanner.TryCompleteAutoDockingSearch(fallbackDockingContext, out fallbackResolution))
						{
							result.dockingPoseFound = fallbackResolution.dockingPoseFound;
							result.baseMoveRequired = fallbackResolution.baseMoveRequired;
							result.armReachableFromCurrentBase = fallbackResolution.armReachableFromCurrentBase;
							result.armReachabilityIsLoosePrecheck = fallbackResolution.armReachabilityIsLoosePrecheck;
							result.dockingSummary = fallbackResolution.dockingSummary;
							result.coarseCandidateCount = fallbackResolution.coarseCandidateCount;
							result.fineCandidateCount = fallbackResolution.fineCandidateCount;
							result.ikSolveCount = fallbackResolution.ikSolveCount;
							result.basePathCheckCount = fallbackResolution.basePathCheckCount;
							result.dockingFailureCategory = fallbackResolution.dockingFailureCategory;
							result.resolvedBaseStopWorldPosition = fallbackResolution.resolvedBaseStopWorldPosition;
							result.resolvedBaseStopYawDeg = fallbackResolution.resolvedBaseStopYawDeg;
							state.ResolvedBaseGoal = fallbackResolution.resolvedBaseStopWorldPosition;
							state.ResolvedBaseYaw = fallbackResolution.resolvedBaseStopYawDeg;
							state.ResolvedBaseGoal.y = manager.diffDriveController.rb.position.y;
							if (fallbackResolution.hasPreferredArmSolveSeed)
							{
								state.PreferredArmSolveSeed = CloneAngles(fallbackResolution.preferredArmSolveSeedAnglesDeg);
							}

							AppendSummary(summaryParts, fallbackResolution.dockingSummary);
							if (fallbackResolution.armReachableFromCurrentBase)
							{
								AppendSummary(summaryParts, L(
									"重新搜索后确认当前底盘位姿已经支持机械臂目标，无需再次移动底盘。",
									"After the retry search, the current base pose now supports the arm target, so no further base motion is required."));
								yield break;
							}

							lastFailureReason = string.IsNullOrEmpty(fallbackResolution.failureReason)
								? lastFailureReason
								: fallbackResolution.failureReason;
						}
						else
						{
							lastFailureReason = string.IsNullOrEmpty(fallbackResolution.failureReason)
								? lastFailureReason
								: fallbackResolution.failureReason;
						}
					}
					else
					{
						lastFailureReason = string.IsNullOrEmpty(fallbackResolution.failureReason)
							? lastFailureReason
							: fallbackResolution.failureReason;
					}
				}
			}

			if (!request.requireBaseMove || !request.allowReplan)
			{
				result.failedAtStage = RobotPlanningStage.CurrentBaseReachabilityCheck;
				result.failureReason = lastFailureReason;
				yield break;
			}

			Vector3 currentSettledBasePosition = manager.diffDriveController.rb.position;
			float driftFromPlannedDocking = Vector3.Distance(ProjectXZ(currentSettledBasePosition), ProjectXZ(state.ResolvedBaseGoal));
			if (driftFromPlannedDocking <= BasePositionToleranceMeters)
			{
				result.failedAtStage = attemptedFallbackDockingSearch
					? RobotPlanningStage.DockingSearch
					: RobotPlanningStage.CurrentBaseReachabilityCheck;
				result.failureReason = lastFailureReason;
				yield break;
			}

			int correctionAttemptBudget = Mathf.Min(maxLocalReplans, MaxPostSettleCorrectionAttempts);
			for (int correctiveAttempt = 1; correctiveAttempt <= correctionAttemptBudget; correctiveAttempt++)
			{
				Vector3 currentBasePosition = manager.diffDriveController.rb.position;
				Vector3 correctionGoal = state.ResolvedBaseGoal;
				correctionGoal.y = currentBasePosition.y;
				float correctionDistance = Vector3.Distance(ProjectXZ(currentBasePosition), ProjectXZ(correctionGoal));
				if (correctionDistance <= BasePositionToleranceMeters)
				{
					result.failedAtStage = RobotPlanningStage.CurrentBaseReachabilityCheck;
					result.failureReason = lastFailureReason;
					yield break;
				}

				result.replanCount++;
				AppendSummary(summaryParts, L(
					$"底盘实际停靠位偏离了原计划停靠位，开始第 {correctiveAttempt} 次短距离回停修正。",
					$"The real settled base pose drifted from the planned docking pose, so return-to-docking correction {correctiveAttempt} is starting."));

				currentStage = RobotPlanningStage.LocalReplan;
				yield return null;

				_distanceFieldSampler.Build(currentBasePosition, state.ResolvedBaseGoal, obstacles, distanceFieldResolution, basePlanningMargin);
				if (!_basePlanner.TryPlan(
					currentBasePosition,
					manager.diffDriveController.rb.rotation.eulerAngles.y,
					state.ResolvedBaseGoal,
					state.ResolvedBaseYaw,
					baseRadius,
					_distanceFieldSampler,
					_physicsQueries,
					obstacles,
					out List<Vector3> correctionWaypoints,
					out string correctionFailure))
				{
					result.failedAtStage = RobotPlanningStage.LocalReplan;
					result.failureReason = correctionFailure;
					yield break;
				}

				result.baseWaypoints = correctionWaypoints;
				currentStage = RobotPlanningStage.BaseShadowValidation;
				yield return null;

				lastShadowValidationResult = _shadowGate.ValidateBasePath(correctionWaypoints, baseRadius, _physicsQueries, _distanceFieldSampler, obstacles);
				if (!lastShadowValidationResult.passed)
				{
					result.failedAtStage = RobotPlanningStage.BaseShadowValidation;
					result.failureReason = lastShadowValidationResult.message;
					yield break;
				}

				currentStage = RobotPlanningStage.BaseExecution;
				float correctionExecutionDeadline = Time.realtimeSinceStartup + ComputeBaseExecutionBudgetSeconds(
					manager.diffDriveController,
					manager.diffDriveController.rb.position,
					correctionWaypoints,
					Mathf.Max(4f, baseExecutionBudgetSeconds * 0.5f));
				yield return ExecuteBasePath(result, request, state.ResolvedBaseGoal, state.ResolvedBaseYaw, baseRadius, obstacles, correctionExecutionDeadline);
				if (!string.IsNullOrEmpty(result.failureReason))
				{
					yield break;
				}

				currentStage = RobotPlanningStage.BaseSettling;
				float correctionSettlingDeadline = Time.realtimeSinceStartup + Mathf.Max(2f, baseExecutionBudgetSeconds * 0.25f);
				yield return WaitForBaseSettled(result, state.ResolvedBaseGoal, correctionSettlingDeadline);
				if (!string.IsNullOrEmpty(result.failureReason))
				{
					yield break;
				}

				manager.CompleteShadowBaseSession(resetShadowToLivePose: false);
				manager.SyncArmToCurrentBasePoseImmediate();
				manager.arm6DOFFKController?.RefreshRuntimeState();
				yield return new WaitForFixedUpdate();

				currentStage = RobotPlanningStage.CurrentBaseReachabilityCheck;
				yield return null;

				evaluation = _coordinatedTaskPlanner.EvaluateCurrentBaseExecution(
					request,
					manager.diffDriveController,
					manager.arm6DOFFKController);
				result.armReachableFromCurrentBase = evaluation.armReachableFromCurrentBase;
				result.armReachabilityIsLoosePrecheck = evaluation.armReachabilityIsLoosePrecheck;
				if (evaluation.armReachableFromCurrentBase)
				{
					if (evaluation.hasPreferredArmSolveSeed)
					{
						state.PreferredArmSolveSeed = CloneAngles(evaluation.preferredArmSolveSeedAnglesDeg);
					}

					AppendSummary(summaryParts, L(
						"底盘修正停靠后，当前真实底盘位姿已经支持机械臂目标。",
						"After the corrective docking move, the current real base pose supports the arm target."));
					yield break;
				}

				lastFailureReason = string.IsNullOrEmpty(evaluation.failureReason)
					? lastFailureReason
					: evaluation.failureReason;
				if (IsSoftArmPlanningFailure(lastFailureReason))
				{
					AppendSummary(summaryParts, L(
						"底盘回停修正后仍未在快速检查阶段直接收敛，转交正式机械臂规划阶段继续求解。",
						"Return-to-docking correction still did not converge during the fast reachability check, so control is being handed off to the full arm-planning stage."));
					yield break;
				}
			}

			result.failedAtStage = RobotPlanningStage.CurrentBaseReachabilityCheck;
			result.failureReason = lastFailureReason;
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

			EnterShadowBasePreview();
			if (!manager.BeginShadowBasePlannerSession("CoordinatedBaseExecution", obstacles, baseRadius, out string shadowSessionError))
			{
				result.failedAtStage = RobotPlanningStage.BaseExecution;
				result.failureReason = shadowSessionError;
				yield break;
			}

			DiffDriveTwinController shadowController = manager.ShadowBaseTwin != null ? manager.ShadowBaseTwin.ShadowController : null;
			if (shadowController == null || shadowController.rb == null)
			{
				result.failedAtStage = RobotPlanningStage.BaseExecution;
				result.failureReason = L("影子底盘执行副本缺失。", "Shadow base execution twin is missing.");
				yield break;
			}

			int replanCount = result.replanCount;
			List<Vector3> currentWaypoints = result.baseWaypoints != null ? new List<Vector3>(result.baseWaypoints) : new List<Vector3>();
			SafetyGateRuntimeContext gateContext = BuildSafetyGateRuntimeContext(request, obstacles, baseRadius);
			_safetyGateTimelineBuffer.Clear();
			float[] armPreviewStartAngles = manager.arm6DOFFKController != null
				? manager.arm6DOFFKController.CaptureMeasuredJointAngles()
				: null;
			yield return TraceStartupCommandBundleIfNeeded(
				result,
				request,
				gateContext,
				manager.diffDriveController.rb.position,
				finalBaseYaw,
				currentWaypoints,
				armPreviewStartAngles,
				result.armTrajectorySamples);
			if (IsStartupCommandTraceEnabled(request))
			{
				Debug.Log($"[SafetyGate] Base queue initialized: pending={_safetyGateTimelineBuffer.Count}");
			}

			bool gateBlockPending = false;
			int gateBlockDecelFramesRemaining = 0;
			string gateBlockReason = string.Empty;
			bool TryQueueLiveReplicaVelocity(float linearVelocity, float angularVelocity, Vector3 trackingPoint, out string queueError)
			{
				return manager.TryQueueLiveShadowBaseVelocityReplica(linearVelocity, angularVelocity, trackingPoint, out queueError);
			}

			void BeginBaseShadowControllerGoalHandoff(Vector3 finalGoal)
			{
				if (!manager.TryQueueLiveShadowBaseTargetPointReplica(finalGoal, out string queueError))
				{
					manager.FailShadowBaseSession(queueError);
					gateBlockPending = true;
					gateBlockReason = queueError;
				}
			}

			void StepBaseShadowControllerGoalHandoff()
			{
			}
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

				if (manager.TryGetShadowBaseSessionFault(out string shadowFault))
				{
					result.failedAtStage = RobotPlanningStage.BaseExecution;
					result.failureReason = shadowFault;
					yield break;
				}

				Vector3 currentPosition = shadowController.rb.position;
				if (!ValidateBasePathSegments(currentPosition, currentWaypoints, baseRadius, obstacles, out ShadowValidationResult segmentValidation))
				{
					lastShadowValidationResult = segmentValidation;
					if (request.allowReplan && replanCount < maxLocalReplans)
					{
						replanCount++;
						result.replanCount = replanCount;
						currentStage = RobotPlanningStage.LocalReplan;
						_distanceFieldSampler.Build(currentPosition, finalBaseGoal, obstacles, distanceFieldResolution, basePlanningMargin);
						if (_localReplanner.TryReplanBase(_basePlanner, currentPosition, shadowController.rb.rotation.eulerAngles.y, finalBaseGoal, finalBaseYaw, baseRadius, _distanceFieldSampler, _physicsQueries, obstacles, out List<Vector3> replannedWaypoints, out string replanFailure))
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
				bool TryInterceptBaseCommand(
					float linearVelocity,
					float angularVelocity,
					Vector3 trackingPoint,
					out float adjustedLinearVelocity,
					out float adjustedAngularVelocity,
					out string blockReason)
				{
					adjustedLinearVelocity = linearVelocity;
					adjustedAngularVelocity = angularVelocity;
					blockReason = string.Empty;
					if (manager.TryGetShadowBaseSessionFault(out blockReason))
					{
						return false;
					}

					if (!gateContext.enabled)
					{
						return TryQueueLiveReplicaVelocity(adjustedLinearVelocity, adjustedAngularVelocity, trackingPoint, out blockReason);
					}

					if (gateBlockPending)
					{
						if (gateBlockDecelFramesRemaining > 0)
						{
							gateBlockDecelFramesRemaining--;
							adjustedLinearVelocity = linearVelocity * 0.2f;
							adjustedAngularVelocity = angularVelocity * 0.2f;
							return TryQueueLiveReplicaVelocity(adjustedLinearVelocity, adjustedAngularVelocity, trackingPoint, out blockReason);
						}

						manager.AbortShadowBaseSession(resetShadowToLivePose: false);
						blockReason = gateBlockReason;
						return false;
					}

					if (gateContext.dynamicBaseLockWhenEeWithinTolerance
						&& request != null
						&& request.requireArmMove
						&& IsArmTargetWithinTolerance(request.armTargetWorldPosition, GetRequestedEeTolerance(request)))
					{
						if (!_loggedDynamicBaseLock)
						{
							_loggedDynamicBaseLock = true;
							Debug.Log("[SafetyGate] Base command suppressed because EE is already within tolerance.");
						}

						adjustedLinearVelocity = 0f;
						adjustedAngularVelocity = 0f;
						return TryQueueLiveReplicaVelocity(adjustedLinearVelocity, adjustedAngularVelocity, trackingPoint, out blockReason);
					}

					_loggedDynamicBaseLock = false;
					BaseGateCommand pendingCommand = new BaseGateCommand
					{
						fromWorldPosition = shadowController.rb.position,
						toWorldPosition = trackingPoint,
						targetYawDeg = finalBaseYaw
					};

					RefreshSafetyGateLoadFactor(gateContext);
					MirrorSnapshot mirror = _mirrorStateProvider.Capture(shadowController, manager.arm6DOFFKController);
					SafetyGateDecision decision = _shadowGate.EvaluateBaseCommand(mirror, pendingCommand, gateContext);
					ApplySafetyGateDecisionTelemetry(result, RobotPlanningStage.BaseExecution, decision);
					LogSafetyGateDecision(RobotPlanningStage.BaseExecution, decision);
					float leadSeconds = ComputeSafetyGateLeadSeconds(gateContext, mirror, armMode: false, result);
					long sequenceId = NextSafetyGateSequenceId();

					if (decision.type == SafetyGateDecisionType.Block)
					{
						RecordSafetyGateQueueFlush(result, sequenceId);
						gateBlockPending = true;
						gateBlockDecelFramesRemaining = Mathf.Max(1, gateContext.decelFramesBeforeStop) - 1;
						gateBlockReason = BuildSafetyGateFailureReason(RobotPlanningStage.BaseExecution, decision);
						adjustedLinearVelocity = linearVelocity * 0.2f;
						adjustedAngularVelocity = angularVelocity * 0.2f;
						return TryQueueLiveReplicaVelocity(adjustedLinearVelocity, adjustedAngularVelocity, trackingPoint, out blockReason);
					}

					float commandThrottleRatio = decision.type == SafetyGateDecisionType.Throttle
						? Mathf.Clamp(decision.throttleRatio, 0.2f, 1f)
						: 1f;
					SafetyGateTimelineCommand timelineCommand = new SafetyGateTimelineCommand
					{
						sequenceId = sequenceId,
						enqueueRealtime = Time.realtimeSinceStartup,
						kind = SafetyGateTimelineCommandKind.Base,
						baseCommand = pendingCommand,
						baseLinearVelocity = linearVelocity * commandThrottleRatio,
						baseAngularVelocity = angularVelocity * commandThrottleRatio,
						predictedBaseWorldPosition = decision.predictorState.predictedBaseWorldPosition,
						predictedBaseWorldRotation = decision.predictorState.predictedBaseWorldRotation,
						throttleRatio = commandThrottleRatio,
						predictedLeadSeconds = leadSeconds,
						readyRealtime = Time.realtimeSinceStartup + Mathf.Max(SafetyGateMinLookaheadFloorSeconds, leadSeconds),
						predictedRiskSummary = decision.reason
					};
					_safetyGateTimelineBuffer.Enqueue(timelineCommand);
					LogTimelineQueueEnqueueIfNeeded(request, armQueue: false, timelineCommand);

					bool hasHeadCommand = false;
					SafetyGateTimelineCommand headCommand = default;
					float requiredLeadSeconds = leadSeconds;
					if (_safetyGateTimelineBuffer.TryPeek(out headCommand))
					{
						hasHeadCommand = true;
						requiredLeadSeconds = Mathf.Max(SafetyGateMinLookaheadFloorSeconds, headCommand.predictedLeadSeconds);
					}

					if (_safetyGateTimelineBuffer.TryDequeueReady(Time.realtimeSinceStartup, requiredLeadSeconds, out SafetyGateTimelineCommand executableCommand))
					{
						if (executableCommand.kind != SafetyGateTimelineCommandKind.Base)
						{
							RecordSafetyGateQueueFlush(result, executableCommand.sequenceId);
							gateBlockPending = true;
							gateBlockDecelFramesRemaining = Mathf.Max(1, gateContext.decelFramesBeforeStop) - 1;
							gateBlockReason = "SafetyGate timeline kind mismatch while executing base command.";
							adjustedLinearVelocity = linearVelocity * 0.2f;
							adjustedAngularVelocity = angularVelocity * 0.2f;
							return TryQueueLiveReplicaVelocity(adjustedLinearVelocity, adjustedAngularVelocity, trackingPoint, out blockReason);
						}

						float executionRatio = Mathf.Clamp(executableCommand.throttleRatio, 0.2f, 1f);
						adjustedLinearVelocity = linearVelocity * executionRatio;
						adjustedAngularVelocity = angularVelocity * executionRatio;
						LogTimelineQueueDequeuedIfNeeded(request, armQueue: false, executableCommand);
						return TryQueueLiveReplicaVelocity(adjustedLinearVelocity, adjustedAngularVelocity, trackingPoint, out blockReason);
					}

					if (hasHeadCommand)
					{
						LogTimelineQueueWaitingIfNeeded(request, armQueue: false, headCommand, requiredLeadSeconds);
					}

					adjustedLinearVelocity = 0f;
					adjustedAngularVelocity = 0f;
					return TryQueueLiveReplicaVelocity(adjustedLinearVelocity, adjustedAngularVelocity, trackingPoint, out blockReason);
				}

				yield return _basePathFollower.FollowWaypoints(
					shadowController,
					currentWaypoints,
					finalBaseYaw,
					false,
					() => _stopRequested || Time.realtimeSinceStartup > deadline || manager.TryGetShadowBaseSessionFault(out _),
					(ok, text) =>
					{
						success = ok;
						message = text;
						done = true;
					},
					TryInterceptBaseCommand,
					BeginBaseShadowControllerGoalHandoff,
					StepBaseShadowControllerGoalHandoff);

				while (!done)
				{
					yield return null;
				}

				if (!success)
				{
					result.failedAtStage = RobotPlanningStage.BaseExecution;
					if (manager.TryGetShadowBaseSessionFault(out string shadowFaultAfterFollow))
					{
						result.failureReason = shadowFaultAfterFollow;
					}
					else
					{
						result.failureReason = gateBlockPending && !string.IsNullOrEmpty(gateBlockReason)
							? gateBlockReason
							: message;
					}
					yield break;
				}

				_safetyGateTimelineBuffer.Clear();
				yield break;
			}
		}

		private IEnumerator ExecuteArmTrajectory(RobotPlanResult result, RobotPlanRequest request, List<Collider> obstacles, float deadline, float executionTimeScale)
		{
			EnsurePlannerInternals();
			EnterShadowArmPreviewFromLiveBase();
			int replanCount = result.replanCount;
			List<RobotPlanJointSample> currentSamples = result.armTrajectorySamples != null ? new List<RobotPlanJointSample>(result.armTrajectorySamples) : new List<RobotPlanJointSample>();
			float[] previousAngles = manager.arm6DOFFKController.CaptureMeasuredJointAngles();
			float previousSampleTimeSeconds = 0f;
			float nextCommandTime = Time.realtimeSinceStartup;
			float baseRadius = _physicsQueries.EstimateBaseRadius(manager.diffDriveController);
			SafetyGateRuntimeContext gateContext = BuildSafetyGateRuntimeContext(request, obstacles, baseRadius);
			_safetyGateTimelineBuffer.Clear();
			yield return TraceStartupCommandBundleIfNeeded(
				result,
				request,
				gateContext,
				manager != null && manager.diffDriveController != null && manager.diffDriveController.rb != null
					? manager.diffDriveController.rb.position
					: Vector3.zero,
				manager != null && manager.diffDriveController != null && manager.diffDriveController.rb != null
					? manager.diffDriveController.rb.rotation.eulerAngles.y
					: 0f,
				result.baseWaypoints,
				previousAngles,
				currentSamples);
			if (IsStartupCommandTraceEnabled(request))
			{
				Debug.Log($"[SafetyGate] Arm queue initialized: pending={_safetyGateTimelineBuffer.Count}");
			}

			int recoveryAttemptCount = result.safetyGateRecoveryAttemptCount;
			int maxRecoveryAttempts = Mathf.Max(0, gateContext.maxRecoveryReplans);

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
							previousSampleTimeSeconds = 0f;
							nextCommandTime = Time.realtimeSinceStartup;
							_safetyGateTimelineBuffer.Clear();
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

				ArmGateCommand pendingCommand = new ArmGateCommand
				{
					fromAnglesDeg = previousAngles != null ? (float[])previousAngles.Clone() : null,
					toAnglesDeg = sample.jointAnglesDeg != null ? (float[])sample.jointAnglesDeg.Clone() : null
				};
				float commandThrottleRatio = 1f;
				if (gateContext.enabled)
				{
					RefreshSafetyGateLoadFactor(gateContext);
					MirrorSnapshot mirror = _mirrorStateProvider.Capture(manager.diffDriveController, manager.arm6DOFFKController);
					SafetyGateDecision decision = _shadowGate.EvaluateArmCommand(mirror, pendingCommand, gateContext, manager.arm6DOFFKController);
					ApplySafetyGateDecisionTelemetry(result, RobotPlanningStage.ArmExecution, decision);
					LogSafetyGateDecision(RobotPlanningStage.ArmExecution, decision);
					PushShadowPredictionPose(decision);
					float leadSeconds = ComputeSafetyGateLeadSeconds(gateContext, mirror, armMode: true, result);
					long sequenceId = NextSafetyGateSequenceId();

					if (decision.type == SafetyGateDecisionType.Block)
					{
						RecordSafetyGateQueueFlush(result, sequenceId);
						manager.arm6DOFFKController.HoldCurrentPose();
						yield return new WaitForFixedUpdate();
						float eeTolerance = GetEffectiveEeTolerance(result, request);
						float eeError = Vector3.Distance(manager.arm6DOFFKController.EndEffectorWorldPosition, request.armTargetWorldPosition);
						string gateFailureReason = BuildSafetyGateFailureReason(RobotPlanningStage.ArmExecution, decision);
						if (recoveryAttemptCount < maxRecoveryAttempts && eeError > eeTolerance)
						{
							recoveryAttemptCount++;
							result.safetyGateRecoveryAttemptCount = recoveryAttemptCount;
							bool recovered = false;
							string recoveryFailureReason = string.Empty;
							ArmStagePreparationState recoveryState = new ArmStagePreparationState
							{
								ResolvedBaseGoal = manager.diffDriveController.rb.position,
								ResolvedBaseYaw = manager.diffDriveController.rb.rotation.eulerAngles.y,
								PreferredArmSolveSeed = null
							};
							List<string> recoverySummaryParts = new List<string>();
							float remainingBudget = Mathf.Max(
								SafetyGateRecoveryBudgetFloorSeconds,
								Mathf.Min(
									SafetyGateRecoveryBudgetCeilingSeconds,
									Mathf.Max(0f, deadline - Time.realtimeSinceStartup)));
							float recoveryDeadline = Time.realtimeSinceStartup + remainingBudget;
							result.failureReason = string.Empty;
							result.failedAtStage = RobotPlanningStage.None;
							yield return RetryDockingSearchAndMoveBaseIfNeeded(
								request,
								result,
								recoverySummaryParts,
								obstacles,
								baseRadius,
								recoveryDeadline,
								recoveryState);
							recovered = string.IsNullOrEmpty(result.failureReason);
							recoveryFailureReason = result.failureReason;
							if (recovered)
							{
								_distanceFieldSampler.Build(
									manager.arm6DOFFKController.EndEffectorWorldPosition,
									request.armTargetWorldPosition,
									obstacles,
									Mathf.Max(0.2f, distanceFieldResolution),
									2.5f);
								if (_localReplanner.TryReplanArm(
									_armMotionPlanner,
									manager.arm6DOFFKController,
									request.armTargetWorldPosition,
									_distanceFieldSampler,
									out List<RobotPlanJointSample> replannedAfterRecovery,
									out _))
								{
									currentSamples = replannedAfterRecovery;
									result.armTrajectorySamples = replannedAfterRecovery;
								}

								_safetyGateTimelineBuffer.Clear();
								previousAngles = manager.arm6DOFFKController.CaptureMeasuredJointAngles();
								previousSampleTimeSeconds = 0f;
								nextCommandTime = Time.realtimeSinceStartup;
								sampleIndex = -1;
								currentStage = RobotPlanningStage.ArmExecution;
								continue;
							}

							result.failedAtStage = RobotPlanningStage.ArmExecution;
							result.failureReason = string.IsNullOrWhiteSpace(recoveryFailureReason)
								? gateFailureReason
								: $"{gateFailureReason} {recoveryFailureReason}";
							yield break;
						}

						result.failedAtStage = RobotPlanningStage.ArmExecution;
						result.failureReason = gateFailureReason;
						yield break;
					}

					commandThrottleRatio = decision.type == SafetyGateDecisionType.Throttle
						? Mathf.Clamp(decision.throttleRatio, 0.2f, 1f)
						: 1f;
					bool armExecutionPipelineWarmed = previousSampleTimeSeconds > 0f;
					float commandReadyRealtime = armExecutionPipelineWarmed
						? nextCommandTime
						: Time.realtimeSinceStartup + Mathf.Max(SafetyGateMinLookaheadFloorSeconds, leadSeconds);
					SafetyGateTimelineCommand timelineCommand = new SafetyGateTimelineCommand
					{
						sequenceId = sequenceId,
						enqueueRealtime = Time.realtimeSinceStartup,
						kind = SafetyGateTimelineCommandKind.Arm,
						armCommand = pendingCommand,
						throttleRatio = commandThrottleRatio,
						predictedLeadSeconds = leadSeconds,
						readyRealtime = commandReadyRealtime,
						predictedRiskSummary = decision.reason
					};
					_safetyGateTimelineBuffer.Enqueue(timelineCommand);
					LogTimelineQueueEnqueueIfNeeded(request, armQueue: true, timelineCommand);
					manager.shadowRobotVisualizer?.ApplyShadowStep(timelineCommand);
				}

				bool commandExecuted = false;
				while (!commandExecuted)
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

					if (HasObservedArmCollision())
					{
						result.failedAtStage = RobotPlanningStage.ArmExecution;
						result.failureReason = string.IsNullOrEmpty(manager.armCollisionMonitor.ActiveCollisionMessage)
							? L("机械臂在轨迹执行过程中发生碰撞。", "Arm collided during trajectory execution.")
							: manager.armCollisionMonitor.ActiveCollisionMessage;
						yield break;
					}

					float requiredLeadSeconds = 0f;
					bool hasHeadCommand = false;
					SafetyGateTimelineCommand headCommand = default;
					if (gateContext.enabled && _safetyGateTimelineBuffer.TryPeek(out headCommand))
					{
						hasHeadCommand = true;
						requiredLeadSeconds = Mathf.Max(SafetyGateMinLookaheadFloorSeconds, headCommand.predictedLeadSeconds);
					}

					if (!_safetyGateTimelineBuffer.TryDequeueReady(Time.realtimeSinceStartup, requiredLeadSeconds, out SafetyGateTimelineCommand executableCommand))
					{
						if (hasHeadCommand)
						{
							LogTimelineQueueWaitingIfNeeded(request, armQueue: true, headCommand, requiredLeadSeconds);
						}

						yield return new WaitForFixedUpdate();
						continue;
					}

					if (executableCommand.kind != SafetyGateTimelineCommandKind.Arm)
					{
						RecordSafetyGateQueueFlush(result, executableCommand.sequenceId);
						result.failedAtStage = RobotPlanningStage.ArmExecution;
						result.failureReason = "SafetyGate timeline kind mismatch while executing arm command.";
						yield break;
					}

					float segmentThrottleTimeScale = 1f / Mathf.Clamp(executableCommand.throttleRatio, 0.2f, 1f);
					LogTimelineQueueDequeuedIfNeeded(request, armQueue: true, executableCommand);
					manager.arm6DOFFKController.ApplyAllJointTargetsRaw(executableCommand.armCommand.toAnglesDeg);
					float plannedSegmentSeconds = Mathf.Max(ArmExecutionMinSegmentSeconds, sample.timeSeconds - previousSampleTimeSeconds);
					float targetCommandTime = nextCommandTime + (plannedSegmentSeconds * Mathf.Max(1f, executionTimeScale) * Mathf.Max(1f, segmentThrottleTimeScale));
					previousSampleTimeSeconds = sample.timeSeconds;
					nextCommandTime = targetCommandTime;

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

						if (HasObservedArmCollision())
						{
							result.failedAtStage = RobotPlanningStage.ArmExecution;
							result.failureReason = string.IsNullOrEmpty(manager.armCollisionMonitor.ActiveCollisionMessage)
								? L("机械臂在轨迹执行过程中发生碰撞。", "Arm collided during trajectory execution.")
								: manager.armCollisionMonitor.ActiveCollisionMessage;
							yield break;
						}

						yield return new WaitForFixedUpdate();
					}

					previousAngles = executableCommand.armCommand.toAnglesDeg != null
						? (float[])executableCommand.armCommand.toAnglesDeg.Clone()
						: previousAngles;
					commandExecuted = true;
				}
			}

			if (currentSamples.Count > 0)
			{
				yield return WaitForArmTrajectorySettled(
					result,
					request,
					currentSamples[currentSamples.Count - 1].jointAnglesDeg,
					deadline);
				if (!string.IsNullOrEmpty(result.failureReason))
				{
					yield break;
				}
			}

			_safetyGateTimelineBuffer.Clear();
			manager.arm6DOFFKController.HoldCurrentPose();
		}

		private IEnumerator WaitForArmTrajectorySettled(RobotPlanResult result, RobotPlanRequest request, float[] finalTargetAnglesDeg, float deadline)
		{
			const float finalJointToleranceDeg = 3.0f;
			const int requiredStableFrames = 3;
			int stableFrames = 0;
			float finalPositionToleranceMeters = GetEffectiveEeTolerance(result, request);
			bool correctionAttempted = false;
			float correctionDeadline = deadline + ArmExecutionCorrectionGraceSeconds;

			while (Time.realtimeSinceStartup <= correctionDeadline)
			{
				if (_stopRequested)
				{
					result.failedAtStage = RobotPlanningStage.ArmExecution;
					result.failureReason = L("规划已停止。", "Planning was stopped.");
					yield break;
				}

				if (HasObservedArmCollision())
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

				if (Time.realtimeSinceStartup > deadline && !correctionAttempted)
				{
					float correctionResidualDeg = manager.arm6DOFFKController.GetMaxJointAngleError(finalTargetAnglesDeg);
					float correctionResidualMeters = Vector3.Distance(manager.arm6DOFFKController.EndEffectorWorldPosition, request.armTargetWorldPosition);
					if (correctionResidualMeters <= ArmExecutionCorrectionMaxResidualMeters
						&& correctionResidualDeg <= ArmExecutionCorrectionMaxJointErrorDeg)
					{
						correctionAttempted = true;
						finalPositionToleranceMeters = Mathf.Max(finalPositionToleranceMeters, ArmExecutionCorrectionToleranceMeters);
						manager.arm6DOFFKController.ApplyAllJointTargetsRaw(finalTargetAnglesDeg);
						LogArmSettleDiagnostics(request, finalTargetAnglesDeg);
						Debug.LogWarning(
							$"[RobotTrajectoryPlanner] Arm settle timeout reached, attempting terminal correction. residualDeg={correctionResidualDeg:F2}, residualMeters={correctionResidualMeters:F3}, tolerance={finalPositionToleranceMeters:F3}, extraWindow={ArmExecutionCorrectionGraceSeconds:F1}s");
						yield return new WaitForFixedUpdate();
						continue;
					}

					break;
				}

				yield return new WaitForFixedUpdate();
			}

			float residualDeg = manager.arm6DOFFKController.GetMaxJointAngleError(finalTargetAnglesDeg);
			float residualMeters = Vector3.Distance(manager.arm6DOFFKController.EndEffectorWorldPosition, request.armTargetWorldPosition);
			LogArmSettleDiagnostics(request, finalTargetAnglesDeg);
			Debug.LogWarning(
				$"[RobotTrajectoryPlanner] Arm settle failed after correction window. residualDeg={residualDeg:F2}, residualMeters={residualMeters:F3}, tolerance={finalPositionToleranceMeters:F3}");
			result.failedAtStage = RobotPlanningStage.ArmExecution;
			result.failureReason = L(
				$"机械臂轨迹未能平滑稳定到位，最终关节误差={residualDeg:F2}°，末端误差={residualMeters:F3}m，收敛容差={finalPositionToleranceMeters:F3}m。",
				$"Arm trajectory did not settle smoothly enough. Final joint error={residualDeg:F2}deg, EE error={residualMeters:F3}m, settle tolerance={finalPositionToleranceMeters:F3}m.");
		}

		private SafetyGateRuntimeContext BuildSafetyGateRuntimeContext(
			RobotPlanRequest request,
			IReadOnlyList<Collider> obstacles,
			float baseRadius)
		{
			SafetyGateRequestSettings settings = request != null ? request.safetyGate : null;
			SafetyGateRuntimeContext context = new SafetyGateRuntimeContext
			{
				enabled = settings == null || settings.enabled,
				throttleOnRisk = settings == null || settings.throttleOnRisk,
				commLatencySeconds = settings != null ? Mathf.Max(0f, settings.commLatencySeconds) : 0.04f,
				computeBudgetSeconds = settings != null ? Mathf.Max(0f, settings.computeBudgetSeconds) : 0.02f,
				minLookaheadSeconds = settings != null ? Mathf.Max(SafetyGateMinLookaheadFloorSeconds, settings.minLookaheadSeconds) : 0.08f,
				baseInflationMeters = settings != null ? Mathf.Max(0f, settings.baseInflationMeters) : 0.02f,
				armInflationMeters = settings != null ? Mathf.Max(0f, settings.armInflationMeters) : 0.015f,
				queueLeadTimeSeconds = settings != null ? Mathf.Max(0f, settings.queueLeadTimeSeconds) : 0f,
				maxRecoveryReplans = settings != null ? Mathf.Max(0, settings.maxRecoveryReplans) : 3,
				dynamicBaseLockWhenEeWithinTolerance = settings == null || settings.dynamicBaseLockWhenEeWithinTolerance,
				decelFramesBeforeStop = settings != null ? Mathf.Max(1, settings.decelFramesBeforeStop) : SafetyGateBlockDecelFrames,
				baseRadiusMeters = Mathf.Max(0.05f, baseRadius),
				obstacles = obstacles,
				obstacleLookup = _physicsQueries.GetObstacleLookup(obstacles)
			};

			RefreshSafetyGateLoadFactor(context);
			return context;
		}

		private bool HasObservedArmCollision()
		{
			return manager != null
				&& manager.armCollisionMonitor != null
				&& manager.armCollisionMonitor.GetObservedCollisionState();
		}

		private void ResetSafetyGateTimelineState()
		{
			_safetyGateTimelineBuffer.Clear();
			_safetyGateSequenceCounter = 0;
			_loggedDynamicBaseLock = false;
			_startupCommandTraceLogged = false;
			_startupTraceSeedInitialized = false;
			_startupTraceSequenceSeed = 0;
			_startupTraceBaseStartWorldPosition = Vector3.zero;
			_startupTraceBaseTargetYawDeg = 0f;
			_startupTraceBaseWaypoints.Clear();
			_lastBaseQueueWaitSequenceId = 0;
			_lastArmQueueWaitSequenceId = 0;
		}

		private void StartOfflineComputeSession()
		{
			CancelOfflineComputeSession();
			_offlineComputeCts = new CancellationTokenSource();
		}

		private void CancelOfflineComputeSession()
		{
			if (_offlineComputeCts == null)
			{
				return;
			}

			try
			{
				_offlineComputeCts.Cancel();
			}
			catch
			{
				// ignored
			}
			finally
			{
				_offlineComputeCts.Dispose();
				_offlineComputeCts = null;
			}
		}

		private long NextSafetyGateSequenceId()
		{
			_safetyGateSequenceCounter++;
			return _safetyGateSequenceCounter;
		}

		private float ComputeSafetyGateLeadSeconds(
			SafetyGateRuntimeContext context,
			MirrorSnapshot snapshot,
			bool armMode,
			RobotPlanResult result = null)
		{
			if (context == null)
			{
				return 0.08f;
			}

			float baseWindow = Mathf.Max(
				Mathf.Max(context.minLookaheadSeconds, SafetyGateMinLookaheadFloorSeconds),
				context.commLatencySeconds + context.computeBudgetSeconds);
			float brakeWindow = 0f;
			if (!armMode)
			{
				brakeWindow = snapshot.basePlanarSpeed / Mathf.Max(0.05f, context.baseBrakeDecelMetersPerSecond2);
			}
			else if (snapshot.armJointVelocitiesDegPerSecond != null)
			{
				float maxJointSpeed = 0f;
				for (int i = 0; i < snapshot.armJointVelocitiesDegPerSecond.Length; i++)
				{
					maxJointSpeed = Mathf.Max(maxJointSpeed, Mathf.Abs(snapshot.armJointVelocitiesDegPerSecond[i]));
				}

				brakeWindow = maxJointSpeed / Mathf.Max(1f, context.armBrakeDecelDegPerSecond2);
			}

			float leadSeconds = Mathf.Max(Mathf.Max(baseWindow, brakeWindow), context.queueLeadTimeSeconds);
			if (result != null)
			{
				result.safetyGateLastDeltaTSeconds = leadSeconds;
			}

			return leadSeconds;
		}

		private void RecordSafetyGateQueueFlush(
			RobotPlanResult result,
			long blockedSequenceId)
		{
			int flushedCount = _safetyGateTimelineBuffer.Flush();
			if (flushedCount > 0)
			{
				Debug.LogWarning($"[SafetyGate] Timeline queue flushed: count={flushedCount}, blockedSeq={blockedSequenceId}");
			}

			if (result == null)
			{
				return;
			}

			result.safetyGateQueueFlushCount++;
			result.safetyGateLastBlockSequenceId = blockedSequenceId;
		}

		private void RefreshSafetyGateLoadFactor(SafetyGateRuntimeContext context)
		{
			if (context == null)
			{
				return;
			}

			context.loadFactor = manager != null && manager.onlineCalibration != null
				? Mathf.Clamp01(manager.onlineCalibration.ArmZeroOffsetBlend)
				: 0f;
		}

		private static void ApplySafetyGateDecisionTelemetry(
			RobotPlanResult result,
			RobotPlanningStage stage,
			SafetyGateDecision decision)
		{
			if (result == null || decision == null)
			{
				return;
			}

			if (decision.type != SafetyGateDecisionType.Allow)
			{
				result.safetyGateInterceptCount++;
			}

			if (!float.IsNaN(decision.leadTimeMs) && !float.IsInfinity(decision.leadTimeMs))
			{
				result.safetyGateMinLeadTimeMs = float.IsInfinity(result.safetyGateMinLeadTimeMs)
					? decision.leadTimeMs
					: Mathf.Min(result.safetyGateMinLeadTimeMs, decision.leadTimeMs);
			}

			if (!float.IsNaN(decision.minPredictedClearanceMeters) && !float.IsInfinity(decision.minPredictedClearanceMeters))
			{
				result.safetyGateMinPredictedClearanceMeters = float.IsInfinity(result.safetyGateMinPredictedClearanceMeters)
					? decision.minPredictedClearanceMeters
					: Mathf.Min(result.safetyGateMinPredictedClearanceMeters, decision.minPredictedClearanceMeters);
			}

			result.safetyGateLastDecision = decision.type.ToString();
			string stageName = RobotSimulationLocalization.PlanningStage(stage);
			string reason = string.IsNullOrWhiteSpace(decision.reason)
				? decision.blockCategory.ToString()
				: decision.reason;
			result.safetyGateLastReason = $"{stageName}: {reason}";
		}

		private static void LogSafetyGateDecision(RobotPlanningStage stage, SafetyGateDecision decision)
		{
			if (decision == null)
			{
				return;
			}

			string stageName = RobotSimulationLocalization.PlanningStage(stage);
			string reason = string.IsNullOrWhiteSpace(decision.reason) ? "none" : decision.reason;
			string message =
				$"[SafetyGate] stage={stageName}, decision={decision.type}, leadTimeMs={decision.leadTimeMs:F1}, dMin={decision.minPredictedClearanceMeters:F3}, throttleRatio={decision.throttleRatio:F2}, blockCategory={decision.blockCategory}, reason={reason}";
			if (decision.type == SafetyGateDecisionType.Allow)
			{
				Debug.Log(message);
			}
			else
			{
				Debug.LogWarning(message);
			}
		}

		private static string BuildSafetyGateFailureReason(RobotPlanningStage stage, SafetyGateDecision decision)
		{
			string stageName = RobotSimulationLocalization.PlanningStage(stage);
			string category = decision != null ? decision.blockCategory.ToString() : SafetyGateBlockCategory.None.ToString();
			string reason = decision != null && !string.IsNullOrWhiteSpace(decision.reason)
				? decision.reason
				: "Safety gate blocked execution.";
			return $"SafetyGate[{stageName}/{category}] {reason}";
		}

		private void PushShadowPredictionPose(SafetyGateDecision decision)
		{
			if (manager == null || manager.shadowRobotVisualizer == null || decision == null)
			{
				return;
			}

			Vector3 predictedPosition = decision.predictorState.predictedBaseWorldPosition;
			Quaternion predictedRotation = decision.predictorState.predictedBaseWorldRotation;
			float holdSeconds = Mathf.Max(0.08f, decision.predictorState.predictedLeadTimeSeconds);
			if (predictedRotation == default)
			{
				predictedRotation = manager.diffDriveController != null && manager.diffDriveController.rb != null
					? manager.diffDriveController.rb.rotation
					: Quaternion.identity;
			}

			manager.shadowRobotVisualizer.PushPredictedBasePose(predictedPosition, predictedRotation, holdSeconds);
		}

		private static bool IsStartupCommandTraceEnabled(RobotPlanRequest request)
		{
			return request == null || request.safetyGate == null || request.safetyGate.enableStartupCommandTrace;
		}

		private static int ResolveStartupPreviewMaxItems(RobotPlanRequest request)
		{
			if (request == null || request.safetyGate == null)
			{
				return 8;
			}

			return Mathf.Clamp(request.safetyGate.startupPreviewMaxItems, 1, 32);
		}

		private IEnumerator TraceStartupCommandBundleIfNeeded(
			RobotPlanResult result,
			RobotPlanRequest request,
			SafetyGateRuntimeContext context,
			Vector3 baseStartWorldPosition,
			float baseTargetYawDeg,
			IReadOnlyList<Vector3> baseWaypoints,
			float[] armStartAnglesDeg,
			IReadOnlyList<RobotPlanJointSample> armSamples)
		{
			if (_startupCommandTraceLogged || result == null || !IsStartupCommandTraceEnabled(request))
			{
				yield break;
			}

			if (!_startupTraceSeedInitialized)
			{
				_startupTraceSeedInitialized = true;
				_startupTraceSequenceSeed = _safetyGateSequenceCounter + 1;
				_startupTraceBaseStartWorldPosition = baseStartWorldPosition;
				_startupTraceBaseTargetYawDeg = baseTargetYawDeg;
				_startupTraceBaseWaypoints.Clear();
				_startupTraceBaseWaypoints.AddRange(CloneWaypoints(baseWaypoints));
			}
			else if (_startupTraceBaseWaypoints.Count == 0 && baseWaypoints != null && baseWaypoints.Count > 0)
			{
				_startupTraceBaseWaypoints.AddRange(CloneWaypoints(baseWaypoints));
			}

			List<Vector3> effectiveBaseWaypoints = _startupTraceBaseWaypoints.Count > 0
				? new List<Vector3>(_startupTraceBaseWaypoints)
				: CloneWaypoints(baseWaypoints);
			Vector3 effectiveBaseStart = _startupTraceSeedInitialized
				? _startupTraceBaseStartWorldPosition
				: baseStartWorldPosition;
			float effectiveBaseYawDeg = _startupTraceSeedInitialized
				? _startupTraceBaseTargetYawDeg
				: baseTargetYawDeg;

			MirrorSnapshot mirror = _mirrorStateProvider.Capture(manager != null ? manager.diffDriveController : null, manager != null ? manager.arm6DOFFKController : null);
			float baseLeadSeconds = effectiveBaseWaypoints.Count > 0
				? ComputeSafetyGateLeadSeconds(context, mirror, armMode: false, result)
				: 0f;
			float armLeadSeconds = armSamples != null && armSamples.Count > 0
				? ComputeSafetyGateLeadSeconds(context, mirror, armMode: true, result)
				: 0f;
			int maxPreviewItems = ResolveStartupPreviewMaxItems(request);
			float baseNominalSpeed = manager != null && manager.diffDriveController != null
				? Mathf.Max(0.1f, manager.diffDriveController.vMax)
				: 0.6f;
			List<Vector3> baseWaypointsCopy = CloneWaypoints(effectiveBaseWaypoints);
			List<RobotPlanJointSample> armSamplesCopy = CloneJointSamplesForPreview(armSamples);
			float[] armStartCopy = armStartAnglesDeg != null ? (float[])armStartAnglesDeg.Clone() : new float[6];

			StartupPreviewBuildResult previewResult = default;
			yield return RunOfflineComputation(
				token => BuildStartupPreviewItems(
					effectiveBaseStart,
					effectiveBaseYawDeg,
					baseWaypointsCopy,
					baseNominalSpeed,
					baseLeadSeconds,
					armStartCopy,
					armSamplesCopy,
					armLeadSeconds,
					Math.Max(1L, _startupTraceSequenceSeed),
					maxPreviewItems,
					token),
				value => previewResult = value,
				"startup-command-preview");

			if (previewResult.previewItems == null)
			{
				yield break;
			}

			result.startupCommandPreview = previewResult.previewItems;
			result.startupCommandCount = previewResult.totalCount;
			Debug.Log($"[SafetyGate] StartupCommandBundle: total={previewResult.totalCount}, preview={previewResult.previewItems.Count}, hasBase={previewResult.hasBaseCommands}, hasArm={previewResult.hasArmCommands}");
			for (int i = 0; i < previewResult.previewItems.Count; i++)
			{
				SafetyGateStartupPreviewItem item = previewResult.previewItems[i];
				Debug.Log($"[SafetyGate] StartupPreview[{i}] seq={item.sequenceId}, kind={item.kind}, t={item.plannedTimeSeconds:F3}s, lead={item.leadTimeSeconds:F3}s, throttle={item.throttleRatio:F2}, from={item.fromSummary}, to={item.toSummary}");
			}

			if (!previewResult.hasBaseCommands)
			{
				Debug.Log("[SafetyGate] Startup command preview contains no base commands (base movement may be unnecessary for this request).");
			}

			if (request != null && request.requireArmMove && !previewResult.hasArmCommands)
			{
				bool armSamplesPending = armSamples == null || armSamples.Count == 0;
				if (armSamplesPending)
				{
					Debug.Log("[SafetyGate] Startup command preview currently contains no arm commands because arm trajectory samples are not available yet. The preview will retry after arm planning finishes.");
				}
				else
				{
					Debug.LogWarning("[SafetyGate] Startup command preview still contains no arm commands even though arm trajectory samples are already available. Please inspect arm command generation.");
				}

				yield break;
			}

			if (!previewResult.hasArmCommands && (request == null || !request.requireArmMove))
			{
				Debug.Log("[SafetyGate] Startup command preview contains no arm commands (arm movement is not required for this request).");
			}

			_startupCommandTraceLogged = true;
		}

		private static float ResolveTimelineCommandReadyRealtime(SafetyGateTimelineCommand command, float requiredLeadSeconds)
		{
			float required = Mathf.Max(Mathf.Max(0f, requiredLeadSeconds), command.predictedLeadSeconds);
			float inferredReady = command.enqueueRealtime + required;
			return command.readyRealtime > 0f ? Mathf.Max(command.readyRealtime, inferredReady) : inferredReady;
		}

		private static string FormatBaseCommandSummary(BaseGateCommand command)
		{
			return $"to=({command.toWorldPosition.x:F3},{command.toWorldPosition.y:F3},{command.toWorldPosition.z:F3}), yaw={command.targetYawDeg:F1}";
		}

		private static string FormatArmCommandSummary(ArmGateCommand command)
		{
			return command.toAnglesDeg == null || command.toAnglesDeg.Length < 6
				? "to=j[n/a]"
				: $"to=j[{command.toAnglesDeg[0]:F1},{command.toAnglesDeg[1]:F1},{command.toAnglesDeg[2]:F1},{command.toAnglesDeg[3]:F1},{command.toAnglesDeg[4]:F1},{command.toAnglesDeg[5]:F1}]";
		}

		private void LogTimelineQueueEnqueueIfNeeded(RobotPlanRequest request, bool armQueue, SafetyGateTimelineCommand command)
		{
			if (!IsStartupCommandTraceEnabled(request))
			{
				return;
			}

			float readyAt = ResolveTimelineCommandReadyRealtime(command, command.predictedLeadSeconds);
			string queueName = armQueue ? "Arm" : "Base";
			string targetSummary = armQueue ? FormatArmCommandSummary(command.armCommand) : FormatBaseCommandSummary(command.baseCommand);
			Debug.Log($"[SafetyGate] {queueName} queue enqueue: pending={_safetyGateTimelineBuffer.Count}, seq={command.sequenceId}, readyIn={Mathf.Max(0f, readyAt - Time.realtimeSinceStartup):F3}s, {targetSummary}");
		}

		private void LogTimelineQueueWaitingIfNeeded(
			RobotPlanRequest request,
			bool armQueue,
			SafetyGateTimelineCommand headCommand,
			float requiredLeadSeconds)
		{
			if (!IsStartupCommandTraceEnabled(request))
			{
				return;
			}

			long lastWaitSequenceId = armQueue ? _lastArmQueueWaitSequenceId : _lastBaseQueueWaitSequenceId;
			if (lastWaitSequenceId == headCommand.sequenceId)
			{
				return;
			}

			float now = Time.realtimeSinceStartup;
			float readyAt = ResolveTimelineCommandReadyRealtime(headCommand, requiredLeadSeconds);
			string queueName = armQueue ? "Arm" : "Base";
			Debug.Log($"[SafetyGate] {queueName} queue waiting: pending={_safetyGateTimelineBuffer.Count}, headSeq={headCommand.sequenceId}, firstReadyIn={Mathf.Max(0f, readyAt - now):F3}s");
			if (armQueue)
			{
				_lastArmQueueWaitSequenceId = headCommand.sequenceId;
			}
			else
			{
				_lastBaseQueueWaitSequenceId = headCommand.sequenceId;
			}
		}

		private void LogTimelineQueueDequeuedIfNeeded(
			RobotPlanRequest request,
			bool armQueue,
			SafetyGateTimelineCommand command)
		{
			if (!IsStartupCommandTraceEnabled(request))
			{
				return;
			}

			string queueName = armQueue ? "Arm" : "Base";
			string targetSummary = armQueue ? FormatArmCommandSummary(command.armCommand) : FormatBaseCommandSummary(command.baseCommand);
			Debug.Log($"[SafetyGate] {queueName} queue dequeue: pending={_safetyGateTimelineBuffer.Count}, seq={command.sequenceId}, throttle={command.throttleRatio:F2}, {targetSummary}");
			if (armQueue)
			{
				_lastArmQueueWaitSequenceId = 0;
			}
			else
			{
				_lastBaseQueueWaitSequenceId = 0;
			}
		}

		private IEnumerator ComputeArmExecutionTimingEstimateAsync(
			Arm6DOFFKController armController,
			List<RobotPlanJointSample> armSamples,
			Action<ArmExecutionTimingEstimate> onComplete)
		{
			if (armController == null || armSamples == null || armSamples.Count == 0)
			{
				onComplete?.Invoke(new ArmExecutionTimingEstimate
				{
					executionTimeScale = 1f,
					scaledExecutionSeconds = 0f
				});
				yield break;
			}

			ArmTimingSnapshot snapshot = CaptureArmTimingSnapshot(armController, armSamples);
			ArmExecutionTimingEstimate estimate = default;
			yield return RunOfflineComputation(
				token => ComputeArmExecutionTimingEstimate(snapshot, token),
				value => estimate = value,
				"arm-execution-timing");

			if (estimate.executionTimeScale <= 0f)
			{
				estimate.executionTimeScale = 1f;
			}

			if (estimate.scaledExecutionSeconds <= 0f)
			{
				estimate.scaledExecutionSeconds = ComputeArmScaledExecutionSeconds(armSamples, estimate.executionTimeScale);
			}

			onComplete?.Invoke(estimate);
		}

		private IEnumerator RunOfflineComputation<T>(
			Func<CancellationToken, T> worker,
			Action<T> onSuccess,
			string label)
		{
			CancellationToken token = _offlineComputeCts != null ? _offlineComputeCts.Token : CancellationToken.None;
			Task<T> task;
			try
			{
				task = Task.Run(() =>
				{
					token.ThrowIfCancellationRequested();
					_offlineComputeSemaphore.Wait(token);
					try
					{
						token.ThrowIfCancellationRequested();
						return worker(token);
					}
					finally
					{
						_offlineComputeSemaphore.Release();
					}
				}, token);
			}
			catch (Exception ex)
			{
				Debug.LogWarning($"[RobotTrajectoryPlanner] Failed to start offline computation '{label}': {ex.Message}");
				yield break;
			}

			while (!task.IsCompleted)
			{
				yield return null;
			}

			if (task.IsCanceled || token.IsCancellationRequested)
			{
				yield break;
			}

			if (task.IsFaulted)
			{
				Debug.LogWarning($"[RobotTrajectoryPlanner] Offline computation '{label}' failed: {task.Exception?.GetBaseException().Message}");
				yield break;
			}

			onSuccess?.Invoke(task.Result);
		}

		private static List<Vector3> CloneWaypoints(IReadOnlyList<Vector3> waypoints)
		{
			List<Vector3> copy = new List<Vector3>();
			if (waypoints == null)
			{
				return copy;
			}

			for (int i = 0; i < waypoints.Count; i++)
			{
				copy.Add(waypoints[i]);
			}

			return copy;
		}

		private static List<RobotPlanJointSample> CloneJointSamplesForPreview(IReadOnlyList<RobotPlanJointSample> samples)
		{
			List<RobotPlanJointSample> copy = new List<RobotPlanJointSample>();
			if (samples == null)
			{
				return copy;
			}

			for (int i = 0; i < samples.Count; i++)
			{
				RobotPlanJointSample sample = samples[i];
				if (sample == null || sample.jointAnglesDeg == null || sample.jointAnglesDeg.Length < 6)
				{
					continue;
				}

				copy.Add(new RobotPlanJointSample
				{
					timeSeconds = sample.timeSeconds,
					jointAnglesDeg = (float[])sample.jointAnglesDeg.Clone(),
					clearance = sample.clearance,
					singularityPenalty = sample.singularityPenalty
				});
			}

			return copy;
		}

		private static ArmTimingSnapshot CaptureArmTimingSnapshot(
			Arm6DOFFKController armController,
			IReadOnlyList<RobotPlanJointSample> armSamples)
		{
			ArmTimingSnapshot snapshot = new ArmTimingSnapshot
			{
				startAnglesDeg = armController != null ? armController.CaptureMeasuredJointAngles() : new float[6],
				maxJointSpeedDegPerSecond = new float[6],
				samples = CloneJointSamplesForPreview(armSamples)
			};

			for (int i = 0; i < snapshot.maxJointSpeedDegPerSecond.Length; i++)
			{
				float maxSpeed = 60f;
				if (armController != null
					&& armController.coordinatedJointControllers != null
					&& i < armController.coordinatedJointControllers.Length
					&& armController.coordinatedJointControllers[i] != null)
				{
					maxSpeed = Mathf.Max(1f, armController.coordinatedJointControllers[i].vMaxDeg);
				}

				snapshot.maxJointSpeedDegPerSecond[i] = maxSpeed;
			}

			return snapshot;
		}

		private static ArmExecutionTimingEstimate ComputeArmExecutionTimingEstimate(ArmTimingSnapshot snapshot, CancellationToken token)
		{
			ArmExecutionTimingEstimate estimate = new ArmExecutionTimingEstimate
			{
				executionTimeScale = 1f,
				scaledExecutionSeconds = 0f
			};
			if (snapshot.samples == null || snapshot.samples.Count == 0)
			{
				return estimate;
			}

			float[] previousAngles = snapshot.startAnglesDeg != null && snapshot.startAnglesDeg.Length >= 6
				? (float[])snapshot.startAnglesDeg.Clone()
				: new float[6];
			float previousTime = 0f;
			float requiredScale = 1f;
			float lastSampleTime = 0f;
			for (int sampleIndex = 0; sampleIndex < snapshot.samples.Count; sampleIndex++)
			{
				token.ThrowIfCancellationRequested();
				RobotPlanJointSample sample = snapshot.samples[sampleIndex];
				if (sample == null || sample.jointAnglesDeg == null || sample.jointAnglesDeg.Length < 6)
				{
					continue;
				}

				float plannedDeltaSeconds = Mathf.Max(ArmExecutionMinSegmentSeconds, sample.timeSeconds - previousTime);
				float requiredSegmentSeconds = EstimateArmSegmentRequiredSecondsOffline(previousAngles, sample.jointAnglesDeg, snapshot.maxJointSpeedDegPerSecond);
				requiredScale = Mathf.Max(requiredScale, requiredSegmentSeconds / plannedDeltaSeconds);
				previousAngles = (float[])sample.jointAnglesDeg.Clone();
				previousTime = sample.timeSeconds;
				lastSampleTime = Mathf.Max(lastSampleTime, sample.timeSeconds);
			}

			estimate.executionTimeScale = Mathf.Clamp(requiredScale, 1f, 8f);
			estimate.scaledExecutionSeconds = Mathf.Max(ArmExecutionMinSegmentSeconds, lastSampleTime * Mathf.Max(1f, estimate.executionTimeScale));
			return estimate;
		}

		private static float EstimateArmSegmentRequiredSecondsOffline(float[] startAnglesDeg, float[] targetAnglesDeg, float[] maxJointSpeedDegPerSecond)
		{
			float worstSeconds = ArmExecutionMinSegmentSeconds;
			for (int jointIndex = 0; jointIndex < 6; jointIndex++)
			{
				float maxSpeed = maxJointSpeedDegPerSecond != null && maxJointSpeedDegPerSecond.Length > jointIndex
					? Mathf.Max(1f, maxJointSpeedDegPerSecond[jointIndex])
					: 60f;
				float startDeg = startAnglesDeg != null && startAnglesDeg.Length > jointIndex ? startAnglesDeg[jointIndex] : 0f;
				float targetDeg = targetAnglesDeg != null && targetAnglesDeg.Length > jointIndex ? targetAnglesDeg[jointIndex] : startDeg;
				float jointDeltaDeg = Mathf.Abs(Mathf.DeltaAngle(startDeg, targetDeg));
				float jointRequiredSeconds = (jointDeltaDeg / maxSpeed) * ArmExecutionSpeedSafetyFactor;
				worstSeconds = Mathf.Max(worstSeconds, jointRequiredSeconds);
			}

			return Mathf.Max(ArmExecutionMinSegmentSeconds, worstSeconds);
		}

		private static StartupPreviewBuildResult BuildStartupPreviewItems(
			Vector3 baseStartWorldPosition,
			float baseTargetYawDeg,
			IReadOnlyList<Vector3> baseWaypoints,
			float baseNominalSpeedMetersPerSecond,
			float baseLeadSeconds,
			float[] armStartAnglesDeg,
			IReadOnlyList<RobotPlanJointSample> armSamples,
			float armLeadSeconds,
			long startingSequenceId,
			int maxPreviewItems,
			CancellationToken token)
		{
			StartupPreviewBuildResult result = new StartupPreviewBuildResult
			{
				totalCount = 0,
				hasBaseCommands = false,
				hasArmCommands = false,
				previewItems = new List<SafetyGateStartupPreviewItem>()
			};

			long sequenceId = startingSequenceId;
			float plannedTime = 0f;
			Vector3 previousBase = baseStartWorldPosition;
			if (baseWaypoints != null)
			{
				for (int i = 0; i < baseWaypoints.Count; i++)
				{
					token.ThrowIfCancellationRequested();
					Vector3 target = baseWaypoints[i];
					float planarDistance = Vector3.Distance(new Vector3(previousBase.x, 0f, previousBase.z), new Vector3(target.x, 0f, target.z));
					float dt = Mathf.Max(0.02f, planarDistance / Mathf.Max(0.1f, baseNominalSpeedMetersPerSecond));
					plannedTime += dt;
					result.totalCount++;
					result.hasBaseCommands = true;
					if (result.previewItems.Count < maxPreviewItems)
					{
						result.previewItems.Add(new SafetyGateStartupPreviewItem
						{
							sequenceId = sequenceId,
							kind = SafetyGateTimelineCommandKind.Base.ToString(),
							plannedTimeSeconds = plannedTime,
							leadTimeSeconds = baseLeadSeconds,
							throttleRatio = 1f,
							fromSummary = $"pos=({previousBase.x:F3},{previousBase.y:F3},{previousBase.z:F3})",
							toSummary = $"pos=({target.x:F3},{target.y:F3},{target.z:F3}), yaw={baseTargetYawDeg:F1}"
						});
					}

					sequenceId++;
					previousBase = target;
				}
			}

			float armTimelineOffset = plannedTime;
			float[] previousArmAngles = armStartAnglesDeg != null && armStartAnglesDeg.Length >= 6
				? (float[])armStartAnglesDeg.Clone()
				: new float[6];
			if (armSamples != null)
			{
				for (int i = 0; i < armSamples.Count; i++)
				{
					token.ThrowIfCancellationRequested();
					RobotPlanJointSample sample = armSamples[i];
					if (sample == null || sample.jointAnglesDeg == null || sample.jointAnglesDeg.Length < 6)
					{
						continue;
					}

					result.totalCount++;
					result.hasArmCommands = true;
					if (result.previewItems.Count < maxPreviewItems)
					{
						result.previewItems.Add(new SafetyGateStartupPreviewItem
						{
							sequenceId = sequenceId,
							kind = SafetyGateTimelineCommandKind.Arm.ToString(),
							plannedTimeSeconds = armTimelineOffset + Mathf.Max(0f, sample.timeSeconds),
							leadTimeSeconds = armLeadSeconds,
							throttleRatio = 1f,
							fromSummary = FormatAnglesSummary(previousArmAngles),
							toSummary = FormatAnglesSummary(sample.jointAnglesDeg)
						});
					}

					previousArmAngles = (float[])sample.jointAnglesDeg.Clone();
					sequenceId++;
				}
			}

			return result;
		}

		private static string FormatAnglesSummary(float[] anglesDeg)
		{
			if (anglesDeg == null || anglesDeg.Length < 6)
			{
				return "j=[n/a]";
			}

			return $"j=[{anglesDeg[0]:F1},{anglesDeg[1]:F1},{anglesDeg[2]:F1},{anglesDeg[3]:F1},{anglesDeg[4]:F1},{anglesDeg[5]:F1}]";
		}

		private IEnumerator RetryDockingSearchAndMoveBaseIfNeeded(
			RobotPlanRequest request,
			RobotPlanResult result,
			List<string> summaryParts,
			List<Collider> obstacles,
			float baseRadius,
			float deadline,
			ArmStagePreparationState state)
		{
			currentStage = RobotPlanningStage.DockingSearch;
			yield return null;

			if (!_coordinatedTaskPlanner.TryPrepareAutoDockingSearch(
				request,
				manager.diffDriveController,
				manager.arm6DOFFKController,
				baseRadius,
				_physicsQueries,
				obstacles,
				out CoordinatedTaskPlanner.DockingSearchContext dockingContext,
				out CoordinatedTaskResolution resolution))
			{
				result.failedAtStage = RobotPlanningStage.DockingSearch;
				result.failureReason = resolution.failureReason;
				yield break;
			}

			if (dockingContext != null)
			{
				yield return null;
				if (!_coordinatedTaskPlanner.TryCompleteAutoDockingSearch(dockingContext, out resolution))
				{
					result.failedAtStage = RobotPlanningStage.DockingSearch;
					result.failureReason = resolution.failureReason;
					yield break;
				}
			}

			result.dockingPoseFound = resolution.dockingPoseFound;
			result.baseMoveRequired = resolution.baseMoveRequired;
			result.armReachableFromCurrentBase = resolution.armReachableFromCurrentBase;
			result.armReachabilityIsLoosePrecheck = resolution.armReachabilityIsLoosePrecheck;
			result.dockingSummary = resolution.dockingSummary;
			result.coarseCandidateCount = resolution.coarseCandidateCount;
			result.fineCandidateCount = resolution.fineCandidateCount;
			result.ikSolveCount = resolution.ikSolveCount;
			result.basePathCheckCount = resolution.basePathCheckCount;
			result.dockingFailureCategory = resolution.dockingFailureCategory;
			result.resolvedBaseStopWorldPosition = resolution.resolvedBaseStopWorldPosition;
			result.resolvedBaseStopYawDeg = resolution.resolvedBaseStopYawDeg;
			state.ResolvedBaseGoal = resolution.resolvedBaseStopWorldPosition;
			state.ResolvedBaseYaw = resolution.resolvedBaseStopYawDeg;
			state.ResolvedBaseGoal.y = manager.diffDriveController.rb.position.y;
			state.PreferredArmSolveSeed = resolution.hasPreferredArmSolveSeed
				? CloneAngles(resolution.preferredArmSolveSeedAnglesDeg)
				: null;
			AppendSummary(summaryParts, resolution.dockingSummary);

			if (resolution.armReachableFromCurrentBase
				|| Vector3.Distance(ProjectXZ(manager.diffDriveController.rb.position), ProjectXZ(state.ResolvedBaseGoal)) <= BasePositionToleranceMeters)
			{
				yield break;
			}

			currentStage = RobotPlanningStage.BasePlanning;
			_distanceFieldSampler.Build(manager.diffDriveController.rb.position, state.ResolvedBaseGoal, obstacles, distanceFieldResolution, basePlanningMargin);
			if (!_basePlanner.TryPlan(
				manager.diffDriveController.rb.position,
				manager.diffDriveController.rb.rotation.eulerAngles.y,
				state.ResolvedBaseGoal,
				state.ResolvedBaseYaw,
				baseRadius,
				_distanceFieldSampler,
				_physicsQueries,
				obstacles,
				out List<Vector3> baseWaypoints,
				out string baseFailure))
			{
				result.failedAtStage = RobotPlanningStage.BasePlanning;
				result.failureReason = baseFailure;
				yield break;
			}

			result.baseWaypoints = baseWaypoints;
			currentStage = RobotPlanningStage.BaseShadowValidation;
			lastShadowValidationResult = _shadowGate.ValidateBasePath(baseWaypoints, baseRadius, _physicsQueries, _distanceFieldSampler, obstacles);
			if (!lastShadowValidationResult.passed)
			{
				result.failedAtStage = RobotPlanningStage.BaseShadowValidation;
				result.failureReason = lastShadowValidationResult.message;
				yield break;
			}

			currentStage = RobotPlanningStage.BaseExecution;
			float retryExecutionBudgetSeconds = ComputeBaseExecutionBudgetSeconds(
				manager.diffDriveController,
				manager.diffDriveController.rb.position,
				baseWaypoints,
				Mathf.Max(4f, deadline - Time.realtimeSinceStartup));
			yield return ExecuteBasePath(
				result,
				request,
				state.ResolvedBaseGoal,
				state.ResolvedBaseYaw,
				baseRadius,
				obstacles,
				Time.realtimeSinceStartup + retryExecutionBudgetSeconds);
			if (!string.IsNullOrEmpty(result.failureReason))
			{
				yield break;
			}

			currentStage = RobotPlanningStage.BaseSettling;
			yield return WaitForBaseSettled(result, state.ResolvedBaseGoal, deadline);
			if (!string.IsNullOrEmpty(result.failureReason))
			{
				yield break;
			}

			manager.CompleteShadowBaseSession(resetShadowToLivePose: false);
			EnterShadowArmPreviewFromLiveBase();
		}

		private bool ValidateBasePathSegments(Vector3 segmentStartWorldPosition, List<Vector3> waypoints, float baseRadius, List<Collider> obstacles, out ShadowValidationResult validation)
		{
			validation = new ShadowValidationResult();
			if (manager == null || manager.diffDriveController == null || manager.diffDriveController.rb == null)
			{
				validation.passed = false;
				validation.message = L("底盘控制器缺失。", "Base controller is missing.");
				return false;
			}

			Vector3 segmentStart = segmentStartWorldPosition;
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
			float effectiveDeadline = deadline + (manager != null ? manager.BaseShadowLeadSeconds : 0f);
			while (Time.realtimeSinceStartup <= effectiveDeadline)
			{
				if (manager != null && manager.TryGetShadowBaseSessionFault(out string shadowFault))
				{
					result.failedAtStage = RobotPlanningStage.BaseSettling;
					result.failureReason = shadowFault;
					yield break;
				}

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
				bool liveReplicaQueueDrained = !manager.HasPendingShadowBaseLiveCommands();
				if (!liveReplicaQueueDrained)
				{
					stableFrames = 0;
					yield return new WaitForFixedUpdate();
					continue;
				}

				float effectiveNearGoalTolerance = Mathf.Max(
					Mathf.Max(controller.posTolerance, controller.arrivalSnapDistance),
					BaseSettlingNearGoalAcceptanceMeters) + BaseSettlingToleranceEpsilonMeters;
				float effectiveStopTolerance = Mathf.Min(
					BaseStopAcceptanceMeters + BaseSettlingToleranceEpsilonMeters,
					effectiveNearGoalTolerance);
				float positionError = Vector3.Distance(ProjectXZ(controller.rb.position), ProjectXZ(goalWorldPosition));
				float linearSpeed = controller.CurrentPlanarSpeedMeasured;
				float angularSpeed = controller.CurrentYawRateMeasured;
				bool controllerReleasedGoal = !controller.hasTargetPoint;
				bool strictSettled = controllerReleasedGoal
					&& positionError <= effectiveStopTolerance
					&& linearSpeed <= BaseLinearSpeedToleranceMetersPerSecond
					&& angularSpeed <= BaseAngularSpeedToleranceRadPerSecond;
				bool nearGoalSettled = controllerReleasedGoal
					&& positionError <= effectiveNearGoalTolerance
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
				DiffDriveTwinController controller = manager.diffDriveController;
				float effectiveNearGoalTolerance = Mathf.Max(
					Mathf.Max(controller.posTolerance, controller.arrivalSnapDistance),
					BaseSettlingNearGoalAcceptanceMeters) + BaseSettlingToleranceEpsilonMeters;
				float finalPositionError = Vector3.Distance(ProjectXZ(controller.rb.position), ProjectXZ(goalWorldPosition));
				float finalLinearSpeed = controller.CurrentPlanarSpeedMeasured;
				float finalAngularSpeed = controller.CurrentYawRateMeasured;
				if (!controller.hasTargetPoint
					&& !manager.HasPendingShadowBaseLiveCommands()
					&& finalPositionError <= effectiveNearGoalTolerance
					&& finalLinearSpeed <= BaseLinearSpeedToleranceMetersPerSecond
					&& finalAngularSpeed <= BaseAngularSpeedToleranceRadPerSecond)
				{
					result.failedAtStage = RobotPlanningStage.None;
					result.failureReason = string.Empty;
					yield break;
				}

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

		private static float ComputeArmTrajectoryFinalWorldResidualMeters(
			Arm6DOFFKController armController,
			IReadOnlyList<RobotPlanJointSample> armSamples,
			Vector3 targetWorldPosition)
		{
			if (armController == null || armSamples == null || armSamples.Count == 0)
			{
				return float.PositiveInfinity;
			}

			RobotPlanJointSample lastSample = armSamples[armSamples.Count - 1];
			if (lastSample == null || lastSample.jointAnglesDeg == null || lastSample.jointAnglesDeg.Length < 6)
			{
				return float.PositiveInfinity;
			}

			Pose finalPoseBase = armController.ForwardPoe(lastSample.jointAnglesDeg);
			Transform baseFrame = armController.BaseFrameTransform;
			Vector3 finalWorldPosition = baseFrame != null
				? baseFrame.TransformPoint(finalPoseBase.position)
				: finalPoseBase.position;
			return Vector3.Distance(finalWorldPosition, targetWorldPosition);
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
			_safetyGateTimelineBuffer.Clear();
			CancelOfflineComputeSession();
			result.success = true;
			result.failedAtStage = RobotPlanningStage.None;
			result.failureReason = string.Empty;
			result.summary = BuildSuccessSummary(summaryParts, _planOnly);
			lastSummary = result.summary;
			isPlanning = false;
			_planningRoutine = null;
			ReturnShadowToMirror();
			onComplete?.Invoke(result);
		}

		private void CompleteFailure(RobotPlanResult result, RobotPlanningStage stage, string reason, Action<RobotPlanResult> onComplete)
		{
			currentStage = RobotPlanningStage.Failed;
			_safetyGateTimelineBuffer.Clear();
			CancelOfflineComputeSession();
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
			manager?.AbortShadowBaseSession(resetShadowToLivePose: true);
			ReturnShadowToMirror();
			onComplete?.Invoke(result);
		}

		private void EnterShadowBasePreview()
		{
			if (_planOnly || manager == null || manager.shadowRobotVisualizer == null)
			{
				return;
			}

			manager.shadowRobotVisualizer.EnterBasePreview();
		}

		private void EnterShadowArmPreviewFromLiveBase()
		{
			if (_planOnly
				|| manager == null
				|| manager.shadowRobotVisualizer == null
				|| manager.diffDriveController == null
				|| manager.diffDriveController.rb == null)
			{
				return;
			}

			manager.shadowRobotVisualizer.EnterArmPreview(
				manager.diffDriveController.rb.position,
				manager.diffDriveController.rb.rotation);
		}

		private void ReturnShadowToMirror()
		{
			if (manager == null || manager.shadowRobotVisualizer == null)
			{
				return;
			}

			manager.shadowRobotVisualizer.ReturnToMirror();
		}

		private void OnDestroy()
		{
			CancelOfflineComputeSession();
			_offlineComputeSemaphore.Dispose();
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
			_safetyGateTimelineBuffer.Clear();
			CancelOfflineComputeSession();
			isPlanning = false;
			_planningRoutine = null;
			lastSummary = failed.summary;
			lastShadowValidationResult = new ShadowValidationResult
			{
				passed = false,
				message = failed.summary
			};
			Debug.LogError($"[RobotTrajectoryPlanner] {exception}");
			ReturnShadowToMirror();
			onComplete?.Invoke(failed);
		}

		private static float ComputeArmExecutionTimeScale(Arm6DOFFKController armController, List<RobotPlanJointSample> armSamples)
		{
			if (armController == null || armSamples == null || armSamples.Count == 0)
			{
				return 1f;
			}

			float[] previousAngles = armController.CaptureMeasuredJointAngles();
			float previousTime = 0f;
			float requiredScale = 1f;
			for (int sampleIndex = 0; sampleIndex < armSamples.Count; sampleIndex++)
			{
				RobotPlanJointSample sample = armSamples[sampleIndex];
				if (sample == null || sample.jointAnglesDeg == null || sample.jointAnglesDeg.Length < 6)
				{
					continue;
				}

				float plannedDeltaSeconds = Mathf.Max(ArmExecutionMinSegmentSeconds, sample.timeSeconds - previousTime);
				float requiredSegmentSeconds = EstimateArmSegmentRequiredSeconds(armController, previousAngles, sample.jointAnglesDeg);
				requiredScale = Mathf.Max(requiredScale, requiredSegmentSeconds / plannedDeltaSeconds);
				previousAngles = (float[])sample.jointAnglesDeg.Clone();
				previousTime = sample.timeSeconds;
			}

			return Mathf.Clamp(requiredScale, 1f, 8f);
		}

		private static float ComputeArmScaledExecutionSeconds(List<RobotPlanJointSample> armSamples, float executionTimeScale)
		{
			if (armSamples == null || armSamples.Count == 0)
			{
				return 0f;
			}

			float lastSampleTime = 0f;
			for (int sampleIndex = 0; sampleIndex < armSamples.Count; sampleIndex++)
			{
				RobotPlanJointSample sample = armSamples[sampleIndex];
				if (sample == null)
				{
					continue;
				}

				lastSampleTime = Mathf.Max(lastSampleTime, sample.timeSeconds);
			}

			return Mathf.Max(ArmExecutionMinSegmentSeconds, lastSampleTime * Mathf.Max(1f, executionTimeScale));
		}

		private static float EstimateArmSegmentRequiredSeconds(Arm6DOFFKController armController, float[] startAnglesDeg, float[] targetAnglesDeg)
		{
			float worstSeconds = ArmExecutionMinSegmentSeconds;
			for (int jointIndex = 0; jointIndex < 6; jointIndex++)
			{
				float maxSpeedDegPerSecond = 60f;
				if (armController.coordinatedJointControllers != null
					&& jointIndex < armController.coordinatedJointControllers.Length
					&& armController.coordinatedJointControllers[jointIndex] != null)
				{
					maxSpeedDegPerSecond = Mathf.Max(1f, armController.coordinatedJointControllers[jointIndex].vMaxDeg);
				}

				float startDeg = startAnglesDeg != null && startAnglesDeg.Length > jointIndex ? startAnglesDeg[jointIndex] : 0f;
				float targetDeg = targetAnglesDeg != null && targetAnglesDeg.Length > jointIndex ? targetAnglesDeg[jointIndex] : startDeg;
				float jointDeltaDeg = Mathf.Abs(Mathf.DeltaAngle(startDeg, targetDeg));
				float jointRequiredSeconds = (jointDeltaDeg / maxSpeedDegPerSecond) * ArmExecutionSpeedSafetyFactor;
				worstSeconds = Mathf.Max(worstSeconds, jointRequiredSeconds);
			}

			return Mathf.Max(ArmExecutionMinSegmentSeconds, worstSeconds);
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

			return Mathf.Max(
				ArmWorldTargetToleranceMeters,
				request.eePositionToleranceMeters > 0f ? request.eePositionToleranceMeters : ArmWorldTargetToleranceMeters);
		}

		private static float GetEffectiveEeTolerance(RobotPlanResult result, RobotPlanRequest request)
		{
			if (result != null && result.effectiveEePositionToleranceMeters > 0f)
			{
				return Mathf.Max(0.001f, result.effectiveEePositionToleranceMeters);
			}

			return GetRequestedEeTolerance(request);
		}

		private string BuildArmSettleDiagnosticReport(RobotPlanRequest request, float[] finalTargetAnglesDeg)
		{
			if (manager == null || manager.arm6DOFFKController == null)
			{
				return "[RobotTrajectoryPlanner] Arm coordinated diagnostics unavailable.";
			}

			Vector3? targetWorldPosition = request != null ? request.armTargetWorldPosition : (Vector3?)null;
			return manager.arm6DOFFKController.BuildCoordinatedDiagnosticReport(
				manager.armBinder,
				targetWorldPosition,
				finalTargetAnglesDeg);
		}

		private void LogArmSettleDiagnostics(RobotPlanRequest request, float[] finalTargetAnglesDeg)
		{
			Debug.LogWarning(BuildArmSettleDiagnosticReport(request, finalTargetAnglesDeg));
		}

		private static Vector3 ProjectXZ(Vector3 value)
		{
			return new Vector3(value.x, 0f, value.z);
		}

		private static float ComputeBaseExecutionBudgetSeconds(
			DiffDriveTwinController controller,
			Vector3 currentBasePosition,
			IReadOnlyList<Vector3> waypoints,
			float minimumBudgetSeconds)
		{
			float totalLength = ComputePlanarPathLength(currentBasePosition, waypoints);
			if (controller == null)
			{
				return Mathf.Max(5f, minimumBudgetSeconds);
			}

			float cruiseSpeed = Mathf.Max(0.15f, controller.vMax * 0.7f);
			float driveSeconds = totalLength / cruiseSpeed;
			float followerStyleBudget = Mathf.Max(5f, (driveSeconds * 4f) + 2.5f);
			return Mathf.Max(minimumBudgetSeconds, followerStyleBudget + 2f);
		}

		private static float ComputePlanarPathLength(Vector3 start, IReadOnlyList<Vector3> waypoints)
		{
			float total = 0f;
			Vector3 previous = ProjectXZ(start);
			if (waypoints == null)
			{
				return total;
			}

			for (int i = 0; i < waypoints.Count; i++)
			{
				Vector3 current = ProjectXZ(waypoints[i]);
				total += Vector3.Distance(previous, current);
				previous = current;
			}

			return total;
		}

		private static bool TryGetRelaxedArmPlanningTolerance(float strictToleranceMeters, out float relaxedToleranceMeters)
		{
			float strictTolerance = Mathf.Max(0.001f, strictToleranceMeters);
			relaxedToleranceMeters = Mathf.Max(ArmPlanningFallbackToleranceMeters, strictTolerance + ArmPlanningFallbackToleranceSlackMeters);
			return relaxedToleranceMeters > strictTolerance + 1e-4f;
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

		private static bool IsLargeResidualArmPlanningFailure(string reason)
		{
			return IsSoftArmPlanningFailure(reason)
				&& TryExtractArmPlanningResidualMeters(reason, out float residualMeters)
				&& residualMeters >= ArmPlanningDockingRetryResidualThresholdMeters;
		}

		private static bool TryExtractArmPlanningResidualMeters(string reason, out float residualMeters)
		{
			residualMeters = 0f;
			if (string.IsNullOrWhiteSpace(reason))
			{
				return false;
			}

			string[] markers =
			{
				"最佳残差为 ",
				"Best residual=",
			};

			for (int markerIndex = 0; markerIndex < markers.Length; markerIndex++)
			{
				string marker = markers[markerIndex];
				int startIndex = reason.IndexOf(marker, StringComparison.OrdinalIgnoreCase);
				if (startIndex < 0)
				{
					continue;
				}

				startIndex += marker.Length;
				int endIndex = startIndex;
				while (endIndex < reason.Length && (char.IsDigit(reason[endIndex]) || reason[endIndex] == '.' || reason[endIndex] == '-'))
				{
					endIndex++;
				}

				if (endIndex <= startIndex)
				{
					continue;
				}

				string raw = reason.Substring(startIndex, endIndex - startIndex);
				if (float.TryParse(raw, System.Globalization.NumberStyles.Float, System.Globalization.CultureInfo.InvariantCulture, out residualMeters)
					|| float.TryParse(raw, out residualMeters))
				{
					return true;
				}
			}

			return false;
		}

		private static string L(string chinese, string english)
		{
			return RobotSimulationLocalization.Text(chinese, english);
		}
	}
}
