using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using TMPro;
using System.IO;
using System.Text.Json;
using System.Text;

namespace RobotSimulation
{
	/// <summary>
	/// Central manager for robot simulation. Handles initialization, control coordination, and state management.
	/// </summary>
	public class RobotSimulationManager : MonoBehaviour
	{
		public static RobotSimulationManager Instance { get; private set; }

		[Header("Robot Controllers")]
		public DiffDriveTwinController diffDriveController;
		public OneJointTrapezoidController[] armJointControllers;
		public ArticulationArmBindToCar armBinder;

		[Header("6-DOF Arm Controllers")]
		public Arm6DOFFKController arm6DOFFKController;
		public Arm6DOFIKController arm6DOFIKController;
		public ArmCollisionMonitor armCollisionMonitor;
		public RobotTrajectoryPlanner trajectoryPlanner;
		public RobotModelResidualTracker residualTracker;
		public OnlineModelCalibration onlineCalibration;

		[Header("Shadow Visualization")]
		public ShadowRobotVisualizer shadowRobotVisualizer;
		[SerializeField] private bool enableShadowRobotVisualization = true;

		[Header("Robot State")]
		[SerializeField] private RobotState _robotState;
		[SerializeField] private ArmMoveResult _lastArmMoveResult = new ArmMoveResult();
		[SerializeField] private bool _isArmMoveInProgress;
		[SerializeField] private bool _isKinematicsSelfTestInProgress;
		[SerializeField] private bool _hasArmCollision;
		[SerializeField] private string _lastArmCollisionMessage = string.Empty;
		[SerializeField] private ArmCollisionGuardResult _lastArmCollisionGuardResult = new ArmCollisionGuardResult();
		[SerializeField] private bool _isArmWorldCoordinateMotionLocked;
		[SerializeField] private string _armWorldCoordinateMotionLockReason = string.Empty;
		[SerializeField] private RobotPlanResult _lastRobotPlanResult = new RobotPlanResult();
		[SerializeField] private bool _isRobotTaskPlanningInProgress;
		[SerializeField] private ShadowValidationResult _lastShadowValidationResult = new ShadowValidationResult();
		[SerializeField] private string _lastPlanningSummary = "规划器空闲 / Planner idle.";
		[SerializeField] private string _lastResidualCalibrationSummary = "Residual tracker idle.";
		public RobotState RobotState => _robotState;
		public ArmMoveResult LastArmMoveResult => _lastArmMoveResult;
		public bool IsArmMoveInProgress => _isArmMoveInProgress;
		public bool IsKinematicsSelfTestInProgress => _isKinematicsSelfTestInProgress;
		public bool HasArmCollision => _hasArmCollision;
		public string LastArmCollisionMessage => _lastArmCollisionMessage;
		public ArmCollisionGuardResult LastArmCollisionGuardResult => _lastArmCollisionGuardResult;
		public bool IsArmWorldCoordinateMotionLocked => _isArmWorldCoordinateMotionLocked;
		public string ArmWorldCoordinateMotionLockReason => _armWorldCoordinateMotionLockReason;
		public RobotPlanResult LastRobotPlanResult => _lastRobotPlanResult;
		public bool IsRobotTaskPlanningInProgress => _isRobotTaskPlanningInProgress;
		public ShadowValidationResult LastShadowValidationResult => _lastShadowValidationResult;
		public string LastPlanningSummary => _lastPlanningSummary;
		public string LastResidualCalibrationSummary => _lastResidualCalibrationSummary;

		[Header("Simulation Settings")]
		public bool enableSimulation = true;
		public float simulationSpeed = 1.0f;
		public bool showDebugInfo = false;
		public bool resetBaseToOriginOnPlay = true;
		public RobotSimulationLanguage displayLanguage = RobotSimulationLanguage.Chinese;

		[Header("UI References")]
		public GameObject controlPanel;
		public GameObject statusPanel;
		public TextMeshProUGUI statusText;
		public TextMeshProUGUI debugText;

		private float _simulationTime;
		private bool _isInitialized = false;
		private Coroutine _armMoveRoutine;
		private Coroutine _kinematicsSelfTestRoutine;
		private Vector3 _capturedStartPosition;
		private Quaternion _capturedStartRotation = Quaternion.identity;
		private bool _hasCapturedStartPose;
		private bool _armCollisionResponseLatched;
		private bool _armWorldCoordinateCommandActive;
		private bool _armWorldCoordinateCommandIssuedSinceUnlock;
		private Coroutine _armHomeUnlockRoutine;
		private bool _startupCoordinatedDiagnosticsLogged;
		private bool _postBindCoordinatedDiagnosticsLogged;

		void Awake()
		{
			if (Instance == null)
			{
				Instance = this;
				DontDestroyOnLoad(gameObject);
			}
			else
			{
				Destroy(gameObject);
				return;
			}

			_robotState = new RobotState();
			RobotSimulationLocalization.SetLanguage(displayLanguage);
		}

		void Start()
		{
			InitializeRobot();
			SetupUI();
		}

		void Update()
		{
			if (!enableSimulation || !_isInitialized) return;

			RobotSimulationLocalization.SetLanguage(displayLanguage);
			Time.timeScale = simulationSpeed;
			_simulationTime += Time.deltaTime * simulationSpeed;

			UpdateRobotState();
			UpdateUI();

			if (residualTracker != null)
			{
				residualTracker.Tick();
				_lastResidualCalibrationSummary = residualTracker.ResidualSummary;
			}

			if (onlineCalibration != null)
			{
				onlineCalibration.Tick();
				_lastResidualCalibrationSummary = $"{_lastResidualCalibrationSummary} | {onlineCalibration.CalibrationSummary}";
			}

			if (trajectoryPlanner != null)
			{
				_lastPlanningSummary = trajectoryPlanner.LastSummary;
				_lastShadowValidationResult = trajectoryPlanner.LastShadowValidationResult;
				_isRobotTaskPlanningInProgress = trajectoryPlanner.IsPlanning;
			}

			if (shadowRobotVisualizer != null)
			{
				shadowRobotVisualizer.SetVisualizationEnabled(enableShadowRobotVisualization);
			}

			SyncArmWorldCoordinateCommandActivity();
		}

		void FixedUpdate()
		{
			if (!enableSimulation || !_isInitialized) return;

			if (armCollisionMonitor != null)
			{
				bool collided = armCollisionMonitor.EvaluateCollisionState();
				if (collided && !_armCollisionResponseLatched)
				{
					if (arm6DOFIKController != null)
					{
						arm6DOFIKController.StopCurrentMove(false);
					}

					if (arm6DOFFKController != null)
					{
						arm6DOFFKController.HoldCurrentPose();
					}

					if (trajectoryPlanner != null && trajectoryPlanner.IsPlanning)
					{
						trajectoryPlanner.StopPlanning(false);
					}

					if (_armWorldCoordinateCommandIssuedSinceUnlock || _armWorldCoordinateCommandActive || _isArmMoveInProgress)
					{
						EngageArmWorldCoordinateMotionLock(string.IsNullOrEmpty(armCollisionMonitor.ActiveCollisionMessage)
							? "World-coordinate arm motion was locked after a collision."
							: armCollisionMonitor.ActiveCollisionMessage);
					}

					_armCollisionResponseLatched = true;
				}
				else if (!collided)
				{
					_armCollisionResponseLatched = false;
				}
			}
		}

		/// <summary>
		/// Initialize robot components and verify references
		/// </summary>
		public void InitializeRobot()
		{
			// Find controllers if not assigned
			if (diffDriveController == null)
			{
				diffDriveController = FindObjectOfType<DiffDriveTwinController>();
			}

			if (armJointControllers == null || armJointControllers.Length == 0)
			{
				armJointControllers = FindObjectsOfType<OneJointTrapezoidController>();
			}

			armJointControllers = BuildOrderedArmJointControllerArray(armJointControllers);

			if (armBinder == null)
			{
				armBinder = FindObjectOfType<ArticulationArmBindToCar>();
			}

			// Find 6-DOF arm controllers
			if (arm6DOFFKController == null)
			{
				arm6DOFFKController = FindObjectOfType<Arm6DOFFKController>();
			}

			if (arm6DOFIKController == null)
			{
				arm6DOFIKController = FindObjectOfType<Arm6DOFIKController>();
			}

			if (armCollisionMonitor == null)
			{
				armCollisionMonitor = FindObjectOfType<ArmCollisionMonitor>();
			}

			CaptureRobotStartPoseIfNeeded();
			EnsureArmCoordinateControllers();
			EnsurePlanningSupportComponents();

			if (resetBaseToOriginOnPlay)
			{
				ResetRobotToCapturedStartPose();
			}

			SyncArmToCurrentBasePoseImmediate();
			LogStartupCoordinatedDiagnosticsIfNeeded();
			StartCoroutine(LogPostBindCoordinatedDiagnosticsAfterFirstFixedUpdate());

			// Validate initialization
			if (diffDriveController == null)
			{
				Debug.LogWarning("[RobotSimulation] DiffDrive controller not found!");
			}

			_isInitialized = diffDriveController != null;
			Debug.Log($"[RobotSimulation] Initialized: {_isInitialized}");
		}

		public bool EnsurePlanningSupportComponents()
		{
			if (trajectoryPlanner == null)
			{
				trajectoryPlanner = GetComponent<RobotTrajectoryPlanner>();
				if (trajectoryPlanner == null)
				{
					trajectoryPlanner = gameObject.AddComponent<RobotTrajectoryPlanner>();
				}
			}

			if (residualTracker == null)
			{
				residualTracker = GetComponent<RobotModelResidualTracker>();
				if (residualTracker == null)
				{
					residualTracker = gameObject.AddComponent<RobotModelResidualTracker>();
				}
			}

			if (onlineCalibration == null)
			{
				onlineCalibration = GetComponent<OnlineModelCalibration>();
				if (onlineCalibration == null)
				{
					onlineCalibration = gameObject.AddComponent<OnlineModelCalibration>();
				}
			}

			if (shadowRobotVisualizer == null)
			{
				shadowRobotVisualizer = GetComponent<ShadowRobotVisualizer>();
				if (shadowRobotVisualizer == null)
				{
					shadowRobotVisualizer = gameObject.AddComponent<ShadowRobotVisualizer>();
				}
			}

			trajectoryPlanner.Configure(this);
			residualTracker.Configure(this);
			onlineCalibration.Configure(residualTracker);
			shadowRobotVisualizer.Configure(this);
			shadowRobotVisualizer.SetVisualizationEnabled(enableShadowRobotVisualization);
			return trajectoryPlanner != null
				&& residualTracker != null
				&& onlineCalibration != null
				&& shadowRobotVisualizer != null;
		}

		private bool EnsureArmCoordinateControllers()
		{
			if (armJointControllers == null || armJointControllers.Length < 6)
			{
				return false;
			}

			if (arm6DOFFKController == null)
			{
				arm6DOFFKController = FindObjectOfType<Arm6DOFFKController>();
			}

			if (arm6DOFIKController == null)
			{
				arm6DOFIKController = FindObjectOfType<Arm6DOFIKController>();
			}

			if (arm6DOFFKController == null || arm6DOFIKController == null)
			{
				GameObject armControllerGo = new GameObject("Arm6DOFController");
				arm6DOFFKController = armControllerGo.AddComponent<Arm6DOFFKController>();
				arm6DOFIKController = armControllerGo.AddComponent<Arm6DOFIKController>();
				Debug.Log("[RobotSimulation] Auto-created Arm6DOFController for coordinate-space arm control.");
			}

			if (NeedsArmCoordinateControllerConfiguration())
			{
				ConfigureArmCoordinateControllersFromJointControllers();
			}
			else
			{
				arm6DOFIKController.armController = arm6DOFFKController;
				EnsureArmCollisionMonitorConfigured();
			}

			return arm6DOFFKController != null && arm6DOFFKController.IsInitialized && arm6DOFIKController != null;
		}

		private bool NeedsArmCoordinateControllerConfiguration()
		{
			if (arm6DOFFKController == null || arm6DOFIKController == null)
			{
				return true;
			}

			if (!arm6DOFFKController.IsInitialized || !arm6DOFFKController.KinematicsReady)
			{
				return true;
			}

			if (arm6DOFIKController.armController != arm6DOFFKController)
			{
				return true;
			}

			if (arm6DOFFKController.joints == null || arm6DOFFKController.joints.Length < 6
				|| arm6DOFFKController.jointTransforms == null || arm6DOFFKController.jointTransforms.Length < 6
				|| arm6DOFFKController.coordinatedJointControllers == null || arm6DOFFKController.coordinatedJointControllers.Length < 6)
			{
				return true;
			}

			for (int i = 0; i < 6; i++)
			{
				OneJointTrapezoidController ctrl = armJointControllers[i];
				if (ctrl == null || ctrl.joint == null || ctrl.joint.jointPosition.dofCount <= 0)
				{
					return true;
				}

				if (arm6DOFFKController.coordinatedJointControllers[i] != ctrl
					|| arm6DOFFKController.joints[i] != ctrl.joint
					|| arm6DOFFKController.jointTransforms[i] != ctrl.joint.transform)
				{
					return true;
				}
			}

			return false;
		}

		private void ConfigureArmCoordinateControllersFromJointControllers()
		{
			if (arm6DOFFKController == null || arm6DOFIKController == null || armJointControllers == null || armJointControllers.Length < 6)
			{
				return;
			}

			arm6DOFFKController.joints = new ArticulationBody[6];
			arm6DOFFKController.jointTransforms = new Transform[6];
			arm6DOFFKController.coordinatedJointControllers = new OneJointTrapezoidController[6];

			for (int i = 0; i < 6; i++)
			{
				OneJointTrapezoidController ctrl = armJointControllers[i];
				if (ctrl == null || ctrl.joint == null || ctrl.joint.jointPosition.dofCount <= 0)
				{
					Debug.LogWarning($"[RobotSimulation] Cannot configure 6-DOF arm controller: joint controller {i} is missing a valid DOF joint.");
					return;
				}

				arm6DOFFKController.joints[i] = ctrl.joint;
				arm6DOFFKController.jointTransforms[i] = ctrl.joint.transform;
				arm6DOFFKController.coordinatedJointControllers[i] = ctrl;
			}

			arm6DOFFKController.preferredLinkNames = new string[] { "Link_01", "Link_02", "Link_03", "Link_04", "Link_05", "Link_06" };
			arm6DOFFKController.expectedJointNames = new string[] { "Joint01", "Joint02", "Joint03", "Joint04", "Joint05", "Joint06" };
			arm6DOFFKController.armHierarchyRoot = arm6DOFFKController.joints[0].transform.root;
			arm6DOFFKController.searchWholeSceneIfLocalSearchFails = false;
			arm6DOFFKController.autoBindByJointName = true;
			arm6DOFFKController.preferLinkNameBinding = true;
			arm6DOFFKController.autoRebindOnMismatch = false;

			Transform lastJointTransform = arm6DOFFKController.jointTransforms[5];
			if (lastJointTransform != null)
			{
				arm6DOFFKController.endEffector = lastJointTransform;
			}

			arm6DOFIKController.armController = arm6DOFFKController;
			arm6DOFFKController.InitializeJoints();
			EnsureArmCollisionMonitorConfigured();
		}

		private void EnsureArmCollisionMonitorConfigured()
		{
			if (arm6DOFFKController == null || arm6DOFFKController.joints == null || arm6DOFFKController.joints.Length == 0 || arm6DOFFKController.joints[0] == null)
			{
				return;
			}

			if (armCollisionMonitor == null)
			{
				GameObject monitorGo = GameObject.Find("ArmCollisionMonitor");
				if (monitorGo == null)
				{
					monitorGo = new GameObject("ArmCollisionMonitor");
				}

				armCollisionMonitor = monitorGo.GetComponent<ArmCollisionMonitor>();
				if (armCollisionMonitor == null)
				{
					armCollisionMonitor = monitorGo.AddComponent<ArmCollisionMonitor>();
				}
			}

			Transform armRoot = null;
			if (armBinder != null && armBinder.armRoot != null)
			{
				armRoot = armBinder.armRoot.transform;
			}
			else if (arm6DOFFKController.BaseFrameTransform != null)
			{
				armRoot = arm6DOFFKController.BaseFrameTransform;
			}
			else
			{
				armRoot = arm6DOFFKController.joints[0].transform.parent != null
					? arm6DOFFKController.joints[0].transform.parent
					: arm6DOFFKController.joints[0].transform;
			}

			Transform forbiddenRoot = null;
			if (armBinder != null && armBinder.carMount != null)
			{
				forbiddenRoot = armBinder.carMount.root;
			}
			else if (diffDriveController != null && diffDriveController.rb != null)
			{
				forbiddenRoot = diffDriveController.rb.transform.root;
			}

			armCollisionMonitor.Configure(armRoot, forbiddenRoot);
			armCollisionMonitor.ignoreArmBaseColliders = true;
			armCollisionMonitor.ignoredArmColliderNameContains = new[] { "Link_00" };
		}

		private OneJointTrapezoidController[] BuildOrderedArmJointControllerArray(OneJointTrapezoidController[] controllers)
		{
			if (controllers == null || controllers.Length == 0)
			{
				return controllers;
			}

			List<KeyValuePair<int, OneJointTrapezoidController>> mapped = new List<KeyValuePair<int, OneJointTrapezoidController>>();
			for (int i = 0; i < controllers.Length; i++)
			{
				OneJointTrapezoidController ctrl = controllers[i];
				if (ctrl == null || ctrl.joint == null || ctrl.joint.jointPosition.dofCount <= 0)
				{
					continue;
				}

				int ordinal = ExtractTrailingNumber(ctrl.joint.name);
				if (ordinal <= 0)
				{
					continue;
				}

				mapped.Add(new KeyValuePair<int, OneJointTrapezoidController>(ordinal, ctrl));
			}

			mapped.Sort((a, b) => a.Key.CompareTo(b.Key));

			List<OneJointTrapezoidController> ordered = new List<OneJointTrapezoidController>();
			HashSet<int> usedOrdinals = new HashSet<int>();
			for (int i = 0; i < mapped.Count; i++)
			{
				int ordinal = mapped[i].Key;
				if (ordinal < 1 || ordinal > 6) continue;
				if (usedOrdinals.Contains(ordinal)) continue;

				ordered.Add(mapped[i].Value);
				usedOrdinals.Add(ordinal);
			}

			if (ordered.Count > 0)
			{
				string orderInfo = "";
				for (int i = 0; i < ordered.Count; i++)
				{
					orderInfo += i == 0 ? ordered[i].joint.name : ", " + ordered[i].joint.name;
				}
				Debug.Log($"[RobotSimulation] Ordered arm controllers by joint name: {orderInfo}");
				return ordered.ToArray();
			}

			Debug.LogWarning("[RobotSimulation] No valid OneJointTrapezoidController with assigned DOF joint was found after filtering.");
			return controllers;
		}

		private int ExtractTrailingNumber(string value)
		{
			if (string.IsNullOrEmpty(value))
			{
				return -1;
			}

			int number = 0;
			int factor = 1;
			bool foundDigit = false;
			for (int i = value.Length - 1; i >= 0; i--)
			{
				char c = value[i];
				if (c >= '0' && c <= '9')
				{
					number += (c - '0') * factor;
					factor *= 10;
					foundDigit = true;
				}
				else if (foundDigit)
				{
					break;
				}
			}

			return foundDigit ? number : -1;
		}

		/// <summary>
		/// Setup UI panels and references
		/// </summary>
		private void SetupUI()
		{
			if (controlPanel == null)
			{
				Debug.Log("[RobotSimulation] Control panel not assigned");
			}

			if (statusPanel == null)
			{
				Debug.Log("[RobotSimulation] Status panel not assigned");
			}
		}

		/// <summary>
		/// Update robot state from controllers
		/// </summary>
		private void UpdateRobotState()
		{
			if (diffDriveController != null)
			{
				_robotState.position = diffDriveController.rb.position;
				_robotState.rotation = diffDriveController.rb.rotation.eulerAngles.y;
				_robotState.linearVelocity = diffDriveController.CurrentPlanarSpeedMeasured;
				_robotState.angularVelocity = diffDriveController.CurrentYawRateMeasured;
				_robotState.leftWheelVelocity = diffDriveController.vLeft;
				_robotState.rightWheelVelocity = diffDriveController.vRight;
				_robotState.targetPoint = diffDriveController.targetPointWorld;
				_robotState.hasTargetPoint = diffDriveController.hasTargetPoint;
				_robotState.targetYaw = diffDriveController.targetYawDeg;
				_robotState.controlMode = diffDriveController.mode.ToString();
			}

			if (armJointControllers != null)
			{
				_robotState.jointAngles = new float[armJointControllers.Length];
				for (int i = 0; i < armJointControllers.Length; i++)
				{
					if (armJointControllers[i]?.joint != null)
					{
						_robotState.jointAngles[i] = armJointControllers[i].joint.jointPosition[0] * Mathf.Rad2Deg;
					}
				}
			}

			// Update 6-DOF arm state
			if (arm6DOFFKController != null && arm6DOFFKController.IsInitialized)
			{
				Vector3 endEffectorBase = arm6DOFFKController.EndEffectorPosition;
				Vector3 endEffectorWorld = arm6DOFFKController.EndEffectorWorldPosition;
				_robotState.endEffectorPositionBase = endEffectorBase;
				_robotState.endEffectorPositionWorld = endEffectorWorld;
				_robotState.armJointAngles = arm6DOFFKController.CurrentJointAngles;
			}

			_robotState.simulationTime = _simulationTime;
			_robotState.isRobotTaskPlanning = _isRobotTaskPlanningInProgress;
			_robotState.robotPlanningStage = trajectoryPlanner != null
				? RobotSimulationLocalization.PlanningStage(trajectoryPlanner.CurrentStage)
				: RobotSimulationLocalization.PlanningStage(RobotPlanningStage.None);
			_robotState.lastPlanningSummary = _lastPlanningSummary;
			if (armCollisionMonitor != null)
			{
				_hasArmCollision = armCollisionMonitor.HasCollision;
				if (!string.IsNullOrEmpty(armCollisionMonitor.ActiveCollisionMessage))
				{
					_lastArmCollisionMessage = armCollisionMonitor.ActiveCollisionMessage;
				}
			}
		}

		/// <summary>
		/// Update UI displays
		/// </summary>
		private void UpdateUI()
		{
			if (statusText != null)
			{
				statusText.text = FormatLocalizedStatusText();
			}

			if (debugText != null && showDebugInfo)
			{
				debugText.text = FormatLocalizedDebugText();
			}
		}

		private string FormatLocalizedStatusText()
		{
			string text = $"<b>{RobotSimulationLocalization.Text("机器人状态", "Robot Status")}</b>\n" +
				   $"鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€\n" +
				   $"{RobotSimulationLocalization.Text("模式", "Mode")}: {RobotSimulationLocalization.ControlMode(_robotState.controlMode)}\n" +
				   $"{RobotSimulationLocalization.Text("位置", "Position")}: ({_robotState.position.x:F2}, {_robotState.position.z:F2})\n" +
				   $"{RobotSimulationLocalization.Text("旋转", "Rotation")}: {_robotState.rotation:F1}掳\n" +
				   $"{RobotSimulationLocalization.Text("线速度", "Linear Vel")}: {_robotState.linearVelocity:F3} m/s\n" +
				   $"{RobotSimulationLocalization.Text("角速度", "Angular Vel")}: {_robotState.angularVelocity:F3} rad/s\n" +
				   $"{RobotSimulationLocalization.Text("目标", "Target")}: {(_robotState.hasTargetPoint ? RobotSimulationLocalization.Text("已设置", "Set") : RobotSimulationLocalization.Text("无", "None"))}\n";

			if (arm6DOFFKController != null && arm6DOFFKController.IsInitialized)
			{
				text += $"\n<b>{RobotSimulationLocalization.Text("末端执行器", "End Effector")}</b>\n" +
						$"{RobotSimulationLocalization.Text("世界坐标", "World")}: ({_robotState.endEffectorPositionWorld.x:F3}, {_robotState.endEffectorPositionWorld.y:F3}, {_robotState.endEffectorPositionWorld.z:F3})\n" +
						$"{RobotSimulationLocalization.Text("基座坐标", "Base")}: ({_robotState.endEffectorPositionBase.x:F3}, {_robotState.endEffectorPositionBase.y:F3}, {_robotState.endEffectorPositionBase.z:F3})\n";
				text += $"{RobotSimulationLocalization.Text("机械臂碰撞", "Arm Collision")}: {(_hasArmCollision ? RobotSimulationLocalization.Text("错误", "ERROR") : RobotSimulationLocalization.Text("无", "None"))}\n";
				if (_lastArmCollisionGuardResult != null && _lastArmCollisionGuardResult.blockedByForbiddenCollision)
				{
					text += $"{RobotSimulationLocalization.Text("预检阻止", "Guard Block")}: {_lastArmCollisionGuardResult.message}\n";
				}
			}

			text += $"{RobotSimulationLocalization.Text("整机规划", "Task Planning")}: {(_isRobotTaskPlanningInProgress ? RobotSimulationLocalization.Text("进行中", "Running") : RobotSimulationLocalization.Text("空闲", "Idle"))}\n";
			text += $"{RobotSimulationLocalization.Text("当前阶段", "Current Stage")}: {_robotState.robotPlanningStage}\n";
			text += $"\n{RobotSimulationLocalization.Text("仿真时间", "Sim Time")}: {_robotState.simulationTime:F1}s";
			return text;
		}

		private string FormatLocalizedDebugText()
		{
			return $"<b>{RobotSimulationLocalization.Text("调试信息", "Debug Info")}</b>\n" +
				   $"鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€\n" +
				   $"{RobotSimulationLocalization.Text("已初始化", "Initialized")}: {_isInitialized}\n" +
				   $"{RobotSimulationLocalization.Text("关节数", "Joints")}: {armJointControllers?.Length ?? 0}\n" +
				   $"{RobotSimulationLocalization.Text("速度倍率", "Speed")}: {simulationSpeed:F1}x";
		}

		private string FormatStatusText()
		{
			string text = $"<b>Robot Status</b>\n" +
				   $"─────────────\n" +
				   $"Mode: {_robotState.controlMode}\n" +
				   $"Position: ({_robotState.position.x:F2}, {_robotState.position.z:F2})\n" +
				   $"Rotation: {_robotState.rotation:F1}°\n" +
				   $"Linear Vel: {_robotState.linearVelocity:F3} m/s\n" +
				   $"Angular Vel: {_robotState.angularVelocity:F3} rad/s\n" +
				   $"Target: {(_robotState.hasTargetPoint ? "Set" : "None")}\n";

			// Add 6-DOF arm info if available
			if (arm6DOFFKController != null && arm6DOFFKController.IsInitialized)
			{
				text += $"\n<b>End Effector</b>\n" +
						$"World: ({_robotState.endEffectorPositionWorld.x:F3}, {_robotState.endEffectorPositionWorld.y:F3}, {_robotState.endEffectorPositionWorld.z:F3})\n" +
						$"Base: ({_robotState.endEffectorPositionBase.x:F3}, {_robotState.endEffectorPositionBase.y:F3}, {_robotState.endEffectorPositionBase.z:F3})\n";
				text += $"Arm Collision: {(_hasArmCollision ? "ERROR" : "None")}\n";
				if (_lastArmCollisionGuardResult != null && _lastArmCollisionGuardResult.blockedByForbiddenCollision)
				{
					text += $"Guard Block: {_lastArmCollisionGuardResult.message}\n";
				}
			}

			text += $"Task Planning: {(_isRobotTaskPlanningInProgress ? "Running" : "Idle")}\n";
			text += $"Current Stage: {_robotState.robotPlanningStage}\n";
			text += $"\nSim Time: {_robotState.simulationTime:F1}s";
			return text;
		}

		private string FormatDebugText()
		{
			return $"<b>Debug Info</b>\n" +
				   $"─────────────\n" +
				   $"Initialized: {_isInitialized}\n" +
				   $"Joints: {armJointControllers?.Length ?? 0}\n" +
				   $"Speed: {simulationSpeed:F1}x";
		}

		/// <summary>
		/// Set target point for navigation
		/// </summary>
		public void SetTargetPoint(Vector3 point)
		{
			if (diffDriveController != null)
			{
				if (diffDriveController.rb != null)
				{
					point.y = diffDriveController.rb.position.y;
				}

				diffDriveController.SetTargetPointGoal(point);
			}
		}

		/// <summary>
		/// Set target yaw angle
		/// </summary>
		public void SetTargetYaw(float yawDegrees)
		{
			if (diffDriveController != null)
			{
				diffDriveController.SetTargetYawGoal(yawDegrees);
			}
		}

		/// <summary>
		/// Set joint target angle for a specific joint
		/// </summary>
		public void SetJointTarget(int jointIndex, float angleDegrees)
		{
			if (armJointControllers != null && jointIndex >= 0 && jointIndex < armJointControllers.Length)
			{
				armJointControllers[jointIndex].goalDeg = angleDegrees;
			}
		}

		/// <summary>
		/// Set 6-DOF arm joint target
		/// </summary>
		public void SetArmJointTarget(int jointIndex, float angleDegrees)
		{
			TrySetArmJointTarget(jointIndex, angleDegrees);
		}

		public bool TrySetArmJointTarget(int jointIndex, float angleDegrees)
		{
			if (!EnsureArmCoordinateControllers() || arm6DOFFKController == null)
			{
				return false;
			}

			bool success = arm6DOFFKController.TrySetJointTarget(jointIndex, angleDegrees);
			_lastArmCollisionGuardResult = arm6DOFFKController.LastCollisionGuardResult;
			return success;
		}

		/// <summary>
		/// Move end effector to target position using IK
		/// Returns true if successful, false if unreachable (and logs error)
		/// </summary>
		public bool MoveArmToPosition(Vector3 position)
		{
			if (_isArmWorldCoordinateMotionLocked)
			{
				_lastArmMoveResult = CreateLockedArmMoveResult(position);
				return false;
			}

			if (!EnsureArmCoordinateControllers() || arm6DOFIKController == null)
			{
				Debug.LogError("[RobotSimulation] IK Controller not found!");
				return false;
			}

			bool success = arm6DOFIKController.TryStartMoveToWorldPosition(position);
			_lastArmCollisionGuardResult = arm6DOFIKController.LastCollisionGuardResult;
			if (success)
			{
				_armWorldCoordinateCommandActive = true;
				_armWorldCoordinateCommandIssuedSinceUnlock = true;
			}
			if (!success)
			{
				_lastArmMoveResult = arm6DOFIKController.LastMoveResult;
				if (_lastArmCollisionGuardResult != null && _lastArmCollisionGuardResult.blockedByForbiddenCollision)
				{
					EngageArmWorldCoordinateMotionLock(_lastArmCollisionGuardResult.message);
				}
			}

			return success;
		}

		public Coroutine MoveArmToWorldPositionAndWait(ArmMoveRequest request, System.Action<ArmMoveResult> onComplete = null)
		{
			Vector3 targetPosition = request != null ? request.worldPosition : Vector3.zero;
			if (_isArmWorldCoordinateMotionLocked)
			{
				ArmMoveResult lockedResult = CreateLockedArmMoveResult(targetPosition);
				onComplete?.Invoke(lockedResult);
				return null;
			}

			if (!EnsureArmCoordinateControllers() || arm6DOFIKController == null)
			{
				Debug.LogError("[RobotSimulation] IK Controller not found!");
				return null;
			}

			if (_armMoveRoutine != null && arm6DOFIKController != null)
			{
				arm6DOFIKController.StopCurrentMove(false);
				_armMoveRoutine = null;
			}

			_isArmMoveInProgress = true;
			_armWorldCoordinateCommandActive = true;
			_armWorldCoordinateCommandIssuedSinceUnlock = true;
			_armMoveRoutine = arm6DOFIKController.MoveToWorldPositionAndWait(request, result =>
			{
				_lastArmMoveResult = result;
				_lastArmCollisionGuardResult = result.collisionGuardResult ?? arm6DOFIKController.LastCollisionGuardResult;
				_isArmMoveInProgress = false;
				_armMoveRoutine = null;
				if (result != null && (result.collided || result.blockedByCollisionGuard))
				{
					EngageArmWorldCoordinateMotionLock(!string.IsNullOrEmpty(result.summary)
						? result.summary
						: "World-coordinate arm motion was locked after a collision.");
				}
				else
				{
					_armWorldCoordinateCommandActive = false;
				}
				onComplete?.Invoke(result);
			});
			return _armMoveRoutine;
		}

		public void StopArmMove(bool emergencyStopArm = true)
		{
			if (arm6DOFIKController != null)
			{
				arm6DOFIKController.StopCurrentMove(emergencyStopArm);
				_lastArmMoveResult = arm6DOFIKController.LastMoveResult;
				_lastArmCollisionGuardResult = arm6DOFIKController.LastCollisionGuardResult;
			}

			_isArmMoveInProgress = false;
			_armMoveRoutine = null;
			_armWorldCoordinateCommandActive = false;
		}

		public Coroutine PlanAndExecuteRobotTask(RobotPlanRequest request, System.Action<RobotPlanResult> onComplete = null)
		{
			if (!Application.isPlaying)
			{
				_lastPlanningSummary = "PlanAndExecuteRobotTask can only run in Play Mode.";
				Debug.LogError("[RobotSimulation] PlanAndExecuteRobotTask can only run in Play Mode.");
				return null;
			}

			if (!EnsurePlanningSupportComponents())
			{
				_lastPlanningSummary = RobotSimulationLocalization.Text("轨迹规划器不可用。", "Trajectory planner is not available.");
				Debug.LogError("[RobotSimulation] Trajectory planner is not available.");
				return null;
			}

			_isRobotTaskPlanningInProgress = true;
			_lastPlanningSummary = RobotSimulationLocalization.Text("开始规划。", "Planning started.");
			Debug.Log("[RobotSimulation] PlanAndExecuteRobotTask started.");
			Coroutine routine = trajectoryPlanner.PlanAndExecute(request, true, result =>
			{
				_lastRobotPlanResult = result;
				_lastPlanningSummary = result.summary;
				_lastShadowValidationResult = trajectoryPlanner != null ? trajectoryPlanner.LastShadowValidationResult : _lastShadowValidationResult;
				_isRobotTaskPlanningInProgress = false;
				onComplete?.Invoke(result);
			});
			if (routine == null)
			{
				_isRobotTaskPlanningInProgress = false;
				Debug.LogWarning($"[RobotSimulation] PlanAndExecuteRobotTask did not start. Summary={_lastPlanningSummary}");
			}

			return routine;
		}

		public Coroutine PlanRobotTaskOnly(RobotPlanRequest request, System.Action<RobotPlanResult> onComplete = null)
		{
			if (!Application.isPlaying)
			{
				_lastPlanningSummary = RobotSimulationLocalization.Text("仅规划模式只能在 Play Mode 下运行。", "PlanRobotTaskOnly can only run in Play Mode.");
				Debug.LogError("[RobotSimulation] PlanRobotTaskOnly can only run in Play Mode.");
				return null;
			}

			if (!EnsurePlanningSupportComponents())
			{
				_lastPlanningSummary = RobotSimulationLocalization.Text("轨迹规划器不可用。", "Trajectory planner is not available.");
				Debug.LogError("[RobotSimulation] Trajectory planner is not available.");
				return null;
			}

			_isRobotTaskPlanningInProgress = true;
			_lastPlanningSummary = RobotSimulationLocalization.Text("开始规划（仅规划）。", "Planning started (plan only).");
			Debug.Log("[RobotSimulation] PlanRobotTaskOnly started.");
			Coroutine routine = trajectoryPlanner.PlanAndExecute(request, false, result =>
			{
				_lastRobotPlanResult = result;
				_lastPlanningSummary = result.summary;
				_lastShadowValidationResult = trajectoryPlanner != null ? trajectoryPlanner.LastShadowValidationResult : _lastShadowValidationResult;
				_isRobotTaskPlanningInProgress = false;
				onComplete?.Invoke(result);
			});
			if (routine == null)
			{
				_isRobotTaskPlanningInProgress = false;
				Debug.LogWarning($"[RobotSimulation] PlanRobotTaskOnly did not start. Summary={_lastPlanningSummary}");
			}

			return routine;
		}

		public void StopRobotTaskPlanning(bool emergencyStop = true)
		{
			if (trajectoryPlanner != null)
			{
				trajectoryPlanner.StopPlanning(emergencyStop);
			}

			_isRobotTaskPlanningInProgress = false;
		}

		/// <summary>
		/// Freeze FK binding to the currently discovered J1..J6 controller joints.
		/// This prevents future scene-order changes from remapping joints unexpectedly.
		/// </summary>
		public bool FreezeArmBindingConfiguration()
		{
			if (arm6DOFFKController == null)
			{
				Debug.LogError("[RobotSimulation] Cannot freeze arm binding: Arm6DOFFKController not found.");
				return false;
			}

			if (armJointControllers == null || armJointControllers.Length < 6)
			{
				armJointControllers = FindObjectsOfType<OneJointTrapezoidController>();
				armJointControllers = BuildOrderedArmJointControllerArray(armJointControllers);
			}

			if (armJointControllers == null || armJointControllers.Length < 6)
			{
				Debug.LogError("[RobotSimulation] Cannot freeze arm binding: less than 6 valid OneJointTrapezoidController were found.");
				return false;
			}

			arm6DOFFKController.joints = new ArticulationBody[6];
			arm6DOFFKController.jointTransforms = new Transform[6];
			arm6DOFFKController.coordinatedJointControllers = new OneJointTrapezoidController[6];

			for (int i = 0; i < 6; i++)
			{
				if (armJointControllers[i] == null || armJointControllers[i].joint == null)
				{
					Debug.LogError($"[RobotSimulation] Cannot freeze arm binding: armJointControllers[{i}] has no assigned joint.");
					return false;
				}

				arm6DOFFKController.joints[i] = armJointControllers[i].joint;
				arm6DOFFKController.jointTransforms[i] = armJointControllers[i].joint.transform;
				arm6DOFFKController.coordinatedJointControllers[i] = armJointControllers[i];
			}

			arm6DOFFKController.preferredLinkNames = new string[] { "Link_01", "Link_02", "Link_03", "Link_04", "Link_05", "Link_06" };
			arm6DOFFKController.expectedJointNames = new string[] { "Joint01", "Joint02", "Joint03", "Joint04", "Joint05", "Joint06" };
			arm6DOFFKController.armHierarchyRoot = arm6DOFFKController.joints[0].transform.root;
			arm6DOFFKController.searchWholeSceneIfLocalSearchFails = false;
			arm6DOFFKController.autoBindByJointName = true;
			arm6DOFFKController.preferLinkNameBinding = true;
			arm6DOFFKController.autoRebindOnMismatch = false;

			arm6DOFFKController.InitializeJoints();
			Debug.Log("[RobotSimulation] Arm binding configuration frozen to current J1..J6 joints.");
			return arm6DOFFKController.IsInitialized;
		}

		public Coroutine RunIKRegressionTest(System.Action<string> onComplete = null)
		{
			if (arm6DOFIKController == null)
			{
				Debug.LogError("[RobotSimulation] IK regression test failed to start: IK controller not found.");
				onComplete?.Invoke("IK controller not found.");
				return null;
			}

			return arm6DOFIKController.RunRegressionTest(onComplete);
		}

		public Coroutine RunKinematicsSelfTest(System.Action<string> onComplete = null)
		{
			if (_kinematicsSelfTestRoutine != null)
			{
				onComplete?.Invoke("Kinematics self-test is already running.");
				return _kinematicsSelfTestRoutine;
			}

			_isKinematicsSelfTestInProgress = true;
			_kinematicsSelfTestRoutine = StartCoroutine(RunKinematicsSelfTestCoroutine(summary =>
			{
				_isKinematicsSelfTestInProgress = false;
				_kinematicsSelfTestRoutine = null;
				onComplete?.Invoke(summary);
			}));
			return _kinematicsSelfTestRoutine;
		}

		public void StopKinematicsSelfTest(bool stopArmMove = true)
		{
			if (_kinematicsSelfTestRoutine != null)
			{
				StopCoroutine(_kinematicsSelfTestRoutine);
				_kinematicsSelfTestRoutine = null;
			}

			_isKinematicsSelfTestInProgress = false;
			if (stopArmMove)
			{
				StopArmMove(true);
			}
		}

		public string GetArmKinematicsParameterReport()
		{
			if (!EnsureArmCoordinateControllers() || arm6DOFFKController == null)
			{
				return "Arm FK controller is not initialized.";
			}

			return arm6DOFFKController.GetKinematicsParameterReport();
		}

		/// <summary>
		/// Check if position is reachable
		/// </summary>
		public bool IsArmPositionReachable(Vector3 position)
		{
			EnsureArmCoordinateControllers();
			if (arm6DOFIKController != null)
			{
				return arm6DOFIKController.IsPositionReachable(position);
			}
			return false;
		}

		/// <summary>
		/// Get arm reachability info
		/// </summary>
		public string GetArmReachabilityInfo(Vector3 position)
		{
			EnsureArmCoordinateControllers();
			if (arm6DOFIKController != null)
			{
				return arm6DOFIKController.GetReachabilityInfo(position);
			}
			return "IK Controller not initialized";
		}

		/// <summary>
		/// Move arm to home position
		/// </summary>
		public void MoveArmHome()
		{
			TryMoveArmHome();
		}

		public bool TryMoveArmHome()
		{
			if (!EnsureArmCoordinateControllers() || arm6DOFFKController == null)
			{
				return false;
			}

			bool success = arm6DOFFKController.TryGoHome();
			_lastArmCollisionGuardResult = arm6DOFFKController.LastCollisionGuardResult;
			if (success)
			{
				if (_armHomeUnlockRoutine != null)
				{
					StopCoroutine(_armHomeUnlockRoutine);
				}

				_armHomeUnlockRoutine = StartCoroutine(WaitForHomePoseAndUnlockCoroutine());
			}
			return success;
		}

		private IEnumerator RunKinematicsSelfTestCoroutine(System.Action<string> onComplete)
		{
			if (!EnsureArmCoordinateControllers() || arm6DOFFKController == null || !arm6DOFFKController.KinematicsReady)
			{
				string failed = "Kinematics self-test aborted: FK controller or URDF model is not ready.";
				Debug.LogError($"[RobotSimulation] {failed}");
				onComplete?.Invoke(failed);
				yield break;
			}

			bool passed = true;
			float worstError = 0f;
			StringBuilder issueBuilder = new StringBuilder();
			float[] homeAngles = arm6DOFFKController.configuredHomeJointAnglesDeg != null && arm6DOFFKController.configuredHomeJointAnglesDeg.Length >= 6
				? (float[])arm6DOFFKController.configuredHomeJointAnglesDeg.Clone()
				: Arm6DOFFKController.CreateDefaultConfiguredHomeJointAnglesDeg();
			float homeError = Vector3.Distance(arm6DOFFKController.ForwardUrdfPoe(homeAngles).position, arm6DOFFKController.ForwardUrdfChain(homeAngles).position);
			worstError = Mathf.Max(worstError, homeError);
			if (homeError > 1e-4f)
			{
				passed = false;
				issueBuilder.AppendLine($"Configured-home PoE vs URDF-chain mismatch: {homeError:F6}m");
			}

			float[][] samples = new float[][]
			{
				new float[] { 10f, -20f, 160f, -10f, 80f, 15f },
				new float[] { -15f, 25f, 120f, 5f, 110f, -25f },
				new float[] { 5f, 10f, 135f, -5f, 70f, 5f }
			};

			for (int i = 0; i < samples.Length; i++)
			{
				float sampleError = Vector3.Distance(arm6DOFFKController.ForwardUrdfPoe(samples[i]).position, arm6DOFFKController.ForwardUrdfChain(samples[i]).position);
				worstError = Mathf.Max(worstError, sampleError);
				if (sampleError > 1e-3f)
				{
					passed = false;
					issueBuilder.AppendLine($"Sample {i} PoE vs URDF-chain mismatch: {sampleError:F6}m");
				}
			}

			float measuredError = arm6DOFFKController.ModelVsMeasuredPositionError;
			worstError = Mathf.Max(worstError, measuredError);
			if (measuredError > 0.05f)
			{
				passed = false;
				issueBuilder.AppendLine($"Model vs measured end-effector residual is high: {measuredError:F6}m");
			}

			ArmMoveResult moveResult = null;
			if (arm6DOFIKController != null)
			{
				Vector3 currentWorld = arm6DOFFKController.EndEffectorWorldPosition;
				bool done = false;
				MoveArmToWorldPositionAndWait(new ArmMoveRequest
				{
					worldPosition = currentWorld + new Vector3(0.03f, 0.02f, -0.02f),
					positionToleranceMeters = 0.015f,
					stableFixedFrames = 3,
					timeoutSeconds = 4f
				}, result =>
				{
					moveResult = result;
					done = true;
				});

				while (!done)
				{
					yield return null;
				}

				if (moveResult == null || !moveResult.success)
				{
					passed = false;
					issueBuilder.AppendLine($"Move-and-wait probe failed: {(moveResult == null ? "no result" : moveResult.summary)}");
				}
				else
				{
					worstError = Mathf.Max(worstError, moveResult.finalPositionError);
				}
			}

			string summary = $"Kinematics self-test {(passed ? "passed" : "failed")}. home={homeError:F4}m, measured={measuredError:F4}m, worst={worstError:F4}m";
			if (moveResult != null)
			{
				summary += $", move={(moveResult.success ? "ok" : moveResult.summary)}";
			}

			if (passed)
			{
				Debug.Log($"[RobotSimulation] {summary}");
			}
			else
			{
				string details = issueBuilder.Length > 0 ? $"\n{issueBuilder.ToString().TrimEnd()}" : string.Empty;
				Debug.LogWarning($"[RobotSimulation] {summary}{details}");
			}

			onComplete?.Invoke(summary);
		}

		/// <summary>
		/// Check if arm is at target
		/// </summary>
		public bool IsArmAtTarget(float toleranceDeg = 1f)
		{
			if (arm6DOFFKController != null)
			{
				return arm6DOFFKController.IsAtTarget(toleranceDeg);
			}
			return false;
		}

		/// <summary>
		/// Clear current target point
		/// </summary>
		public void ClearTarget()
		{
			if (diffDriveController != null)
			{
				diffDriveController.hasTargetPoint = false;
				diffDriveController.targetPointWorld = diffDriveController.rb != null ? diffDriveController.rb.position : Vector3.zero;
				diffDriveController.mode = DiffDriveTwinController.ControlMode.TargetPoint;
				diffDriveController.HardStopAtGoal();
			}
		}

		private void SyncArmWorldCoordinateCommandActivity()
		{
			if (!_armWorldCoordinateCommandActive || arm6DOFIKController == null)
			{
				return;
			}

			if (!arm6DOFIKController.IsSolving && !arm6DOFIKController.IsMoveInProgress)
			{
				_armWorldCoordinateCommandActive = false;
			}
		}

		private void EngageArmWorldCoordinateMotionLock(string reason)
		{
			_isArmWorldCoordinateMotionLocked = true;
			_armWorldCoordinateMotionLockReason = string.IsNullOrWhiteSpace(reason)
				? "World-coordinate arm commands are locked until the arm returns to Home."
				: reason;
			_armWorldCoordinateCommandActive = false;
			_lastArmMoveResult = new ArmMoveResult
			{
				accepted = false,
				success = false,
				collided = true,
				finalWorldPosition = arm6DOFFKController != null ? arm6DOFFKController.EndEffectorWorldPosition : Vector3.zero,
				summary = _armWorldCoordinateMotionLockReason
			};
			Debug.LogWarning($"[RobotSimulation] World-coordinate arm motion locked: {_armWorldCoordinateMotionLockReason}");
		}

		private void ClearArmWorldCoordinateMotionLock(string reason = null)
		{
			_isArmWorldCoordinateMotionLocked = false;
			_armWorldCoordinateMotionLockReason = string.IsNullOrWhiteSpace(reason)
				? string.Empty
				: reason;
			_armWorldCoordinateCommandActive = false;
			_armWorldCoordinateCommandIssuedSinceUnlock = false;
		}

		private ArmMoveResult CreateLockedArmMoveResult(Vector3 targetWorldPosition)
		{
			return new ArmMoveResult
			{
				accepted = false,
				success = false,
				targetWorldPosition = targetWorldPosition,
				finalWorldPosition = arm6DOFFKController != null ? arm6DOFFKController.EndEffectorWorldPosition : Vector3.zero,
				finalPositionError = arm6DOFFKController != null ? Vector3.Distance(arm6DOFFKController.EndEffectorWorldPosition, targetWorldPosition) : 0f,
				summary = string.IsNullOrEmpty(_armWorldCoordinateMotionLockReason)
					? "World-coordinate arm commands are locked until the arm returns to Home."
					: _armWorldCoordinateMotionLockReason
			};
		}

		private IEnumerator WaitForHomePoseAndUnlockCoroutine()
		{
			float elapsed = 0f;
			const float timeoutSeconds = 8f;
			int stableFrames = 0;
			const int requiredStableFrames = 3;
			while (elapsed < timeoutSeconds)
			{
				yield return new WaitForFixedUpdate();
				elapsed += Time.fixedDeltaTime;

				if (arm6DOFFKController == null)
				{
					continue;
				}

				bool homeReached = arm6DOFFKController.IsAtTarget(1f) && IsArmNearConfiguredHome(1.5f);
				bool collisionCleared = armCollisionMonitor == null || !armCollisionMonitor.HasCollision;
				if (homeReached && collisionCleared)
				{
					stableFrames++;
					if (stableFrames >= requiredStableFrames)
					{
						ClearArmWorldCoordinateMotionLock("Unlocked after returning to Home.");
						_armHomeUnlockRoutine = null;
						yield break;
					}
				}
				else
				{
					stableFrames = 0;
				}
			}

			_armHomeUnlockRoutine = null;
		}

		private bool IsArmNearConfiguredHome(float toleranceDeg)
		{
			if (arm6DOFFKController == null || arm6DOFFKController.configuredHomeJointAnglesDeg == null)
			{
				return false;
			}

			float[] measured = arm6DOFFKController.CaptureMeasuredJointAngles();
			for (int i = 0; i < Mathf.Min(6, measured.Length); i++)
			{
				if (Mathf.Abs(measured[i] - arm6DOFFKController.configuredHomeJointAnglesDeg[i]) > toleranceDeg)
				{
					return false;
				}
			}

			return true;
		}

		/// <summary>
		/// Emergency stop - halt all motion
		/// </summary>
		public void EmergencyStop()
		{
			if (diffDriveController != null)
			{
				diffDriveController.HardStopAtGoal();
			}

			if (arm6DOFFKController != null)
			{
				arm6DOFFKController.EmergencyStop();
			}
		}

		/// <summary>
		/// Reset robot to initial state
		/// </summary>
		public void ResetRobot(Vector3 position, Quaternion rotation)
		{
			if (diffDriveController?.rb != null)
			{
				diffDriveController.rb.position = position;
				diffDriveController.rb.rotation = rotation;
				diffDriveController.rb.velocity = Vector3.zero;
				diffDriveController.rb.angularVelocity = Vector3.zero;
				diffDriveController.targetPointWorld = position;
				diffDriveController.mode = DiffDriveTwinController.ControlMode.TargetPoint;
				diffDriveController.HardStopAtGoal();
			}

			RebindArmToCarMountImmediately();

			ClearTarget();
			_simulationTime = 0f;
		}

		public void ResetRobotToCapturedStartPose()
		{
			CaptureRobotStartPoseIfNeeded();
			if (!_hasCapturedStartPose)
			{
				ResetRobot(Vector3.zero, Quaternion.identity);
				return;
			}

			ResetRobot(_capturedStartPosition, _capturedStartRotation);
		}

		private void CaptureRobotStartPoseIfNeeded()
		{
			if (_hasCapturedStartPose)
			{
				return;
			}

			if (diffDriveController?.rb == null)
			{
				return;
			}

			_capturedStartPosition = diffDriveController.rb.position;
			_capturedStartRotation = diffDriveController.rb.rotation;
			_hasCapturedStartPose = true;
		}

		private void RebindArmToCarMountImmediately()
		{
			if (armBinder == null || armBinder.armRoot == null || armBinder.carMount == null)
			{
				return;
			}

			armBinder.armRoot.TeleportRoot(armBinder.carMount.position, armBinder.carMount.rotation);
			Physics.SyncTransforms();
		}

		public void SyncArmToCurrentBasePoseImmediate()
		{
			RebindArmToCarMountImmediately();
			if (arm6DOFFKController != null)
			{
				arm6DOFFKController.RefreshRuntimeState();
			}
		}

		private void LogStartupCoordinatedDiagnosticsIfNeeded()
		{
			if (_startupCoordinatedDiagnosticsLogged || arm6DOFFKController == null)
			{
				return;
			}

			string report = arm6DOFFKController.BuildCoordinatedDiagnosticReport(armBinder);
			if (string.IsNullOrWhiteSpace(report))
			{
				return;
			}

			Debug.Log(report);
			_startupCoordinatedDiagnosticsLogged = true;
		}

		private IEnumerator LogPostBindCoordinatedDiagnosticsAfterFirstFixedUpdate()
		{
			if (_postBindCoordinatedDiagnosticsLogged)
			{
				yield break;
			}

			yield return new WaitForFixedUpdate();
			SyncArmToCurrentBasePoseImmediate();
			if (arm6DOFFKController == null)
			{
				yield break;
			}

			string report = arm6DOFFKController.BuildCoordinatedDiagnosticReport(armBinder);
			if (string.IsNullOrWhiteSpace(report))
			{
				yield break;
			}

			Debug.Log(report.Replace("[Arm6DOF] Coordinated diagnostics", "[Arm6DOF] Coordinated diagnostics (post-bind)"));
			_postBindCoordinatedDiagnosticsLogged = true;
		}

		/// <summary>
		/// Get current robot configuration for saving
		/// </summary>
		public RobotConfig GetRobotConfig()
		{
			var config = new RobotConfig();

			if (diffDriveController != null)
			{
				config.controllerConfig = new DiffDriveConfig
				{
					vMax = diffDriveController.vMax,
					aMax = diffDriveController.aMax,
					wMax = diffDriveController.wMax,
					alphaMax = diffDriveController.alphaMax,
					kDist = diffDriveController.kDist,
					kYaw = diffDriveController.kYaw,
					rotateInPlaceAngleDeg = diffDriveController.rotateInPlaceAngleDeg,
					posTolerance = diffDriveController.posTolerance,
					yawToleranceDeg = diffDriveController.yawToleranceDeg,
					headingOffsetDeg = diffDriveController.headingOffsetDeg,
					alignYawAtGoal = diffDriveController.alignYawAtGoal,
					rotateAtGoal = diffDriveController.rotateAtGoal
				};
			}

			if (armJointControllers != null)
			{
				config.jointConfigs = new JointConfig[armJointControllers.Length];
				for (int i = 0; i < armJointControllers.Length; i++)
				{
					var ctrl = armJointControllers[i];
					config.jointConfigs[i] = new JointConfig
					{
						jointIndex = i,
						jointName = ctrl.joint != null ? ctrl.joint.name : $"Joint_{i}",
						vMaxDeg = ctrl.vMaxDeg,
						aMaxDeg = ctrl.aMaxDeg,
						stiffness = ctrl.stiffness,
						damping = ctrl.damping,
						forceLimit = ctrl.forceLimit
					};
				}
			}


			return config;
		}

		/// <summary>
		/// Apply robot configuration
		/// </summary>
		public void ApplyRobotConfig(RobotConfig config)
		{
			if (config.controllerConfig != null && diffDriveController != null)
			{
				var c = config.controllerConfig;
				diffDriveController.vMax = c.vMax;
				diffDriveController.aMax = c.aMax;
				diffDriveController.wMax = c.wMax;
				diffDriveController.alphaMax = c.alphaMax;
				diffDriveController.kDist = c.kDist;
				diffDriveController.kYaw = c.kYaw;
				diffDriveController.rotateInPlaceAngleDeg = c.rotateInPlaceAngleDeg;
				diffDriveController.posTolerance = c.posTolerance;
				diffDriveController.yawToleranceDeg = c.yawToleranceDeg;
				diffDriveController.headingOffsetDeg = c.headingOffsetDeg;
				diffDriveController.alignYawAtGoal = c.alignYawAtGoal;
				diffDriveController.rotateAtGoal = c.rotateAtGoal;
			}

			if (config.jointConfigs != null && armJointControllers != null)
			{
				for (int i = 0; i < Mathf.Min(config.jointConfigs.Length, armJointControllers.Length); i++)
				{
					var jc = config.jointConfigs[i];
					var ctrl = armJointControllers[i];
					ctrl.vMaxDeg = jc.vMaxDeg;
					ctrl.aMaxDeg = jc.aMaxDeg;
					ctrl.stiffness = jc.stiffness;
					ctrl.damping = jc.damping;
					ctrl.forceLimit = jc.forceLimit;
				}
			}
		}
	}

	/// <summary>
	/// Runtime robot state data
	/// </summary>
	[System.Serializable]
	public class RobotState
	{
		public Vector3 position;
		public float rotation;
		public float linearVelocity;
		public float angularVelocity;
		public float leftWheelVelocity;
		public float rightWheelVelocity;
		public Vector3 targetPoint;
		public bool hasTargetPoint;
		public float targetYaw;
		public string controlMode = "TargetPoint";
		public float[] jointAngles = new float[6];
		public Vector3 endEffectorPositionWorld;
		public Vector3 endEffectorPositionBase;
		public float[] armJointAngles = new float[6];
		public bool isRobotTaskPlanning;
		public string robotPlanningStage = "无 / None";
		public string lastPlanningSummary = string.Empty;
		public float simulationTime;
	}

	/// <summary>
	/// Robot configuration for saving/loading
	/// </summary>
	[System.Serializable]
	public class RobotConfig
	{
		public DiffDriveConfig controllerConfig;
		public JointConfig[] jointConfigs;
	}

	[System.Serializable]
	public class DiffDriveConfig
	{
		public float vMax;
		public float aMax;
		public float wMax;
		public float alphaMax;
		public float kDist;
		public float kYaw;
		public float rotateInPlaceAngleDeg;
		public float posTolerance;
		public float yawToleranceDeg;
		public float headingOffsetDeg;
		public bool alignYawAtGoal;
		public bool rotateAtGoal;
	}

	[System.Serializable]
	public class JointConfig
	{
		public int jointIndex;
		public string jointName;
		public float vMaxDeg;
		public float aMaxDeg;
		public float stiffness;
		public float damping;
		public float forceLimit;
	}
}
