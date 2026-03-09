using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using TMPro;
using System.IO;
using System.Text.Json;

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

		[Header("Robot State")]
		[SerializeField] private RobotState _robotState;
		public RobotState RobotState => _robotState;

		[Header("Simulation Settings")]
		public bool enableSimulation = true;
		public float simulationSpeed = 1.0f;
		public bool showDebugInfo = false;

		[Header("UI References")]
		public GameObject controlPanel;
		public GameObject statusPanel;
		public TextMeshProUGUI statusText;
		public TextMeshProUGUI debugText;

		private float _simulationTime;
		private bool _isInitialized = false;

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
		}

		void Start()
		{
			InitializeRobot();
			SetupUI();
		}

		void Update()
		{
			if (!enableSimulation || !_isInitialized) return;

			Time.timeScale = simulationSpeed;
			_simulationTime += Time.deltaTime * simulationSpeed;

			UpdateRobotState();
			UpdateUI();
		}

		void FixedUpdate()
		{
			if (!enableSimulation || !_isInitialized) return;
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

			// Initialize 6-DOF arm if found
			if (arm6DOFFKController != null)
			{
				arm6DOFFKController.InitializeJoints();
			}

			// Validate initialization
			if (diffDriveController == null)
			{
				Debug.LogWarning("[RobotSimulation] DiffDrive controller not found!");
			}

			_isInitialized = diffDriveController != null;
			Debug.Log($"[RobotSimulation] Initialized: {_isInitialized}");
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
				_robotState.linearVelocity = diffDriveController.CurrentLinearVelocity;
				_robotState.angularVelocity = diffDriveController.CurrentAngularVelocity;
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
				_robotState.endEffectorPosition = arm6DOFFKController.EndEffectorPosition;
				_robotState.armJointAngles = arm6DOFFKController.CurrentJointAngles;
			}

			_robotState.simulationTime = _simulationTime;
		}

		/// <summary>
		/// Update UI displays
		/// </summary>
		private void UpdateUI()
		{
			if (statusText != null)
			{
				statusText.text = FormatStatusText();
			}

			if (debugText != null && showDebugInfo)
			{
				debugText.text = FormatDebugText();
			}
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
						$"X: {_robotState.endEffectorPosition.x:F3}\n" +
						$"Y: {_robotState.endEffectorPosition.y:F3}\n" +
						$"Z: {_robotState.endEffectorPosition.z:F3}\n";
			}

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
				diffDriveController.targetPointWorld = point;
				diffDriveController.hasTargetPoint = true;
				diffDriveController.mode = DiffDriveTwinController.ControlMode.TargetPoint;
			}
		}

		/// <summary>
		/// Set target yaw angle
		/// </summary>
		public void SetTargetYaw(float yawDegrees)
		{
			if (diffDriveController != null)
			{
				diffDriveController.targetYawDeg = yawDegrees;
				diffDriveController.mode = DiffDriveTwinController.ControlMode.TargetYaw;
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
			// First try to use armJointControllers (OneJointTrapezoidController) directly
			if (armJointControllers != null && jointIndex >= 0 && jointIndex < armJointControllers.Length)
			{
				var ctrl = armJointControllers[jointIndex];
				if (ctrl != null)
				{
					ctrl.goalDeg = angleDegrees;
					Debug.Log($"[RobotSimulation] Set Joint {jointIndex} to {angleDegrees}");
				}
			}

			// Also update Arm6DOFFKController if it exists
			if (arm6DOFFKController != null)
			{
				arm6DOFFKController.SetJointTarget(jointIndex, angleDegrees);
			}
		}

		/// <summary>
		/// Move end effector to target position using IK
		/// Returns true if successful, false if unreachable (and logs error)
		/// </summary>
		public bool MoveArmToPosition(Vector3 position)
		{
			if (arm6DOFIKController == null)
			{
				Debug.LogError("[RobotSimulation] IK Controller not found!");
				return false;
			}

			return arm6DOFIKController.MoveToPosition(position);
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

			for (int i = 0; i < 6; i++)
			{
				if (armJointControllers[i] == null || armJointControllers[i].joint == null)
				{
					Debug.LogError($"[RobotSimulation] Cannot freeze arm binding: armJointControllers[{i}] has no assigned joint.");
					return false;
				}

				arm6DOFFKController.joints[i] = armJointControllers[i].joint;
				arm6DOFFKController.jointTransforms[i] = armJointControllers[i].joint.transform;
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

		/// <summary>
		/// Check if position is reachable
		/// </summary>
		public bool IsArmPositionReachable(Vector3 position)
		{
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
			if (arm6DOFFKController != null)
			{
				arm6DOFFKController.GoHome();
			}
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
			}
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
			}

			ClearTarget();
			_simulationTime = 0f;
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
		public Vector3 endEffectorPosition;
		public float[] armJointAngles = new float[6];
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
