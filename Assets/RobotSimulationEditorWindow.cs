using UnityEngine;
using UnityEditor;
using System;
using System.IO;
using RobotSimulation;

namespace RobotSimulation.Editor
{
	/// <summary>
	/// Unity Editor Window for Robot Simulation
	/// </summary>
	public class RobotSimulationEditorWindow : EditorWindow
	{
		private RobotSimulationManager _manager;
		private ConfigManager _configManager;
		private StatusDisplayPanel _statusPanel;
		private InteractiveControlPanel _controlPanel;

		private Vector2 _scrollPosition;
		private Vector2 _configScrollPosition;

		// Foldout states
		private bool _showRobotSettings = true;
		private bool _showControlSettings = true;
		private bool _showArmControl = true;
		private bool _showStatusDisplay = true;
		private bool _showConfigManagement = true;
		private bool _showDebugInfo = false;

		// Quick control values
		private float _quickTargetYaw = 0f;
		private Vector3 _quickTargetPoint = Vector3.zero;
		private float _simulationSpeed = 1f;

		// Arm control values
		private Vector3 _armTargetPosition = new Vector3(0.3f, 0.2f, 0.1f);
		private float[] _armJointAngles = new float[] { 0f, 0f, 0f, 0f, 0f, 0f };
		private string _ikRegressionSummary = "Not run";

		[MenuItem("Robot Simulation/Open Simulation Window")]
		public static void ShowWindow()
		{
			GetWindow<RobotSimulationEditorWindow>("Robot Simulation");
		}

		void OnEnable()
		{
			FindReferences();
		}

		void OnGUI()
		{
			if (_manager == null)
			{
				FindReferences();
			}

			_scrollPosition = EditorGUILayout.BeginScrollView(_scrollPosition);
			try
			{

				// Header
				EditorGUILayout.Space();
				EditorGUILayout.LabelField("Robot Simulation Manager", EditorStyles.boldLabel);
				EditorGUILayout.Space();

				// Robot Setup Section
				_showRobotSettings = EditorGUILayout.Foldout(_showRobotSettings, "Robot Setup");
				if (_showRobotSettings)
				{
					EditorGUI.indentLevel++;
					_manager = (RobotSimulationManager)EditorGUILayout.ObjectField(
						"Manager", _manager, typeof(RobotSimulationManager), true);

					if (GUILayout.Button("Auto-Find Robot Components"))
					{
						AutoFindRobot();
					}

					if (_manager != null)
					{
						EditorGUILayout.LabelField("Controller Status:");
						EditorGUILayout.LabelField($"  DiffDrive: {(_manager.diffDriveController != null ? "Found" : "Not Found")}");
						EditorGUILayout.LabelField($"  Arm Joints: {_manager.armJointControllers?.Length ?? 0}");
						EditorGUILayout.LabelField($"  Arm Binder: {(_manager.armBinder != null ? "Found" : "Not Found")}");
					}
					EditorGUI.indentLevel--;
					EditorGUILayout.Space();
				}

				// Control Section
				_showControlSettings = EditorGUILayout.Foldout(_showControlSettings, "Quick Controls");
				if (_showControlSettings)
				{
					EditorGUI.indentLevel++;

					_simulationSpeed = EditorGUILayout.Slider("Simulation Speed", _simulationSpeed, 0.1f, 5f);
					if (_manager != null)
					{
						_manager.simulationSpeed = _simulationSpeed;
					}

					EditorGUILayout.Space();

					// Target Yaw Control
					EditorGUILayout.LabelField("Target Yaw Control", EditorStyles.miniBoldLabel);
					_quickTargetYaw = EditorGUILayout.Slider("Yaw Angle", _quickTargetYaw, -180f, 180f);

					EditorGUILayout.BeginHorizontal();
					if (GUILayout.Button("Set Target Yaw"))
					{
						if (_manager != null)
						{
							_manager.SetTargetYaw(_quickTargetYaw);
						}
					}
					if (GUILayout.Button("Clear Target"))
					{
						if (_manager != null)
						{
							_manager.ClearTarget();
						}
					}
					EditorGUILayout.EndHorizontal();

					EditorGUILayout.Space();

					// Target Point Control
					EditorGUILayout.LabelField("Target Point Control", EditorStyles.miniBoldLabel);
					_quickTargetPoint = EditorGUILayout.Vector3Field("Position", _quickTargetPoint);

					EditorGUILayout.BeginHorizontal();
					if (GUILayout.Button("Set Target Point"))
					{
						if (_manager != null)
						{
							_manager.SetTargetPoint(_quickTargetPoint);
						}
					}
					if (GUILayout.Button("Reset Robot"))
					{
						if (_manager != null)
						{
							_manager.ResetRobot(Vector3.zero, Quaternion.identity);
						}
					}
					EditorGUILayout.EndHorizontal();

					EditorGUILayout.Space();

					// Emergency Control
					EditorGUILayout.BeginHorizontal();
					if (GUILayout.Button("EMERGENCY STOP", EditorStyles.miniButtonMid))
					{
						if (_manager != null)
						{
							_manager.EmergencyStop();
						}
					}
					EditorGUILayout.EndHorizontal();

					EditorGUI.indentLevel--;
					EditorGUILayout.Space();
				}

				// 6-DOF Arm Control Section
				_showArmControl = EditorGUILayout.Foldout(_showArmControl, "6-DOF Arm Control");
				if (_showArmControl)
				{
					EditorGUI.indentLevel++;

					if (_manager?.arm6DOFFKController == null)
					{
						EditorGUILayout.LabelField("6-DOF Arm Controller: Not Found");

						// Show existing joint controllers status
						if (_manager?.armJointControllers != null && _manager.armJointControllers.Length > 0)
						{
							EditorGUILayout.LabelField($"Found {_manager.armJointControllers.Length} joint controllers:");
							for (int i = 0; i < _manager.armJointControllers.Length; i++)
							{
								var ctrl = _manager.armJointControllers[i];
								EditorGUILayout.LabelField($"  Joint {i}: {GetControllerJointDisplayName(ctrl)}");
							}
						}

						EditorGUILayout.Space();
						if (GUILayout.Button("Create Arm Controller"))
						{
							CreateArmController();
						}
					}
					else
					{
						bool isInit = _manager.arm6DOFFKController.IsInitialized;
						EditorGUILayout.LabelField($"Status: {(isInit ? "Initialized" : "Not Initialized")}");

						EditorGUILayout.Space();

						// IK Position Control
						EditorGUILayout.LabelField("IK Position Control", EditorStyles.miniBoldLabel);
						_armTargetPosition = EditorGUILayout.Vector3Field("Target Position", _armTargetPosition);

						EditorGUILayout.BeginHorizontal();
						if (GUILayout.Button("Move to Position"))
						{
							if (_manager != null)
							{
								bool success = _manager.MoveArmToPosition(_armTargetPosition);
								if (!success)
								{
									Debug.LogWarning("IK: Failed to reach target position!");
								}
							}
						}
						if (GUILayout.Button("Check Reachability"))
						{
							if (_manager != null)
							{
								bool reachable = _manager.IsArmPositionReachable(_armTargetPosition);
								Debug.Log($"Position {_armTargetPosition} is {(reachable ? "REACHABLE" : "OUT OF REACH")}");
							}
						}
						EditorGUILayout.EndHorizontal();

						EditorGUILayout.Space();

						// Current End Effector Position
						if (_manager.RobotState != null)
						{
							var eePos = _manager.RobotState.endEffectorPosition;
							EditorGUILayout.LabelField($"Current EE: ({eePos.x:F3}, {eePos.y:F3}, {eePos.z:F3})");
						}

						EditorGUILayout.Space();

						// Joint Angle Control
						EditorGUILayout.LabelField("Joint Angle Control", EditorStyles.miniBoldLabel);

						// Get actual joint names from armJointControllers
						string[] jointNames = new string[6];
						for (int i = 0; i < 6; i++)
						{
							jointNames[i] = $"J{i + 1}";
						}

						// If we have armJointControllers, show actual names
						if (_manager?.armJointControllers != null)
						{
							for (int i = 0; i < Mathf.Min(6, _manager.armJointControllers.Length); i++)
							{
								var ctrl = _manager.armJointControllers[i];
								if (ctrl != null && ctrl.joint != null)
								{
									jointNames[i] = ctrl.joint.name;
								}
							}
						}

						for (int i = 0; i < 6; i++)
						{
							EditorGUILayout.BeginHorizontal();
							_armJointAngles[i] = EditorGUILayout.Slider($"{jointNames[i]}:", _armJointAngles[i], -180f, 180f);
							if (GUILayout.Button("Set", GUILayout.Width(50)))
							{
								if (_manager != null)
								{
									_manager.SetArmJointTarget(i, _armJointAngles[i]);
								}
							}
							EditorGUILayout.EndHorizontal();
						}

						EditorGUILayout.Space();

						// Utility Buttons
						EditorGUILayout.BeginHorizontal();
						if (GUILayout.Button("Go Home"))
						{
							if (_manager != null)
							{
								_manager.MoveArmHome();
							}
						}
						if (GUILayout.Button("Arm Stop"))
						{
							if (_manager?.arm6DOFFKController != null)
							{
								_manager.arm6DOFFKController.EmergencyStop();
							}
						}
						EditorGUILayout.EndHorizontal();

						EditorGUILayout.Space();
						EditorGUILayout.BeginHorizontal();
						if (GUILayout.Button("Freeze Binding"))
						{
							if (_manager != null)
							{
								bool frozen = _manager.FreezeArmBindingConfiguration();
								Debug.Log($"[RobotSimulationEditor] Freeze binding result: {frozen}");
							}
						}
						if (GUILayout.Button("Run IK Regression"))
						{
							if (_manager != null)
							{
								_manager.RunIKRegressionTest(summary =>
								{
									_ikRegressionSummary = summary;
									Repaint();
								});
							}
						}
						EditorGUILayout.EndHorizontal();
						EditorGUILayout.LabelField($"IK Regression: {_ikRegressionSummary}");

						// Joint Status
						if (_manager.RobotState?.armJointAngles != null)
						{
							EditorGUILayout.Space();
							EditorGUILayout.LabelField("Current Joint Angles:", EditorStyles.miniBoldLabel);
							var angles = _manager.RobotState.armJointAngles;
							for (int i = 0; i < Mathf.Min(6, angles.Length); i++)
							{
								EditorGUILayout.LabelField($"  {jointNames[i]}: {angles[i]:F1}");
							}
						}
					}

					EditorGUI.indentLevel--;
					EditorGUILayout.Space();
				}

				// Status Display Section
				_showStatusDisplay = EditorGUILayout.Foldout(_showStatusDisplay, "Status Display");
				if (_showStatusDisplay)
				{
					EditorGUI.indentLevel++;
					_statusPanel = (StatusDisplayPanel)EditorGUILayout.ObjectField(
						"Status Panel", _statusPanel, typeof(StatusDisplayPanel), true);

					if (_manager != null && _manager.RobotState != null)
					{
						var state = _manager.RobotState;
						EditorGUILayout.LabelField("Current State:");
						EditorGUILayout.LabelField($"  Position: ({state.position.x:F2}, {state.position.z:F2})");
						EditorGUILayout.LabelField($"  Rotation: {state.rotation:F1}°");
						EditorGUILayout.LabelField($"  Velocity: {state.linearVelocity:F3} m/s");
						EditorGUILayout.LabelField($"  Angular: {state.angularVelocity:F3} rad/s");
						EditorGUILayout.LabelField($"  Mode: {state.controlMode}");
						EditorGUILayout.LabelField($"  Sim Time: {state.simulationTime:F1}s");
					}
					EditorGUI.indentLevel--;
					EditorGUILayout.Space();
				}

				// Configuration Management Section
				_showConfigManagement = EditorGUILayout.Foldout(_showConfigManagement, "Configuration");
				if (_showConfigManagement)
				{
					EditorGUI.indentLevel++;
					_configManager = (ConfigManager)EditorGUILayout.ObjectField(
						"Config Manager", _configManager, typeof(ConfigManager), true);

					if (_configManager == null)
					{
						if (GUILayout.Button("Create Config Manager"))
						{
							CreateConfigManager();
						}
					}
					else
					{
						EditorGUILayout.LabelField("Quick Save/Load:", EditorStyles.miniBoldLabel);
						EditorGUILayout.BeginHorizontal();
						if (GUILayout.Button("Quick Save"))
						{
							_configManager.QuickSave();
						}
						if (GUILayout.Button("Quick Load"))
						{
							_configManager.QuickLoad();
						}
						EditorGUILayout.EndHorizontal();

						EditorGUILayout.Space();

						EditorGUILayout.LabelField("Export/Import:", EditorStyles.miniBoldLabel);
						EditorGUILayout.BeginHorizontal();
						if (GUILayout.Button("Export"))
						{
							_configManager.ExportConfig();
						}
						if (GUILayout.Button("Import"))
						{
							_configManager.ImportConfig();
						}
						EditorGUILayout.EndHorizontal();

						EditorGUILayout.Space();

						EditorGUILayout.LabelField($"Config Directory: {_configManager.ConfigFolderPath}");
					}
					EditorGUI.indentLevel--;
					EditorGUILayout.Space();
				}

				// Debug Section
				_showDebugInfo = EditorGUILayout.Foldout(_showDebugInfo, "Debug Info");
				if (_showDebugInfo)
				{
					EditorGUI.indentLevel++;
					if (_manager != null)
					{
						_manager.showDebugInfo = EditorGUILayout.Toggle("Show Debug Info", _manager.showDebugInfo);

						if (_manager.diffDriveController != null)
						{
							EditorGUILayout.Space();
							EditorGUILayout.LabelField("DiffDrive Controller:", EditorStyles.miniBoldLabel);
							EditorGUILayout.LabelField($"  Track Width: {(_manager.diffDriveController as DiffDriveTwinController)?.TrackWidth:F4} m");
							EditorGUILayout.LabelField($"  Wheel Radius: {(_manager.diffDriveController as DiffDriveTwinController)?.WheelRadius:F4} m");
							EditorGUILayout.LabelField($"  vMax: {(_manager.diffDriveController as DiffDriveTwinController)?.vMax:F2}");
							EditorGUILayout.LabelField($"  wMax: {(_manager.diffDriveController as DiffDriveTwinController)?.wMax:F2}");
						}
					}
					EditorGUI.indentLevel--;
				}

			}
			catch (Exception ex)
			{
				Debug.LogError($"[RobotSimulationEditorWindow] OnGUI exception: {ex.Message}\n{ex.StackTrace}");
			}
			finally
			{
				EditorGUILayout.EndScrollView();
			}

			// Footer with scene buttons
			EditorGUILayout.Space();
			EditorGUILayout.BeginHorizontal();
			if (GUILayout.Button("Refresh"))
			{
				FindReferences();
			}
			EditorGUILayout.EndHorizontal();
		}

		private string GetControllerJointDisplayName(OneJointTrapezoidController ctrl)
		{
			if (ctrl == null)
			{
				return "controller=null";
			}

			if (ctrl.joint == null)
			{
				return "joint=unassigned";
			}

			return ctrl.joint.name;
		}

		private void FindReferences()
		{
			if (_manager == null)
			{
				_manager = FindObjectOfType<RobotSimulationManager>();
			}

			if (_configManager == null)
			{
				_configManager = FindObjectOfType<ConfigManager>();
			}

			if (_statusPanel == null)
			{
				_statusPanel = FindObjectOfType<StatusDisplayPanel>();
			}

			if (_controlPanel == null)
			{
				_controlPanel = FindObjectOfType<InteractiveControlPanel>();
			}
		}

		private void AutoFindRobot()
		{
			if (_manager == null)
			{
				_manager = FindObjectOfType<RobotSimulationManager>();
				if (_manager == null)
				{
					GameObject go = new GameObject("RobotSimulationManager");
					_manager = go.AddComponent<RobotSimulationManager>();
				}
			}

			_manager.InitializeRobot();
		}

		private void CreateConfigManager()
		{
			if (_configManager == null)
			{
				GameObject go = new GameObject("ConfigManager");
				_configManager = go.AddComponent<ConfigManager>();
				_configManager.robotManager = _manager;
			}
		}

		private void CreateArmController()
		{
			// Create a new GameObject for the arm controller
			GameObject armGo = new GameObject("Arm6DOFController");

			// Add Arm6DOFFKController
			var fkController = armGo.AddComponent<Arm6DOFFKController>();

			// Add Arm6DOFIKController
			var ikController = armGo.AddComponent<Arm6DOFIKController>();

			// Link IK to FK controller
			ikController.armController = fkController;

			// Try to find existing joint controllers and link them
			if (_manager != null && _manager.armJointControllers != null && _manager.armJointControllers.Length >= 6)
			{
				fkController.joints = new ArticulationBody[6];
				fkController.jointTransforms = new Transform[6];

				// Build expected joint names from actual joint names
				string[] actualJointNames = new string[6];
				for (int i = 0; i < 6; i++)
				{
					if (_manager.armJointControllers[i] != null && _manager.armJointControllers[i].joint != null)
					{
						fkController.joints[i] = _manager.armJointControllers[i].joint;
						fkController.jointTransforms[i] = _manager.armJointControllers[i].joint.transform;
						actualJointNames[i] = _manager.armJointControllers[i].joint.name;

						if (fkController.armHierarchyRoot == null)
						{
							fkController.armHierarchyRoot = _manager.armJointControllers[i].joint.transform.root;
						}
					}
				}

				// Keep auto binding and rebind enabled so FK can correct mapping mismatches.
				fkController.autoBindByJointName = true;
				fkController.autoRebindOnMismatch = true;

				// Debug: log each joint binding
				for (int i = 0; i < 6; i++)
				{
					if (fkController.joints[i] != null)
						Debug.Log($"[RobotSimulationEditor] Bound J{i}: {fkController.joints[i].name}");
					else
						Debug.LogWarning($"[RobotSimulationEditor] J{i} is NULL!");
				}

				// Try to find end effector (last joint's child or last transform)
				if (fkController.jointTransforms[5] != null)
				{
					fkController.endEffector = fkController.jointTransforms[5];
				}
			}

			// Update manager references
			_manager.arm6DOFFKController = fkController;
			_manager.arm6DOFIKController = ikController;

			// Initialize
			fkController.InitializeJoints();

			Debug.Log("[RobotSimulationEditor] Created Arm6DOFFKController and Arm6DOFIKController");
		}
	}

}
