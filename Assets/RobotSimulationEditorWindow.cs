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
		private bool _showTaskPlanning = true;
		private bool _showStatusDisplay = true;
		private bool _showConfigManagement = true;
		private bool _showDebugInfo = false;

		// Quick control values
		private float _quickTargetYaw = 0f;
		private Vector3 _quickTargetPoint = Vector3.zero;
		private float _simulationSpeed = 1f;

		// Arm control values
		private Vector3 _armTargetPosition = new Vector3(0.3f, 0.2f, 0.1f);
		private float _armPositionTolerance = 0.015f;
		private int _armStableFrames = 3;
		private float _armTimeoutSeconds = 4f;
		private float _armMoveSpeedScale = 1f;
		private float[] _armJointAngles = Arm6DOFFKController.CreateDefaultConfiguredHomeJointAnglesDeg();
		private string _armMoveSummary = "No move executed";
		private bool _moveAndWaitButtonPressedLastGui;
		private bool _moveAndWaitHoldActive;
		private bool _planAllowReplan = true;
		private float _planTimeoutSeconds = 12f;
		private float _planBaseSamplingStep = 0.7f;
		private float _planDistanceFieldResolution = 0.4f;
		private float _planArmSampleDensityDeg = 4f;
		private float _planShadowSpeedScale = 4f;
		private string _planningSummary = "未执行规划 / No task planned";

		[MenuItem("Robot Simulation/Open Simulation Window")]
		public static void ShowWindow()
		{
			GetWindow<RobotSimulationEditorWindow>("Robot Simulation");
		}

		void OnEnable()
		{
			FindReferences();
		}

		void OnDisable()
		{
			StopHeldMoveAndWait(updateSummary: false);
			_moveAndWaitButtonPressedLastGui = false;
		}

		void OnInspectorUpdate()
		{
			if (_manager != null && _manager.IsArmWorldCoordinateMotionLocked && !string.IsNullOrEmpty(_manager.ArmWorldCoordinateMotionLockReason))
			{
				_armMoveSummary = _manager.ArmWorldCoordinateMotionLockReason;
			}

			if (Application.isPlaying)
			{
				Repaint();
			}
		}

		void OnGUI()
		{
			if (_manager == null)
			{
				FindReferences();
			}

			if (_manager != null)
			{
				RobotSimulationLocalization.SetLanguage(_manager.displayLanguage);
			}

			_scrollPosition = EditorGUILayout.BeginScrollView(_scrollPosition);
			try
			{

				// Header
				EditorGUILayout.Space();
				EditorGUILayout.LabelField(T("机器人仿真管理器", "Robot Simulation Manager"), EditorStyles.boldLabel);
				if (_manager != null)
				{
					_manager.displayLanguage = (RobotSimulationLanguage)EditorGUILayout.EnumPopup(T("界面语言", "UI Language"), _manager.displayLanguage);
					RobotSimulationLocalization.SetLanguage(_manager.displayLanguage);
				}
				EditorGUILayout.Space();

				// Robot Setup Section
				_showRobotSettings = EditorGUILayout.Foldout(_showRobotSettings, T("机器人装配", "Robot Setup"));
				if (_showRobotSettings)
				{
					EditorGUI.indentLevel++;
					_manager = (RobotSimulationManager)EditorGUILayout.ObjectField(
						T("管理器", "Manager"), _manager, typeof(RobotSimulationManager), true);

					if (GUILayout.Button(T("自动查找机器人组件", "Auto-Find Robot Components")))
					{
						AutoFindRobot();
					}

					if (GUILayout.Button(T("生成方形障碍测试场景", "Create Square Obstacle Test Scene")))
					{
						string scenePath = SquareObstacleTestSceneBuilder.CreateAndOpenScene(true);
						if (!string.IsNullOrEmpty(scenePath))
						{
							Debug.Log($"[RobotSimulationEditor] Created test scene: {scenePath}");
							FindReferences();
						}
					}

					if (_manager != null)
					{
						EditorGUILayout.LabelField(T("控制器状态：", "Controller Status:"));
						EditorGUILayout.LabelField($"  DiffDrive: {RobotSimulationLocalization.Found(_manager.diffDriveController != null)}");
						EditorGUILayout.LabelField($"  {T("机械臂关节数", "Arm Joints")}: {_manager.armJointControllers?.Length ?? 0}");
						EditorGUILayout.LabelField($"  {T("机械臂绑定器", "Arm Binder")}: {RobotSimulationLocalization.Found(_manager.armBinder != null)}");
					}
					EditorGUI.indentLevel--;
					EditorGUILayout.Space();
				}

				// Control Section
				_showControlSettings = EditorGUILayout.Foldout(_showControlSettings, T("快速控制", "Quick Controls"));
				if (_showControlSettings)
				{
					EditorGUI.indentLevel++;

					_simulationSpeed = EditorGUILayout.Slider(T("仿真速度", "Simulation Speed"), _simulationSpeed, 0.1f, 5f);
					if (_manager != null)
					{
						_manager.simulationSpeed = _simulationSpeed;
					}

					EditorGUILayout.Space();

					float currentBaseYaw = _manager != null && _manager.diffDriveController != null && _manager.diffDriveController.rb != null
						? _manager.diffDriveController.rb.rotation.eulerAngles.y
						: 0f;
					EditorGUILayout.LabelField(T("底盘朝向（世界偏航角）", "Base Heading (World Yaw)"), EditorStyles.miniBoldLabel);
					EditorGUILayout.HelpBox(
						T("这里控制的是同一个全局世界坐标系下的底盘偏航角，只改变车体朝向，不改变车体参考点位置。",
							"This control uses the same shared global world frame and changes only the base yaw, not the base reference-point position."),
						MessageType.Info);
					EditorGUILayout.LabelField($"{T("当前底盘朝向（世界偏航角）", "Current Base Heading (World Yaw)")}: {currentBaseYaw:F1}°");
					_quickTargetYaw = EditorGUILayout.Slider(T("目标偏航角", "Target Yaw"), _quickTargetYaw, -180f, 180f);

					EditorGUILayout.BeginHorizontal();
					if (GUILayout.Button(T("发送底盘朝向", "Send Base Heading")))
					{
						if (_manager != null)
						{
							_manager.SetTargetYaw(_quickTargetYaw);
						}
					}
					if (GUILayout.Button(T("使用当前朝向", "Use Current Heading")))
					{
						_quickTargetYaw = currentBaseYaw;
					}
					EditorGUILayout.EndHorizontal();

					EditorGUILayout.BeginHorizontal();
					if (GUILayout.Button(T("停止朝向控制", "Stop Heading Control")))
					{
						if (_manager != null)
						{
							_manager.ClearTarget();
						}
					}
					if (GUILayout.Button(T("清零到 0°", "Zero To 0°")))
					{
						_quickTargetYaw = 0f;
					}
					EditorGUILayout.EndHorizontal();

					#pragma warning disable 162
					if (false)
					{
					// Target Yaw Control
					EditorGUILayout.LabelField(T("目标偏航控制", "Target Yaw Control"), EditorStyles.miniBoldLabel);
					_quickTargetYaw = EditorGUILayout.Slider(T("偏航角", "Yaw Angle"), _quickTargetYaw, -180f, 180f);

					EditorGUILayout.BeginHorizontal();
					if (GUILayout.Button(T("设置目标偏航", "Set Target Yaw")))
					{
						if (_manager != null)
						{
							_manager.SetTargetYaw(_quickTargetYaw);
						}
					}
					if (GUILayout.Button(T("清除目标", "Clear Target")))
					{
						if (_manager != null)
						{
							_manager.ClearTarget();
						}
					}
					EditorGUILayout.EndHorizontal();
					}
					#pragma warning restore 162

					EditorGUILayout.Space();

					// Base target point control
					EditorGUILayout.LabelField(T("底盘目标点（世界车体参考点）", "Base Target (World Base Ref)"), EditorStyles.miniBoldLabel);
					Vector3 currentBaseTargetRef = _manager != null && _manager.diffDriveController != null && _manager.diffDriveController.rb != null
						? _manager.diffDriveController.rb.position
						: Vector3.zero;
					_quickTargetPoint.y = currentBaseTargetRef.y;
					EditorGUILayout.HelpBox(
						T("底盘目标与机械臂目标使用同一个全局世界坐标系，但这里控制的是车体参考点。底盘只在 XZ 平面移动，Y 会自动跟随当前底盘高度。",
							"The base target and arm target share the same global world frame, but this control drives the base reference point. The base moves on the XZ plane only, and Y follows the current base height automatically."),
						MessageType.Info);
					EditorGUILayout.LabelField($"{T("当前底盘（世界车体参考点）", "Current Base (World Base Ref)")}: ({currentBaseTargetRef.x:F3}, {currentBaseTargetRef.y:F3}, {currentBaseTargetRef.z:F3})");
					_quickTargetPoint.x = EditorGUILayout.FloatField(T("目标 X", "Target X"), _quickTargetPoint.x);
					_quickTargetPoint.z = EditorGUILayout.FloatField(T("目标 Z", "Target Z"), _quickTargetPoint.z);
					EditorGUILayout.LabelField($"{T("目标 Y（自动）", "Target Y (Auto)")}: {_quickTargetPoint.y:F3}");

					EditorGUILayout.BeginHorizontal();
					if (GUILayout.Button(T("发送底盘目标", "Send Base Target")))
					{
						if (_manager != null)
						{
							_manager.SetTargetPoint(_quickTargetPoint);
						}
					}
					if (GUILayout.Button(T("使用当前底盘", "Use Current Base")))
					{
						_quickTargetPoint = currentBaseTargetRef;
					}
					EditorGUILayout.EndHorizontal();

					EditorGUILayout.BeginHorizontal();
					if (GUILayout.Button(T("停止底盘", "Stop Base")))
					{
						if (_manager != null)
						{
							_manager.ClearTarget();
						}
					}
					if (GUILayout.Button(T("重置机器人", "Reset Robot")))
					{
						if (_manager != null)
						{
							_manager.ResetRobotToCapturedStartPose();
						}
					}
					EditorGUILayout.EndHorizontal();

					EditorGUILayout.Space();

					// Emergency Control
					EditorGUILayout.BeginHorizontal();
					if (GUILayout.Button(T("紧急停止", "EMERGENCY STOP"), EditorStyles.miniButtonMid))
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
				_showArmControl = EditorGUILayout.Foldout(_showArmControl, T("六轴机械臂控制", "6-DOF Arm Control"));
				if (_showArmControl)
				{
					EditorGUI.indentLevel++;

					if (_manager?.arm6DOFFKController == null)
					{
						EditorGUILayout.LabelField(T("六轴机械臂控制器：未找到", "6-DOF Arm Controller: Not Found"));

						// Show existing joint controllers status
						if (_manager?.armJointControllers != null && _manager.armJointControllers.Length > 0)
						{
							EditorGUILayout.LabelField(string.Format(T("已找到 {0} 个关节控制器：", "Found {0} joint controllers:"), _manager.armJointControllers.Length));
							for (int i = 0; i < _manager.armJointControllers.Length; i++)
							{
								var ctrl = _manager.armJointControllers[i];
								EditorGUILayout.LabelField($"  {T("关节", "Joint")} {i}: {GetControllerJointDisplayName(ctrl)}");
							}
						}

						EditorGUILayout.Space();
						if (GUILayout.Button(T("创建机械臂控制器", "Create Arm Controller")))
						{
							CreateArmController();
						}
					}
					else
					{
						bool isInit = _manager.arm6DOFFKController.IsInitialized;
						EditorGUILayout.LabelField($"{T("状态", "Status")}: {(isInit ? T("已初始化", "Initialized") : T("未初始化", "Not Initialized"))}");
						EditorGUILayout.LabelField($"{T("运动学", "Kinematics")}: {(_manager.arm6DOFFKController.KinematicsReady ? T("就绪", "Ready") : T("缺少 URDF/未就绪", "Missing URDF/Not Ready"))}");
						EditorGUILayout.LabelField($"{T("播放模式", "Play Mode")}: {(Application.isPlaying ? T("是", "Yes") : T("否", "No"))}");

						EditorGUILayout.Space();

						// IK Position Control
						EditorGUILayout.LabelField(T("世界坐标控制", "World Position Control"), EditorStyles.miniBoldLabel);
						_armTargetPosition = EditorGUILayout.Vector3Field(T("目标世界坐标", "Target World Position"), _armTargetPosition);
						_armPositionTolerance = EditorGUILayout.FloatField(T("位置容差", "Position Tolerance"), _armPositionTolerance);
						_armStableFrames = EditorGUILayout.IntField(T("稳定帧数", "Stable Frames"), _armStableFrames);
						_armTimeoutSeconds = EditorGUILayout.FloatField(T("超时时间", "Timeout Seconds"), _armTimeoutSeconds);
						_armMoveSpeedScale = EditorGUILayout.Slider(T("Move And Wait 速度", "Move And Wait Speed"), _armMoveSpeedScale, 0.1f, 3f);
						EditorGUILayout.LabelField(T("Move To World Position：只发起一次 IK，不等待是否真正到位。", "Move To World Position: only starts IK once, does not wait for arrival."));
						EditorGUILayout.LabelField(T("Move And Wait：持续驱动末端到目标，并监控到位/碰撞/超时。", "Move And Wait: keeps driving toward the target and monitors arrival/collision/timeout."));
						if (_manager.arm6DOFFKController != null && _manager.arm6DOFFKController.KinematicsReady)
						{
							Vector3 targetBase = _manager.arm6DOFFKController.WorldToBasePosition(_armTargetPosition);
							EditorGUILayout.LabelField($"{T("目标基座坐标", "Target Base Position")}: ({targetBase.x:F3}, {targetBase.y:F3}, {targetBase.z:F3})");
						}

						bool worldCoordinateButtonsLocked = _manager != null && _manager.IsArmWorldCoordinateMotionLocked;
						if (worldCoordinateButtonsLocked)
						{
							EditorGUILayout.HelpBox(
								string.IsNullOrEmpty(_manager.ArmWorldCoordinateMotionLockReason)
									? T("世界坐标移动已锁定，需先回到 Home。", "World-coordinate arm motion is locked until the arm returns to Home.")
									: _manager.ArmWorldCoordinateMotionLockReason,
								MessageType.Warning);
						}

						EditorGUILayout.BeginHorizontal();
						using (new EditorGUI.DisabledScope(_manager == null || worldCoordinateButtonsLocked))
						{
							if (GUILayout.Button(T("移动到世界坐标", "Move To World Position")))
							{
								if (_manager != null)
								{
									bool success = _manager.MoveArmToPosition(_armTargetPosition);
									if (!success)
									{
										_armMoveSummary = _manager.LastArmCollisionGuardResult != null && _manager.LastArmCollisionGuardResult.blockedByForbiddenCollision
											? _manager.LastArmCollisionGuardResult.message
											: _manager.LastArmMoveResult != null && !string.IsNullOrEmpty(_manager.LastArmMoveResult.summary)
												? _manager.LastArmMoveResult.summary
												: T("IK：到达目标位置失败。", "IK: Failed to reach target position!");
										Debug.LogWarning(_armMoveSummary);
									}
									else
									{
										_armMoveSummary = T("移动命令已接受。", "Move command accepted.");
									}
								}
							}
						}
						using (new EditorGUI.DisabledScope(!Application.isPlaying || _manager == null || !_manager.arm6DOFFKController.KinematicsReady || worldCoordinateButtonsLocked))
						{
							bool moveAndWaitPressed = GUILayout.RepeatButton(_moveAndWaitHoldActive ? T("Move And Wait（松开停止）", "Move And Wait (Release To Stop)") : T("Move And Wait（按住运行）", "Move And Wait (Hold)"));
							HandleMoveAndWaitHoldButton(moveAndWaitPressed);
						}
						if (GUILayout.Button(T("检查可达性", "Check Reachability")))
						{
							if (_manager != null)
							{
								bool reachable = _manager.IsArmPositionReachable(_armTargetPosition);
								Debug.Log(T($"位置 {_armTargetPosition} {(reachable ? "可达" : "不可达")}", $"Position {_armTargetPosition} is {(reachable ? "REACHABLE" : "OUT OF REACH")}"));
							}
						}
						EditorGUILayout.EndHorizontal();
						EditorGUILayout.LabelField($"{T("最近一次移动", "Last Move")}: {_armMoveSummary}");

						EditorGUILayout.Space();

						// Current End Effector Position
						if (_manager.RobotState != null)
						{
							Vector3 eeWorld = _manager.RobotState.endEffectorPositionWorld;
							Vector3 eeBase = _manager.RobotState.endEffectorPositionBase;
							EditorGUILayout.LabelField($"{T("机械臂目标（世界末端点）", "Arm Target (World EE)")}: ({_armTargetPosition.x:F3}, {_armTargetPosition.y:F3}, {_armTargetPosition.z:F3})");
							EditorGUILayout.LabelField(T("使用同一个全局世界坐标系，这里控制的是末端执行器点，不是车体参考点。", "Uses the shared global world frame. This field controls the end-effector point, not the base reference point."));
							EditorGUILayout.LabelField($"{T("当前末端（世界末端点）", "Current EE (World EE)")}: ({eeWorld.x:F3}, {eeWorld.y:F3}, {eeWorld.z:F3})");
							EditorGUILayout.LabelField($"{T("当前末端（基座）", "Current EE (Base)")}: ({eeBase.x:F3}, {eeBase.y:F3}, {eeBase.z:F3})");
						}

						EditorGUILayout.Space();

						// Joint Angle Control
						EditorGUILayout.LabelField(T("关节角控制", "Joint Angle Control"), EditorStyles.miniBoldLabel);

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
							Vector2 jointLimits = _manager?.arm6DOFFKController != null
								? _manager.arm6DOFFKController.GetJointLimits(i)
								: new Vector2(-180f, 180f);
							EditorGUILayout.BeginHorizontal();
							_armJointAngles[i] = EditorGUILayout.Slider($"{jointNames[i]}:", _armJointAngles[i], jointLimits.x, jointLimits.y);
							if (GUILayout.Button(T("设置", "Set"), GUILayout.Width(50)))
							{
								if (_manager != null)
								{
									bool success = _manager.TrySetArmJointTarget(i, _armJointAngles[i]);
									if (!success && _manager.LastArmCollisionGuardResult != null && _manager.LastArmCollisionGuardResult.blockedByForbiddenCollision)
									{
										_armMoveSummary = _manager.LastArmCollisionGuardResult.message;
									}
								}
							}
							EditorGUILayout.EndHorizontal();
						}

						EditorGUILayout.Space();

						// Utility Buttons
						EditorGUILayout.BeginHorizontal();
						if (GUILayout.Button(T("回到 Home", "Go Home")))
						{
							if (_manager != null)
							{
								bool success = _manager.TryMoveArmHome();
								if (!success && _manager.LastArmCollisionGuardResult != null && _manager.LastArmCollisionGuardResult.blockedByForbiddenCollision)
								{
									_armMoveSummary = _manager.LastArmCollisionGuardResult.message;
								}
							}
						}
						if (GUILayout.Button(T("机械臂停止", "Arm Stop")))
						{
							if (_manager?.arm6DOFFKController != null)
							{
								_manager.arm6DOFFKController.EmergencyStop();
							}
						}
						EditorGUILayout.EndHorizontal();

						EditorGUILayout.Space();
						EditorGUILayout.BeginHorizontal();
						if (GUILayout.Button(T("冻结绑定", "Freeze Binding")))
						{
							if (_manager != null)
							{
								bool frozen = _manager.FreezeArmBindingConfiguration();
								Debug.Log($"[RobotSimulationEditor] Freeze binding result: {frozen}");
							}
						}
						EditorGUILayout.EndHorizontal();

						EditorGUILayout.Space();
						EditorGUILayout.LabelField($"{T("模型误差", "Model Error")}: {_manager.arm6DOFFKController.ModelVsMeasuredPositionError:F4}m");
						EditorGUILayout.LabelField($"{T("机械臂碰撞", "Arm Collision")}: {(_manager.HasArmCollision ? _manager.LastArmCollisionMessage : T("无", "None"))}");
						EditorGUILayout.LabelField($"{T("预检阻止", "Guard Block")}: {(_manager.LastArmCollisionGuardResult != null && _manager.LastArmCollisionGuardResult.blockedByForbiddenCollision ? _manager.LastArmCollisionGuardResult.message : T("无", "None"))}");
						EditorGUILayout.LabelField($"{T("世界坐标锁定", "World Motion Lock")}: {(_manager.IsArmWorldCoordinateMotionLocked ? _manager.ArmWorldCoordinateMotionLockReason : T("无", "None"))}");
						if (_manager.armCollisionMonitor != null)
						{
							EditorGUILayout.LabelField($"{T("碰撞监视器", "Collision Monitor")}: arm={_manager.armCollisionMonitor.ArmColliderCount}, forbidden={_manager.armCollisionMonitor.ForbiddenColliderCount}");
						}

						// Joint Status
						if (_manager.RobotState?.armJointAngles != null)
						{
							EditorGUILayout.Space();
							EditorGUILayout.LabelField(T("当前关节角：", "Current Joint Angles:"), EditorStyles.miniBoldLabel);
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

				_showTaskPlanning = EditorGUILayout.Foldout(_showTaskPlanning, T("整机协同规划", "Coordinated Task Planning"));
				if (_showTaskPlanning)
				{
					EditorGUI.indentLevel++;

					EditorGUILayout.LabelField(T("自动停靠模式", "Auto Docking Mode"), EditorStyles.miniBoldLabel);
					_armTargetPosition = EditorGUILayout.Vector3Field(T("目标点（世界末端）", "Target Point (World EE)"), _armTargetPosition);
					EditorGUILayout.HelpBox(
						T("所有输入输出都使用同一个全局世界坐标系。这里输入的是末端执行器世界点；系统会自动求解车体参考点停靠位。",
							"All inputs and outputs use the same shared global world frame. This field controls the end-effector world point; the planner resolves the base-reference docking pose automatically."),
						MessageType.Info);
					_planAllowReplan = EditorGUILayout.Toggle(T("允许局部重规划", "Allow Replan"), _planAllowReplan);
					_planTimeoutSeconds = EditorGUILayout.FloatField(T("规划超时", "Planning Timeout"), _planTimeoutSeconds);

					if (_manager != null)
					{
						_manager.EnsurePlanningSupportComponents();
						if (_manager.trajectoryPlanner != null)
						{
							_planBaseSamplingStep = EditorGUILayout.Slider(T("底盘采样步长", "Base Sampling Step"), _planBaseSamplingStep, 0.2f, 2.0f);
							_planDistanceFieldResolution = EditorGUILayout.Slider(T("距离场分辨率", "Distance Field Resolution"), _planDistanceFieldResolution, 0.1f, 1.5f);
							_planArmSampleDensityDeg = EditorGUILayout.Slider(T("机械臂轨迹采样密度", "Arm Trajectory Density"), _planArmSampleDensityDeg, 1f, 12f);
							_planShadowSpeedScale = EditorGUILayout.Slider(T("影子验证加速倍率", "Shadow Validation Speed"), _planShadowSpeedScale, 1f, 8f);
							_manager.trajectoryPlanner.BaseSamplingStep = _planBaseSamplingStep;
							_manager.trajectoryPlanner.DistanceFieldResolution = _planDistanceFieldResolution;
							_manager.trajectoryPlanner.ArmTrajectorySampleDensity = _planArmSampleDensityDeg;
							_manager.trajectoryPlanner.ShadowSimulationSpeedMultiplier = _planShadowSpeedScale;
						}
					}

					EditorGUILayout.BeginHorizontal();
					using (new EditorGUI.DisabledScope(!Application.isPlaying || _manager == null))
					{
						if (GUILayout.Button(T("规划并执行", "Plan And Execute")))
						{
							StartRobotTaskPlanning(planOnly: false);
						}

						if (GUILayout.Button(T("仅规划", "Plan Only")))
						{
							StartRobotTaskPlanning(planOnly: true);
						}

						if (GUILayout.Button(T("停止规划", "Stop Planning")))
						{
							if (_manager != null)
							{
								_manager.StopRobotTaskPlanning(true);
								_planningSummary = T("规划已停止。", "Planning stopped.");
							}
						}
					}
					EditorGUILayout.EndHorizontal();

					if (_manager != null)
					{
						EditorGUILayout.Space();
						RobotPlanResult lastPlanResult = _manager.LastRobotPlanResult;
						Vector3 baseWorld = _manager.RobotState != null ? _manager.RobotState.position : Vector3.zero;
						Vector3 eeWorld = _manager.RobotState != null ? _manager.RobotState.endEffectorPositionWorld : Vector3.zero;
						Vector3 resolvedBaseStop = lastPlanResult != null ? lastPlanResult.resolvedBaseStopWorldPosition : baseWorld;
						float resolvedBaseYaw = lastPlanResult != null ? lastPlanResult.resolvedBaseStopYawDeg : 0f;
						string localizedPlanningStage = RobotSimulationLocalization.PlanningStage(_manager.RobotState?.robotPlanningStage);
						string dockingSummary = lastPlanResult != null && !string.IsNullOrEmpty(lastPlanResult.dockingSummary)
							? lastPlanResult.dockingSummary
							: T("无", "None");
						EditorGUILayout.LabelField($"{T("当前底盘（世界车体参考点）", "Current Base (World Base Ref)")}: ({baseWorld.x:F3}, {baseWorld.y:F3}, {baseWorld.z:F3})");
						EditorGUILayout.LabelField($"{T("当前末端（世界末端点）", "Current EE (World EE)")}: ({eeWorld.x:F3}, {eeWorld.y:F3}, {eeWorld.z:F3})");
						EditorGUILayout.LabelField($"{T("推荐停靠位（世界车体参考点）", "Resolved Base Stop (World Base Ref)")}: ({resolvedBaseStop.x:F3}, {resolvedBaseStop.y:F3}, {resolvedBaseStop.z:F3})");
						EditorGUILayout.LabelField($"{T("推荐停靠朝向", "Resolved Base Yaw")}: {resolvedBaseYaw:F1}");
						EditorGUILayout.LabelField($"{T("停靠搜索结果", "Docking Search Result")}: {dockingSummary}");
						EditorGUILayout.LabelField($"{T("当前阶段", "Current Stage")}: {_manager.RobotState?.robotPlanningStage ?? RobotPlanningStage.None.ToString()}");
						EditorGUILayout.LabelField($"{T("规划进行中", "Planning In Progress")}: {(_manager.IsRobotTaskPlanningInProgress ? T("是", "Yes") : T("否", "No"))}");
						EditorGUILayout.LabelField($"{T("重规划次数", "Replan Count")}: {_manager.LastRobotPlanResult?.replanCount ?? 0}");
						ShadowValidationResult shadow = _manager.LastShadowValidationResult;
						string shadowText = shadow == null
							? T("无", "None")
							: shadow.passed
								? T("通过", "Passed")
								: string.IsNullOrEmpty(shadow.message) ? T("失败", "Failed") : shadow.message;
						EditorGUILayout.LabelField($"{T("影子验证", "Shadow Validation")}: {shadowText}");
						string effectivePlanningSummary = !string.IsNullOrEmpty(_planningSummary)
							&& (string.IsNullOrEmpty(_manager.LastPlanningSummary)
								|| _manager.LastPlanningSummary == "Planner idle."
								|| _manager.LastPlanningSummary == "规划器空闲 / Planner idle.")
							? _planningSummary
							: (!string.IsNullOrEmpty(_manager.LastPlanningSummary) ? _manager.LastPlanningSummary : _planningSummary);
						EditorGUILayout.LabelField($"{T("规划摘要", "Planning Summary")}: {effectivePlanningSummary}");
						EditorGUILayout.LabelField($"{T("残差/校准", "Residual / Calibration")}: {_manager.LastResidualCalibrationSummary}");
					}

					EditorGUI.indentLevel--;
					EditorGUILayout.Space();
				}
				// Status Display Section
				_showStatusDisplay = EditorGUILayout.Foldout(_showStatusDisplay, T("状态显示", "Status Display"));
				if (_showStatusDisplay)
				{
					EditorGUI.indentLevel++;
					_statusPanel = (StatusDisplayPanel)EditorGUILayout.ObjectField(
						T("状态面板", "Status Panel"), _statusPanel, typeof(StatusDisplayPanel), true);

					if (_manager != null && _manager.RobotState != null)
					{
						var state = _manager.RobotState;
						EditorGUILayout.LabelField(T("当前状态：", "Current State:"));
						EditorGUILayout.LabelField($"  {T("位置", "Position")}: ({state.position.x:F2}, {state.position.z:F2})");
						EditorGUILayout.LabelField($"  Rotation: {state.rotation:F1}°");
						EditorGUILayout.LabelField($"  {T("速度", "Velocity")}: {state.linearVelocity:F3} m/s");
						EditorGUILayout.LabelField($"  {T("角速度", "Angular")}: {state.angularVelocity:F3} rad/s");
						EditorGUILayout.LabelField($"  {T("模式", "Mode")}: {RobotSimulationLocalization.ControlMode(state.controlMode)}");
						EditorGUILayout.LabelField($"  {T("仿真时间", "Sim Time")}: {state.simulationTime:F1}s");
					}
					EditorGUI.indentLevel--;
					EditorGUILayout.Space();
				}

				// Configuration Management Section
				_showConfigManagement = EditorGUILayout.Foldout(_showConfigManagement, T("配置管理", "Configuration"));
				if (_showConfigManagement)
				{
					EditorGUI.indentLevel++;
					_configManager = (ConfigManager)EditorGUILayout.ObjectField(
						T("配置管理器", "Config Manager"), _configManager, typeof(ConfigManager), true);

					if (_configManager == null)
					{
						if (GUILayout.Button(T("创建配置管理器", "Create Config Manager")))
						{
							CreateConfigManager();
						}
					}
					else
					{
						EditorGUILayout.LabelField(T("快速存取：", "Quick Save/Load:"), EditorStyles.miniBoldLabel);
						EditorGUILayout.BeginHorizontal();
						if (GUILayout.Button(T("快速保存", "Quick Save")))
						{
							_configManager.QuickSave();
						}
						if (GUILayout.Button(T("快速加载", "Quick Load")))
						{
							_configManager.QuickLoad();
						}
						EditorGUILayout.EndHorizontal();

						EditorGUILayout.Space();

						EditorGUILayout.LabelField(T("导出 / 导入：", "Export/Import:"), EditorStyles.miniBoldLabel);
						EditorGUILayout.BeginHorizontal();
						if (GUILayout.Button(T("导出", "Export")))
						{
							_configManager.ExportConfig();
						}
						if (GUILayout.Button(T("导入", "Import")))
						{
							_configManager.ImportConfig();
						}
						EditorGUILayout.EndHorizontal();

						EditorGUILayout.Space();

						EditorGUILayout.LabelField($"{T("配置目录", "Config Directory")}: {_configManager.ConfigFolderPath}");
					}
					EditorGUI.indentLevel--;
					EditorGUILayout.Space();
				}

				// Debug Section
				_showDebugInfo = EditorGUILayout.Foldout(_showDebugInfo, T("调试信息", "Debug Info"));
				if (_showDebugInfo)
				{
					EditorGUI.indentLevel++;
					if (_manager != null)
					{
						_manager.showDebugInfo = EditorGUILayout.Toggle(T("显示调试信息", "Show Debug Info"), _manager.showDebugInfo);

						if (_manager.diffDriveController != null)
						{
							EditorGUILayout.Space();
							EditorGUILayout.LabelField(T("差速底盘控制器：", "DiffDrive Controller:"), EditorStyles.miniBoldLabel);
							EditorGUILayout.LabelField($"  {T("轮距", "Track Width")}: {(_manager.diffDriveController as DiffDriveTwinController)?.TrackWidth:F4} m");
							EditorGUILayout.LabelField($"  {T("轮半径", "Wheel Radius")}: {(_manager.diffDriveController as DiffDriveTwinController)?.WheelRadius:F4} m");
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
			if (GUILayout.Button(T("刷新", "Refresh")))
			{
				FindReferences();
			}
			EditorGUILayout.EndHorizontal();
		}

		private static string T(string chinese, string english)
		{
			return RobotSimulationLocalization.Text(chinese, english);
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

		private void HandleMoveAndWaitHoldButton(bool isPressed)
		{
			if (isPressed && !_moveAndWaitButtonPressedLastGui)
			{
				StartHeldMoveAndWait();
			}
			else if (!isPressed && _moveAndWaitButtonPressedLastGui)
			{
				StopHeldMoveAndWait(updateSummary: true);
			}

			_moveAndWaitButtonPressedLastGui = isPressed;
		}

		private void StartHeldMoveAndWait()
		{
			if (_manager == null)
			{
				return;
			}

			_moveAndWaitHoldActive = true;
			_armMoveSummary = "Holding Move And Wait... release to stop.";
			_manager.MoveArmToWorldPositionAndWait(new ArmMoveRequest
			{
				worldPosition = _armTargetPosition,
				positionToleranceMeters = Mathf.Max(0.001f, _armPositionTolerance),
				stableFixedFrames = Mathf.Max(1, _armStableFrames),
				timeoutSeconds = 3600f,
				speedScale = _armMoveSpeedScale
			}, result =>
			{
				_moveAndWaitHoldActive = false;
				_armMoveSummary = $"{(result.success ? "Success" : result.blockedByCollisionGuard ? "Guard Blocked" : result.collided ? "Collision" : result.timedOut ? "Timeout" : result.unreachable ? "Unreachable" : "Failed")} | err={result.finalPositionError:F4} | final=({result.finalWorldPosition.x:F3}, {result.finalWorldPosition.y:F3}, {result.finalWorldPosition.z:F3})";
				Repaint();
			});
		}

		private void StopHeldMoveAndWait(bool updateSummary)
		{
			if (!_moveAndWaitHoldActive)
			{
				return;
			}

			_moveAndWaitHoldActive = false;
			if (_manager != null)
			{
				_manager.StopArmMove(true);
			}

			if (updateSummary)
			{
				_armMoveSummary = "Move stopped by button release.";
				Repaint();
			}
		}

		private void StartRobotTaskPlanning(bool planOnly)
		{
			if (_manager == null)
			{
				_planningSummary = T("未找到 RobotSimulationManager", "RobotSimulationManager was not found.");
				Debug.LogWarning("[RobotSimulationEditor] Planning button clicked but RobotSimulationManager was not found.");
				return;
			}

			if (!Application.isPlaying)
			{
				_planningSummary = T("整机规划只能在 Play Mode 运行", "Task planning can only run in Play Mode.");
				Debug.LogWarning("[RobotSimulationEditor] Planning button clicked outside Play Mode.");
				return;
			}

			StopHeldMoveAndWait(updateSummary: false);
			RobotPlanRequest request = new RobotPlanRequest
			{
				armTargetWorldPosition = _armTargetPosition,
				autoResolveBaseDockingPose = true,
				eePositionToleranceMeters = Mathf.Max(0.005f, _armPositionTolerance),
				requireBaseMove = true,
				requireArmMove = true,
				allowReplan = _planAllowReplan,
				planningTimeoutSeconds = Mathf.Max(1f, _planTimeoutSeconds)
			};

			_planningSummary = planOnly ? T("规划中...", "Planning...") : T("规划并执行中...", "Planning and executing...");
			Debug.Log($"[RobotSimulationEditor] {(planOnly ? "Plan only" : "Plan and execute")} requested. arm={_armTargetPosition}, autoDocking=true");
			System.Action<RobotPlanResult> onComplete = result =>
			{
				_planningSummary = $"{(result.success ? T("成功", "Success") : T("失败", "Failed"))}: {result.summary}";
				if (result.success)
				{
					Debug.Log($"[RobotSimulationEditor] Planning completed successfully. {result.summary}");
				}
				else
				{
					Debug.LogWarning($"[RobotSimulationEditor] Planning finished with failure. Stage={result.failedAtStage}, summary={result.summary}");
				}
				Repaint();
			};

			if (planOnly)
			{
				if (_manager.PlanRobotTaskOnly(request, onComplete) == null)
				{
					_planningSummary = string.IsNullOrEmpty(_manager.LastPlanningSummary)
						? T("规划启动失败", "Failed to start planning.")
						: _manager.LastPlanningSummary;
					Debug.LogWarning($"[RobotSimulationEditor] Plan-only request did not start. {_planningSummary}");
				}
			}
			else
			{
				if (_manager.PlanAndExecuteRobotTask(request, onComplete) == null)
				{
					_planningSummary = string.IsNullOrEmpty(_manager.LastPlanningSummary)
						? T("规划启动失败", "Failed to start planning.")
						: _manager.LastPlanningSummary;
					Debug.LogWarning($"[RobotSimulationEditor] Plan-and-execute request did not start. {_planningSummary}");
				}
			}
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
			GameObject armGo = new GameObject("Arm6DOFController");
			var fkController = armGo.AddComponent<Arm6DOFFKController>();
			var ikController = armGo.AddComponent<Arm6DOFIKController>();
			ikController.armController = fkController;
			fkController.urdfAssetPath = FindDefaultUrdfPath();

			if (_manager != null && _manager.armJointControllers != null && _manager.armJointControllers.Length >= 6)
			{
				fkController.joints = new ArticulationBody[6];
				fkController.jointTransforms = new Transform[6];
				for (int i = 0; i < 6; i++)
				{
					if (_manager.armJointControllers[i] != null && _manager.armJointControllers[i].joint != null)
					{
						fkController.joints[i] = _manager.armJointControllers[i].joint;
						fkController.jointTransforms[i] = _manager.armJointControllers[i].joint.transform;
						if (fkController.armHierarchyRoot == null)
						{
							fkController.armHierarchyRoot = _manager.armJointControllers[i].joint.transform.root;
						}
					}
				}

				fkController.autoBindByJointName = true;
				fkController.autoRebindOnMismatch = true;
				for (int i = 0; i < 6; i++)
				{
					if (fkController.joints[i] != null)
					{
						Debug.Log($"[RobotSimulationEditor] Bound J{i}: {fkController.joints[i].name}");
					}
					else
					{
						Debug.LogWarning($"[RobotSimulationEditor] J{i} is NULL!");
					}
				}

				if (fkController.jointTransforms[5] != null)
				{
					fkController.endEffector = fkController.jointTransforms[5];
				}
			}

			_manager.arm6DOFFKController = fkController;
			_manager.arm6DOFIKController = ikController;
			fkController.InitializeJoints();
			Debug.Log($"[RobotSimulationEditor] Created Arm6DOFFKController and Arm6DOFIKController. URDF={(string.IsNullOrEmpty(fkController.urdfAssetPath) ? "missing" : fkController.urdfAssetPath)}");
		}

		private string FindDefaultUrdfPath()
		{
			string[] guids = AssetDatabase.FindAssets("Zu5_LDASM_unity_fixed");
			for (int i = 0; i < guids.Length; i++)
			{
				string path = AssetDatabase.GUIDToAssetPath(guids[i]);
				if (!path.EndsWith(".urdf", StringComparison.OrdinalIgnoreCase))
				{
					continue;
				}

				if (path.IndexOf("Zone.Identifier", StringComparison.OrdinalIgnoreCase) >= 0)
				{
					continue;
				}

				return path;
			}

			Debug.LogError("[RobotSimulationEditor] Could not find a valid Zu5_LDASM_unity_fixed.urdf asset. Move-and-wait and kinematics self-test will stay disabled.");
			return string.Empty;
		}
	}

}
