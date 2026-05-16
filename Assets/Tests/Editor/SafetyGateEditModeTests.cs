using System.Collections.Generic;
using System.Reflection;
using NUnit.Framework;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.TestTools;

namespace RobotSimulation.Tests.Editor
{
	public class SafetyGateEditModeTests
	{
		[Test]
		public void EvaluateBaseCommand_BlocksWhenPredictedPoseOverlapsObstacle()
		{
			GameObject obstacle = GameObject.CreatePrimitive(PrimitiveType.Cube);
			obstacle.transform.position = Vector3.zero;
			obstacle.transform.localScale = new Vector3(0.5f, 0.5f, 0.5f);

			try
			{
				PlannerPhysicsQueries physicsQueries = new PlannerPhysicsQueries();
				List<Collider> obstacles = new List<Collider> { obstacle.GetComponent<Collider>() };
				SafetyGateRuntimeContext context = new SafetyGateRuntimeContext
				{
					enabled = true,
					obstacles = obstacles,
					obstacleLookup = physicsQueries.GetObstacleLookup(obstacles),
					baseRadiusMeters = 0.2f
				};
				ShadowSimulationGate gate = new ShadowSimulationGate();
				SafetyGateDecision decision = gate.EvaluateBaseCommand(
					new MirrorSnapshot
					{
						baseWorldPosition = new Vector3(-0.6f, 0f, 0f),
						baseWorldRotation = Quaternion.identity
					},
					new BaseGateCommand
					{
						fromWorldPosition = new Vector3(-0.6f, 0f, 0f),
						toWorldPosition = Vector3.zero,
						targetYawDeg = 0f
					},
					context);

				Assert.AreEqual(SafetyGateDecisionType.Block, decision.type);
				Assert.AreEqual(SafetyGateBlockCategory.GateBlockedByCollision, decision.blockCategory);
			}
			finally
			{
				Object.DestroyImmediate(obstacle);
			}
		}

		[Test]
		public void EvaluateBaseCommand_ThrottlesWhenClearanceFallsBelowThrottleThreshold()
		{
			GameObject obstacle = GameObject.CreatePrimitive(PrimitiveType.Cube);
			obstacle.transform.position = new Vector3(0.75f, 0f, 0f);
			obstacle.transform.localScale = new Vector3(0.2f, 0.2f, 0.2f);

			try
			{
				PlannerPhysicsQueries physicsQueries = new PlannerPhysicsQueries();
				List<Collider> obstacles = new List<Collider> { obstacle.GetComponent<Collider>() };
				SafetyGateRuntimeContext context = new SafetyGateRuntimeContext
				{
					enabled = true,
					throttleOnRisk = true,
					obstacles = obstacles,
					obstacleLookup = physicsQueries.GetObstacleLookup(obstacles),
					baseRadiusMeters = 0.1f,
					gateBlockClearanceMeters = 0.05f,
					gateThrottleClearanceMeters = 0.7f
				};
				ShadowSimulationGate gate = new ShadowSimulationGate();
				SafetyGateDecision decision = gate.EvaluateBaseCommand(
					new MirrorSnapshot
					{
						baseWorldPosition = Vector3.zero,
						baseWorldRotation = Quaternion.identity
					},
					new BaseGateCommand
					{
						fromWorldPosition = Vector3.zero,
						toWorldPosition = Vector3.zero,
						targetYawDeg = 0f
					},
					context);

				Assert.AreEqual(SafetyGateDecisionType.Throttle, decision.type);
				Assert.Greater(decision.throttleRatio, 0.19f);
				Assert.Less(decision.throttleRatio, 1f);
			}
			finally
			{
				Object.DestroyImmediate(obstacle);
			}
		}

		[Test]
		public void EvaluateArmCommand_BlocksWhenSingularityThresholdIsForcedHigh()
		{
			RobotSimulationManager manager = CreateAutoDiscoverManager();
			try
			{
				float[] currentAngles = manager.arm6DOFFKController.CaptureMeasuredJointAngles();
				ShadowSimulationGate gate = new ShadowSimulationGate
				{
					armSingularityDeterminantHardThreshold = float.MaxValue
				};
				SafetyGateRuntimeContext context = new SafetyGateRuntimeContext
				{
					enabled = true,
					obstacles = new List<Collider>(),
					obstacleLookup = new HashSet<Collider>()
				};
				SafetyGateDecision decision = gate.EvaluateArmCommand(
					new UnityMirrorStateProvider().Capture(manager.diffDriveController, manager.arm6DOFFKController),
					new ArmGateCommand
					{
						fromAnglesDeg = (float[])currentAngles.Clone(),
						toAnglesDeg = (float[])currentAngles.Clone()
					},
					context,
					manager.arm6DOFFKController);

				Assert.AreEqual(SafetyGateDecisionType.Block, decision.type);
				Assert.AreEqual(SafetyGateBlockCategory.GateBlockedBySingularity, decision.blockCategory);
			}
			finally
			{
				Object.DestroyImmediate(manager.gameObject);
			}
		}

		[Test]
		public void EvaluateArmCommand_BlocksWhenPredictedLinkPointHitsObstacle()
		{
			RobotSimulationManager manager = CreateAutoDiscoverManager();
			GameObject obstacle = null;
			try
			{
				float[] currentAngles = manager.arm6DOFFKController.CaptureMeasuredJointAngles();
				Pose[] linkWorldPoses = manager.arm6DOFFKController.ComputeLinkPosesWorld(currentAngles);
				Assert.IsNotNull(linkWorldPoses);
				Assert.IsNotEmpty(linkWorldPoses);

				obstacle = GameObject.CreatePrimitive(PrimitiveType.Cube);
				obstacle.transform.position = linkWorldPoses[linkWorldPoses.Length - 1].position;
				obstacle.transform.localScale = Vector3.one * 0.05f;

				PlannerPhysicsQueries physicsQueries = new PlannerPhysicsQueries();
				List<Collider> obstacles = new List<Collider> { obstacle.GetComponent<Collider>() };
				ShadowSimulationGate gate = new ShadowSimulationGate
				{
					armSingularityDeterminantHardThreshold = 0f,
					armSingularityDeterminantThrottleThreshold = 0f
				};
				SafetyGateRuntimeContext context = new SafetyGateRuntimeContext
				{
					enabled = true,
					obstacles = obstacles,
					obstacleLookup = physicsQueries.GetObstacleLookup(obstacles),
					armInflationMeters = 0.01f
				};
				SafetyGateDecision decision = gate.EvaluateArmCommand(
					new UnityMirrorStateProvider().Capture(manager.diffDriveController, manager.arm6DOFFKController),
					new ArmGateCommand
					{
						fromAnglesDeg = (float[])currentAngles.Clone(),
						toAnglesDeg = (float[])currentAngles.Clone()
					},
					context,
					manager.arm6DOFFKController);

				Assert.AreEqual(SafetyGateDecisionType.Block, decision.type);
				Assert.AreEqual(SafetyGateBlockCategory.GateBlockedByCollision, decision.blockCategory);
			}
			finally
			{
				if (obstacle != null)
				{
					Object.DestroyImmediate(obstacle);
				}

				Object.DestroyImmediate(manager.gameObject);
			}
		}

		[Test]
		public void ShadowBaseFaultHandler_EmergencyStopsBaseOutsideManualPreview()
		{
			GameObject managerGo = new GameObject("RobotSimulationManager");
			GameObject baseGo = new GameObject("DiffDriveBase");
			try
			{
				RobotSimulationManager manager = managerGo.AddComponent<RobotSimulationManager>();
				Rigidbody rb = baseGo.AddComponent<Rigidbody>();
				DiffDriveTwinController diffDrive = baseGo.AddComponent<DiffDriveTwinController>();
				diffDrive.rb = rb;
				diffDrive.hasTargetPoint = true;
				diffDrive.targetPointWorld = new Vector3(5f, 0f, 0f);
				rb.velocity = new Vector3(1f, 0f, 2f);
				rb.angularVelocity = new Vector3(0f, 1f, 0f);
				manager.diffDriveController = diffDrive;

				System.Type shadowBaseType = LoadRuntimeType("RobotSimulation.ShadowBaseTwinRuntime");
				object shadowBaseRuntime = System.Activator.CreateInstance(shadowBaseType);
				const string faultMessage = "Synthetic shadow base collision.";
				LogAssert.Expect(LogType.Error, $"[ShadowBaseTwinRuntime] {faultMessage}");
				shadowBaseType.GetMethod("FailSession").Invoke(shadowBaseRuntime, new object[] { faultMessage });

				typeof(RobotSimulationManager)
					.GetField("_shadowBaseTwinRuntime", BindingFlags.Instance | BindingFlags.NonPublic)
					.SetValue(manager, shadowBaseRuntime);

				LogAssert.Expect(LogType.Error, $"[RobotSimulation] Shadow base fault triggered emergency stop: {faultMessage}");
				typeof(RobotSimulationManager)
					.GetMethod("HandleShadowBaseTwinFaultIfNeeded", BindingFlags.Instance | BindingFlags.NonPublic)
					.Invoke(manager, null);

				Assert.IsFalse(diffDrive.hasTargetPoint, "Shadow base faults must clear the live base target so the base cannot resume on the next physics tick.");
				Assert.AreEqual(rb.position, diffDrive.targetPointWorld);
				Assert.AreEqual(0f, rb.velocity.x, 1e-5f);
				Assert.AreEqual(0f, rb.velocity.z, 1e-5f);
				Assert.AreEqual(0f, rb.angularVelocity.y, 1e-5f);
			}
			finally
			{
				Object.DestroyImmediate(managerGo);
				Object.DestroyImmediate(baseGo);
			}
		}

		[Test]
		public void GoHome_BypassesCollisionGuardForRecovery()
		{
			RobotSimulationManager manager = CreateAutoDiscoverManager();
			GameObject forbiddenRoot = new GameObject("HomeRecoveryForbiddenRoot");
			GameObject blocker = GameObject.CreatePrimitive(PrimitiveType.Cube);
			try
			{
				float[] homeAngles = manager.arm6DOFFKController.configuredHomeJointAnglesDeg;
				Pose[] homeLinkPoses = manager.arm6DOFFKController.ComputeLinkPosesWorld(homeAngles);
				Assert.IsNotNull(homeLinkPoses);
				Assert.IsNotEmpty(homeLinkPoses);

				blocker.name = "HomeRecoveryBlocker";
				blocker.transform.SetParent(forbiddenRoot.transform, false);
				blocker.transform.position = homeLinkPoses[homeLinkPoses.Length - 1].position;
				blocker.transform.localScale = Vector3.one * 0.25f;
				manager.armCollisionMonitor.Configure(manager.armBinder.armRoot.transform, forbiddenRoot.transform);

				LogAssert.Expect(LogType.Warning, new System.Text.RegularExpressions.Regex(@"\[Arm6DOF\].*"));
				Assert.IsFalse(manager.arm6DOFFKController.TryGoHome(), "Normal arm targeting should still honor the collision guard.");
				Assert.IsTrue(manager.arm6DOFFKController.TryGoHome(bypassCollisionGuard: true), "Go Home recovery must be able to command Home even while the guard still sees the collision.");
			}
			finally
			{
				Object.DestroyImmediate(blocker);
				Object.DestroyImmediate(forbiddenRoot);
				Object.DestroyImmediate(manager.gameObject);
			}
		}

		private static RobotSimulationManager CreateAutoDiscoverManager()
		{
			EditorSceneManager.OpenScene("Assets/Scenes/SampleScene.unity");
			RobotSimulationManager existing = Object.FindObjectOfType<RobotSimulationManager>();
			if (existing != null)
			{
				Object.DestroyImmediate(existing.gameObject);
			}

			GameObject managerGo = new GameObject("RobotSimulationManager");
			RobotSimulationManager manager = managerGo.AddComponent<RobotSimulationManager>();
			manager.bootstrapMode = RobotSimulationManager.BootstrapMode.AutoDiscover;
			manager.InitializeRobot();
			Assert.IsNotNull(manager.arm6DOFFKController, "FK controller should be discoverable in SampleScene.");
			Assert.IsTrue(manager.arm6DOFFKController.KinematicsReady, "FK controller should be initialized for SafetyGate tests.");
			return manager;
		}

		private static System.Type LoadRuntimeType(string fullName)
		{
			System.Type type = System.Type.GetType($"{fullName}, Assembly-CSharp");
			Assert.IsNotNull(type, $"Runtime type '{fullName}' could not be loaded from Assembly-CSharp.");
			return type;
		}
	}
}
