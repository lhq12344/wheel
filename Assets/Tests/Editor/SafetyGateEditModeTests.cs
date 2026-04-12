using System.Collections.Generic;
using NUnit.Framework;
using UnityEditor.SceneManagement;
using UnityEngine;

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
	}
}
