using NUnit.Framework;
using UnityEditor.SceneManagement;
using UnityEngine;

namespace RobotSimulation.Tests.Editor
{
	public class Arm6DOFSceneSetupEditModeTests
	{
		[Test]
		public void SampleScene_ManagerCanInitializeArmKinematics()
		{
			EditorSceneManager.OpenScene("Assets/Scenes/SampleScene.unity");

			RobotSimulationManager existing = Object.FindObjectOfType<RobotSimulationManager>();
			if (existing != null)
			{
				Object.DestroyImmediate(existing.gameObject);
			}

			GameObject managerGo = new GameObject("RobotSimulationManager");
			RobotSimulationManager manager = managerGo.AddComponent<RobotSimulationManager>();
			manager.InitializeRobot();

			Assert.IsNotNull(manager.arm6DOFFKController, "FK controller should be auto-created or discovered.");
			Assert.IsNotNull(manager.arm6DOFIKController, "IK controller should be auto-created or discovered.");
			Assert.IsTrue(manager.arm6DOFFKController.KinematicsReady, "URDF kinematics should be initialized.");
			Assert.GreaterOrEqual(manager.armJointControllers.Length, 6, "Expected six joint controllers in SampleScene.");

			Object.DestroyImmediate(managerGo);
		}
	}
}
