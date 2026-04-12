using NUnit.Framework;
using UnityEditor.SceneManagement;
using UnityEngine;
using RobotSimulation.Editor;

namespace RobotSimulation.Tests.Editor
{
	public class SquareObstacleTestSceneContractEditModeTests
	{
		[Test]
		public void SquareObstacleTestScene_SatisfiesExplicitContract()
		{
			var scene = EditorSceneManager.OpenScene("Assets/Scenes/SquareObstacleTestScene.unity");
			bool valid = SquareObstacleTestSceneContractValidator.TryValidateScene(scene, out string report);

			Assert.IsTrue(valid, report);
		}

		[Test]
		public void SquareObstacleTestScene_ExplicitModeDoesNotRediscoverMissingMonitor()
		{
			EditorSceneManager.OpenScene("Assets/Scenes/SquareObstacleTestScene.unity");
			RobotSimulationManager manager = Object.FindObjectOfType<RobotSimulationManager>();
			Assert.IsNotNull(manager, "SquareObstacleTestScene should contain RobotSimulationManager.");

			ArmCollisionMonitor originalMonitor = manager.armCollisionMonitor;
			try
			{
				manager.armCollisionMonitor = null;

				bool valid = RobotSimulationBootstrapContract.TryValidateExplicitContract(manager, out string report);
				Assert.IsFalse(valid, "Explicit contract validation should fail when the collision monitor reference is cleared.");
				StringAssert.Contains("armCollisionMonitor", report);

				manager.InitializeRobot();
				Assert.IsNull(manager.armCollisionMonitor, "ExplicitContract mode must not rediscover or recreate ArmCollisionMonitor at runtime.");
			}
			finally
			{
				manager.armCollisionMonitor = originalMonitor;
			}
		}
	}
}
