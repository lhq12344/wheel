#if UNITY_EDITOR
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;

namespace RobotSimulation.Editor
{
	public static class SquareObstacleTestSceneContractValidator
	{
		public static bool TryValidateScene(Scene scene, out string report)
		{
			report = string.Empty;
			if (!scene.IsValid())
			{
				report = "Scene is invalid.";
				return false;
			}

			RobotSimulationManager manager = FindManager(scene);
			if (manager == null)
			{
				report = "[RobotSimulation] SquareObstacleTestScene contract validation failed:\n- RobotSimulationManager root object is missing.";
				return false;
			}

			if (manager.bootstrapMode != RobotSimulationManager.BootstrapMode.ExplicitContract)
			{
				report = $"[RobotSimulation] SquareObstacleTestScene must use ExplicitContract bootstrap mode, current={manager.bootstrapMode}.";
				return false;
			}

			return RobotSimulationBootstrapContract.TryValidateExplicitContract(manager, out report);
		}

		public static bool TryValidateOpenSquareObstacleScene(out string report)
		{
			Scene scene = SceneManager.GetActiveScene();
			return TryValidateScene(scene, out report);
		}

		public static string ValidateAndReturnReport(string scenePath)
		{
			Scene scene = EditorSceneManager.OpenScene(scenePath);
			return TryValidateScene(scene, out string report) ? string.Empty : report;
		}

		private static RobotSimulationManager FindManager(Scene scene)
		{
			GameObject[] roots = scene.GetRootGameObjects();
			for (int i = 0; i < roots.Length; i++)
			{
				RobotSimulationManager manager = roots[i].GetComponent<RobotSimulationManager>();
				if (manager != null)
				{
					return manager;
				}
			}

			return null;
		}
	}
}
#endif
