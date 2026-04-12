#if UNITY_EDITOR
using System;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;

namespace RobotSimulation.Editor
{
	public static class SquareObstacleTestSceneBuilder
	{
		public const string ScenePath = "Assets/Scenes/SquareObstacleTestScene.unity";
		private const string GeneratedRootName = "SquareObstacleTestEnvironment";
		private const string GeneratedFolderPath = "Assets/Scenes/Generated";
		private const string MaterialFolderPath = "Assets/Scenes/Generated/Materials";

		[MenuItem("Robot Simulation/Create Square Obstacle Test Scene")]
		public static void CreateAndOpenSceneMenu()
		{
			CreateAndOpenScene(true);
		}

		public static string CreateAndOpenScene(bool promptToSaveCurrentScene)
		{
			if (promptToSaveCurrentScene && !EditorSceneManager.SaveCurrentModifiedScenesIfUserWantsTo())
			{
				return string.Empty;
			}

			EnsureFolder("Assets/Scenes");
			EnsureFolder(GeneratedFolderPath);
			EnsureFolder(MaterialFolderPath);

			Scene scene = PrepareTargetScene();
			if (!scene.IsValid())
			{
				return string.Empty;
			}

			BuildScene(scene);
			if (!SquareObstacleTestSceneContractValidator.TryValidateScene(scene, out string validationReport))
			{
				throw new InvalidOperationException(validationReport);
			}

			EditorSceneManager.SaveScene(scene);
			AssetDatabase.SaveAssets();
			AssetDatabase.Refresh();

			Debug.Log($"[RobotSimulation] Created square obstacle test scene from current scene: {ScenePath}");
			return ScenePath;
		}

		private static void BuildScene(Scene scene)
		{
			RenderSettings.ambientMode = UnityEngine.Rendering.AmbientMode.Flat;
			RenderSettings.ambientLight = new Color(0.72f, 0.74f, 0.78f);

			GameObject existingRoot = GameObject.Find(GeneratedRootName);
			if (existingRoot != null)
			{
				UnityEngine.Object.DestroyImmediate(existingRoot);
			}

			GameObject root = new GameObject(GeneratedRootName);
			SceneManager.MoveGameObjectToScene(root, scene);

			Transform environmentRoot = CreateChild(root.transform, "Environment");
			Transform obstacleRoot = CreateChild(root.transform, "Obstacles");
			Transform markerRoot = CreateChild(root.transform, "Markers");
			Transform utilityRoot = CreateChild(root.transform, "Utility");

			Material floorMaterial = LoadOrCreateMaterial("ArenaFloor.mat", new Color(0.33f, 0.36f, 0.39f));
			Material wallMaterial = LoadOrCreateMaterial("ArenaWall.mat", new Color(0.62f, 0.64f, 0.68f));
			Material obstacleMaterial = LoadOrCreateMaterial("ArenaObstacle.mat", new Color(0.78f, 0.49f, 0.22f));
			Material markerMaterial = LoadOrCreateMaterial("ArenaMarker.mat", new Color(0.11f, 0.66f, 0.54f));

			CreatePrimitive(PrimitiveType.Cube, "Floor", environmentRoot, new Vector3(0f, -0.1f, 0f), new Vector3(10f, 0.2f, 10f), floorMaterial);

			CreatePrimitive(PrimitiveType.Cube, "Wall_North", environmentRoot, new Vector3(0f, 0.7f, 4.9f), new Vector3(10f, 1.4f, 0.2f), wallMaterial);
			CreatePrimitive(PrimitiveType.Cube, "Wall_South", environmentRoot, new Vector3(0f, 0.7f, -4.9f), new Vector3(10f, 1.4f, 0.2f), wallMaterial);
			CreatePrimitive(PrimitiveType.Cube, "Wall_East", environmentRoot, new Vector3(4.9f, 0.7f, 0f), new Vector3(0.2f, 1.4f, 10f), wallMaterial);
			CreatePrimitive(PrimitiveType.Cube, "Wall_West", environmentRoot, new Vector3(-4.9f, 0.7f, 0f), new Vector3(0.2f, 1.4f, 10f), wallMaterial);

			CreatePrimitive(PrimitiveType.Cube, "Obstacle_Box_A", obstacleRoot, new Vector3(-2.2f, 0.4f, 1.7f), new Vector3(0.9f, 0.8f, 0.9f), obstacleMaterial);
			CreatePrimitive(PrimitiveType.Cube, "Obstacle_Box_B", obstacleRoot, new Vector3(2.1f, 0.55f, -1.8f), new Vector3(1.1f, 1.1f, 0.8f), obstacleMaterial);
			CreatePrimitive(PrimitiveType.Cube, "Obstacle_Wall_Left", obstacleRoot, new Vector3(-3.0f, 0.65f, -0.2f), new Vector3(0.35f, 1.3f, 2.6f), obstacleMaterial);
			CreatePrimitive(PrimitiveType.Cube, "Obstacle_Wall_Right", obstacleRoot, new Vector3(3.1f, 0.65f, 0.9f), new Vector3(0.35f, 1.3f, 2.2f), obstacleMaterial);
			CreatePrimitive(PrimitiveType.Cube, "Obstacle_LowBar", obstacleRoot, new Vector3(0.1f, 0.25f, 3.0f), new Vector3(1.8f, 0.5f, 0.45f), obstacleMaterial);
			CreatePrimitive(PrimitiveType.Cylinder, "Obstacle_Column_A", obstacleRoot, new Vector3(-1.7f, 0.75f, -2.8f), new Vector3(0.6f, 0.75f, 0.6f), obstacleMaterial);
			CreatePrimitive(PrimitiveType.Cylinder, "Obstacle_Column_B", obstacleRoot, new Vector3(2.7f, 0.9f, 2.2f), new Vector3(0.5f, 0.9f, 0.5f), obstacleMaterial);

			CreatePrimitive(PrimitiveType.Cylinder, "OriginMarker", markerRoot, new Vector3(0f, 0.02f, 0f), new Vector3(0.35f, 0.02f, 0.35f), markerMaterial);
			CreatePrimitive(PrimitiveType.Cylinder, "RecommendedRobotSpawn", markerRoot, new Vector3(-3.5f, 0.02f, -3.5f), new Vector3(0.35f, 0.02f, 0.35f), markerMaterial);
			CreatePrimitive(PrimitiveType.Cylinder, "RecommendedTargetMarker", markerRoot, new Vector3(3.5f, 0.02f, 3.5f), new Vector3(0.35f, 0.02f, 0.35f), markerMaterial);

			GameObject lightGo = new GameObject("Directional Light");
			lightGo.transform.SetParent(utilityRoot, false);
			Light light = lightGo.AddComponent<Light>();
			light.type = LightType.Directional;
			light.intensity = 1.15f;
			light.transform.rotation = Quaternion.Euler(50f, -35f, 0f);

			GameObject cameraGo = new GameObject("Overview Camera");
			cameraGo.transform.SetParent(utilityRoot, false);
			Camera camera = cameraGo.AddComponent<Camera>();
			camera.transform.position = new Vector3(0f, 10.5f, -8.5f);
			camera.transform.rotation = Quaternion.Euler(40f, 0f, 0f);
			camera.clearFlags = CameraClearFlags.Skybox;
			camera.fieldOfView = 52f;
		}

		private static Scene PrepareTargetScene()
		{
			Scene activeScene = SceneManager.GetActiveScene();
			if (activeScene.IsValid() && activeScene.isLoaded)
			{
				if (!string.IsNullOrEmpty(activeScene.path))
				{
					EditorSceneManager.SaveScene(activeScene);
					EditorSceneManager.SaveScene(activeScene, ScenePath, true);
					return EditorSceneManager.OpenScene(ScenePath, OpenSceneMode.Single);
				}

				EditorSceneManager.SaveScene(activeScene, ScenePath);
				return SceneManager.GetActiveScene();
			}

			Scene scene = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);
			EditorSceneManager.SaveScene(scene, ScenePath);
			return scene;
		}

		private static Transform CreateChild(Transform parent, string name)
		{
			GameObject go = new GameObject(name);
			go.transform.SetParent(parent, false);
			return go.transform;
		}

		private static GameObject CreatePrimitive(PrimitiveType primitiveType, string name, Transform parent, Vector3 position, Vector3 scale, Material material)
		{
			GameObject go = GameObject.CreatePrimitive(primitiveType);
			go.name = name;
			go.transform.SetParent(parent, false);
			go.transform.localPosition = position;
			go.transform.localRotation = Quaternion.identity;
			go.transform.localScale = scale;

			Renderer renderer = go.GetComponent<Renderer>();
			if (renderer != null && material != null)
			{
				renderer.sharedMaterial = material;
				renderer.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.On;
				renderer.receiveShadows = true;
			}

			GameObjectUtility.SetStaticEditorFlags(go, StaticEditorFlags.BatchingStatic | StaticEditorFlags.ContributeGI);
			return go;
		}

		private static Material LoadOrCreateMaterial(string fileName, Color color)
		{
			string assetPath = $"{MaterialFolderPath}/{fileName}";
			Material material = AssetDatabase.LoadAssetAtPath<Material>(assetPath);
			if (material != null)
			{
				return material;
			}

			Shader shader = Shader.Find("Universal Render Pipeline/Lit");
			if (shader == null)
			{
				shader = Shader.Find("Standard");
			}

			material = new Material(shader);
			material.color = color;
			AssetDatabase.CreateAsset(material, assetPath);
			return material;
		}

		private static void EnsureFolder(string folderPath)
		{
			if (AssetDatabase.IsValidFolder(folderPath))
			{
				return;
			}

			int lastSlash = folderPath.LastIndexOf('/');
			string parent = lastSlash > 0 ? folderPath.Substring(0, lastSlash) : "Assets";
			string name = lastSlash > 0 ? folderPath.Substring(lastSlash + 1) : folderPath;

			if (!AssetDatabase.IsValidFolder(parent))
			{
				EnsureFolder(parent);
			}

			AssetDatabase.CreateFolder(parent, name);
		}
	}
}
#endif
