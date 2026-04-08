using NUnit.Framework;
using UnityEditor;
using UnityEngine;

namespace RobotSimulation.Tests.Editor
{
	public class Arm6DOFKinematicsEditModeTests
	{
		[Test]
		public void UrdfModel_ParsesExpectedJoints_AndPoeMatchesUrdfChain()
		{
			TextAsset urdf = LoadUrdf();
			Assert.IsNotNull(urdf, "Zu5_LDASM_unity_fixed.urdf was not found.");

			float[] homeAngles = Arm6DOFFKController.CreateDefaultConfiguredHomeJointAnglesDeg();
			Arm6DOFKinematicsModel model = new Arm6DOFKinematicsModel();
			Assert.IsTrue(model.Initialize(urdf, null, null, null, homeAngles), model.ErrorMessage);

			DhRow[] rows = model.GetDhRows();
			Assert.AreEqual(6, rows.Length);
			for (int i = 0; i < rows.Length; i++)
			{
				Assert.AreEqual($"Joint{i + 1:00}", rows[i].jointName);
			}

			float homeError = Vector3.Distance(model.ForwardPoe(homeAngles).position, model.ForwardUrdfChain(homeAngles).position);
			Assert.Less(homeError, 1e-4f, "Configured home pose PoE vs URDF chain mismatch is too large.");

			float[][] samples = new float[][]
			{
				new float[] { 10f, -20f, 160f, -10f, 80f, 15f },
				new float[] { -15f, 25f, 120f, 5f, 110f, -25f },
				new float[] { 5f, 10f, 135f, -5f, 70f, 5f }
			};

			for (int i = 0; i < samples.Length; i++)
			{
				float error = Vector3.Distance(model.ForwardPoe(samples[i]).position, model.ForwardUrdfChain(samples[i]).position);
				Assert.Less(error, 1e-3f, $"Sample {i} PoE vs URDF chain mismatch is too large.");
			}
		}

		[Test]
		public void ManipulabilityIndex_IsPositive_ForNominalConfigurations()
		{
			TextAsset urdf = LoadUrdf();
			Assert.IsNotNull(urdf, "Zu5_LDASM_unity_fixed.urdf was not found.");

			float[] homeAngles = Arm6DOFFKController.CreateDefaultConfiguredHomeJointAnglesDeg();
			Arm6DOFKinematicsModel model = new Arm6DOFKinematicsModel();
			Assert.IsTrue(model.Initialize(urdf, null, null, null, homeAngles), model.ErrorMessage);

			float[][] samples =
			{
				homeAngles,
				new float[] { 10f, -20f, 160f, -10f, 80f, 15f },
				new float[] { -15f, 25f, 120f, 5f, 110f, -25f }
			};

			for (int i = 0; i < samples.Length; i++)
			{
				float manipulability = model.ComputeManipulabilityIndex(samples[i]);
				Assert.That(manipulability, Is.GreaterThan(0f), $"Sample {i} should have a positive manipulability index.");
				Assert.That(float.IsNaN(manipulability) || float.IsInfinity(manipulability), Is.False, $"Sample {i} manipulability should stay finite.");
			}
		}

		private static TextAsset LoadUrdf()
		{
			string[] guids = AssetDatabase.FindAssets("Zu5_LDASM_unity_fixed t:TextAsset");
			if (guids.Length == 0)
			{
				guids = AssetDatabase.FindAssets("Zu5_LDASM_unity_fixed.urdf");
			}

			if (guids.Length == 0)
			{
				return null;
			}

			string path = AssetDatabase.GUIDToAssetPath(guids[0]);
			return AssetDatabase.LoadAssetAtPath<TextAsset>(path);
		}
	}
}
