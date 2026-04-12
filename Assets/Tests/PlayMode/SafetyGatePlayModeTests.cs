using System.Collections;
using System.Reflection;
using NUnit.Framework;
using UnityEngine;
using UnityEngine.TestTools;

namespace RobotSimulation.Tests.PlayMode
{
	public class SafetyGatePlayModeTests
	{
		[UnityTest]
		public IEnumerator TimelineBuffer_DequeuesOnlyAfterLeadTimeIsSatisfied()
		{
			System.Type bufferType = LoadRuntimeType("RobotSimulation.SafetyGateTimelineBuffer");
			System.Type commandType = LoadRuntimeType("RobotSimulation.SafetyGateTimelineCommand");
			System.Type kindType = LoadRuntimeType("RobotSimulation.SafetyGateTimelineCommandKind");
			object buffer = System.Activator.CreateInstance(bufferType);
			float enqueueRealtime = Time.realtimeSinceStartup;
			object command = System.Activator.CreateInstance(commandType);
			commandType.GetField("enqueueRealtime").SetValue(command, enqueueRealtime);
			commandType.GetField("readyRealtime").SetValue(command, enqueueRealtime + 0.15f);
			commandType.GetField("predictedLeadSeconds").SetValue(command, 0.15f);
			commandType.GetField("kind").SetValue(command, System.Enum.Parse(kindType, "Base"));
			bufferType.GetMethod("Enqueue").Invoke(buffer, new[] { command });

			object[] earlyArgs = { Time.realtimeSinceStartup, 0.15f, null };
			bool earlyReady = (bool)bufferType.GetMethod("TryDequeueReady").Invoke(buffer, earlyArgs);
			Assert.IsFalse(earlyReady);

			yield return new WaitForSecondsRealtime(0.18f);

			object[] lateArgs = { Time.realtimeSinceStartup, 0.15f, null };
			bool lateReady = (bool)bufferType.GetMethod("TryDequeueReady").Invoke(buffer, lateArgs);
			Assert.IsTrue(lateReady);
			Assert.AreEqual("Base", lateArgs[2].GetType().GetField("kind").GetValue(lateArgs[2]).ToString());
		}

		[UnityTest]
		public IEnumerator ArmCollisionMonitor_UpdatesCachedCollisionStateInFixedUpdate()
		{
			GameObject armRoot = new GameObject("ArmRoot");
			GameObject forbiddenRoot = new GameObject("ForbiddenRoot");
			GameObject armColliderGo = GameObject.CreatePrimitive(PrimitiveType.Cube);
			GameObject forbiddenColliderGo = GameObject.CreatePrimitive(PrimitiveType.Cube);
			GameObject monitorGo = new GameObject("ArmCollisionMonitor");

			armColliderGo.transform.SetParent(armRoot.transform, false);
			forbiddenColliderGo.transform.SetParent(forbiddenRoot.transform, false);
			armColliderGo.transform.localPosition = Vector3.zero;
			forbiddenColliderGo.transform.localPosition = Vector3.zero;

			System.Type monitorType = LoadRuntimeType("RobotSimulation.ArmCollisionMonitor");
			Component monitor = monitorGo.AddComponent(monitorType);
			monitorType.GetField("monitorInFixedUpdate").SetValue(monitor, true);
			monitorType.GetMethod("Configure").Invoke(monitor, new object[] { armRoot.transform, forbiddenRoot.transform });

			yield return new WaitForFixedUpdate();

			bool hasCollision = (bool)monitorType.GetMethod("GetObservedCollisionState").Invoke(monitor, null);
			Assert.IsTrue(hasCollision, "The monitor should cache the overlapping collision state after a fixed step.");

			Object.Destroy(monitorGo);
			Object.Destroy(armColliderGo);
			Object.Destroy(forbiddenColliderGo);
			Object.Destroy(armRoot);
			Object.Destroy(forbiddenRoot);
		}

		private static System.Type LoadRuntimeType(string fullName)
		{
			System.Type type = System.Type.GetType($"{fullName}, Assembly-CSharp");
			Assert.IsNotNull(type, $"Runtime type '{fullName}' could not be loaded from Assembly-CSharp.");
			return type;
		}
	}
}
