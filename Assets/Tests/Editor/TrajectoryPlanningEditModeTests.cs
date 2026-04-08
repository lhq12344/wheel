using System.Collections.Generic;
using NUnit.Framework;
using UnityEngine;

namespace RobotSimulation.Tests.Editor
{
	public class TrajectoryPlanningEditModeTests
	{
		[Test]
		public void SceneDistanceFieldSampler_ReturnsLowerDistanceNearObstacle()
		{
			GameObject obstacle = GameObject.CreatePrimitive(PrimitiveType.Cube);
			obstacle.name = "DistanceFieldObstacle";
			obstacle.transform.position = Vector3.zero;
			obstacle.transform.localScale = new Vector3(2f, 2f, 2f);

			try
			{
				PlannerPhysicsQueries physicsQueries = new PlannerPhysicsQueries();
				List<Collider> obstacles = physicsQueries.CollectObstacleColliders(null);
				SceneDistanceFieldSampler sampler = new SceneDistanceFieldSampler();
				sampler.Build(new Vector3(-4f, 0f, -4f), new Vector3(4f, 0f, 4f), obstacles, 0.25f, 2f);

				float nearDistance = sampler.SampleDistance(new Vector3(0.6f, 0f, 0.6f));
				float farDistance = sampler.SampleDistance(new Vector3(4f, 0f, 4f));

				Assert.Less(nearDistance, farDistance, "Distance field should report a smaller clearance near the obstacle.");
			}
			finally
			{
				Object.DestroyImmediate(obstacle);
			}
		}

		[Test]
		public void BaseRrtStarPlanner_FindsPathAroundBlockingObstacle()
		{
			GameObject obstacle = GameObject.CreatePrimitive(PrimitiveType.Cube);
			obstacle.name = "PlannerObstacle";
			obstacle.transform.position = Vector3.zero;
			obstacle.transform.localScale = new Vector3(2f, 2f, 2f);

			try
			{
				PlannerPhysicsQueries physicsQueries = new PlannerPhysicsQueries
				{
					baseHalfHeight = 0.4f,
					sampleSpacing = 0.2f
				};
				List<Collider> obstacles = physicsQueries.CollectObstacleColliders(null);
				SceneDistanceFieldSampler sampler = new SceneDistanceFieldSampler();
				Vector3 start = new Vector3(-4f, 0f, -4f);
				Vector3 goal = new Vector3(4f, 0f, 4f);
				sampler.Build(start, goal, obstacles, 0.3f, 3f);

				BaseRrtStarPlanner planner = new BaseRrtStarPlanner();
				planner.settings.maxIterations = 1500;
				planner.settings.stepLength = 0.75f;
				planner.settings.goalThreshold = 0.9f;
				bool success = planner.TryPlan(start, 0f, goal, 45f, 0.4f, sampler, physicsQueries, obstacles, out List<Vector3> path, out string failureReason);

				Assert.IsTrue(success, failureReason);
				Assert.IsNotNull(path);
				Assert.IsNotEmpty(path);
				for (int i = 0; i < path.Count; i++)
				{
					Assert.IsTrue(physicsQueries.IsBasePoseCollisionFree(path[i], 0.4f, obstacles, out _), $"Waypoint {i} should be collision-free.");
				}
			}
			finally
			{
				Object.DestroyImmediate(obstacle);
			}
		}

		[Test]
		public void BasePoseBoxCollisionQuery_DetectsBlockingObstacle()
		{
			GameObject obstacle = GameObject.CreatePrimitive(PrimitiveType.Cube);
			obstacle.name = "BoxFilterObstacle";
			obstacle.transform.position = Vector3.zero;
			obstacle.transform.localScale = new Vector3(1.5f, 1f, 1.5f);

			try
			{
				PlannerPhysicsQueries physicsQueries = new PlannerPhysicsQueries
				{
					baseHalfHeight = 0.5f
				};
				List<Collider> obstacles = physicsQueries.CollectObstacleColliders(null);

				bool blockedAtOrigin = physicsQueries.IsBasePoseCheckBoxCollisionFree(
					Vector3.zero,
					Quaternion.identity,
					new Vector3(0.45f, 0.5f, 0.45f),
					obstacles,
					out Collider hitCollider);
				bool clearAwayFromObstacle = physicsQueries.IsBasePoseCheckBoxCollisionFree(
					new Vector3(3f, 0f, 3f),
					Quaternion.identity,
					new Vector3(0.45f, 0.5f, 0.45f),
					obstacles,
					out _);

				Assert.IsFalse(blockedAtOrigin, "The overlap-box query should detect the blocking obstacle at the origin.");
				Assert.IsNotNull(hitCollider, "The blocking collider should be reported.");
				Assert.IsTrue(clearAwayFromObstacle, "A distant base pose should remain collision-free.");
			}
			finally
			{
				Object.DestroyImmediate(obstacle);
			}
		}
	}
}
