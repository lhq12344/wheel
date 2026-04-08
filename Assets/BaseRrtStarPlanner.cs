using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace RobotSimulation
{
	[Serializable]
	public sealed class BaseRrtStarPlannerSettings
	{
		public int maxIterations = 1200;
		public float stepLength = 0.7f;
		public float goalBias = 0.2f;
		public float rewireRadius = 1.5f;
		public float goalThreshold = 0.8f;
		public float waypointMergeDistance = 0.25f;
		public float clearanceWeight = 0.5f;
	}

	public sealed class BaseRrtStarPlanner
	{
		private sealed class Node
		{
			public int parentIndex = -1;
			public Vector3 position;
			public float yawDeg;
			public float cost;
			public float clearance;
		}

		private readonly System.Random _random = new System.Random();

		public BaseRrtStarPlannerSettings settings = new BaseRrtStarPlannerSettings();

		public bool TryPlan(
			Vector3 startWorld,
			float startYawDeg,
			Vector3 goalWorld,
			float goalYawDeg,
			float baseRadius,
			SceneDistanceFieldSampler sampler,
			PlannerPhysicsQueries physicsQueries,
			IReadOnlyList<Collider> obstacles,
			out List<Vector3> waypoints,
			out string failureReason)
		{
			waypoints = new List<Vector3>();
			failureReason = string.Empty;

			if (physicsQueries == null)
			{
				failureReason = "Physics planner is not configured.";
				return false;
			}

			if (sampler == null || !sampler.IsReady)
			{
				failureReason = "Distance-field sampler is not ready.";
				return false;
			}

			if (!physicsQueries.IsBasePoseCollisionFree(goalWorld, baseRadius, obstacles, out Collider blockedGoal))
			{
				failureReason = $"Goal pose is occupied by '{blockedGoal?.name}'.";
				return false;
			}

			bool startOccupied = !physicsQueries.IsBasePoseCollisionFree(startWorld, baseRadius, obstacles, out Collider blockedStart);
			if (startOccupied)
			{
				Debug.LogWarning($"[BaseRrtStarPlanner] Start pose is currently occupied by '{blockedStart?.name}'. Planning will attempt an escape path instead of failing immediately.");
			}

			if (!physicsQueries.HasDirectRaycastBlock(startWorld, goalWorld, obstacles, out _)
				&& physicsQueries.IsSegmentCollisionFree(startWorld, goalWorld, baseRadius, obstacles, out _, startOccupied))
			{
				waypoints.Add(goalWorld);
				return true;
			}

			List<Node> nodes = new List<Node>(settings.maxIterations + 4)
			{
				new Node
				{
					position = startWorld,
					yawDeg = startYawDeg,
					cost = 0f,
					clearance = sampler.SampleDistance(startWorld)
				}
			};

			int goalNodeIndex = -1;
			Bounds bounds = sampler.Bounds;
			for (int iteration = 0; iteration < settings.maxIterations; iteration++)
			{
				Vector3 sample = SamplePoint(bounds, goalWorld);
				int nearestIndex = FindNearestNode(nodes, sample);
				Vector3 newPoint = Steer(nodes[nearestIndex].position, sample, settings.stepLength);

				if (!physicsQueries.IsBasePoseCollisionFree(newPoint, baseRadius, obstacles, out _))
				{
					continue;
				}

				if (physicsQueries.HasDirectRaycastBlock(nodes[nearestIndex].position, newPoint, obstacles, out _))
				{
					continue;
				}

				bool allowSegmentFromOccupiedRoot = startOccupied && nearestIndex == 0;
				if (!physicsQueries.IsSegmentCollisionFree(nodes[nearestIndex].position, newPoint, baseRadius, obstacles, out _, allowSegmentFromOccupiedRoot))
				{
					continue;
				}

				List<int> nearIndices = CollectNearNodes(nodes, newPoint, settings.rewireRadius);
				int bestParent = nearestIndex;
				float bestCost = ComputeTransitionCost(nodes[nearestIndex], newPoint, sampler);
				for (int i = 0; i < nearIndices.Count; i++)
				{
					int candidateIndex = nearIndices[i];
					Node candidate = nodes[candidateIndex];
					bool allowCandidateFromOccupiedRoot = startOccupied && candidateIndex == 0;
					if (!physicsQueries.IsSegmentCollisionFree(candidate.position, newPoint, baseRadius, obstacles, out _, allowCandidateFromOccupiedRoot))
					{
						continue;
					}

					float candidateCost = ComputeTransitionCost(candidate, newPoint, sampler);
					if (candidateCost < bestCost)
					{
						bestCost = candidateCost;
						bestParent = candidateIndex;
					}
				}

				Vector3 parentPosition = nodes[bestParent].position;
				Node newNode = new Node
				{
					parentIndex = bestParent,
					position = newPoint,
					yawDeg = DirectionToYawDeg(newPoint - parentPosition),
					cost = bestCost,
					clearance = sampler.SampleDistance(newPoint)
				};
				int newIndex = nodes.Count;
				nodes.Add(newNode);

				for (int i = 0; i < nearIndices.Count; i++)
				{
					int rewireIndex = nearIndices[i];
					if (rewireIndex == bestParent)
					{
						continue;
					}

					Node existing = nodes[rewireIndex];
					if (!physicsQueries.IsSegmentCollisionFree(newPoint, existing.position, baseRadius, obstacles, out _))
					{
						continue;
					}

					float rewireCost = ComputeTransitionCost(newNode, existing.position, sampler);
					if (rewireCost < existing.cost)
					{
						existing.parentIndex = newIndex;
						existing.cost = rewireCost;
						existing.yawDeg = DirectionToYawDeg(existing.position - newPoint);
					}
				}

				if (Vector3.Distance(newPoint, goalWorld) <= settings.goalThreshold
					&& physicsQueries.IsSegmentCollisionFree(newPoint, goalWorld, baseRadius, obstacles, out _))
				{
					goalNodeIndex = nodes.Count;
					nodes.Add(new Node
					{
						parentIndex = newIndex,
						position = goalWorld,
						yawDeg = goalYawDeg,
						cost = ComputeTransitionCost(newNode, goalWorld, sampler),
						clearance = sampler.SampleDistance(goalWorld)
					});
					break;
				}
			}

			if (goalNodeIndex < 0)
			{
				failureReason = startOccupied
					? $"RRT* could not find a collision-free escape path from the occupied start pose near '{blockedStart?.name}'."
					: "RRT* failed to connect the goal within the iteration budget.";
				return false;
			}

			waypoints = ReconstructWaypoints(nodes, goalNodeIndex);
			SimplifyWaypoints(waypoints, baseRadius, physicsQueries, obstacles);
			MergeCloseWaypoints(waypoints);
			return waypoints.Count > 0;
		}

		private Vector3 SamplePoint(Bounds bounds, Vector3 goalWorld)
		{
			if (_random.NextDouble() < settings.goalBias)
			{
				return goalWorld;
			}

			float x = Mathf.Lerp(bounds.min.x, bounds.max.x, (float)_random.NextDouble());
			float z = Mathf.Lerp(bounds.min.z, bounds.max.z, (float)_random.NextDouble());
			return new Vector3(x, goalWorld.y, z);
		}

		private static int FindNearestNode(List<Node> nodes, Vector3 sample)
		{
			int bestIndex = 0;
			float bestDistance = float.MaxValue;
			for (int i = 0; i < nodes.Count; i++)
			{
				float distance = Vector3.SqrMagnitude(nodes[i].position - sample);
				if (distance < bestDistance)
				{
					bestDistance = distance;
					bestIndex = i;
				}
			}

			return bestIndex;
		}

		private static Vector3 Steer(Vector3 from, Vector3 to, float maxStep)
		{
			Vector3 direction = to - from;
			float distance = direction.magnitude;
			if (distance <= maxStep || distance <= 1e-5f)
			{
				return to;
			}

			return from + (direction / distance) * maxStep;
		}

		private List<int> CollectNearNodes(List<Node> nodes, Vector3 point, float radius)
		{
			float sqrRadius = radius * radius;
			List<int> result = new List<int>();
			for (int i = 0; i < nodes.Count; i++)
			{
				if ((nodes[i].position - point).sqrMagnitude <= sqrRadius)
				{
					result.Add(i);
				}
			}

			return result;
		}

		private float ComputeTransitionCost(Node parent, Vector3 point, SceneDistanceFieldSampler sampler)
		{
			float distance = Vector3.Distance(parent.position, point);
			float repulsive = sampler.ComputeRepulsivePotential(point);
			return parent.cost + distance + (repulsive * settings.clearanceWeight);
		}

		private static List<Vector3> ReconstructWaypoints(List<Node> nodes, int goalNodeIndex)
		{
			List<Vector3> reverse = new List<Vector3>();
			int index = goalNodeIndex;
			while (index >= 0 && index < nodes.Count)
			{
				reverse.Add(nodes[index].position);
				index = nodes[index].parentIndex;
			}

			reverse.Reverse();
			if (reverse.Count > 0)
			{
				reverse.RemoveAt(0);
			}

			return reverse;
		}

		private static void SimplifyWaypoints(List<Vector3> waypoints, float baseRadius, PlannerPhysicsQueries physicsQueries, IReadOnlyList<Collider> obstacles)
		{
			if (waypoints == null || waypoints.Count < 3)
			{
				return;
			}

			int startIndex = 0;
			while (startIndex < waypoints.Count - 2)
			{
				int farthest = startIndex + 2;
				while (farthest < waypoints.Count)
				{
					if (!physicsQueries.IsSegmentCollisionFree(waypoints[startIndex], waypoints[farthest], baseRadius, obstacles, out _))
					{
						break;
					}

					farthest++;
				}

				int removeUntil = farthest - 1;
				while (removeUntil > startIndex + 1)
				{
					waypoints.RemoveAt(startIndex + 1);
					removeUntil--;
				}

				startIndex++;
			}
		}

		private void MergeCloseWaypoints(List<Vector3> waypoints)
		{
			if (waypoints == null || waypoints.Count < 2)
			{
				return;
			}

			for (int i = waypoints.Count - 2; i >= 0; i--)
			{
				if (Vector3.Distance(waypoints[i], waypoints[i + 1]) <= settings.waypointMergeDistance)
				{
					waypoints.RemoveAt(i);
				}
			}
		}

		private static float DirectionToYawDeg(Vector3 direction)
		{
			direction.y = 0f;
			if (direction.sqrMagnitude <= 1e-6f)
			{
				return 0f;
			}

			return Mathf.Atan2(direction.x, direction.z) * Mathf.Rad2Deg;
		}
	}

	public sealed class BasePathFollower
	{
		private const float LookaheadDistance = 0.35f;
		private const float DockingDistance = 0.40f;
		private const float PositionTolerance = 0.02f;
		private const float HandoffTolerance = 0.025f;
		private const float NearGoalStallGraceTolerance = 0.035f;
		private const float NearGoalTimeoutAcceptanceTolerance = 0.04f;
		private const float RelaxedSpeedTolerance = 0.06f;
		private const float RelaxedYawRateTolerance = 0.10f;
		private const float HandoffSpeedTolerance = 0.10f;
		private const float HandoffYawRateTolerance = 0.12f;
		private const float MinimumCruiseSpeed = 0.28f;
		private const float MinimumDockingSpeed = 0.18f;
		private const float NearGoalDockingSpeed = 0.10f;
		private const float StallTimeoutSeconds = 2.5f;

		private sealed class PreparedPath
		{
			public readonly List<Vector3> points = new List<Vector3>();
			public float[] cumulativeDistances = Array.Empty<float>();
			public float totalLength;
		}

		public IEnumerator FollowWaypoints(
			DiffDriveTwinController controller,
			IReadOnlyList<Vector3> waypoints,
			float finalYawDeg,
			bool alignFinalYaw,
			System.Func<bool> shouldCancel,
			System.Action<bool, string> onComplete)
		{
			if (controller == null || controller.rb == null)
			{
				onComplete?.Invoke(false, "DiffDrive controller is missing.");
				yield break;
			}

			if (!controller.EnsureDriveGeometryReady())
			{
				onComplete?.Invoke(false, "DiffDrive geometry is invalid. Check wheel references or auto-detection.");
				yield break;
			}

			PreparedPath path = BuildPath(controller.rb.position, waypoints);
			if (path.points.Count <= 1 || path.totalLength <= 1e-4f)
			{
				controller.ClearVelocityCommand(true);
				onComplete?.Invoke(true, "Base path is empty, so no base motion is required.");
				yield break;
			}

			float timeout = EstimatePathTimeout(controller, path.totalLength);
			float elapsed = 0f;
			float stallElapsed = 0f;
			float bestRemaining = path.totalLength;
			Vector3 finalGoal = path.points[path.points.Count - 1];

			while (elapsed < timeout)
			{
				if (shouldCancel != null && shouldCancel())
				{
					controller.ClearVelocityCommand(true);
					onComplete?.Invoke(false, "Base path following was cancelled.");
					yield break;
				}

				Vector3 current = controller.rb.position;
				float progress = FindProgressAlongPath(current, path, out _, out _);
				float remaining = ComputeRemainingDistance(path, progress);
				float planarGoalError = Vector3.Distance(ProjectXZ(current), ProjectXZ(finalGoal));
				bool insideNearGoalStallGrace = remaining <= DockingDistance && planarGoalError <= NearGoalStallGraceTolerance;
				if (remaining < bestRemaining - 0.01f)
				{
					bestRemaining = remaining;
					stallElapsed = 0f;
				}
				else if (!insideNearGoalStallGrace && planarGoalError > HandoffTolerance)
				{
					stallElapsed += Time.fixedDeltaTime;
				}
				else
				{
					stallElapsed = 0f;
				}

				if (planarGoalError <= HandoffTolerance
					&& remaining <= DockingDistance)
				{
					controller.CompletePointGoal(finalGoal);
					onComplete?.Invoke(true, "Base path tracking completed with a terminal stop.");
					yield break;
				}

			if (planarGoalError <= NearGoalTimeoutAcceptanceTolerance
				&& remaining <= DockingDistance
				&& controller.CurrentPlanarSpeedMeasured <= RelaxedSpeedTolerance
				&& controller.CurrentYawRateMeasured <= RelaxedYawRateTolerance)
			{
				controller.CompletePointGoal(finalGoal);
					onComplete?.Invoke(true, "Base path tracking accepted near-goal settling and stopped.");
					yield break;
				}

				if (planarGoalError <= PositionTolerance)
				{
					controller.CompletePointGoal(finalGoal);
					onComplete?.Invoke(true, "Base path tracking reached the terminal docking zone and stopped.");
					yield break;
				}
				else if (stallElapsed > StallTimeoutSeconds)
				{
					controller.ClearVelocityCommand(true);
					onComplete?.Invoke(false, $"Base stalled while tracking the planned path. Remaining planar distance={planarGoalError:F3}m.");
					yield break;
				}
				else
				{
					Vector3 trackingPoint = remaining > DockingDistance
						? SamplePathPointAtDistance(path, progress + LookaheadDistance)
						: finalGoal;
					if (remaining > DockingDistance)
					{
						ComputePurePursuitCommand(controller, trackingPoint, remaining, out float linearVelocity, out float angularVelocity);
						controller.SetVelocityCommand(linearVelocity, angularVelocity, trackingPoint);
					}
					else
					{
						ComputeDockingCommand(controller, finalGoal, remaining, out float linearVelocity, out float angularVelocity);
						controller.SetVelocityCommand(linearVelocity, angularVelocity, finalGoal);
					}
				}

				elapsed += Time.fixedDeltaTime;
				yield return new WaitForFixedUpdate();
			}

			controller.ClearVelocityCommand(true);
			float finalError = Vector3.Distance(ProjectXZ(controller.rb.position), ProjectXZ(finalGoal));
			if (finalError <= NearGoalTimeoutAcceptanceTolerance
				&& controller.CurrentPlanarSpeedMeasured <= RelaxedSpeedTolerance
				&& controller.CurrentYawRateMeasured <= RelaxedYawRateTolerance)
			{
				controller.CompletePointGoal(finalGoal);
				onComplete?.Invoke(true, $"Base path tracking accepted near-goal completion on timeout. Remaining planar distance={finalError:F3}m.");
				yield break;
			}

			onComplete?.Invoke(false, $"Timed out while tracking the base path. Remaining planar distance={finalError:F3}m.");
		}

		private static void ComputePurePursuitCommand(DiffDriveTwinController controller, Vector3 trackingPoint, float remainingDistance, out float linearVelocity, out float angularVelocity)
		{
			Vector3 localTarget = controller.transform.InverseTransformPoint(trackingPoint);
			localTarget.y = 0f;
			float lookahead = Mathf.Max(0.15f, new Vector2(localTarget.x, localTarget.z).magnitude);
			float headingError = Mathf.Atan2(localTarget.x, Mathf.Max(0.02f, localTarget.z));
			float curvature = (2f * localTarget.x) / Mathf.Max(0.04f, lookahead * lookahead);
			float cornerSpeedLimit = Mathf.Abs(curvature) > 1e-3f
				? controller.wMax / Mathf.Abs(curvature)
				: controller.vMax;
			float brakingBound = Mathf.Sqrt(Mathf.Max(0f, 2f * Mathf.Max(controller.aMax, 0.1f) * Mathf.Max(remainingDistance - DockingDistance, 0f))) + MinimumDockingSpeed;
			float headingScale = Mathf.Clamp01((localTarget.z / lookahead + 1f) * 0.5f);
			linearVelocity = Mathf.Min(controller.vMax, Mathf.Min(cornerSpeedLimit, brakingBound));
			linearVelocity = Mathf.Max(Mathf.Min(linearVelocity, controller.vMax), MinimumCruiseSpeed);
			linearVelocity *= Mathf.Lerp(0.55f, 1f, headingScale);
			if (localTarget.z < 0.02f && Mathf.Abs(localTarget.x) > 0.08f)
			{
				linearVelocity = 0f;
			}

			float pursuitAngular = linearVelocity * curvature;
			float headingAngular = controller.kYaw * headingError;
			angularVelocity = Mathf.Clamp(Mathf.Lerp(headingAngular, pursuitAngular, 0.65f), -controller.wMax, controller.wMax);
		}

		private static void ComputeDockingCommand(DiffDriveTwinController controller, Vector3 finalGoal, float remainingDistance, out float linearVelocity, out float angularVelocity)
		{
			Vector3 localGoal = controller.transform.InverseTransformPoint(finalGoal);
			localGoal.y = 0f;
			float planarDistance = new Vector2(localGoal.x, localGoal.z).magnitude;
			float headingError = Mathf.Atan2(localGoal.x, Mathf.Max(0.02f, localGoal.z));
			float brakingDistance = Mathf.Max(0f, remainingDistance - PositionTolerance);
			float brakingBound = Mathf.Sqrt(Mathf.Max(0f, 2f * Mathf.Max(controller.aMax, 0.1f) * brakingDistance));
			float headingScale = Mathf.Clamp01(Mathf.Cos(headingError));
			linearVelocity = Mathf.Min(controller.vMax * 0.55f, brakingBound);
			if (planarDistance > 0.08f)
			{
				linearVelocity = Mathf.Max(linearVelocity, MinimumDockingSpeed);
			}
			else if (planarDistance > PositionTolerance)
			{
				linearVelocity = Mathf.Max(linearVelocity, NearGoalDockingSpeed);
			}

			linearVelocity *= Mathf.Lerp(0.45f, 1f, headingScale);
			if (planarDistance <= PositionTolerance)
			{
				linearVelocity = 0f;
			}
			else if (localGoal.z < 0.02f && Mathf.Abs(localGoal.x) > 0.05f)
			{
				linearVelocity = 0f;
			}

			float curvature = (2f * localGoal.x) / Mathf.Max(0.03f, planarDistance * planarDistance);
			float pursuitAngular = linearVelocity * curvature;
			float headingAngular = controller.kYaw * headingError;
			float maxYawRate = Mathf.Lerp(0.35f, 0.9f, Mathf.InverseLerp(PositionTolerance, DockingDistance, planarDistance));
			angularVelocity = Mathf.Clamp(
				Mathf.Abs(linearVelocity) > 0.02f ? Mathf.Lerp(headingAngular, pursuitAngular, 0.5f) : headingAngular,
				-maxYawRate,
				maxYawRate);
		}

		private static PreparedPath BuildPath(Vector3 start, IReadOnlyList<Vector3> waypoints)
		{
			PreparedPath path = new PreparedPath();
			Vector3 normalizedStart = start;
			path.points.Add(normalizedStart);
			if (waypoints == null)
			{
				UpdatePathMetrics(path);
				return path;
			}

			for (int i = 0; i < waypoints.Count; i++)
			{
				Vector3 waypoint = waypoints[i];
				waypoint.y = normalizedStart.y;
				if (path.points.Count == 0 || Vector3.Distance(ProjectXZ(path.points[path.points.Count - 1]), ProjectXZ(waypoint)) > 0.01f)
				{
					path.points.Add(waypoint);
				}
			}

			UpdatePathMetrics(path);
			return path;
		}

		private static void UpdatePathMetrics(PreparedPath path)
		{
			if (path == null)
			{
				return;
			}

			int pointCount = path.points.Count;
			if (path.cumulativeDistances == null || path.cumulativeDistances.Length != pointCount)
			{
				path.cumulativeDistances = pointCount > 0 ? new float[pointCount] : Array.Empty<float>();
			}

			float cumulative = 0f;
			for (int i = 1; i < pointCount; i++)
			{
				cumulative += Vector3.Distance(ProjectXZ(path.points[i - 1]), ProjectXZ(path.points[i]));
				path.cumulativeDistances[i] = cumulative;
			}

			path.totalLength = cumulative;
		}

		private static float EstimatePathTimeout(DiffDriveTwinController controller, float totalLength)
		{
			float driveSeconds = totalLength / Mathf.Max(0.15f, controller.vMax * 0.7f);
			return Mathf.Max(5f, (driveSeconds * 4f) + 2.5f);
		}

		private static float FindProgressAlongPath(Vector3 position, PreparedPath path, out int segmentIndex, out Vector3 projectedPoint)
		{
			segmentIndex = 0;
			projectedPoint = path != null && path.points.Count > 0 ? path.points[0] : position;
			if (path == null || path.points.Count < 2)
			{
				return 0f;
			}

			float bestDistanceSqr = float.MaxValue;
			float bestProgress = 0f;
			for (int i = 0; i < path.points.Count - 1; i++)
			{
				Vector3 a = ProjectXZ(path.points[i]);
				Vector3 b = ProjectXZ(path.points[i + 1]);
				Vector3 segment = b - a;
				float segmentLength = segment.magnitude;
				if (segmentLength <= 1e-5f)
				{
					continue;
				}

				float t = Mathf.Clamp01(Vector3.Dot(ProjectXZ(position) - a, segment) / Mathf.Max(1e-5f, segment.sqrMagnitude));
				Vector3 projected = a + segment * t;
				float distanceSqr = (ProjectXZ(position) - projected).sqrMagnitude;
				if (distanceSqr < bestDistanceSqr)
				{
					bestDistanceSqr = distanceSqr;
					bestProgress = path.cumulativeDistances[i] + (segmentLength * t);
					segmentIndex = i;
					projectedPoint = new Vector3(projected.x, path.points[i].y, projected.z);
				}
			}

			return bestProgress;
		}

		private static float ComputeRemainingDistance(PreparedPath path, float progress)
		{
			return path == null ? 0f : Mathf.Max(0f, path.totalLength - progress);
		}

		private static Vector3 SamplePathPointAtDistance(PreparedPath path, float distanceAlongPath)
		{
			if (path == null || path.points.Count == 0)
			{
				return Vector3.zero;
			}

			if (path.points.Count == 1)
			{
				return path.points[0];
			}

			float targetDistance = Mathf.Clamp(distanceAlongPath, 0f, path.totalLength);
			for (int i = 0; i < path.points.Count - 1; i++)
			{
				Vector3 from = path.points[i];
				Vector3 to = path.points[i + 1];
				float segmentLength = Vector3.Distance(ProjectXZ(from), ProjectXZ(to));
				if (segmentLength <= 1e-5f)
				{
					continue;
				}

				float segmentStartDistance = path.cumulativeDistances[i];
				if (segmentStartDistance + segmentLength >= targetDistance)
				{
					float t = (targetDistance - segmentStartDistance) / segmentLength;
					return Vector3.Lerp(from, to, t);
				}
			}

			return path.points[path.points.Count - 1];
		}

		private static Vector3 ProjectXZ(Vector3 value)
		{
			return new Vector3(value.x, 0f, value.z);
		}
	}
}
