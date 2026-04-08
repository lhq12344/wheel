using System.Collections.Generic;
using UnityEngine;

namespace RobotSimulation
{
	/// <summary>
	/// Shared physics queries used by the planning layer.
	/// </summary>
	public sealed class PlannerPhysicsQueries
	{
		private static readonly HashSet<int> UnsupportedClosestPointWarnings = new HashSet<int>();
		private struct BaseRadiusCacheEntry
		{
			public int colliderCount;
			public bool usedDriveGeometry;
			public float trackWidth;
			public float wheelRadius;
			public float radius;
		}

		private readonly Collider[] _overlapBuffer = new Collider[128];
		private readonly Dictionary<int, BaseRadiusCacheEntry> _baseRadiusCache = new Dictionary<int, BaseRadiusCacheEntry>();

		public float baseHalfHeight = 0.35f;
		public float sampleSpacing = 0.25f;

		public List<Collider> CollectObstacleColliders(params Transform[] ignoreRoots)
		{
			Collider[] colliders = Object.FindObjectsOfType<Collider>();
			List<Collider> result = new List<Collider>(colliders.Length);
			for (int i = 0; i < colliders.Length; i++)
			{
				Collider collider = colliders[i];
				if (!IsValidObstacle(collider, ignoreRoots))
				{
					continue;
				}

				result.Add(collider);
			}

			return result;
		}

		public float EstimateBaseRadius(DiffDriveTwinController controller)
		{
			if (controller == null)
			{
				return 0.32f;
			}

			controller.EnsureDriveGeometryReady();
			float maxRadius = 0.30f;
			Transform root = controller.rb != null ? controller.rb.transform : controller.transform;
			Collider[] colliders = root.GetComponentsInChildren<Collider>(true);
			int cacheKey = root != null ? root.GetInstanceID() : controller.GetInstanceID();
			bool usingDriveGeometry = controller.HasValidDriveGeometry;
			float trackWidth = usingDriveGeometry ? controller.TrackWidth : 0f;
			float wheelRadius = usingDriveGeometry ? controller.WheelRadius : 0f;
			if (_baseRadiusCache.TryGetValue(cacheKey, out BaseRadiusCacheEntry cachedEntry)
				&& cachedEntry.colliderCount == colliders.Length
				&& cachedEntry.usedDriveGeometry == usingDriveGeometry
				&& (!usingDriveGeometry
					|| (Mathf.Abs(cachedEntry.trackWidth - trackWidth) <= 1e-4f
						&& Mathf.Abs(cachedEntry.wheelRadius - wheelRadius) <= 1e-4f)))
			{
				return cachedEntry.radius;
			}

			Vector3 center = root.position;
			for (int i = 0; i < colliders.Length; i++)
			{
				Collider collider = colliders[i];
				if (!IsRelevantBaseCollider(collider, center.y))
				{
					continue;
				}

				Bounds bounds = collider.bounds;
				Vector3[] points =
				{
					bounds.center,
					bounds.min,
					bounds.max,
					new Vector3(bounds.min.x, bounds.center.y, bounds.max.z),
					new Vector3(bounds.max.x, bounds.center.y, bounds.min.z)
				};

				for (int p = 0; p < points.Length; p++)
				{
					Vector3 planar = points[p] - center;
					planar.y = 0f;
					maxRadius = Mathf.Max(maxRadius, planar.magnitude);
				}
			}

			float resolvedRadius = Mathf.Max(0.22f, maxRadius);
			if (usingDriveGeometry)
			{
				float geometryRadius = Mathf.Max(0.2f, (trackWidth * 0.5f) + wheelRadius + 0.05f);
				float cappedRadius = Mathf.Min(resolvedRadius, geometryRadius + 0.08f);
				if (cappedRadius < resolvedRadius - 1e-4f)
				{
					Debug.Log($"[PlannerPhysicsQueries] Base radius capped by drive geometry. colliderRadius={resolvedRadius:F3}, geometryRadius={geometryRadius:F3}, cappedRadius={cappedRadius:F3}");
					resolvedRadius = cappedRadius;
				}
			}

			_baseRadiusCache[cacheKey] = new BaseRadiusCacheEntry
			{
				colliderCount = colliders.Length,
				usedDriveGeometry = usingDriveGeometry,
				trackWidth = trackWidth,
				wheelRadius = wheelRadius,
				radius = resolvedRadius
			};
			return resolvedRadius;
		}

		private static bool IsRelevantBaseCollider(Collider collider, float baseCenterY)
		{
			if (collider == null || !collider.enabled || collider.isTrigger)
			{
				return false;
			}

			if (IsInternalRobotOrPreviewCollider(collider) || IsSupportSurface(collider))
			{
				return false;
			}

			Transform transform = collider.transform;
			if (HierarchyContainsToken(transform, "ArmCollisionPreview")
				|| HierarchyContainsToken(transform, "arm")
				|| HierarchyContainsToken(transform, "joint")
				|| HierarchyContainsToken(transform, "wrist"))
			{
				return false;
			}

			ArticulationBody ownerBody = collider.GetComponentInParent<ArticulationBody>();
			if (ownerBody != null)
			{
				return false;
			}

			Bounds bounds = collider.bounds;
			float colliderCenterOffsetY = Mathf.Abs(bounds.center.y - baseCenterY);
			if (colliderCenterOffsetY > 0.75f && bounds.extents.y < 0.5f)
			{
				return false;
			}

			return true;
		}

		public bool IsBasePoseCollisionFree(Vector3 position, float radius, IReadOnlyList<Collider> obstacles, out Collider hitCollider)
		{
			hitCollider = null;
			if (obstacles == null)
			{
				return true;
			}

			float halfHeight = Mathf.Max(0.1f, baseHalfHeight);
			Vector3 bottom = position + Vector3.up * radius;
			Vector3 top = bottom + Vector3.up * Mathf.Max(0.05f, (halfHeight * 2f) - (radius * 2f));
			int hitCount = Physics.OverlapCapsuleNonAlloc(bottom, top, Mathf.Max(0.05f, radius), _overlapBuffer, ~0, QueryTriggerInteraction.Ignore);
			for (int i = 0; i < hitCount; i++)
			{
				Collider overlap = _overlapBuffer[i];
				if (overlap == null)
				{
					continue;
				}

				if (!ContainsCollider(obstacles, overlap))
				{
					continue;
				}

				hitCollider = overlap;
				return false;
			}

			return true;
		}

		public bool IsSegmentCollisionFree(Vector3 start, Vector3 end, float radius, IReadOnlyList<Collider> obstacles, out Collider hitCollider)
		{
			return IsSegmentCollisionFree(start, end, radius, obstacles, out hitCollider, false);
		}

		public bool IsSegmentCollisionFree(Vector3 start, Vector3 end, float radius, IReadOnlyList<Collider> obstacles, out Collider hitCollider, bool allowStartOccupied)
		{
			hitCollider = null;
			float distance = Vector3.Distance(start, end);
			int sampleCount = Mathf.Max(1, Mathf.CeilToInt(distance / Mathf.Max(0.05f, sampleSpacing)));
			bool escapedInitialOverlap = !allowStartOccupied;
			bool sawFreePose = false;
			for (int i = 0; i <= sampleCount; i++)
			{
				float t = sampleCount <= 0 ? 1f : i / (float)sampleCount;
				Vector3 point = Vector3.Lerp(start, end, t);
				if (!IsBasePoseCollisionFree(point, radius, obstacles, out hitCollider))
				{
					if (allowStartOccupied && !escapedInitialOverlap && i < sampleCount)
					{
						continue;
					}

					return false;
				}

				sawFreePose = true;
				escapedInitialOverlap = true;
			}

			return sawFreePose;
		}

		public bool HasDirectRaycastBlock(Vector3 start, Vector3 end, IReadOnlyList<Collider> obstacles, out Collider hitCollider)
		{
			hitCollider = null;
			Vector3 origin = start + Vector3.up * 0.2f;
			Vector3 destination = end + Vector3.up * 0.2f;
			Vector3 direction = destination - origin;
			float distance = direction.magnitude;
			if (distance <= 1e-5f)
			{
				return false;
			}

			if (Physics.Raycast(origin, direction.normalized, out RaycastHit hit, distance, ~0, QueryTriggerInteraction.Ignore))
			{
				if (ContainsCollider(obstacles, hit.collider))
				{
					hitCollider = hit.collider;
					return true;
				}
			}

			return false;
		}

		public float ComputeClearance(Vector3 position, IReadOnlyList<Collider> obstacles, float queryRadius = 0f)
		{
			if (obstacles == null || obstacles.Count == 0)
			{
				return 10f;
			}

			float minDistance = float.MaxValue;
			for (int i = 0; i < obstacles.Count; i++)
			{
				Collider collider = obstacles[i];
				if (collider == null || !collider.enabled || collider.isTrigger)
				{
					continue;
				}

				float distance = ComputeDistanceToCollider(position, collider) - queryRadius;
				minDistance = Mathf.Min(minDistance, distance);
			}

			if (minDistance == float.MaxValue)
			{
				return 10f;
			}

			return Mathf.Max(0f, minDistance);
		}

		private static float ComputeDistanceToCollider(Vector3 point, Collider collider)
		{
			if (SupportsClosestPoint(collider))
			{
				Vector3 closest = collider.ClosestPoint(point);
				return Vector3.Distance(point, closest);
			}

			int instanceId = collider.GetInstanceID();
			if (UnsupportedClosestPointWarnings.Add(instanceId))
			{
				Debug.LogWarning($"[PlannerPhysicsQueries] Collider '{collider.name}' does not support Collider.ClosestPoint. Falling back to bounds-based clearance.");
			}

			Vector3 closestBoundsPoint = collider.bounds.ClosestPoint(point);
			return Vector3.Distance(point, closestBoundsPoint);
		}

		private static bool SupportsClosestPoint(Collider collider)
		{
			if (collider is BoxCollider || collider is SphereCollider || collider is CapsuleCollider)
			{
				return true;
			}

			return collider is MeshCollider meshCollider && meshCollider.convex;
		}

		private static bool IsValidObstacle(Collider collider, Transform[] ignoreRoots)
		{
			if (collider == null || !collider.enabled || collider.isTrigger)
			{
				return false;
			}

			if (IsInternalRobotOrPreviewCollider(collider))
			{
				return false;
			}

			if (ignoreRoots != null)
			{
				for (int i = 0; i < ignoreRoots.Length; i++)
				{
					Transform ignoreRoot = ignoreRoots[i];
					if (ignoreRoot != null && collider.transform.IsChildOf(ignoreRoot))
					{
						return false;
					}
				}
			}

			if (IsSupportSurface(collider))
			{
				return false;
			}

			return collider.gameObject.activeInHierarchy;
		}

		private static bool IsInternalRobotOrPreviewCollider(Collider collider)
		{
			if (collider == null)
			{
				return false;
			}

			if ((collider.hideFlags & HideFlags.DontSave) != 0
				|| (collider.gameObject.hideFlags & HideFlags.DontSave) != 0)
			{
				return true;
			}

			if (StartsWithLinkToken(collider.transform.name)
				|| HierarchyContainsToken(collider.transform, "ArmCollisionPreview")
				|| HierarchyContainsToken(collider.transform, "Preview_"))
			{
				return true;
			}

			ArticulationBody ownerBody = collider.GetComponentInParent<ArticulationBody>();
			if (ownerBody != null && StartsWithLinkToken(ownerBody.name))
			{
				return true;
			}

			return false;
		}

		private static bool StartsWithLinkToken(string value)
		{
			return !string.IsNullOrEmpty(value)
				&& value.StartsWith("Link_", System.StringComparison.OrdinalIgnoreCase);
		}

		private static bool HierarchyContainsToken(Transform transform, string token)
		{
			if (transform == null || string.IsNullOrEmpty(token))
			{
				return false;
			}

			Transform current = transform;
			while (current != null)
			{
				if (current.name.IndexOf(token, System.StringComparison.OrdinalIgnoreCase) >= 0)
				{
					return true;
				}

				current = current.parent;
			}

			return false;
		}

		private static bool IsSupportSurface(Collider collider)
		{
			if (collider == null)
			{
				return false;
			}

			Bounds bounds = collider.bounds;
			Vector3 extents = bounds.extents;
			float maxHorizontalExtent = Mathf.Max(extents.x, extents.z);
			bool looksFlat = extents.y <= 0.15f;
			bool looksLarge = maxHorizontalExtent >= 2f;
			bool facesUp = Vector3.Dot(collider.transform.up, Vector3.up) >= 0.95f;
			bool obviousFloorName =
				collider.name.IndexOf("plane", System.StringComparison.OrdinalIgnoreCase) >= 0 ||
				collider.name.IndexOf("ground", System.StringComparison.OrdinalIgnoreCase) >= 0 ||
				collider.transform.name.IndexOf("plane", System.StringComparison.OrdinalIgnoreCase) >= 0 ||
				collider.transform.name.IndexOf("ground", System.StringComparison.OrdinalIgnoreCase) >= 0;

			return facesUp && (obviousFloorName || (looksFlat && looksLarge));
		}

		private static bool ContainsCollider(IReadOnlyList<Collider> colliders, Collider target)
		{
			if (colliders == null || target == null)
			{
				return false;
			}

			for (int i = 0; i < colliders.Count; i++)
			{
				if (colliders[i] == target)
				{
					return true;
				}
			}

			return false;
		}
	}
}
