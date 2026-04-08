using System.Collections.Generic;
using UnityEngine;

namespace RobotSimulation
{
	/// <summary>
	/// Lightweight 2D distance-field cache backed by scene colliders.
	/// </summary>
	public sealed class SceneDistanceFieldSampler
	{
		private static readonly HashSet<int> UnsupportedClosestPointWarnings = new HashSet<int>();

		private Vector3 _origin;
		private float _cellSize = 0.5f;
		private int _width;
		private int _height;
		private float _sampleHeight;
		private float[] _distances = new float[0];
		private List<Collider> _obstacles = new List<Collider>();

		public Bounds Bounds { get; private set; }
		public float CellSize => _cellSize;
		public bool IsReady => _distances != null && _distances.Length > 0;

		public void Build(Vector3 start, Vector3 goal, IReadOnlyList<Collider> obstacles, float cellSize, float margin)
		{
			_cellSize = Mathf.Max(0.1f, cellSize);
			_sampleHeight = Mathf.Lerp(start.y, goal.y, 0.5f);
			_obstacles = obstacles != null ? new List<Collider>(obstacles) : new List<Collider>();

			Vector3 min = Vector3.Min(start, goal);
			Vector3 max = Vector3.Max(start, goal);
			float safeMargin = Mathf.Max(2f, margin);
			min -= new Vector3(safeMargin, 0f, safeMargin);
			max += new Vector3(safeMargin, 0f, safeMargin);

			for (int i = 0; i < _obstacles.Count; i++)
			{
				Collider collider = _obstacles[i];
				if (collider == null)
				{
					continue;
				}

				Bounds bounds = collider.bounds;
				min = Vector3.Min(min, bounds.min - new Vector3(safeMargin, 0f, safeMargin));
				max = Vector3.Max(max, bounds.max + new Vector3(safeMargin, 0f, safeMargin));
			}

			float sizeX = Mathf.Max(_cellSize, max.x - min.x);
			float sizeZ = Mathf.Max(_cellSize, max.z - min.z);
			_width = Mathf.Max(2, Mathf.CeilToInt(sizeX / _cellSize) + 1);
			_height = Mathf.Max(2, Mathf.CeilToInt(sizeZ / _cellSize) + 1);
			_origin = new Vector3(min.x, _sampleHeight, min.z);
			Bounds = new Bounds(
				new Vector3(min.x + (sizeX * 0.5f), _sampleHeight, min.z + (sizeZ * 0.5f)),
				new Vector3(sizeX, 1f, sizeZ));
			_distances = new float[_width * _height];

			for (int z = 0; z < _height; z++)
			{
				for (int x = 0; x < _width; x++)
				{
					Vector3 point = GridToWorld(x, z);
					_distances[(z * _width) + x] = ComputeRawDistance(point);
				}
			}
		}

		public float SampleDistance(Vector3 worldPosition)
		{
			if (!IsReady)
			{
				return 10f;
			}

			float gx = Mathf.Clamp((worldPosition.x - _origin.x) / _cellSize, 0f, _width - 1);
			float gz = Mathf.Clamp((worldPosition.z - _origin.z) / _cellSize, 0f, _height - 1);

			int x0 = Mathf.FloorToInt(gx);
			int z0 = Mathf.FloorToInt(gz);
			int x1 = Mathf.Min(_width - 1, x0 + 1);
			int z1 = Mathf.Min(_height - 1, z0 + 1);

			float tx = gx - x0;
			float tz = gz - z0;
			float d00 = _distances[(z0 * _width) + x0];
			float d10 = _distances[(z0 * _width) + x1];
			float d01 = _distances[(z1 * _width) + x0];
			float d11 = _distances[(z1 * _width) + x1];

			float a = Mathf.Lerp(d00, d10, tx);
			float b = Mathf.Lerp(d01, d11, tx);
			return Mathf.Lerp(a, b, tz);
		}

		public float ComputeRepulsivePotential(Vector3 worldPosition, float influenceDistance = 1.5f)
		{
			float distance = SampleDistance(worldPosition);
			if (distance >= influenceDistance)
			{
				return 0f;
			}

			float clamped = Mathf.Max(0.01f, distance);
			float normalized = 1f - Mathf.Clamp01(clamped / influenceDistance);
			return normalized * normalized * (1f / clamped);
		}

		private Vector3 GridToWorld(int x, int z)
		{
			return new Vector3(_origin.x + (x * _cellSize), _sampleHeight, _origin.z + (z * _cellSize));
		}

		private float ComputeRawDistance(Vector3 point)
		{
			if (_obstacles == null || _obstacles.Count == 0)
			{
				return 10f;
			}

			float minDistance = float.MaxValue;
			for (int i = 0; i < _obstacles.Count; i++)
			{
				Collider collider = _obstacles[i];
				if (collider == null || !collider.enabled || collider.isTrigger)
				{
					continue;
				}

				float distance = ComputeDistanceToCollider(point, collider);
				minDistance = Mathf.Min(minDistance, distance);
			}

			return minDistance == float.MaxValue ? 10f : minDistance;
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
				Debug.LogWarning($"[SceneDistanceFieldSampler] Collider '{collider.name}' does not support Collider.ClosestPoint. Falling back to bounds-based clearance.");
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
	}
}
