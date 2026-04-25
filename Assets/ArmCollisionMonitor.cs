using System.Collections.Generic;
using UnityEngine;

namespace RobotSimulation
{
	/// <summary>
	/// Monitors collisions between the arm subtree and forbidden colliders such as chassis and wheels.
	/// Uses overlap + penetration checks so it still works when the arm root is teleported each physics step.
	/// </summary>
	public class ArmCollisionMonitor : MonoBehaviour
	{
		private sealed class PreviewColliderEntry
		{
			public GameObject gameObject;
			public Collider collider;
			public string sourceName;
			public string ownerLinkName;
			public Vector3 localPosition;
			public Quaternion localRotation = Quaternion.identity;
			public Vector3 lossyScale = Vector3.one;
		}

		public static ArmCollisionMonitor Instance { get; private set; }

		[Header("References")]
		public Transform armRoot;
		public Transform forbiddenRoot;

		[Header("Detection")]
		public bool monitorInFixedUpdate = true;
		public float penetrationEpsilon = 0.0001f;
		public bool ignoreArmBaseColliders = true;
		public string[] ignoredArmColliderNameContains = { "Link_00" };
		public string[] ignoredForbiddenColliderNameContains = new string[0];

		[Header("Motion Guard Preview")]
		public float previewStepDegrees = 5f;
		public int maxPreviewSamples = 48;

		[Header("Status")]
		[SerializeField] private bool _hasCollision;
		[SerializeField] private string _activeCollisionMessage = string.Empty;
		[SerializeField] private string _lastCollisionMessage = string.Empty;
		[SerializeField] private int _armColliderCount;
		[SerializeField] private int _forbiddenColliderCount;
		[SerializeField] private string _lastRefreshSummary = string.Empty;
		[SerializeField] private string _lastLoggedRefreshSummary = string.Empty;

		private readonly List<Collider> _armColliders = new List<Collider>();
		private readonly List<Collider> _forbiddenColliders = new List<Collider>();
		private readonly List<PreviewColliderEntry> _previewColliders = new List<PreviewColliderEntry>();
		private ArticulationBody _armRootBody;
		private GameObject _previewRoot;
		private float _lastCollisionEvaluationFixedTime = float.NegativeInfinity;

		public bool HasCollision => _hasCollision;
		public string ActiveCollisionMessage => _activeCollisionMessage;
		public string LastCollisionMessage => _lastCollisionMessage;
		public int ArmColliderCount => _armColliderCount;
		public int ForbiddenColliderCount => _forbiddenColliderCount;
		public string LastRefreshSummary => _lastRefreshSummary;

		private void Awake()
		{
			if (Instance == null)
			{
				Instance = this;
			}

			RefreshColliders();
		}

		private void FixedUpdate()
		{
			if (monitorInFixedUpdate)
			{
				GetObservedCollisionState();
			}
		}

		public void Configure(Transform newArmRoot, Transform newForbiddenRoot)
		{
			bool rootsChanged = armRoot != newArmRoot || forbiddenRoot != newForbiddenRoot;
			armRoot = newArmRoot;
			forbiddenRoot = newForbiddenRoot;
			if (!rootsChanged && _armColliderCount > 0 && _forbiddenColliderCount > 0)
			{
				EvaluateCollisionState();
				return;
			}

			RefreshColliders();
			EvaluateCollisionState();
		}

		public void RefreshColliders()
		{
			_armColliders.Clear();
			_forbiddenColliders.Clear();
			_armRootBody = ResolveArmRootBody();

			if (armRoot != null)
			{
				CollectCollidersRecursive(armRoot, _armColliders, null);
			}

			if (forbiddenRoot != null)
			{
				HashSet<Collider> armColliderSet = new HashSet<Collider>(_armColliders);
				CollectCollidersRecursive(forbiddenRoot, _forbiddenColliders, armColliderSet);
			}

			_armColliderCount = CountUsableArmColliders();
			_forbiddenColliderCount = CountUsableForbiddenColliders();
			_lastRefreshSummary = $"armRoot={(armRoot != null ? armRoot.name : "null")}, forbiddenRoot={(forbiddenRoot != null ? forbiddenRoot.name : "null")}, armColliders={_armColliderCount}, forbiddenColliders={_forbiddenColliderCount}";
			if (_lastLoggedRefreshSummary != _lastRefreshSummary)
			{
				_lastLoggedRefreshSummary = _lastRefreshSummary;
				Debug.Log($"[ArmCollision] Monitor configured: {_lastRefreshSummary}");
			}
			if (_armColliderCount == 0 || _forbiddenColliderCount == 0)
			{
				Debug.LogWarning("[ArmCollision] Collision monitor collected zero usable arm colliders or forbidden colliders. Collision guard will not be effective until this is fixed.");
			}

			RebuildPreviewColliders();
		}

		public bool EvaluateCollisionState()
		{
			if (_armColliders.Count == 0 || _forbiddenColliders.Count == 0)
			{
				RefreshColliders();
			}

			string collisionMessage = string.Empty;
			bool collided = TryFindCollision(out collisionMessage);
			_hasCollision = collided;
			_activeCollisionMessage = collided ? collisionMessage : string.Empty;

			if (collided && collisionMessage != _lastCollisionMessage)
			{
				_lastCollisionMessage = collisionMessage;
				Debug.LogError($"[ArmCollision] {collisionMessage}");
			}

			_lastCollisionEvaluationFixedTime = Time.fixedTime;
			return collided;
		}

		public bool GetObservedCollisionState()
		{
			if (!monitorInFixedUpdate)
			{
				return EvaluateCollisionState();
			}

			if (Mathf.Abs(_lastCollisionEvaluationFixedTime - Time.fixedTime) > 1e-5f)
			{
				return EvaluateCollisionState();
			}

			return _hasCollision;
		}

		public bool EvaluatePredictedCollision(Pose[] linkWorldPoses, out ArmCollisionGuardResult result)
		{
			result = new ArmCollisionGuardResult
			{
				allowed = true
			};

			if (_armColliders.Count == 0 || _forbiddenColliders.Count == 0 || _previewColliders.Count == 0)
			{
				RefreshColliders();
			}

			if (linkWorldPoses == null || linkWorldPoses.Length < 7)
			{
				result.message = "Predicted collision check failed: link poses are incomplete.";
				return false;
			}

			if (_previewColliders.Count == 0 || _forbiddenColliders.Count == 0)
			{
				return false;
			}

			for (int i = 0; i < _previewColliders.Count; i++)
			{
				PreviewColliderEntry entry = _previewColliders[i];
				if (entry == null || entry.collider == null)
				{
					continue;
				}

				if (!TryGetLinkWorldPose(entry.ownerLinkName, linkWorldPoses, out Pose ownerPose))
				{
					continue;
				}

				entry.gameObject.transform.SetPositionAndRotation(
					ownerPose.position + ownerPose.rotation * entry.localPosition,
					ownerPose.rotation * entry.localRotation);
				entry.gameObject.transform.localScale = entry.lossyScale;
			}

			for (int i = 0; i < _previewColliders.Count; i++)
			{
				PreviewColliderEntry armEntry = _previewColliders[i];
				if (armEntry == null || !IsUsableCollider(armEntry.collider))
				{
					continue;
				}

				for (int j = 0; j < _forbiddenColliders.Count; j++)
				{
					Collider forbiddenCollider = _forbiddenColliders[j];
					if (!IsUsableCollider(forbiddenCollider) || IsIgnoredForbiddenCollider(forbiddenCollider))
					{
						continue;
					}

					if (!armEntry.collider.bounds.Intersects(forbiddenCollider.bounds))
					{
						continue;
					}

					Vector3 direction;
					float distance;
					bool overlapping = Physics.ComputePenetration(
						armEntry.collider, armEntry.collider.transform.position, armEntry.collider.transform.rotation,
						forbiddenCollider, forbiddenCollider.transform.position, forbiddenCollider.transform.rotation,
						out direction, out distance);

					float effectivePenetrationThreshold = GetEffectivePenetrationThreshold(armEntry.collider, forbiddenCollider);
					if (!overlapping || distance <= effectivePenetrationThreshold)
					{
						continue;
					}

					result.allowed = false;
					result.blockedByForbiddenCollision = true;
					result.armColliderName = armEntry.sourceName;
					result.forbiddenColliderName = forbiddenCollider.transform.name;
					result.message = $"Arm motion blocked: '{armEntry.sourceName}' would collide with '{forbiddenCollider.transform.name}' (penetration {distance:F4}m).";
					ParkPreviewColliders();
					return true;
				}
			}

			ParkPreviewColliders();
			return false;
		}

		private bool TryFindCollision(out string collisionMessage)
		{
			collisionMessage = string.Empty;

			for (int i = 0; i < _armColliders.Count; i++)
			{
				Collider armCollider = _armColliders[i];
				if (!IsUsableCollider(armCollider) || IsIgnoredArmCollider(armCollider))
				{
					continue;
				}

				for (int j = 0; j < _forbiddenColliders.Count; j++)
				{
					Collider forbiddenCollider = _forbiddenColliders[j];
					if (!IsUsableCollider(forbiddenCollider) || IsIgnoredForbiddenCollider(forbiddenCollider))
					{
						continue;
					}

					if (!armCollider.bounds.Intersects(forbiddenCollider.bounds))
					{
						continue;
					}

					Vector3 direction;
					float distance;
					bool overlapping = Physics.ComputePenetration(
						armCollider, armCollider.transform.position, armCollider.transform.rotation,
						forbiddenCollider, forbiddenCollider.transform.position, forbiddenCollider.transform.rotation,
						out direction, out distance);

					float effectivePenetrationThreshold = GetEffectivePenetrationThreshold(armCollider, forbiddenCollider);
					if (!overlapping || distance <= effectivePenetrationThreshold)
					{
						continue;
					}

					collisionMessage = $"Arm collider '{armCollider.transform.name}' collided with '{forbiddenCollider.transform.name}' (penetration {distance:F4}m).";
					return true;
				}
			}

			return false;
		}

		private static bool IsUsableCollider(Collider collider)
		{
			return collider != null
				&& collider.enabled
				&& collider.gameObject.activeInHierarchy;
		}

		private float GetEffectivePenetrationThreshold(Collider primary, Collider secondary)
		{
			return Mathf.Max(
				Mathf.Max(0.0001f, penetrationEpsilon),
				GetSafeContactOffset(primary) + GetSafeContactOffset(secondary));
		}

		private static float GetSafeContactOffset(Collider collider)
		{
			return collider != null ? Mathf.Max(0f, collider.contactOffset) : 0f;
		}

		private bool IsIgnoredArmCollider(Collider collider)
		{
			if (collider == null)
			{
				return true;
			}

			if (ignoreArmBaseColliders && _armRootBody != null)
			{
				ArticulationBody ownerBody = collider.GetComponentInParent<ArticulationBody>();
				if (ownerBody == _armRootBody)
				{
					return true;
				}
			}

			return ColliderOrOwnerMatchesAnyToken(collider, ignoredArmColliderNameContains);
		}

		private bool IsIgnoredForbiddenCollider(Collider collider)
		{
			if (collider == null)
			{
				return true;
			}

			return HierarchyNameMatchesAnyToken(collider.transform, ignoredForbiddenColliderNameContains);
		}

		private ArticulationBody ResolveArmRootBody()
		{
			if (armRoot == null)
			{
				return null;
			}

			ArticulationBody direct = armRoot.GetComponent<ArticulationBody>();
			if (direct != null)
			{
				return direct;
			}

			return armRoot.GetComponentInChildren<ArticulationBody>(true);
		}

		private static bool HierarchyNameMatchesAnyToken(Transform transform, string[] tokens)
		{
			Transform current = transform;
			while (current != null)
			{
				if (NameMatchesAnyToken(current.name, tokens))
				{
					return true;
				}

				current = current.parent;
			}

			return false;
		}

		private static bool NameMatchesAnyToken(string value, string[] tokens)
		{
			if (string.IsNullOrEmpty(value) || tokens == null || tokens.Length == 0)
			{
				return false;
			}

			for (int i = 0; i < tokens.Length; i++)
			{
				string token = tokens[i];
				if (string.IsNullOrWhiteSpace(token))
				{
					continue;
				}

				if (value.IndexOf(token, System.StringComparison.OrdinalIgnoreCase) >= 0)
				{
					return true;
				}
			}

			return false;
		}

		private static bool ColliderOrOwnerMatchesAnyToken(Collider collider, string[] tokens)
		{
			if (collider == null)
			{
				return false;
			}

			if (NameMatchesAnyToken(collider.transform.name, tokens))
			{
				return true;
			}

			ArticulationBody ownerBody = collider.GetComponentInParent<ArticulationBody>();
			if (ownerBody != null && NameMatchesAnyToken(ownerBody.name, tokens))
			{
				return true;
			}

			return false;
		}

		private void CollectCollidersRecursive(Transform root, List<Collider> output, HashSet<Collider> exclusionSet)
		{
			if (root == null)
			{
				return;
			}

			Collider[] colliders = root.GetComponentsInChildren<Collider>(true);
			for (int i = 0; i < colliders.Length; i++)
			{
				Collider collider = colliders[i];
				if (collider == null)
				{
					continue;
				}

				if (armRoot != null && root == forbiddenRoot && collider.transform.IsChildOf(armRoot))
				{
					continue;
				}

				if (exclusionSet != null && exclusionSet.Contains(collider))
				{
					continue;
				}

				if (!output.Contains(collider))
				{
					output.Add(collider);
				}
			}
		}

		private int CountUsableArmColliders()
		{
			int count = 0;
			for (int i = 0; i < _armColliders.Count; i++)
			{
				if (IsUsableCollider(_armColliders[i]) && !IsIgnoredArmCollider(_armColliders[i]))
				{
					count++;
				}
			}

			return count;
		}

		private int CountUsableForbiddenColliders()
		{
			int count = 0;
			for (int i = 0; i < _forbiddenColliders.Count; i++)
			{
				if (IsUsableCollider(_forbiddenColliders[i]) && !IsIgnoredForbiddenCollider(_forbiddenColliders[i]))
				{
					count++;
				}
			}

			return count;
		}

		private void RebuildPreviewColliders()
		{
			DestroyPreviewColliders();
			EnsurePreviewRoot();
			if (_previewRoot == null)
			{
				return;
			}

			for (int i = 0; i < _armColliders.Count; i++)
			{
				Collider source = _armColliders[i];
				if (!IsUsableCollider(source) || IsIgnoredArmCollider(source))
				{
					continue;
				}

				PreviewColliderEntry entry = CreatePreviewCollider(source);
				if (entry != null)
				{
					_previewColliders.Add(entry);
				}
			}

			ParkPreviewColliders();
		}

		private void EnsurePreviewRoot()
		{
			if (_previewRoot != null)
			{
				return;
			}

			_previewRoot = new GameObject("ArmCollisionPreview");
			_previewRoot.hideFlags = HideFlags.HideAndDontSave;
			_previewRoot.transform.SetParent(transform, false);
		}

		private void DestroyPreviewColliders()
		{
			for (int i = 0; i < _previewColliders.Count; i++)
			{
				PreviewColliderEntry entry = _previewColliders[i];
				if (entry?.gameObject == null)
				{
					continue;
				}

				if (Application.isPlaying)
				{
					Destroy(entry.gameObject);
				}
				else
				{
					DestroyImmediate(entry.gameObject);
				}
			}

			_previewColliders.Clear();

			if (_previewRoot != null && _previewRoot.transform.childCount == 0)
			{
				if (Application.isPlaying)
				{
					Destroy(_previewRoot);
				}
				else
				{
					DestroyImmediate(_previewRoot);
				}

				_previewRoot = null;
			}
		}

		private PreviewColliderEntry CreatePreviewCollider(Collider source)
		{
			ArticulationBody ownerBody = source.GetComponentInParent<ArticulationBody>();
			Transform ownerTransform = ownerBody != null ? ownerBody.transform : armRoot;
			if (ownerTransform == null)
			{
				return null;
			}

			GameObject previewObject = new GameObject($"Preview_{source.transform.name}");
			previewObject.hideFlags = HideFlags.HideAndDontSave;
			previewObject.layer = LayerMask.NameToLayer("Ignore Raycast");
			previewObject.transform.SetParent(_previewRoot.transform, false);

			Collider previewCollider = CopyCollider(source, previewObject);
			if (previewCollider == null)
			{
				if (Application.isPlaying)
				{
					Destroy(previewObject);
				}
				else
				{
					DestroyImmediate(previewObject);
				}

				return null;
			}

			previewCollider.isTrigger = true;
			previewCollider.enabled = true;

			return new PreviewColliderEntry
			{
				gameObject = previewObject,
				collider = previewCollider,
				sourceName = source.transform.name,
				ownerLinkName = ownerTransform.name,
				localPosition = ownerTransform.InverseTransformPoint(source.transform.position),
				localRotation = Quaternion.Inverse(ownerTransform.rotation) * source.transform.rotation,
				lossyScale = source.transform.lossyScale
			};
		}

		private static Collider CopyCollider(Collider source, GameObject destination)
		{
			if (source is BoxCollider sourceBox)
			{
				BoxCollider box = destination.AddComponent<BoxCollider>();
				box.center = sourceBox.center;
				box.size = sourceBox.size;
				return box;
			}

			if (source is SphereCollider sourceSphere)
			{
				SphereCollider sphere = destination.AddComponent<SphereCollider>();
				sphere.center = sourceSphere.center;
				sphere.radius = sourceSphere.radius;
				return sphere;
			}

			if (source is CapsuleCollider sourceCapsule)
			{
				CapsuleCollider capsule = destination.AddComponent<CapsuleCollider>();
				capsule.center = sourceCapsule.center;
				capsule.radius = sourceCapsule.radius;
				capsule.height = sourceCapsule.height;
				capsule.direction = sourceCapsule.direction;
				return capsule;
			}

			if (source is MeshCollider sourceMesh && sourceMesh.sharedMesh != null)
			{
				MeshCollider mesh = destination.AddComponent<MeshCollider>();
				mesh.sharedMesh = sourceMesh.sharedMesh;
				mesh.convex = true;
				return mesh;
			}

			return null;
		}

		private void ParkPreviewColliders()
		{
			for (int i = 0; i < _previewColliders.Count; i++)
			{
				PreviewColliderEntry entry = _previewColliders[i];
				if (entry?.gameObject == null)
				{
					continue;
				}

				entry.gameObject.transform.SetPositionAndRotation(new Vector3(10000f + i * 2f, 10000f, 10000f), Quaternion.identity);
			}
		}

		private static bool TryGetLinkWorldPose(string ownerLinkName, Pose[] linkWorldPoses, out Pose pose)
		{
			pose = default;
			if (string.IsNullOrEmpty(ownerLinkName) || linkWorldPoses == null || linkWorldPoses.Length < 7)
			{
				return false;
			}

			int linkIndex = ExtractLinkIndex(ownerLinkName);
			if (linkIndex < 0 || linkIndex >= linkWorldPoses.Length)
			{
				return false;
			}

			pose = linkWorldPoses[linkIndex];
			return true;
		}

		private static int ExtractLinkIndex(string linkName)
		{
			if (string.IsNullOrEmpty(linkName))
			{
				return -1;
			}

			if (linkName.StartsWith("Link_", System.StringComparison.OrdinalIgnoreCase))
			{
				string suffix = linkName.Substring(5);
				if (int.TryParse(suffix, out int value))
				{
					return Mathf.Clamp(value, 0, 6);
				}
			}

			return -1;
		}
	}
}
