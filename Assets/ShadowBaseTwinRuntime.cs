using System.Collections.Generic;
using UnityEngine;

namespace RobotSimulation
{
	internal enum ShadowBaseTwinCommandType
	{
		Velocity,
		TargetPoint,
		TargetYaw
	}

	internal struct ShadowBaseTwinDelayedCommand
	{
		public ShadowBaseTwinCommandType type;
		public float executeRealtime;
		public float linearVelocity;
		public float angularVelocity;
		public Vector3 vectorValue;
		public float floatValue;

		public static ShadowBaseTwinDelayedCommand CreateVelocity(float executeRealtime, float linearVelocity, float angularVelocity, Vector3 trackingPoint)
		{
			return new ShadowBaseTwinDelayedCommand
			{
				type = ShadowBaseTwinCommandType.Velocity,
				executeRealtime = executeRealtime,
				linearVelocity = linearVelocity,
				angularVelocity = angularVelocity,
				vectorValue = trackingPoint
			};
		}

		public static ShadowBaseTwinDelayedCommand CreateTargetPoint(float executeRealtime, Vector3 point)
		{
			return new ShadowBaseTwinDelayedCommand
			{
				type = ShadowBaseTwinCommandType.TargetPoint,
				executeRealtime = executeRealtime,
				vectorValue = point
			};
		}

		public static ShadowBaseTwinDelayedCommand CreateTargetYaw(float executeRealtime, float yawDeg)
		{
			return new ShadowBaseTwinDelayedCommand
			{
				type = ShadowBaseTwinCommandType.TargetYaw,
				executeRealtime = executeRealtime,
				floatValue = yawDeg
			};
		}

		public void Apply(DiffDriveTwinController controller)
		{
			if (controller == null)
			{
				return;
			}

			switch (type)
			{
				case ShadowBaseTwinCommandType.Velocity:
					controller.SetVelocityCommand(linearVelocity, angularVelocity, vectorValue);
					break;

				case ShadowBaseTwinCommandType.TargetPoint:
					controller.SetTargetPointGoal(vectorValue);
					break;

				case ShadowBaseTwinCommandType.TargetYaw:
					controller.SetTargetYawGoal(floatValue);
					break;
			}
		}
	}

	internal sealed class ShadowBaseTwinRuntime
	{
		private readonly PlannerPhysicsQueries _physicsQueries = new PlannerPhysicsQueries();
		private readonly Queue<ShadowBaseTwinDelayedCommand> _liveCommandQueue = new Queue<ShadowBaseTwinDelayedCommand>();
		private readonly List<Collider> _liveRobotColliders = new List<Collider>();
		private readonly List<Collider> _shadowBaseColliders = new List<Collider>();
		private readonly List<Collider> _activeObstacles = new List<Collider>();

		private RobotSimulationManager _manager;
		private DiffDriveTwinController _liveController;
		private DiffDriveTwinController _shadowController;
		private Transform _liveSourceRoot;
		private GameObject _shadowRootObject;
		private float _leadSeconds = 0.35f;
		private float _activeBaseRadius = 0.35f;
		private bool _sessionActive;
		private string _sessionLabel = string.Empty;
		private string _faultMessage = string.Empty;

		public DiffDriveTwinController ShadowController => _shadowController;
		public Transform ShadowRoot => _shadowRootObject != null ? _shadowRootObject.transform : null;
		public bool IsReady => _shadowController != null && _shadowController.rb != null && _shadowRootObject != null;
		public bool IsSessionActive => _sessionActive;
		public bool HasPendingLiveCommands => _liveCommandQueue.Count > 0;
		public bool HasFault => !string.IsNullOrEmpty(_faultMessage);
		public string FaultMessage => _faultMessage;

		public bool Configure(RobotSimulationManager manager, DiffDriveTwinController liveController, float leadSeconds)
		{
			_manager = manager;
			_liveController = liveController;
			_leadSeconds = Mathf.Max(0.05f, leadSeconds);

			if (_liveController == null || _liveController.rb == null)
			{
				DisposeShadowTwin();
				return false;
			}

			if (!IsReady)
			{
				if (!RebuildShadowTwin())
				{
					return false;
				}
			}

			SyncShadowControllerParametersFromLive();
			SyncShadowPhysicsFromLive();
			return IsReady;
		}

		public void Tick()
		{
			if (!IsReady)
			{
				return;
			}

			float nowRealtime = Time.realtimeSinceStartup;
			while (_liveCommandQueue.Count > 0)
			{
				ShadowBaseTwinDelayedCommand nextCommand = _liveCommandQueue.Peek();
				if (nextCommand.executeRealtime > nowRealtime + 1e-4f)
				{
					break;
				}

				_liveCommandQueue.Dequeue();
				nextCommand.Apply(_liveController);
			}

			if (_sessionActive && !HasFault)
			{
				if (TryDetectShadowBaseCollision(out string collisionMessage))
				{
					FailSession(collisionMessage);
				}
			}
		}

		public bool BeginPlannerSession(string label, IReadOnlyList<Collider> obstacles, float baseRadius, out string error)
		{
			error = string.Empty;
			if (!PrepareSession(label, obstacles, baseRadius))
			{
				error = string.IsNullOrEmpty(_faultMessage)
					? "Shadow base twin runtime is unavailable."
					: _faultMessage;
				return false;
			}

			return true;
		}

		public bool StartManualPointLeadRun(Vector3 point, Transform armIgnoreRoot, out string error)
		{
			error = string.Empty;
			if (!PrepareSession(
				"ManualBasePoint",
				CollectManualObstacleSnapshot(armIgnoreRoot),
				_physicsQueries.EstimateBaseRadius(_liveController)))
			{
				error = string.IsNullOrEmpty(_faultMessage)
					? "Shadow base twin runtime is unavailable."
					: _faultMessage;
				return false;
			}

			point.y = _liveController.rb.position.y;
			_shadowController.SetTargetPointGoal(point);
			EnqueueLiveReplica(ShadowBaseTwinDelayedCommand.CreateTargetPoint(Time.realtimeSinceStartup + _leadSeconds, point));
			return true;
		}

		public bool StartManualYawLeadRun(float yawDeg, Transform armIgnoreRoot, out string error)
		{
			error = string.Empty;
			if (!PrepareSession(
				"ManualBaseYaw",
				CollectManualObstacleSnapshot(armIgnoreRoot),
				_physicsQueries.EstimateBaseRadius(_liveController)))
			{
				error = string.IsNullOrEmpty(_faultMessage)
					? "Shadow base twin runtime is unavailable."
					: _faultMessage;
				return false;
			}

			_shadowController.SetTargetYawGoal(yawDeg);
			EnqueueLiveReplica(ShadowBaseTwinDelayedCommand.CreateTargetYaw(Time.realtimeSinceStartup + _leadSeconds, yawDeg));
			return true;
		}

		public bool QueueLiveReplicaVelocity(float linearVelocity, float angularVelocity, Vector3 trackingPoint, out string error)
		{
			error = string.Empty;
			if (!CanQueueReplica(out error))
			{
				return false;
			}

			EnqueueLiveReplica(ShadowBaseTwinDelayedCommand.CreateVelocity(
				Time.realtimeSinceStartup + _leadSeconds,
				linearVelocity,
				angularVelocity,
				trackingPoint));
			return true;
		}

		public bool QueueLiveReplicaTargetPoint(Vector3 point, out string error)
		{
			error = string.Empty;
			if (!CanQueueReplica(out error))
			{
				return false;
			}

			point.y = _liveController != null && _liveController.rb != null ? _liveController.rb.position.y : point.y;
			EnqueueLiveReplica(ShadowBaseTwinDelayedCommand.CreateTargetPoint(Time.realtimeSinceStartup + _leadSeconds, point));
			return true;
		}

		public bool QueueLiveReplicaTargetYaw(float yawDeg, out string error)
		{
			error = string.Empty;
			if (!CanQueueReplica(out error))
			{
				return false;
			}

			EnqueueLiveReplica(ShadowBaseTwinDelayedCommand.CreateTargetYaw(Time.realtimeSinceStartup + _leadSeconds, yawDeg));
			return true;
		}

		public void CompleteSession(bool resetShadowToLivePose)
		{
			_sessionActive = false;
			_sessionLabel = string.Empty;
			_faultMessage = string.Empty;
			_liveCommandQueue.Clear();
			if (resetShadowToLivePose)
			{
				ResetShadowToLivePose();
			}

			ClearObstacleSnapshot();
		}

		public void StopAndReset(bool resetShadowToLivePose)
		{
			StopAllMotion();
			CompleteSession(resetShadowToLivePose);
		}

		public void FailSession(string reason)
		{
			if (string.IsNullOrWhiteSpace(reason))
			{
				reason = "Shadow base twin execution failed.";
			}

			if (HasFault)
			{
				return;
			}

			_faultMessage = reason;
			Debug.LogError($"[ShadowBaseTwinRuntime] {reason}");
			StopAllMotion();
		}

		public bool TryGetShadowPose(out Vector3 worldPosition, out Quaternion worldRotation)
		{
			if (!IsReady)
			{
				worldPosition = default;
				worldRotation = Quaternion.identity;
				return false;
			}

			worldPosition = _shadowController.rb.position;
			worldRotation = _shadowController.rb.rotation;
			return true;
		}

		private bool PrepareSession(string label, IReadOnlyList<Collider> obstacles, float baseRadius)
		{
			_faultMessage = string.Empty;
			_sessionLabel = label ?? string.Empty;

			if (!Configure(_manager, _liveController, _leadSeconds))
			{
				_faultMessage = "Shadow base twin runtime could not be configured.";
				return false;
			}

			SyncShadowControllerParametersFromLive();
			SyncShadowPhysicsFromLive();
			StopAllMotion();
			ResetShadowToLivePose();
			ClearObstacleSnapshot();
			_activeBaseRadius = Mathf.Max(0.1f, baseRadius);
			AppendObstacleSnapshot(obstacles);
			_liveCommandQueue.Clear();
			_sessionActive = true;
			return true;
		}

		private bool CanQueueReplica(out string error)
		{
			error = string.Empty;
			if (!IsReady)
			{
				error = "Shadow base twin runtime is unavailable.";
				return false;
			}

			if (HasFault)
			{
				error = _faultMessage;
				return false;
			}

			return true;
		}

		private void EnqueueLiveReplica(ShadowBaseTwinDelayedCommand command)
		{
			_liveCommandQueue.Enqueue(command);
		}

		private void StopAllMotion()
		{
			_liveCommandQueue.Clear();
			HardStopController(_shadowController);
			HardStopController(_liveController);
		}

		private static void HardStopController(DiffDriveTwinController controller)
		{
			if (controller == null)
			{
				return;
			}

			controller.hasTargetPoint = false;
			if (controller.rb != null)
			{
				controller.targetPointWorld = controller.rb.position;
			}

			controller.mode = DiffDriveTwinController.ControlMode.TargetPoint;
			controller.HardStopAtGoal();
		}

		private void ClearObstacleSnapshot()
		{
			_activeObstacles.Clear();
		}

		private void AppendObstacleSnapshot(IReadOnlyList<Collider> obstacles)
		{
			if (obstacles == null)
			{
				return;
			}

			Transform liveRoot = _liveSourceRoot;
			Transform shadowRoot = ShadowRoot;
			for (int i = 0; i < obstacles.Count; i++)
			{
				Collider obstacle = obstacles[i];
				if (obstacle == null
					|| !obstacle.enabled
					|| !obstacle.gameObject.activeInHierarchy
					|| obstacle.isTrigger)
				{
					continue;
				}

				if ((liveRoot != null && obstacle.transform.IsChildOf(liveRoot))
					|| (shadowRoot != null && obstacle.transform.IsChildOf(shadowRoot)))
				{
					continue;
				}

				if (!_activeObstacles.Contains(obstacle))
				{
					_activeObstacles.Add(obstacle);
				}
			}
		}

		private List<Collider> CollectManualObstacleSnapshot(Transform armIgnoreRoot)
		{
			Transform liveRoot = _liveController != null && _liveController.rb != null
				? _liveController.rb.transform.root
				: null;
			Transform shadowRoot = ShadowRoot;
			return _physicsQueries.CollectObstacleColliders(liveRoot, armIgnoreRoot, shadowRoot);
		}

		private bool TryDetectShadowBaseCollision(out string collisionMessage)
		{
			collisionMessage = string.Empty;
			if (!IsReady || _shadowBaseColliders.Count == 0 || _activeObstacles.Count == 0)
			{
				return false;
			}

			Vector3 shadowPosition = _shadowController.rb.position;
			if (_physicsQueries.IsBasePoseCollisionFree(shadowPosition, _activeBaseRadius, _activeObstacles, out _) && !HasColliderPenetration(out collisionMessage))
			{
				return false;
			}

			if (!string.IsNullOrEmpty(collisionMessage))
			{
				return true;
			}

			return HasColliderPenetration(out collisionMessage);
		}

		private bool HasColliderPenetration(out string collisionMessage)
		{
			collisionMessage = string.Empty;
			for (int shadowIndex = 0; shadowIndex < _shadowBaseColliders.Count; shadowIndex++)
			{
				Collider shadowCollider = _shadowBaseColliders[shadowIndex];
				if (!IsUsableShadowCollider(shadowCollider))
				{
					continue;
				}

				for (int obstacleIndex = 0; obstacleIndex < _activeObstacles.Count; obstacleIndex++)
				{
					Collider obstacle = _activeObstacles[obstacleIndex];
					if (obstacle == null || !obstacle.enabled || !obstacle.gameObject.activeInHierarchy)
					{
						continue;
					}

					if (!shadowCollider.bounds.Intersects(obstacle.bounds))
					{
						continue;
					}

					if (!Physics.ComputePenetration(
						shadowCollider, shadowCollider.transform.position, shadowCollider.transform.rotation,
						obstacle, obstacle.transform.position, obstacle.transform.rotation,
						out Vector3 _, out float distance))
					{
						continue;
					}

					if (distance <= 1e-4f)
					{
						continue;
					}

					Vector3 pose = _shadowController != null && _shadowController.rb != null ? _shadowController.rb.position : Vector3.zero;
					collisionMessage = $"Shadow base '{_sessionLabel}' collided with obstacle '{obstacle.name}' at ({pose.x:F3}, {pose.y:F3}, {pose.z:F3}).";
					return true;
				}
			}

			return false;
		}

		private static bool IsUsableShadowCollider(Collider collider)
		{
			return collider != null
				&& collider.enabled
				&& collider.gameObject.activeInHierarchy;
		}

		private bool RebuildShadowTwin()
		{
			DisposeShadowTwin();
			if (_liveController == null || _liveController.rb == null)
			{
				return false;
			}

			_liveSourceRoot = ResolveExecutionSourceRoot(_liveController);
			if (_liveSourceRoot == null)
			{
				return false;
			}

			_shadowRootObject = Object.Instantiate(_liveSourceRoot.gameObject);
			if (_shadowRootObject == null)
			{
				return false;
			}

			_shadowRootObject.name = $"Preview_ShadowBaseTwin_{_liveSourceRoot.name}";
			_shadowRootObject.hideFlags = HideFlags.DontSave;
			if (_manager != null)
			{
				_shadowRootObject.transform.SetParent(_manager.transform, true);
			}

			ApplyHideFlagsRecursively(_shadowRootObject.transform);
			_shadowController = _shadowRootObject.GetComponentInChildren<DiffDriveTwinController>(true);
			if (_shadowController == null)
			{
				DisposeShadowTwin();
				return false;
			}

			DisableArmSubtreeIfPresent();
			DisableNonControllerBehaviours();
			DisableAllRenderers();
			if (_shadowController.rb == null)
			{
				_shadowController.rb = _shadowController.GetComponent<Rigidbody>();
				if (_shadowController.rb == null)
				{
					_shadowController.rb = _shadowController.GetComponentInChildren<Rigidbody>(true);
				}
			}

			if (_shadowController.rb == null)
			{
				DisposeShadowTwin();
				return false;
			}

			_shadowController.enableMouseClickToSetTargetPoint = false;
			_shadowController.enableKeyToApplyTargetYaw = false;
			_shadowController.forceUseGravityOnStart = _liveController.forceUseGravityOnStart;

			_liveRobotColliders.Clear();
			_shadowBaseColliders.Clear();
			Collider[] liveColliders = _liveSourceRoot.GetComponentsInChildren<Collider>(true);
			for (int i = 0; i < liveColliders.Length; i++)
			{
				Collider liveCollider = liveColliders[i];
				if (liveCollider == null)
				{
					continue;
				}

				_liveRobotColliders.Add(liveCollider);
			}

			Collider[] colliders = _shadowRootObject.GetComponentsInChildren<Collider>(true);
			for (int i = 0; i < colliders.Length; i++)
			{
				Collider collider = colliders[i];
				if (collider == null)
				{
					continue;
				}

				_shadowBaseColliders.Add(collider);
			}

			IgnoreLiveShadowCollisions();
			SyncShadowPhysicsFromLive();
			ResetShadowToLivePose();
			return IsReady;
		}

		private void SyncShadowControllerParametersFromLive()
		{
			if (_liveController == null || _shadowController == null)
			{
				return;
			}

			_shadowController.autoDetectGeometryOnStart = false;
			_shadowController.lateralAxis = _liveController.lateralAxis;
			_shadowController.wheelRollAxis = _liveController.wheelRollAxis;
			_shadowController.animateWheelRoll = _liveController.animateWheelRoll;
			_shadowController.vMax = _liveController.vMax;
			_shadowController.aMax = _liveController.aMax;
			_shadowController.wMax = _liveController.wMax;
			_shadowController.alphaMax = _liveController.alphaMax;
			_shadowController.kDist = _liveController.kDist;
			_shadowController.kYaw = _liveController.kYaw;
			_shadowController.rotateInPlaceAngleDeg = _liveController.rotateInPlaceAngleDeg;
			_shadowController.posTolerance = _liveController.posTolerance;
			_shadowController.arrivalSnapDistance = _liveController.arrivalSnapDistance;
			_shadowController.brakingDistancePadding = _liveController.brakingDistancePadding;
			_shadowController.yawRateStopTolerance = _liveController.yawRateStopTolerance;
			_shadowController.yawToleranceDeg = _liveController.yawToleranceDeg;
			_shadowController.headingOffsetDeg = _liveController.headingOffsetDeg;
			_shadowController.goalReleaseDistance = _liveController.goalReleaseDistance;
			_shadowController.alignYawAtGoal = _liveController.alignYawAtGoal;
			_shadowController.rotateAtGoal = _liveController.rotateAtGoal;
			_shadowController.preserveArrivalHeading = _liveController.preserveArrivalHeading;
			_shadowController.snapPositionToGoalOnArrival = _liveController.snapPositionToGoalOnArrival;
			_shadowController.rotateExitAngleDeg = _liveController.rotateExitAngleDeg;
			_shadowController.cruiseHeadingFloor = _liveController.cruiseHeadingFloor;
			_shadowController.brakeHeadingFloor = _liveController.brakeHeadingFloor;
			_shadowController.brakeMinSpeed = _liveController.brakeMinSpeed;
			_shadowController.finalYawGain = _liveController.finalYawGain;
			_shadowController.finalYawMinRate = _liveController.finalYawMinRate;
			_shadowController.finalYawAccelerationScale = _liveController.finalYawAccelerationScale;
			_shadowController.snapYawToTargetWhenAligned = _liveController.snapYawToTargetWhenAligned;
			_shadowController.forceUseGravityOnStart = _liveController.forceUseGravityOnStart;
			_shadowController.DetectGeometry();
		}

		private void SyncShadowPhysicsFromLive()
		{
			if (_liveController == null
				|| _liveController.rb == null
				|| _shadowController == null
				|| _shadowController.rb == null)
			{
				return;
			}

			Rigidbody liveRb = _liveController.rb;
			Rigidbody shadowRb = _shadowController.rb;
			shadowRb.mass = liveRb.mass;
			shadowRb.drag = liveRb.drag;
			shadowRb.angularDrag = liveRb.angularDrag;
			shadowRb.useGravity = liveRb.useGravity;
			shadowRb.isKinematic = liveRb.isKinematic;
			shadowRb.interpolation = liveRb.interpolation;
			shadowRb.collisionDetectionMode = liveRb.collisionDetectionMode;
			shadowRb.constraints = liveRb.constraints;
			shadowRb.centerOfMass = liveRb.centerOfMass;
			shadowRb.inertiaTensor = liveRb.inertiaTensor;
			shadowRb.inertiaTensorRotation = liveRb.inertiaTensorRotation;
		}

		private void IgnoreLiveShadowCollisions()
		{
			if (_liveRobotColliders.Count == 0 || _shadowBaseColliders.Count == 0)
			{
				return;
			}

			for (int liveIndex = 0; liveIndex < _liveRobotColliders.Count; liveIndex++)
			{
				Collider liveCollider = _liveRobotColliders[liveIndex];
				if (liveCollider == null)
				{
					continue;
				}

				for (int shadowIndex = 0; shadowIndex < _shadowBaseColliders.Count; shadowIndex++)
				{
					Collider shadowCollider = _shadowBaseColliders[shadowIndex];
					if (shadowCollider == null || shadowCollider == liveCollider)
					{
						continue;
					}

					Physics.IgnoreCollision(liveCollider, shadowCollider, true);
				}
			}
		}

		private void ResetShadowToLivePose()
		{
			if (!IsReady || _liveSourceRoot == null || _liveController == null || _liveController.rb == null)
			{
				return;
			}

			_shadowRootObject.transform.position = _liveSourceRoot.position;
			_shadowRootObject.transform.rotation = _liveSourceRoot.rotation;
			_shadowRootObject.transform.localScale = _liveSourceRoot.lossyScale;
			_shadowController.rb.position = _liveController.rb.position;
			_shadowController.rb.rotation = _liveController.rb.rotation;
			_shadowController.rb.velocity = Vector3.zero;
			_shadowController.rb.angularVelocity = Vector3.zero;
			_shadowController.targetPointWorld = _shadowController.rb.position;
			_shadowController.mode = DiffDriveTwinController.ControlMode.TargetPoint;
			_shadowController.HardStopAtGoal();
		}

		private void DisableArmSubtreeIfPresent()
		{
			if (_manager == null || _liveSourceRoot == null || _shadowRootObject == null)
			{
				return;
			}

			Transform liveArmRoot = _manager.GetArmIgnoreRootTransform();
			if (liveArmRoot == null || liveArmRoot == _liveSourceRoot || !liveArmRoot.IsChildOf(_liveSourceRoot))
			{
				return;
			}

			string relativePath = GetRelativePath(_liveSourceRoot, liveArmRoot);
			if (string.IsNullOrEmpty(relativePath))
			{
				return;
			}

			Transform clonedArmRoot = _shadowRootObject.transform.Find(relativePath);
			if (clonedArmRoot != null)
			{
				clonedArmRoot.gameObject.SetActive(false);
			}
		}

		private void DisableNonControllerBehaviours()
		{
			if (_shadowRootObject == null)
			{
				return;
			}

			MonoBehaviour[] behaviours = _shadowRootObject.GetComponentsInChildren<MonoBehaviour>(true);
			for (int i = 0; i < behaviours.Length; i++)
			{
				MonoBehaviour behaviour = behaviours[i];
				if (behaviour == null || behaviour == _shadowController)
				{
					continue;
				}

				behaviour.enabled = false;
			}
		}

		private void DisableAllRenderers()
		{
			if (_shadowRootObject == null)
			{
				return;
			}

			Renderer[] renderers = _shadowRootObject.GetComponentsInChildren<Renderer>(true);
			for (int i = 0; i < renderers.Length; i++)
			{
				if (renderers[i] != null)
				{
					renderers[i].enabled = false;
				}
			}
		}

		private static void ApplyHideFlagsRecursively(Transform root)
		{
			if (root == null)
			{
				return;
			}

			root.gameObject.hideFlags = HideFlags.DontSave;
			for (int childIndex = 0; childIndex < root.childCount; childIndex++)
			{
				ApplyHideFlagsRecursively(root.GetChild(childIndex));
			}
		}

		private static Transform ResolveExecutionSourceRoot(DiffDriveTwinController controller)
		{
			if (controller == null)
			{
				return null;
			}

			Transform controllerTransform = controller.transform;
			Transform rigidbodyTransform = controller.rb != null ? controller.rb.transform : null;
			if (controllerTransform == null)
			{
				return rigidbodyTransform;
			}

			if (rigidbodyTransform == null)
			{
				return controllerTransform;
			}

			return FindLowestCommonAncestor(controllerTransform, rigidbodyTransform) ?? rigidbodyTransform;
		}

		private static Transform FindLowestCommonAncestor(Transform a, Transform b)
		{
			if (a == null)
			{
				return b;
			}

			if (b == null)
			{
				return a;
			}

			HashSet<Transform> ancestors = new HashSet<Transform>();
			Transform current = a;
			while (current != null)
			{
				ancestors.Add(current);
				current = current.parent;
			}

			current = b;
			while (current != null)
			{
				if (ancestors.Contains(current))
				{
					return current;
				}

				current = current.parent;
			}

			return null;
		}

		private static string GetRelativePath(Transform root, Transform target)
		{
			if (root == null || target == null || target == root)
			{
				return string.Empty;
			}

			List<string> segments = new List<string>();
			Transform current = target;
			while (current != null && current != root)
			{
				segments.Add(current.name);
				current = current.parent;
			}

			if (current != root)
			{
				return string.Empty;
			}

			segments.Reverse();
			return string.Join("/", segments);
		}

		private void DisposeShadowTwin()
		{
			_liveRobotColliders.Clear();
			_shadowBaseColliders.Clear();
			if (_shadowRootObject != null)
			{
				Object.Destroy(_shadowRootObject);
			}

			_shadowRootObject = null;
			_shadowController = null;
			_liveSourceRoot = null;
			_liveCommandQueue.Clear();
			ClearObstacleSnapshot();
			_sessionActive = false;
			_faultMessage = string.Empty;
		}
	}
}
