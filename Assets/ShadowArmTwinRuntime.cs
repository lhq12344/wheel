using System.Collections.Generic;
using UnityEngine;

namespace RobotSimulation
{
	internal struct ShadowArmTwinDelayedCommand
	{
		public long sequenceId;
		public float executeRealtime;
		public float[] targetAnglesDeg;

		public static ShadowArmTwinDelayedCommand Create(long sequenceId, float executeRealtime, float[] targetAnglesDeg)
		{
			return new ShadowArmTwinDelayedCommand
			{
				sequenceId = sequenceId,
				executeRealtime = executeRealtime,
				targetAnglesDeg = targetAnglesDeg != null ? (float[])targetAnglesDeg.Clone() : null
			};
		}

		public void Apply(Arm6DOFFKController controller)
		{
			if (controller == null || targetAnglesDeg == null || targetAnglesDeg.Length < 6)
			{
				return;
			}

			controller.ApplyAllJointTargetsRaw(targetAnglesDeg);
		}
	}

	internal sealed class ShadowArmTwinRuntime
	{
		private const string DirectCommandSessionLabel = "DirectArmCommand";

		private readonly PlannerPhysicsQueries _physicsQueries = new PlannerPhysicsQueries();
		private readonly Queue<ShadowArmTwinDelayedCommand> _liveCommandQueue = new Queue<ShadowArmTwinDelayedCommand>();
		private readonly List<Collider> _activeObstacles = new List<Collider>();
		private readonly List<Collider> _liveRobotColliders = new List<Collider>();
		private readonly List<Collider> _shadowArmColliders = new List<Collider>();
		private readonly List<ArticulationBody> _shadowJoints = new List<ArticulationBody>();
		private readonly List<OneJointTrapezoidController> _shadowJointControllers = new List<OneJointTrapezoidController>();

		private RobotSimulationManager _manager;
		private Arm6DOFFKController _liveController;
		private ArticulationArmBindToCar _liveBinder;
		private Transform _liveArmRoot;
		private Transform _liveCarMount;
		private Transform _configuredLiveArmRoot;
		private Transform _configuredLiveCarMount;
		private GameObject _shadowRootObject;
		private ArticulationBody _shadowRootBody;
		private float _leadSeconds = 0.35f;
		private bool _sessionActive;
		private string _sessionLabel = string.Empty;
		private string _faultMessage = string.Empty;

		public Transform ShadowRoot => _shadowRootObject != null ? _shadowRootObject.transform : null;
		public bool IsReady => _shadowRootBody != null && _liveController != null && _liveBinder != null && _liveCarMount != null;
		public bool IsSessionActive => _sessionActive;
		public bool HasPendingLiveCommands => _liveCommandQueue.Count > 0;
		public bool HasFault => !string.IsNullOrEmpty(_faultMessage);
		public string FaultMessage => _faultMessage;
		public bool ShouldDriveVisibleShadow => IsReady && (_sessionActive || HasPendingLiveCommands || HasFault);

		public bool Configure(
			RobotSimulationManager manager,
			ArticulationArmBindToCar liveBinder,
			Arm6DOFFKController liveController,
			float leadSeconds)
		{
			_manager = manager;
			_liveBinder = liveBinder;
			_liveController = liveController;
			_leadSeconds = Mathf.Max(0.05f, leadSeconds);
			_liveArmRoot = liveBinder != null && liveBinder.armRoot != null ? liveBinder.armRoot.transform : null;
			_liveCarMount = liveBinder != null ? liveBinder.carMount : null;

			if (_liveArmRoot == null || _liveCarMount == null || _liveController == null)
			{
				DisposeShadowTwin();
				return false;
			}

			bool sourceChanged = _configuredLiveArmRoot != _liveArmRoot || _configuredLiveCarMount != _liveCarMount;
			if (!IsReady || sourceChanged || ShadowRoot == null || ShadowRoot.name != $"Preview_ShadowArmTwin_{_liveArmRoot.name}")
			{
				if (!RebuildShadowTwin())
				{
					return false;
				}
			}

			SyncShadowConfigurationFromLive();
			SyncMountToLive();
			if (!_sessionActive && !HasFault && _shadowRootObject != null && _shadowRootObject.activeSelf)
			{
				_shadowRootObject.SetActive(false);
			}

			return IsReady;
		}

		public void Tick()
		{
			if (!IsReady)
			{
				return;
			}

			SyncShadowConfigurationFromLive();
			SyncMountToLive();

			float nowRealtime = Time.realtimeSinceStartup;
			while (_liveCommandQueue.Count > 0)
			{
				ShadowArmTwinDelayedCommand nextCommand = _liveCommandQueue.Peek();
				if (nextCommand.executeRealtime > nowRealtime + 1e-4f)
				{
					break;
				}

				_liveCommandQueue.Dequeue();
				nextCommand.Apply(_liveController);
			}

			if (_sessionActive && !HasFault && TryDetectShadowArmCollision(out string collisionMessage))
			{
				FailSession(collisionMessage);
			}
		}

		public bool BeginPlannerSession(string label, IReadOnlyList<Collider> obstacles, out string error)
		{
			return PrepareSession(label, obstacles, out error);
		}

		public bool BeginManualSession(string label, out string error)
		{
			return PrepareSession(label, CollectManualObstacleSnapshot(), out error);
		}

		public bool EnsureDirectCommandSession(out string error)
		{
			error = string.Empty;
			if (_sessionActive && !HasFault && _sessionLabel == DirectCommandSessionLabel)
			{
				return true;
			}

			return PrepareSession(DirectCommandSessionLabel, CollectManualObstacleSnapshot(), out error);
		}

		public bool DispatchTarget(float[] targetAnglesDeg, out string error)
		{
			return DispatchTarget(targetAnglesDeg, 0L, out error);
		}

		public bool DispatchTarget(float[] targetAnglesDeg, long sequenceId, out string error)
		{
			error = string.Empty;
			if (!CanDispatch(out error))
			{
				return false;
			}

			float[] safeAngles = CloneAndClampAngles(targetAnglesDeg);
			if (safeAngles == null)
			{
				error = "Shadow arm twin target angles are invalid.";
				return false;
			}

			ApplyShadowTargetsRaw(safeAngles);
			_liveCommandQueue.Enqueue(ShadowArmTwinDelayedCommand.Create(sequenceId, Time.realtimeSinceStartup + _leadSeconds, safeAngles));
			return true;
		}

		public bool AreShadowJointAnglesNear(float[] targetAnglesDeg, float toleranceDeg)
		{
			if (!IsReady || targetAnglesDeg == null || targetAnglesDeg.Length < 6 || _shadowJoints.Count < 6)
			{
				return false;
			}

			for (int i = 0; i < 6; i++)
			{
				ArticulationBody joint = _shadowJoints[i];
				if (joint == null || joint.jointPosition.dofCount <= 0)
				{
					return false;
				}

				float measuredDeg = joint.jointPosition[0] * Mathf.Rad2Deg;
				if (Mathf.Abs(Mathf.DeltaAngle(measuredDeg, targetAnglesDeg[i])) > toleranceDeg)
				{
					return false;
				}
			}

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

			_activeObstacles.Clear();
			if (_shadowRootObject != null && !HasFault)
			{
				_shadowRootObject.SetActive(false);
			}
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
				reason = "Shadow arm twin execution failed.";
			}

			if (HasFault)
			{
				return;
			}

			_faultMessage = reason;
			Debug.LogError($"[ShadowArmTwinRuntime] {reason}");
			StopAllMotion();
		}

		private bool PrepareSession(string label, IReadOnlyList<Collider> obstacles, out string error)
		{
			error = string.Empty;
			_faultMessage = string.Empty;
			_sessionLabel = label ?? string.Empty;

			if (!Configure(_manager, _liveBinder, _liveController, _leadSeconds))
			{
				error = "Shadow arm twin runtime could not be configured.";
				_faultMessage = error;
				return false;
			}

			if (_shadowRootObject != null && !_shadowRootObject.activeSelf)
			{
				_shadowRootObject.SetActive(true);
			}

			SyncShadowConfigurationFromLive();
			StopAllMotion();
			ResetShadowToLivePose();
			AppendObstacleSnapshot(obstacles);
			_liveCommandQueue.Clear();
			_sessionActive = true;
			return true;
		}

		private bool CanDispatch(out string error)
		{
			error = string.Empty;
			if (!IsReady)
			{
				error = "Shadow arm twin runtime is unavailable.";
				return false;
			}

			if (!_sessionActive)
			{
				error = "Shadow arm twin runtime has no active session.";
				return false;
			}

			if (HasFault)
			{
				error = _faultMessage;
				return false;
			}

			return true;
		}

		private void StopAllMotion()
		{
			_liveCommandQueue.Clear();
			HoldControllerCurrentPose(_liveController);
			HoldShadowCurrentPose();
		}

		private void HoldShadowCurrentPose()
		{
			if (!IsReady || _shadowJoints.Count < 6)
			{
				return;
			}

			float[] measured = CaptureShadowMeasuredAngles();
			ApplyShadowTargetsRaw(measured);
			for (int i = 0; i < 6; i++)
			{
				ArticulationBody joint = _shadowJoints[i];
				if (joint == null)
				{
					continue;
				}

				ArticulationDrive drive = joint.xDrive;
				drive.target = measured[i];
				drive.targetVelocity = 0f;
				joint.xDrive = drive;
				joint.WakeUp();
			}
		}

		private static void HoldControllerCurrentPose(Arm6DOFFKController controller)
		{
			controller?.HoldCurrentPose();
		}

		private float[] CaptureShadowMeasuredAngles()
		{
			float[] measured = new float[6];
			for (int i = 0; i < Mathf.Min(6, _shadowJoints.Count); i++)
			{
				ArticulationBody joint = _shadowJoints[i];
				if (joint == null || joint.jointPosition.dofCount <= 0)
				{
					continue;
				}

				measured[i] = joint.jointPosition[0] * Mathf.Rad2Deg;
			}

			return measured;
		}

		private void AppendObstacleSnapshot(IReadOnlyList<Collider> obstacles)
		{
			_activeObstacles.Clear();
			if (obstacles == null)
			{
				return;
			}

			Transform liveBaseRoot = ResolveLiveBaseRoot();
			Transform shadowBaseRoot = _manager != null ? _manager.GetShadowBaseTwinRoot() : null;
			Transform shadowArmRoot = ShadowRoot;
			for (int i = 0; i < obstacles.Count; i++)
			{
				Collider obstacle = obstacles[i];
				if (!IsUsableCollider(obstacle))
				{
					continue;
				}

				if ((liveBaseRoot != null && obstacle.transform.IsChildOf(liveBaseRoot))
					|| (_liveArmRoot != null && obstacle.transform.IsChildOf(_liveArmRoot))
					|| (shadowBaseRoot != null && obstacle.transform.IsChildOf(shadowBaseRoot))
					|| (shadowArmRoot != null && obstacle.transform.IsChildOf(shadowArmRoot)))
				{
					continue;
				}

				if (!_activeObstacles.Contains(obstacle))
				{
					_activeObstacles.Add(obstacle);
				}
			}
		}

		private List<Collider> CollectManualObstacleSnapshot()
		{
			return _physicsQueries.CollectObstacleColliders(
				ResolveLiveBaseRoot(),
				_liveArmRoot,
				_manager != null ? _manager.GetShadowBaseTwinRoot() : null,
				ShadowRoot);
		}

		private bool TryDetectShadowArmCollision(out string collisionMessage)
		{
			collisionMessage = string.Empty;
			if (!IsReady || _shadowArmColliders.Count == 0 || _activeObstacles.Count == 0)
			{
				return false;
			}

			for (int shadowIndex = 0; shadowIndex < _shadowArmColliders.Count; shadowIndex++)
			{
				Collider shadowCollider = _shadowArmColliders[shadowIndex];
				if (!IsUsableCollider(shadowCollider))
				{
					continue;
				}

				for (int obstacleIndex = 0; obstacleIndex < _activeObstacles.Count; obstacleIndex++)
				{
					Collider obstacle = _activeObstacles[obstacleIndex];
					if (!IsUsableCollider(obstacle) || !shadowCollider.bounds.Intersects(obstacle.bounds))
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

					if (distance <= GetEffectivePenetrationThreshold(shadowCollider, obstacle))
					{
						continue;
					}

					Vector3 pose = _shadowRootBody != null ? _shadowRootBody.worldCenterOfMass : Vector3.zero;
					collisionMessage =
						$"Shadow arm '{_sessionLabel}' collided with obstacle '{obstacle.name}' via '{shadowCollider.name}' at ({pose.x:F3}, {pose.y:F3}, {pose.z:F3}).";
					return true;
				}
			}

			return false;
		}

		private bool RebuildShadowTwin()
		{
			DisposeShadowTwin();
			if (_liveArmRoot == null)
			{
				return false;
			}

			_shadowRootObject = Object.Instantiate(_liveArmRoot.gameObject);
			if (_shadowRootObject == null)
			{
				return false;
			}

			_shadowRootObject.name = $"Preview_ShadowArmTwin_{_liveArmRoot.name}";
			_shadowRootObject.hideFlags = HideFlags.DontSave;
			if (_manager != null)
			{
				_shadowRootObject.transform.SetParent(_manager.transform, true);
			}

			ApplyHideFlagsRecursively(_shadowRootObject.transform);
			bool wasActive = _shadowRootObject.activeSelf;
			_shadowRootObject.SetActive(false);
			_shadowRootBody = _shadowRootObject.GetComponent<ArticulationBody>();
			if (_shadowRootBody == null)
			{
				DisposeShadowTwin();
				return false;
			}

			CollectJointMappingsAndControllers();
			if (_shadowJoints.Count < 6 || _shadowJointControllers.Count < 6)
			{
				DisposeShadowTwin();
				return false;
			}

			DisableNonExecutionBehaviours();
			DisableAllRenderers();
			CollectColliders();
			_shadowRootObject.SetActive(wasActive);
			_configuredLiveArmRoot = _liveArmRoot;
			_configuredLiveCarMount = _liveCarMount;
			IgnoreRobotCollisions();
			SyncShadowConfigurationFromLive();
			ResetShadowToLivePose();
			return IsReady;
		}

		private void CollectJointMappingsAndControllers()
		{
			_shadowJoints.Clear();
			_shadowJointControllers.Clear();
			if (_liveController == null || _liveController.joints == null || _liveController.coordinatedJointControllers == null)
			{
				return;
			}

			for (int jointIndex = 0; jointIndex < 6; jointIndex++)
			{
				ArticulationBody liveJoint = _liveController.joints[jointIndex];
				if (liveJoint == null)
				{
					continue;
				}

				string relativePath = GetRelativePath(_liveArmRoot, liveJoint.transform);
				Transform shadowJointTransform = string.IsNullOrEmpty(relativePath)
					? _shadowRootObject.transform
					: _shadowRootObject.transform.Find(relativePath);
				ArticulationBody shadowJoint = shadowJointTransform != null
					? shadowJointTransform.GetComponent<ArticulationBody>()
					: null;
				if (shadowJoint == null)
				{
					continue;
				}

				OneJointTrapezoidController liveControllerComponent = _liveController.coordinatedJointControllers[jointIndex];
				OneJointTrapezoidController shadowControllerComponent = shadowJointTransform.GetComponent<OneJointTrapezoidController>();
				if (shadowControllerComponent == null)
				{
					shadowControllerComponent = shadowJointTransform.gameObject.AddComponent<OneJointTrapezoidController>();
				}

				shadowControllerComponent.joint = shadowJoint;
				if (liveControllerComponent != null)
				{
					CopyJointControllerConfiguration(liveControllerComponent, shadowControllerComponent);
				}

				_shadowJoints.Add(shadowJoint);
				_shadowJointControllers.Add(shadowControllerComponent);
			}
		}

		private void CollectColliders()
		{
			_liveRobotColliders.Clear();
			_shadowArmColliders.Clear();

			HashSet<Collider> uniqueLiveColliders = new HashSet<Collider>();
			AppendColliderSet(uniqueLiveColliders, ResolveLiveBaseRoot());
			AppendColliderSet(uniqueLiveColliders, _liveArmRoot);
			foreach (Collider collider in uniqueLiveColliders)
			{
				_liveRobotColliders.Add(collider);
			}

			AppendColliderList(_shadowArmColliders, ShadowRoot);
		}

		private void IgnoreRobotCollisions()
		{
			if (_shadowArmColliders.Count == 0)
			{
				return;
			}

			for (int shadowIndex = 0; shadowIndex < _shadowArmColliders.Count; shadowIndex++)
			{
				Collider shadowCollider = _shadowArmColliders[shadowIndex];
				if (shadowCollider == null)
				{
					continue;
				}

				for (int liveIndex = 0; liveIndex < _liveRobotColliders.Count; liveIndex++)
				{
					Collider liveCollider = _liveRobotColliders[liveIndex];
					if (liveCollider == null || liveCollider == shadowCollider)
					{
						continue;
					}

					Physics.IgnoreCollision(liveCollider, shadowCollider, true);
				}
			}

			Transform shadowBaseRoot = _manager != null ? _manager.GetShadowBaseTwinRoot() : null;
			if (shadowBaseRoot != null)
			{
				List<Collider> shadowBaseColliders = new List<Collider>();
				AppendColliderList(shadowBaseColliders, shadowBaseRoot);
				for (int shadowIndex = 0; shadowIndex < _shadowArmColliders.Count; shadowIndex++)
				{
					Collider shadowArmCollider = _shadowArmColliders[shadowIndex];
					if (shadowArmCollider == null)
					{
						continue;
					}

					for (int baseIndex = 0; baseIndex < shadowBaseColliders.Count; baseIndex++)
					{
						Collider shadowBaseCollider = shadowBaseColliders[baseIndex];
						if (shadowBaseCollider == null || shadowBaseCollider == shadowArmCollider)
						{
							continue;
						}

						Physics.IgnoreCollision(shadowBaseCollider, shadowArmCollider, true);
					}
				}
			}
		}

		private void SyncShadowConfigurationFromLive()
		{
			if (_liveController == null
				|| _liveController.joints == null
				|| _liveController.coordinatedJointControllers == null
				|| _shadowJoints.Count < 6
				|| _shadowJointControllers.Count < 6)
			{
				return;
			}

			for (int i = 0; i < 6; i++)
			{
				ArticulationBody liveJoint = _liveController.joints[i];
				ArticulationBody shadowJoint = _shadowJoints[i];
				if (liveJoint == null || shadowJoint == null)
				{
					continue;
				}

				ArticulationDrive liveDrive = liveJoint.xDrive;
				ArticulationDrive shadowDrive = shadowJoint.xDrive;
				shadowDrive.lowerLimit = liveDrive.lowerLimit;
				shadowDrive.upperLimit = liveDrive.upperLimit;
				shadowDrive.stiffness = liveDrive.stiffness;
				shadowDrive.damping = liveDrive.damping;
				shadowDrive.forceLimit = liveDrive.forceLimit;
				shadowJoint.xDrive = shadowDrive;

				OneJointTrapezoidController liveControllerComponent = _liveController.coordinatedJointControllers[i];
				OneJointTrapezoidController shadowControllerComponent = _shadowJointControllers[i];
				if (liveControllerComponent != null && shadowControllerComponent != null)
				{
					CopyJointControllerConfiguration(liveControllerComponent, shadowControllerComponent);
					shadowControllerComponent.joint = shadowJoint;
				}
			}
		}

		private void SyncMountToLive()
		{
			if (_shadowRootBody == null || _liveCarMount == null)
			{
				return;
			}

			_shadowRootBody.TeleportRoot(_liveCarMount.position, _liveCarMount.rotation);
			Physics.SyncTransforms();
		}

		private void ResetShadowToLivePose()
		{
			if (!IsReady)
			{
				return;
			}

			SyncMountToLive();
			float[] measured = _liveController.CaptureMeasuredJointAngles();
			ApplyShadowTargetsRaw(measured);
			if (_shadowRootBody != null)
			{
				_shadowRootBody.velocity = Vector3.zero;
				_shadowRootBody.angularVelocity = Vector3.zero;
			}
		}

		private void ApplyShadowTargetsRaw(float[] anglesDeg)
		{
			if (anglesDeg == null || anglesDeg.Length < 6 || _shadowJoints.Count < 6)
			{
				return;
			}

			for (int i = 0; i < 6; i++)
			{
				ArticulationBody joint = _shadowJoints[i];
				if (joint == null)
				{
					continue;
				}

				float safeAngle = anglesDeg[i];
				if (_liveController != null && _liveController.jointLimits != null && _liveController.jointLimits.Length > i)
				{
					Vector2 limits = _liveController.jointLimits[i];
					safeAngle = Mathf.Clamp(safeAngle, limits.x, limits.y);
				}

				ArticulationDrive drive = joint.xDrive;
				drive.target = safeAngle;
				joint.xDrive = drive;
				joint.WakeUp();

				if (i < _shadowJointControllers.Count && _shadowJointControllers[i] != null)
				{
					_shadowJointControllers[i].goalDeg = safeAngle;
				}
			}
		}

		private float[] CloneAndClampAngles(float[] anglesDeg)
		{
			if (anglesDeg == null || anglesDeg.Length < 6)
			{
				return null;
			}

			float[] clone = (float[])anglesDeg.Clone();
			if (_liveController == null || _liveController.jointLimits == null)
			{
				return clone;
			}

			for (int i = 0; i < Mathf.Min(6, _liveController.jointLimits.Length); i++)
			{
				Vector2 limits = _liveController.jointLimits[i];
				clone[i] = Mathf.Clamp(clone[i], limits.x, limits.y);
			}

			return clone;
		}

		private void DisableNonExecutionBehaviours()
		{
			if (_shadowRootObject == null)
			{
				return;
			}

			MonoBehaviour[] behaviours = _shadowRootObject.GetComponentsInChildren<MonoBehaviour>(true);
			for (int i = 0; i < behaviours.Length; i++)
			{
				MonoBehaviour behaviour = behaviours[i];
				if (behaviour == null || behaviour is OneJointTrapezoidController)
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

		private Transform ResolveLiveBaseRoot()
		{
			return _manager != null
				&& _manager.diffDriveController != null
				&& _manager.diffDriveController.rb != null
					? _manager.diffDriveController.rb.transform.root
					: null;
		}

		private static void AppendColliderSet(HashSet<Collider> result, Transform root)
		{
			if (result == null || root == null)
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

				result.Add(collider);
			}
		}

		private static void AppendColliderList(List<Collider> result, Transform root)
		{
			if (result == null || root == null)
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

				result.Add(collider);
			}
		}

		private static bool IsUsableCollider(Collider collider)
		{
			return collider != null
				&& collider.enabled
				&& collider.gameObject.activeInHierarchy
				&& !collider.isTrigger;
		}

		private static float GetEffectivePenetrationThreshold(Collider primary, Collider secondary)
		{
			return Mathf.Max(0.0001f, GetSafeContactOffset(primary) + GetSafeContactOffset(secondary));
		}

		private static float GetSafeContactOffset(Collider collider)
		{
			return collider != null ? Mathf.Max(0f, collider.contactOffset) : 0f;
		}

		private static void CopyJointControllerConfiguration(
			OneJointTrapezoidController source,
			OneJointTrapezoidController target)
		{
			if (source == null || target == null)
			{
				return;
			}

			target.expectedJointName = source.expectedJointName;
			target.stopToleranceDeg = source.stopToleranceDeg;
			target.syncGoalToCurrentOnAwake = source.syncGoalToCurrentOnAwake;
			target.goalChangeReplanThresholdDeg = source.goalChangeReplanThresholdDeg;
			target.preserveVelocityOnSameDirectionRetarget = source.preserveVelocityOnSameDirectionRetarget;
			target.hardRetargetResetThresholdDeg = source.hardRetargetResetThresholdDeg;
			target.vMaxDeg = source.vMaxDeg;
			target.aMaxDeg = source.aMaxDeg;
			target.stiffness = source.stiffness;
			target.damping = source.damping;
			target.forceLimit = source.forceLimit;
			target.wakeUpEachStep = source.wakeUpEachStep;
			target.enabled = source.enabled;
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
			_shadowArmColliders.Clear();
			_shadowJoints.Clear();
			_shadowJointControllers.Clear();
			if (_shadowRootObject != null)
			{
				Object.Destroy(_shadowRootObject);
			}

			_shadowRootObject = null;
			_shadowRootBody = null;
			_liveCommandQueue.Clear();
			_activeObstacles.Clear();
			_sessionActive = false;
			_faultMessage = string.Empty;
			_configuredLiveArmRoot = null;
			_configuredLiveCarMount = null;
		}
	}
}
