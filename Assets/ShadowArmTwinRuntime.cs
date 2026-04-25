using System.Collections.Generic;
using System.Text;
using UnityEngine;
using UnityEngine.Rendering;

namespace RobotSimulation
{
	internal sealed class ShadowArmTwinRuntime
	{
		private const string DirectCommandSessionLabel = "DirectArmCommand";

		private readonly PlannerPhysicsQueries _physicsQueries = new PlannerPhysicsQueries();
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
		private Material _shadowVisualMaterial;
		private float _leadSeconds = 0.35f;
		private bool _sessionActive;
		private string _sessionLabel = string.Empty;
		private string _faultMessage = string.Empty;
		private string _lastBindingReport = string.Empty;
		private float[] _persistedShadowAnglesDeg;
		private float[] _sessionInitialAnglesDeg;
		private bool _sessionInitialPoseGuardPending;

		public Transform ShadowRoot => _shadowRootObject != null ? _shadowRootObject.transform : null;
		public bool IsReady => _shadowRootBody != null
			&& _liveController != null
			&& _liveBinder != null
			&& _liveCarMount != null
			&& _shadowJoints.Count >= 6
			&& _shadowJointControllers.Count >= 6;
		public bool IsSessionActive => _sessionActive;
		public bool HasPendingLiveCommands => false;
		public bool HasFault => !string.IsNullOrEmpty(_faultMessage);
		public string FaultMessage => _faultMessage;
		public string LastBindingReport => _lastBindingReport;
		public bool ShouldDriveVisibleShadow => IsReady
			&& _shadowRootObject != null
			&& _shadowRootObject.activeInHierarchy;

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
			SyncMountToShadowBase();
			EnsureShadowRootVisible();

			return IsReady;
		}

		public void Tick()
		{
			if (!IsReady)
			{
				return;
			}

			SyncShadowConfigurationFromLive();
			SyncMountToShadowBase();

			if (_sessionActive && !HasFault && TryDetectShadowArmCollision(out string collisionMessage))
			{
				FailSession(collisionMessage);
			}
		}

		public bool BeginPlannerSession(string label, IReadOnlyList<Collider> obstacles, out string error)
		{
			return BeginPlannerSession(label, obstacles, null, out error);
		}

		public bool BeginPlannerSession(string label, IReadOnlyList<Collider> obstacles, float[] initialAnglesDeg, out string error)
		{
			return PrepareSession(label, obstacles, initialAnglesDeg, out error);
		}

		public bool BeginManualSession(string label, out string error)
		{
			return PrepareSession(label, CollectManualObstacleSnapshot(), null, out error);
		}

		public bool BeginManualSession(string label, float[] initialAnglesDeg, out string error)
		{
			return PrepareSession(label, CollectManualObstacleSnapshot(), initialAnglesDeg, out error);
		}

		public bool EnsureDirectCommandSession(out string error)
		{
			error = string.Empty;
			if (_sessionActive && !HasFault && _sessionLabel == DirectCommandSessionLabel)
			{
				return true;
			}

			return PrepareSession(DirectCommandSessionLabel, CollectManualObstacleSnapshot(), null, out error);
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

			SyncMountToShadowBase();
			ApplyPendingInitialPoseGuard();
			float[] liveBefore = _liveController != null ? _liveController.CaptureMeasuredJointAngles() : null;
			float[] shadowBefore = CaptureShadowMeasuredAngles();
			ApplyShadowTargetsRaw(safeAngles);
			RememberShadowAngles(safeAngles);
			float[] shadowAfterCommand = CaptureShadowMeasuredAngles();
			Debug.Log(
				$"[ArmDebug][ShadowTwin][Dispatch] label={_sessionLabel}, seq={sequenceId}, lead={_leadSeconds:F3}s, " +
				$"target={FormatAngles(safeAngles)}, inputClampDelta={ComputeMaxAbsDeltaDeg(targetAnglesDeg, safeAngles):F3}deg, " +
				$"liveBefore={FormatAngles(liveBefore)}, shadowBefore={FormatAngles(shadowBefore)}, " +
				$"shadowAfterCommand={FormatAngles(shadowAfterCommand)}, shadowToTargetDelta={ComputeMaxDeltaAngleDeg(shadowAfterCommand, safeAngles):F3}deg, " +
				$"pendingLiveCommands=0, liveDrivenByShadow=N");
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
			_sessionInitialPoseGuardPending = false;
			_sessionInitialAnglesDeg = null;
			HoldShadowCurrentPose();

			_activeObstacles.Clear();
			if (_shadowRootObject != null)
			{
				EnsureShadowRootVisible();
				RememberShadowAngles(CaptureShadowMeasuredAngles());
			}

			Debug.Log($"[ArmDebug][ShadowTwin][SessionComplete] resetShadowToLivePose={resetShadowToLivePose}, keepVisible=Y, {BuildDebugSummary(null)}");
		}

		public void StopAndReset(bool resetShadowToLivePose)
		{
			StopShadowMotionOnly();
			CompleteSession(resetShadowToLivePose);
		}

		public void SyncToLivePose()
		{
			if (!Configure(_manager, _liveBinder, _liveController, _leadSeconds))
			{
				return;
			}

			SyncShadowConfigurationFromLive();
			// This method is intentionally only called from explicit Home/Reset/initial-state paths.
			// Planner session start must not reset the independent shadow arm, but when the live arm is
			// explicitly returned to Home the shadow arm must be forced to the same measured pose.
			bool restoreInactive = EnsureShadowRootVisible();

			ResetShadowToLivePose();
			Debug.Log($"[ArmDebug][ShadowTwin][ExplicitSyncToLive] {BuildDebugSummary(null)}");
			if (restoreInactive)
			{
				RememberShadowAngles(CaptureShadowMeasuredAngles());
			}
		}

		public void ForceToJointAngles(float[] anglesDeg, string reason)
		{
			if (anglesDeg == null || anglesDeg.Length < 6)
			{
				return;
			}

			if (!Configure(_manager, _liveBinder, _liveController, _leadSeconds))
			{
				return;
			}

			bool restoreInactive = EnsureShadowRootVisible();

			SyncShadowConfigurationFromLive();
			SyncMountToShadowBase();
			float[] safeAngles = CloneAndClampAngles(anglesDeg);
			ApplyShadowTargetsRaw(safeAngles);
			ForceShadowMeasuredPose(safeAngles);
			ResetShadowControllerRuntimeStates(safeAngles);
			RememberShadowAngles(safeAngles);
			if (_shadowRootBody != null)
			{
				_shadowRootBody.velocity = Vector3.zero;
				_shadowRootBody.angularVelocity = Vector3.zero;
			}

			Debug.Log($"[ArmDebug][ShadowTwin][ForceToJointAngles] reason='{reason}', {BuildDebugSummary(safeAngles)}");

			if (restoreInactive)
			{
				RememberShadowAngles(safeAngles);
			}
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
			StopShadowMotionOnly();
		}

		private bool PrepareSession(string label, IReadOnlyList<Collider> obstacles, float[] initialAnglesDeg, out string error)
		{
			error = string.Empty;
			_faultMessage = string.Empty;
			_sessionLabel = label ?? string.Empty;
			_sessionInitialAnglesDeg = null;
			_sessionInitialPoseGuardPending = false;

			if (!Configure(_manager, _liveBinder, _liveController, _leadSeconds))
			{
				error = "Shadow arm twin runtime could not be configured.";
				_faultMessage = error;
				return false;
			}

			_sessionLabel = label ?? string.Empty;

			if (_shadowRootObject != null && !_shadowRootObject.activeSelf)
			{
				EnsureShadowRootVisible();
			}

			SyncShadowConfigurationFromLive();
			SyncMountToShadowBase();
			string initialPoseSource = "provided";
			float[] safeInitialAngles = CloneAndClampAngles(initialAnglesDeg);
			if (safeInitialAngles == null && _liveController != null)
			{
				initialPoseSource = "liveMeasuredFallback";
				safeInitialAngles = CloneAndClampAngles(_liveController.CaptureMeasuredJointAngles());
			}

			if (safeInitialAngles != null && safeInitialAngles.Length >= 6)
			{
				ApplyShadowTargetsRaw(safeInitialAngles);
				ForceShadowMeasuredPose(safeInitialAngles);
				ResetShadowControllerRuntimeStates(safeInitialAngles);
				RememberShadowAngles(safeInitialAngles);
				_sessionInitialAnglesDeg = (float[])safeInitialAngles.Clone();
				_sessionInitialPoseGuardPending = true;
				Debug.Log(
					$"[ArmDebug][ShadowTwin][PrepareInitialPose] label={_sessionLabel}, source={initialPoseSource}, " +
					$"initial={FormatAngles(safeInitialAngles)}, after={FormatAngles(CaptureShadowMeasuredAngles())}, " +
					$"delta={ComputeMaxDeltaAngleDeg(CaptureShadowMeasuredAngles(), safeInitialAngles):F3}deg");
			}
			else
			{
				StopShadowMotionOnly();
			}
			AppendObstacleSnapshot(obstacles);
			_sessionActive = true;
			Debug.Log(
				$"[ArmDebug][ShadowTwin][SessionStart] label={_sessionLabel}, lead={_leadSeconds:F3}s, " +
				$"obstacles={_activeObstacles.Count}, independentShadow=Y, resetToLiveOnStart=N, {BuildDebugSummary(null)}");
			return true;
		}

		private void ApplyPendingInitialPoseGuard()
		{
			if (!_sessionInitialPoseGuardPending)
			{
				return;
			}

			_sessionInitialPoseGuardPending = false;
			if (_sessionInitialAnglesDeg == null || _sessionInitialAnglesDeg.Length < 6 || !IsReady)
			{
				return;
			}

			float[] before = CaptureShadowMeasuredAngles();
			float deltaBefore = ComputeMaxDeltaAngleDeg(before, _sessionInitialAnglesDeg);
			ApplyShadowTargetsRaw(_sessionInitialAnglesDeg);
			ForceShadowMeasuredPose(_sessionInitialAnglesDeg);
			ResetShadowControllerRuntimeStates(_sessionInitialAnglesDeg);
			RememberShadowAngles(_sessionInitialAnglesDeg);
			float[] after = CaptureShadowMeasuredAngles();
			Debug.Log(
				$"[ArmDebug][ShadowTwin][InitialPoseGuard] label={_sessionLabel}, " +
				$"before={FormatAngles(before)}, initial={FormatAngles(_sessionInitialAnglesDeg)}, " +
				$"after={FormatAngles(after)}, deltaBefore={FormatTraceFloat(deltaBefore)}deg, " +
				$"deltaAfter={FormatTraceFloat(ComputeMaxDeltaAngleDeg(after, _sessionInitialAnglesDeg))}deg");
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

		private void StopShadowMotionOnly()
		{
			HoldShadowCurrentPose();
		}

		private void HoldShadowCurrentPose()
		{
			if (!IsReady || _shadowJoints.Count < 6)
			{
				return;
			}

			float[] measured = CaptureShadowMeasuredAngles();
			RememberShadowAngles(measured);
			ApplyShadowTargetsRaw(measured);
			ResetShadowControllerRuntimeStates(measured);
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

		internal bool TryCaptureShadowMeasuredAngles(out float[] anglesDeg)
		{
			anglesDeg = null;
			if (!IsReady)
			{
				return false;
			}

			anglesDeg = CaptureShadowMeasuredAngles();
			return true;
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

			bool previousSuppressAutoBind = OneJointTrapezoidController.SuppressAutoBindOnAwakeForShadowClone;
			bool previousSuppressFkInstance = Arm6DOFFKController.SuppressInstanceRegistrationForShadowClone;
			bool previousSuppressIkInstance = Arm6DOFIKController.SuppressInstanceRegistrationForShadowClone;
			OneJointTrapezoidController.SuppressAutoBindOnAwakeForShadowClone = true;
			Arm6DOFFKController.SuppressInstanceRegistrationForShadowClone = true;
			Arm6DOFIKController.SuppressInstanceRegistrationForShadowClone = true;
			try
			{
				_shadowRootObject = Object.Instantiate(_liveArmRoot.gameObject);
			}
			finally
			{
				OneJointTrapezoidController.SuppressAutoBindOnAwakeForShadowClone = previousSuppressAutoBind;
				Arm6DOFFKController.SuppressInstanceRegistrationForShadowClone = previousSuppressFkInstance;
				Arm6DOFIKController.SuppressInstanceRegistrationForShadowClone = previousSuppressIkInstance;
			}

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

			RemoveCopiedJointControllers();
			CollectJointMappingsAndControllers();
			if (_shadowJoints.Count < 6 || _shadowJointControllers.Count < 6)
			{
				if (!string.IsNullOrWhiteSpace(_lastBindingReport))
				{
					Debug.LogError($"[ShadowArmTwinRuntime] {_lastBindingReport}");
				}

				DisposeShadowTwin();
				return false;
			}

			RemoveCopiedNonExecutionBehaviours();
			ConfigureShadowRenderers();
			CollectColliders();
			_shadowRootObject.SetActive(wasActive);
			_configuredLiveArmRoot = _liveArmRoot;
			_configuredLiveCarMount = _liveCarMount;
			IgnoreRobotCollisions();
			SyncShadowConfigurationFromLive();
			ResetShadowToLivePose();
			SyncMountToShadowBase();
			Debug.Log($"[ArmDebug][ShadowTwin][Rebuild] {BuildDebugSummary(null)}");
			return IsReady;
		}

		private void CollectJointMappingsAndControllers()
		{
			_shadowJoints.Clear();
			_shadowJointControllers.Clear();
			if (_liveController == null
				|| _liveController.joints == null
				|| _liveController.joints.Length < 6
				|| _liveController.coordinatedJointControllers == null
				|| _liveController.coordinatedJointControllers.Length < 6
				|| _liveArmRoot == null
				|| _shadowRootObject == null)
			{
				_lastBindingReport = "Shadow arm binding failed: live FK controller, joint arrays, live arm root, or shadow root is incomplete.";
				return;
			}

			StringBuilder report = new StringBuilder();
			bool hasError = false;
			for (int jointIndex = 0; jointIndex < 6; jointIndex++)
			{
				ArticulationBody liveJoint = _liveController.joints[jointIndex];
				if (liveJoint == null)
				{
					hasError = true;
					report.AppendLine($"J{jointIndex + 1}: live FK joint is not bound.");
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
					hasError = true;
					report.AppendLine($"J{jointIndex + 1}: shadow joint missing at clone path '{relativePath}'.");
					continue;
				}

				OneJointTrapezoidController liveControllerComponent = _liveController.coordinatedJointControllers[jointIndex];
				if (liveControllerComponent == null)
				{
					hasError = true;
					report.AppendLine($"J{jointIndex + 1}: live trapezoid controller is not bound.");
					continue;
				}

				OneJointTrapezoidController shadowControllerComponent = shadowJointTransform.gameObject.AddComponent<OneJointTrapezoidController>();
				shadowControllerComponent.joint = shadowJoint;
				CopyJointControllerConfiguration(liveControllerComponent, shadowControllerComponent);

				_shadowJoints.Add(shadowJoint);
				_shadowJointControllers.Add(shadowControllerComponent);
			}

			if (hasError || _shadowJoints.Count != 6 || _shadowJointControllers.Count != 6)
			{
				if (_shadowJoints.Count != 6 || _shadowJointControllers.Count != 6)
				{
					report.AppendLine($"Mapped {_shadowJoints.Count}/6 joints and {_shadowJointControllers.Count}/6 controllers.");
				}

				_shadowJoints.Clear();
				_shadowJointControllers.Clear();
				_lastBindingReport = report.Length > 0
					? $"Shadow arm binding failed:\n{report.ToString().TrimEnd()}"
					: "Shadow arm binding failed: no detailed binding report was produced.";
				return;
			}

			_lastBindingReport = "Shadow arm binding OK: cloned six live FK joints and six trapezoid controllers.";
		}

		private void RemoveCopiedJointControllers()
		{
			if (_shadowRootObject == null)
			{
				return;
			}

			OneJointTrapezoidController[] copiedControllers = _shadowRootObject.GetComponentsInChildren<OneJointTrapezoidController>(true);
			for (int i = 0; i < copiedControllers.Length; i++)
			{
				OneJointTrapezoidController controller = copiedControllers[i];
				if (controller == null)
				{
					continue;
				}

				Object.DestroyImmediate(controller);
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

		private Transform ResolveShadowBaseMount()
		{
			return _manager != null && _liveCarMount != null
				? _manager.GetShadowBaseTwinMappedTransform(_liveCarMount)
				: null;
		}

		private Transform ResolveActiveMount()
		{
			return ResolveShadowBaseMount() ?? _liveCarMount;
		}

		private void SyncMountToShadowBase()
		{
			Transform mount = ResolveActiveMount();
			if (_shadowRootBody == null || mount == null)
			{
				return;
			}

			_shadowRootBody.TeleportRoot(mount.position, mount.rotation);
			if (Vector3.Distance(_shadowRootBody.transform.position, mount.position) > 0.01f
				|| Quaternion.Angle(_shadowRootBody.transform.rotation, mount.rotation) > 1f)
			{
				_shadowRootBody.transform.SetPositionAndRotation(mount.position, mount.rotation);
			}

			Physics.SyncTransforms();
		}

		private void ResetShadowToLivePose()
		{
			if (!IsReady)
			{
				return;
			}

			SyncMountToShadowBase();
			float[] measured = _liveController.CaptureMeasuredJointAngles();
			ApplyShadowTargetsRaw(measured);
			ForceShadowMeasuredPose(measured);
			ResetShadowControllerRuntimeStates(measured);
			RememberShadowAngles(measured);
			if (_shadowRootBody != null)
			{
				_shadowRootBody.velocity = Vector3.zero;
				_shadowRootBody.angularVelocity = Vector3.zero;
			}
		}

		private void ForceShadowMeasuredPose(float[] anglesDeg)
		{
			if (anglesDeg == null || anglesDeg.Length < 6 || _shadowJoints.Count < 6)
			{
				return;
			}

			for (int i = 0; i < 6; i++)
			{
				ArticulationBody joint = _shadowJoints[i];
				if (joint == null || joint.jointPosition.dofCount <= 0)
				{
					continue;
				}

				ArticulationReducedSpace position = joint.jointPosition;
				position[0] = anglesDeg[i] * Mathf.Deg2Rad;
				joint.jointPosition = position;

				ArticulationReducedSpace velocity = joint.jointVelocity;
				if (velocity.dofCount > 0)
				{
					velocity[0] = 0f;
					joint.jointVelocity = velocity;
				}

				joint.velocity = Vector3.zero;
				joint.angularVelocity = Vector3.zero;
				joint.WakeUp();
			}

			Physics.SyncTransforms();
		}

		private bool EnsureShadowRootVisible()
		{
			if (_shadowRootObject == null)
			{
				return false;
			}

			bool wasInactive = !_shadowRootObject.activeSelf;
			if (wasInactive)
			{
				_shadowRootObject.SetActive(true);
				RestorePersistedShadowPoseAfterActivation();
			}

			return wasInactive;
		}

		private void RestorePersistedShadowPoseAfterActivation()
		{
			if (_persistedShadowAnglesDeg == null || _persistedShadowAnglesDeg.Length < 6 || !IsReady)
			{
				return;
			}

			SyncMountToShadowBase();
			ApplyShadowTargetsRaw(_persistedShadowAnglesDeg);
			ForceShadowMeasuredPose(_persistedShadowAnglesDeg);
			ResetShadowControllerRuntimeStates(_persistedShadowAnglesDeg);
			if (_shadowRootBody != null)
			{
				_shadowRootBody.velocity = Vector3.zero;
				_shadowRootBody.angularVelocity = Vector3.zero;
			}
		}

		private void RememberShadowAngles(float[] anglesDeg)
		{
			if (anglesDeg == null || anglesDeg.Length < 6)
			{
				return;
			}

			if (_persistedShadowAnglesDeg == null || _persistedShadowAnglesDeg.Length != 6)
			{
				_persistedShadowAnglesDeg = new float[6];
			}

			for (int i = 0; i < 6; i++)
			{
				_persistedShadowAnglesDeg[i] = anglesDeg[i];
			}
		}

		private void ResetShadowControllerRuntimeStates(float[] goalAnglesDeg)
		{
			if (goalAnglesDeg == null || goalAnglesDeg.Length < 6 || _shadowJointControllers.Count < 6)
			{
				return;
			}

			for (int i = 0; i < 6; i++)
			{
				OneJointTrapezoidController controller = _shadowJointControllers[i];
				if (controller == null)
				{
					continue;
				}

				controller.ResetRuntimeStateToCurrentJoint(goalAnglesDeg[i]);
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

		private void RemoveCopiedNonExecutionBehaviours()
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

				Object.DestroyImmediate(behaviour);
			}
		}

		private void ConfigureShadowRenderers()
		{
			if (_shadowRootObject == null)
			{
				return;
			}

			Renderer[] renderers = _shadowRootObject.GetComponentsInChildren<Renderer>(true);
			for (int i = 0; i < renderers.Length; i++)
			{
				Renderer renderer = renderers[i];
				if (renderer == null)
				{
					continue;
				}

				renderer.enabled = true;
				renderer.shadowCastingMode = ShadowCastingMode.Off;
				renderer.receiveShadows = false;
				if (EnsureShadowVisualMaterial() != null)
				{
					Material[] materials = renderer.sharedMaterials;
					int materialCount = Mathf.Max(1, materials != null ? materials.Length : 1);
					Material[] shadowMaterials = new Material[materialCount];
					for (int materialIndex = 0; materialIndex < materialCount; materialIndex++)
					{
						shadowMaterials[materialIndex] = _shadowVisualMaterial;
					}

					renderer.sharedMaterials = shadowMaterials;
				}
			}
		}

		private Material EnsureShadowVisualMaterial()
		{
			if (_shadowVisualMaterial != null)
			{
				return _shadowVisualMaterial;
			}

			Shader shader = Shader.Find("Standard");
			if (shader == null)
			{
				return null;
			}

			_shadowVisualMaterial = new Material(shader)
			{
				name = "ShadowArmTwin_Runtime_Material",
				hideFlags = HideFlags.DontSave
			};
			_shadowVisualMaterial.color = new Color(0.1f, 0.85f, 1f, 0.38f);
			_shadowVisualMaterial.SetFloat("_Mode", 3f);
			_shadowVisualMaterial.SetInt("_SrcBlend", (int)BlendMode.SrcAlpha);
			_shadowVisualMaterial.SetInt("_DstBlend", (int)BlendMode.OneMinusSrcAlpha);
			_shadowVisualMaterial.SetInt("_ZWrite", 0);
			_shadowVisualMaterial.DisableKeyword("_ALPHATEST_ON");
			_shadowVisualMaterial.EnableKeyword("_ALPHABLEND_ON");
			_shadowVisualMaterial.DisableKeyword("_ALPHAPREMULTIPLY_ON");
			_shadowVisualMaterial.renderQueue = (int)RenderQueue.Transparent;
			return _shadowVisualMaterial;
		}

		internal string BuildDebugSummary(float[] targetAnglesDeg)
		{
			float[] liveAngles = _liveController != null ? _liveController.CaptureMeasuredJointAngles() : null;
			float[] shadowAngles = IsReady ? CaptureShadowMeasuredAngles() : null;
			Transform shadowBaseMount = ResolveShadowBaseMount();
			Transform activeMount = shadowBaseMount != null ? shadowBaseMount : _liveCarMount;
			Vector3 liveMountPosition = _liveCarMount != null ? _liveCarMount.position : Vector3.zero;
			Quaternion liveMountRotation = _liveCarMount != null ? _liveCarMount.rotation : Quaternion.identity;
			Vector3 activeMountPosition = activeMount != null ? activeMount.position : Vector3.zero;
			Quaternion activeMountRotation = activeMount != null ? activeMount.rotation : Quaternion.identity;
			Vector3 shadowRootPosition = _shadowRootBody != null ? _shadowRootBody.transform.position : Vector3.zero;
			Quaternion shadowRootRotation = _shadowRootBody != null ? _shadowRootBody.transform.rotation : Quaternion.identity;
			float rootPosDelta = activeMount != null && _shadowRootBody != null
				? Vector3.Distance(activeMountPosition, shadowRootPosition)
				: float.NaN;
			float liveShadowDelta = ComputeMaxDeltaAngleDeg(liveAngles, shadowAngles);
			float targetShadowDelta = targetAnglesDeg != null ? ComputeMaxDeltaAngleDeg(shadowAngles, targetAnglesDeg) : float.NaN;
			return
				$"ready={IsReady}, active={_sessionActive}, fault='{_faultMessage}', binding='{_lastBindingReport}', " +
				$"mountSource={(shadowBaseMount != null ? "shadowBase" : "liveFallback")}, liveMount={FormatPose(liveMountPosition, liveMountRotation)}, " +
				$"activeMount={FormatPose(activeMountPosition, activeMountRotation)}, shadowRoot={FormatPose(shadowRootPosition, shadowRootRotation)}, " +
				$"rootPosDelta={FormatTraceFloat(rootPosDelta)}m, live={FormatAngles(liveAngles)}, shadow={FormatAngles(shadowAngles)}, " +
				$"liveShadowDelta={FormatTraceFloat(liveShadowDelta)}deg, target={FormatAngles(targetAnglesDeg)}, " +
				$"targetShadowDelta={FormatTraceFloat(targetShadowDelta)}deg, pendingLiveCommands=0, liveDrivenByShadow=N";
		}

		private static string FormatPose(Vector3 position, Quaternion rotation)
		{
			return $"pos={FormatVector(position)}, rot={FormatQuaternion(rotation)}";
		}

		private static string FormatVector(Vector3 value)
		{
			return $"({value.x:F3},{value.y:F3},{value.z:F3})";
		}

		private static string FormatQuaternion(Quaternion value)
		{
			return $"({value.x:F3},{value.y:F3},{value.z:F3},{value.w:F3})";
		}

		private static string FormatAngles(float[] anglesDeg)
		{
			if (anglesDeg == null || anglesDeg.Length < 6)
			{
				return "j[n/a]";
			}

			return $"j[{anglesDeg[0]:F3},{anglesDeg[1]:F3},{anglesDeg[2]:F3},{anglesDeg[3]:F3},{anglesDeg[4]:F3},{anglesDeg[5]:F3}]";
		}

		private static string FormatTraceFloat(float value)
		{
			return float.IsNaN(value) || float.IsInfinity(value) ? "n/a" : value.ToString("F3");
		}

		private static float ComputeMaxAbsDeltaDeg(float[] expected, float[] actual)
		{
			if (expected == null || actual == null || expected.Length < 6 || actual.Length < 6)
			{
				return float.NaN;
			}

			float maxDelta = 0f;
			for (int i = 0; i < 6; i++)
			{
				maxDelta = Mathf.Max(maxDelta, Mathf.Abs(expected[i] - actual[i]));
			}

			return maxDelta;
		}

		private static float ComputeMaxDeltaAngleDeg(float[] expected, float[] actual)
		{
			if (expected == null || actual == null || expected.Length < 6 || actual.Length < 6)
			{
				return float.NaN;
			}

			float maxDelta = 0f;
			for (int i = 0; i < 6; i++)
			{
				maxDelta = Mathf.Max(maxDelta, Mathf.Abs(Mathf.DeltaAngle(expected[i], actual[i])));
			}

			return maxDelta;
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
			_activeObstacles.Clear();
			_sessionActive = false;
			_faultMessage = string.Empty;
			_configuredLiveArmRoot = null;
			_configuredLiveCarMount = null;
		}
	}
}
