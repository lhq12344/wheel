using System;
using System.Text;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Rendering;

namespace RobotSimulation
{
	[DisallowMultipleComponent]
	public sealed class ShadowRobotVisualizer : MonoBehaviour
	{
		private static ShadowRobotVisualizer _activeInstance;
		private const string ShadowContainerName = "ShadowRobotVisual";

		private enum PreviewMode
		{
			Mirror,
			BasePreview,
			ArmPreview
		}

		[Header("Shadow Visual")]
		[SerializeField] private bool visualizationEnabled = true;
		[SerializeField][Range(0.05f, 1f)] private float shadowAlpha = 0.35f;
		[SerializeField] private Color shadowTint = new Color(0.18f, 0.82f, 1f, 0.35f);
		[SerializeField] private bool usePredictedBasePose = true;
		[SerializeField][Range(0.05f, 1f)] private float fallbackLeadSeconds = 0.20f;
		[SerializeField][Range(0.05f, 0.50f)] private float maxVisualizationLeadSeconds = 0.25f;

		[Header("Debug")]
		[SerializeField] private int bindingCount;
		[SerializeField] private string sourceSummary = "Shadow source is not configured.";
		[SerializeField] private PreviewMode previewMode = PreviewMode.Mirror;
		[SerializeField][TextArea(3, 24)] private string lastArmHierarchySnapshot = string.Empty;
		[SerializeField][TextArea(3, 24)] private string lastArmRuntimeSnapshot = string.Empty;
		[SerializeField][TextArea(3, 24)] private string lastArmNoOpAlignmentSnapshot = string.Empty;

		private RobotSimulationManager _manager;
		private Transform _shadowContainer;
		private readonly List<TransformBinding> _bindings = new List<TransformBinding>();
		private readonly Dictionary<Transform, Transform> _sourceToShadowTransformMap = new Dictionary<Transform, Transform>();
		private readonly List<PendingSkinnedBinding> _pendingSkinnedBindings = new List<PendingSkinnedBinding>();
		private Transform _cachedBaseRoot;
		private Transform _cachedArmRoot;
		private Transform _baseReferenceTransform;
		private Transform _sourceArmMountTransform;
		private Transform _shadowArmMountTransform;
		private Material _sharedShadowMaterial;
		private bool _isBuilt;
		private int _ignoreRaycastLayer = -1;
		private Vector3 _baseRootLocalPositionFromReference;
		private Quaternion _baseRootLocalRotationFromReference = Quaternion.identity;
		private Vector3 _predictedBasePosition;
		private Quaternion _predictedBaseRotation = Quaternion.identity;
		private float _predictedBasePoseExpireRealtime;
		private bool _hasPredictedBasePose;
		private Vector3 _executionBasePosition;
		private Quaternion _executionBaseRotation = Quaternion.identity;
		private bool _hasExecutionBasePose;
		private Vector3 _simulatedBasePosition;
		private Quaternion _simulatedBaseRotation = Quaternion.identity;
		private float _simulatedBaseLinearVelocity;
		private float _simulatedBaseAngularVelocity;

		private float _simulatedBaseCommandExpireRealtime;
		private float _simulatedBaseLastUpdateRealtime;
		private bool _hasSimulatedBasePose;
		private readonly float[] _predictedArmAnglesDeg = new float[6];
		private float _predictedArmPoseExpireRealtime;
		private bool _hasPredictedArmAngles;
		private Vector3 _armPreviewBaseReferencePosition;
		private Quaternion _armPreviewBaseReferenceRotation = Quaternion.identity;
		private bool _hasArmPreviewBaseReferencePose;
		private bool _armDiagnosticsEnabled;
		private bool _armHierarchySnapshotLogged;
		private bool _hasPendingArmTrace;
		private PendingArmTrace _pendingArmTrace;

		private struct TransformBinding
		{
			public Transform source;
			public Transform shadow;
			public bool isRoot;
			public bool rootUsesWorldSpace;
			public Transform rootReference;
		}

		private struct PendingSkinnedBinding
		{
			public SkinnedMeshRenderer sourceRenderer;
			public SkinnedMeshRenderer shadowRenderer;
		}

		private struct PendingArmTrace
		{
			public long sequenceId;
			public int sampleIndex;
			public float leadSeconds;
			public float throttleRatio;
			public float[] targetAnglesDeg;
		}

		public static ShadowRobotVisualizer ActiveInstance => _activeInstance;

		private void Awake()
		{
			EnsureSingleInstance();
		}

		private void OnEnable()
		{
			EnsureSingleInstance();
		}

		private void OnDisable()
		{
			if (_activeInstance == this)
			{
				_activeInstance = null;
			}
		}

		private void EnsureSingleInstance()
		{
			if (_activeInstance != null && _activeInstance != this)
			{
				Debug.LogWarning("[ShadowRobotVisualizer] Duplicate visualizer detected. This instance will be disabled to keep a single shadow visualizer.");
				enabled = false;
				return;
			}

			_activeInstance = this;
		}

		public void Configure(RobotSimulationManager manager)
		{
			EnsureSingleInstance();
			if (_activeInstance != null && _activeInstance != this)
			{
				_activeInstance.Configure(manager);
				return;
			}

			_manager = manager;
			RebuildIfNeeded(forceRebuild: false);
		}

		internal void SetArmDiagnosticsEnabled(bool enabled)
		{
			if (_activeInstance != null && _activeInstance != this)
			{
				_activeInstance.SetArmDiagnosticsEnabled(enabled);
				return;
			}

			_armDiagnosticsEnabled = enabled;
			_armHierarchySnapshotLogged = false;
			_hasPendingArmTrace = false;
		}

		internal void StagePendingArmTrace(long sequenceId, int sampleIndex, float[] targetAnglesDeg, float leadSeconds, float throttleRatio)
		{
			if (_activeInstance != null && _activeInstance != this)
			{
				_activeInstance.StagePendingArmTrace(sequenceId, sampleIndex, targetAnglesDeg, leadSeconds, throttleRatio);
				return;
			}

			if (!_armDiagnosticsEnabled)
			{
				return;
			}

			_pendingArmTrace = new PendingArmTrace
			{
				sequenceId = sequenceId,
				sampleIndex = sampleIndex,
				leadSeconds = leadSeconds,
				throttleRatio = throttleRatio,
				targetAnglesDeg = targetAnglesDeg != null ? (float[])targetAnglesDeg.Clone() : null
			};
			_hasPendingArmTrace = true;
		}

		internal string CaptureArmHierarchySnapshotReport()
		{
			if (_activeInstance != null && _activeInstance != this)
			{
				return _activeInstance.CaptureArmHierarchySnapshotReport();
			}

			RebuildIfNeeded(forceRebuild: false);
			lastArmHierarchySnapshot = BuildArmHierarchySnapshotReport();
			return lastArmHierarchySnapshot;
		}

		internal string CaptureCurrentArmRuntimeStateSummary(float[] targetAnglesDeg)
		{
			if (_activeInstance != null && _activeInstance != this)
			{
				return _activeInstance.CaptureCurrentArmRuntimeStateSummary(targetAnglesDeg);
			}

			lastArmRuntimeSnapshot = BuildArmRuntimeStateSummary(targetAnglesDeg);
			return lastArmRuntimeSnapshot;
		}

		public void SetVisualizationEnabled(bool enabled)
		{
			if (_activeInstance != null && _activeInstance != this)
			{
				_activeInstance.SetVisualizationEnabled(enabled);
				return;
			}

			visualizationEnabled = enabled;
			if (_shadowContainer != null)
			{
				_shadowContainer.gameObject.SetActive(enabled);
			}
		}

		public void PushPredictedBasePose(Vector3 worldPosition, Quaternion worldRotation, float holdSeconds)
		{
			if (_activeInstance != null && _activeInstance != this)
			{
				_activeInstance.PushPredictedBasePose(worldPosition, worldRotation, holdSeconds);
				return;
			}

			_predictedBasePosition = worldPosition;
			_predictedBaseRotation = worldRotation;
			_predictedBasePoseExpireRealtime = Time.realtimeSinceStartup + Mathf.Max(0.05f, holdSeconds);
			_hasPredictedBasePose = true;
		}

		public void EnterBasePreview()
		{
			if (_activeInstance != null && _activeInstance != this)
			{
				_activeInstance.EnterBasePreview();
				return;
			}

			RebuildIfNeeded(forceRebuild: false);
			previewMode = PreviewMode.BasePreview;
			_hasArmPreviewBaseReferencePose = false;
			ClearPredictedBasePose();
			ClearPredictedArmPose();
			if (_manager != null && _manager.TryGetShadowBaseTwinPose(out Vector3 shadowBasePosition, out Quaternion shadowBaseRotation))
			{
				SetExecutionBasePose(shadowBasePosition, shadowBaseRotation);
			}
			else if (_cachedBaseRoot != null)
			{
				SetExecutionBasePose(_cachedBaseRoot.position, _cachedBaseRoot.rotation);
			}
			else
			{
				ClearExecutionBasePose();
			}
			EnsureSimulatedBasePose(Time.realtimeSinceStartup);
		}

		public void EnterArmPreview(Vector3 settledBaseWorldPos, Quaternion settledBaseWorldRot)
		{
			if (_activeInstance != null && _activeInstance != this)
			{
				_activeInstance.EnterArmPreview(settledBaseWorldPos, settledBaseWorldRot);
				return;
			}

			RebuildIfNeeded(forceRebuild: false);
			previewMode = PreviewMode.ArmPreview;
			_armPreviewBaseReferencePosition = settledBaseWorldPos;
			_armPreviewBaseReferenceRotation = settledBaseWorldRot;
			_hasArmPreviewBaseReferencePose = true;
			SetSimulatedBasePose(settledBaseWorldPos, settledBaseWorldRot);
			ClearPredictedBasePose();
			ClearExecutionBasePose();
			TraceArmHierarchySnapshotIfNeeded();
		}

		public void ReturnToMirror()
		{
			if (_activeInstance != null && _activeInstance != this)
			{
				_activeInstance.ReturnToMirror();
				return;
			}

			previewMode = PreviewMode.Mirror;
			_hasArmPreviewBaseReferencePose = false;
			ClearPredictedBasePose();
			ClearPredictedArmPose();
			ClearExecutionBasePose();
			_hasPendingArmTrace = false;
			_armHierarchySnapshotLogged = false;
			if (_baseReferenceTransform != null)
			{
				SetSimulatedBasePose(_baseReferenceTransform.position, _baseReferenceTransform.rotation);
			}
			else if (_cachedBaseRoot != null)
			{
				SetSimulatedBasePose(_cachedBaseRoot.position, _cachedBaseRoot.rotation);
			}
		}

		public void ResetToMirrorSnapshot(MirrorSnapshot snapshot, float[] armAnglesDeg)
		{
			if (_activeInstance != null && _activeInstance != this)
			{
				_activeInstance.ResetToMirrorSnapshot(snapshot, armAnglesDeg);
				return;
			}

			RebuildIfNeeded(forceRebuild: false);
			previewMode = PreviewMode.Mirror;
			_hasArmPreviewBaseReferencePose = false;
			SetSimulatedBasePose(snapshot.baseWorldPosition, snapshot.baseWorldRotation);
			ClearPredictedBasePose();
			ClearPredictedArmPose();
			ClearExecutionBasePose();
			_hasPendingArmTrace = false;
			_armHierarchySnapshotLogged = false;
		}

		public void ApplyShadowStep(SafetyGateTimelineCommand command)
		{
			if (_activeInstance != null && _activeInstance != this)
			{
				_activeInstance.ApplyShadowStep(command);
				return;
			}

			if (command.kind == SafetyGateTimelineCommandKind.Base)
			{
				if (previewMode != PreviewMode.BasePreview)
				{
					return;
				}

				float visualizationLeadSeconds = GetVisualizationLeadSeconds(command.predictedLeadSeconds);
				SetSimulatedBaseCommand(
					command.baseLinearVelocity,
					command.baseAngularVelocity,
					visualizationLeadSeconds);

				if (TryPredictBasePoseFromCommand(command, visualizationLeadSeconds, out Vector3 predictedPosition, out Quaternion predictedRotation))
				{
					PushPredictedBasePose(
						predictedPosition,
						predictedRotation,
						visualizationLeadSeconds);
					return;
				}

				bool hasCommandPredictedPose = command.predictedBaseWorldRotation != default;
				bool hasPredictedPose = command.predictedBaseWorldRotation != default;
				PushPredictedBasePose(
					hasPredictedPose ? command.predictedBaseWorldPosition : command.baseCommand.toWorldPosition,
					hasPredictedPose ? command.predictedBaseWorldRotation : Quaternion.Euler(0f, command.baseCommand.targetYawDeg, 0f),
					visualizationLeadSeconds);
				return;
			}

			if (previewMode == PreviewMode.ArmPreview)
			{
				if (!_hasPendingArmTrace)
				{
					StagePendingArmTrace(
						command.sequenceId,
						-1,
						command.armCommand.toAnglesDeg,
						command.predictedLeadSeconds,
						command.throttleRatio);
				}
				SetPredictedArmAngles(command.armCommand.toAnglesDeg, GetVisualizationLeadSeconds(command.predictedLeadSeconds));
			}
		}

		public void ApplyBaseExecutionPreviewStep(DiffDriveTwinController.DrivePredictionStep step)
		{
			if (_activeInstance != null && _activeInstance != this)
			{
				_activeInstance.ApplyBaseExecutionPreviewStep(step);
				return;
			}

			if (previewMode != PreviewMode.BasePreview)
			{
				return;
			}

			Vector3 worldPosition = DiffDriveTwinController.ResolveExecutionPreviewPosition(step);
			Quaternion worldRotation = DiffDriveTwinController.ResolveExecutionPreviewRotation(step);
			SetExecutionBasePose(worldPosition, worldRotation);
			SetSimulatedBasePose(worldPosition, worldRotation);
		}

		public void ApplyManualBasePreviewStep(
			DiffDriveTwinController.DrivePredictionStep step)
		{
			if (_activeInstance != null && _activeInstance != this)
			{
				_activeInstance.ApplyManualBasePreviewStep(step);
				return;
			}

			ApplyBaseExecutionPreviewStep(step);
		}

		public void ApplyManualArmPreviewSample(RobotPlanJointSample sample, float holdSeconds = -1f)
		{
			if (_activeInstance != null && _activeInstance != this)
			{
				_activeInstance.ApplyManualArmPreviewSample(sample, holdSeconds);
				return;
			}

			if (sample == null)
			{
				return;
			}

			ApplyManualArmPreviewAngles(sample.jointAnglesDeg, holdSeconds);
		}

		public void ApplyManualArmPreviewAngles(float[] armAnglesDeg, float holdSeconds = -1f)
		{
			if (_activeInstance != null && _activeInstance != this)
			{
				_activeInstance.ApplyManualArmPreviewAngles(armAnglesDeg, holdSeconds);
				return;
			}

			if (previewMode != PreviewMode.ArmPreview)
			{
				return;
			}

			float resolvedHoldSeconds = holdSeconds > 0f ? holdSeconds : fallbackLeadSeconds;
			SetPredictedArmAngles(armAnglesDeg, Mathf.Max(0.05f, resolvedHoldSeconds));
		}

		public void RebuildIfNeeded(bool forceRebuild = false)
		{
			EnsureSingleInstance();
			if (_activeInstance != null && _activeInstance != this)
			{
				return;
			}

			if (!TryResolveSourceRoots(out Transform baseRoot, out Transform armRoot))
			{
				sourceSummary = "Shadow source roots are unavailable.";
				ClearShadowHierarchy();
				return;
			}

			UpdateBaseReferenceFrame(baseRoot, armRoot);
			bool rootsChanged = baseRoot != _cachedBaseRoot || armRoot != _cachedArmRoot;
			if (forceRebuild || rootsChanged || !_isBuilt)
			{
				_cachedBaseRoot = baseRoot;
				_cachedArmRoot = armRoot;
				BuildShadowHierarchy();
			}

			if (_shadowContainer != null)
			{
				_shadowContainer.gameObject.SetActive(visualizationEnabled);
			}
		}

		private void LateUpdate()
		{
			RebuildIfNeeded();
			if (!visualizationEnabled || !_isBuilt)
			{
				return;
			}

			SyncShadowTransforms();
		}

		private void OnValidate()
		{
			shadowAlpha = Mathf.Clamp(shadowAlpha, 0.05f, 1f);
			UpdateShadowMaterialTint();
			if (_shadowContainer != null)
			{
				_shadowContainer.gameObject.SetActive(visualizationEnabled);
			}
		}

		private void OnDestroy()
		{
			if (_activeInstance == this)
			{
				_activeInstance = null;
			}

			ClearShadowHierarchy();
			DestroyShadowMaterial();
		}

		private bool TryResolveSourceRoots(out Transform baseRoot, out Transform armRoot)
		{
			if (_manager == null)
			{
				_manager = GetComponent<RobotSimulationManager>();
			}

			baseRoot = null;
			armRoot = null;
			if (_manager == null)
			{
				return false;
			}

			if (_manager.diffDriveController != null && _manager.diffDriveController.rb != null)
			{
				Transform rbTransform = _manager.diffDriveController.rb.transform;
				Transform controllerTransform = _manager.diffDriveController.transform;
				baseRoot = ResolvePreferredBaseRoot(rbTransform, controllerTransform);
			}

			if (_manager.ShouldUseShadowArmTwinSource() && _manager.GetShadowArmTwinRoot() != null)
			{
				armRoot = _manager.GetShadowArmTwinRoot();
			}
			else if (_manager.armBinder != null && _manager.armBinder.armRoot != null)
			{
				armRoot = _manager.armBinder.armRoot.transform;
			}
			else if (_manager.arm6DOFFKController != null && _manager.arm6DOFFKController.BaseFrameTransform != null)
			{
				armRoot = _manager.arm6DOFFKController.BaseFrameTransform;
			}

			if (baseRoot != null && armRoot != null)
			{
				if (armRoot == baseRoot || armRoot.IsChildOf(baseRoot))
				{
					armRoot = null;
				}
				else if (baseRoot.IsChildOf(armRoot))
				{
					armRoot = null;
				}
			}

			sourceSummary = $"baseRoot={(baseRoot != null ? baseRoot.name : "none")}, armRoot={(armRoot != null ? armRoot.name : "none")}";
			return baseRoot != null || armRoot != null;
		}

		private void UpdateBaseReferenceFrame(Transform baseRoot, Transform armRoot)
		{
			_baseReferenceTransform = null;
			_sourceArmMountTransform = null;
			_shadowArmMountTransform = null;
			_baseRootLocalPositionFromReference = Vector3.zero;
			_baseRootLocalRotationFromReference = Quaternion.identity;

			if (_manager != null && _manager.diffDriveController != null && _manager.diffDriveController.rb != null)
			{
				_baseReferenceTransform = _manager.diffDriveController.rb.transform;
			}

			if (_baseReferenceTransform == null)
			{
				_baseReferenceTransform = baseRoot;
			}

			if (baseRoot == null || _baseReferenceTransform == null)
			{
				return;
			}

			Quaternion referenceRotation = _baseReferenceTransform.rotation;
			_baseRootLocalPositionFromReference = Quaternion.Inverse(referenceRotation) * (baseRoot.position - _baseReferenceTransform.position);
			_baseRootLocalRotationFromReference = Quaternion.Inverse(referenceRotation) * baseRoot.rotation;
			_sourceArmMountTransform = ResolveSourceArmMountTransform(baseRoot, armRoot);

			sourceSummary = $"baseRoot={baseRoot.name}, armRoot={(armRoot != null ? armRoot.name : "none")}, baseRef={_baseReferenceTransform.name}, armMount={(_sourceArmMountTransform != null ? _sourceArmMountTransform.name : "none")}";
		}

		private Transform ResolveSourceArmMountTransform(Transform baseRoot, Transform armRoot)
		{
			Transform candidate = null;
			if (_manager != null && _manager.armBinder != null && _manager.armBinder.carMount != null)
			{
				candidate = _manager.armBinder.carMount;
			}
			else if (_manager != null && _manager.arm6DOFFKController != null && _manager.arm6DOFFKController.BaseFrameTransform != null)
			{
				candidate = _manager.arm6DOFFKController.BaseFrameTransform;
			}
			else if (armRoot != null)
			{
				candidate = armRoot.parent;
			}

			if (candidate == null)
			{
				return baseRoot;
			}

			if (baseRoot != null && candidate != baseRoot && !candidate.IsChildOf(baseRoot))
			{
				return baseRoot;
			}

			return candidate;
		}

		private static Transform ResolvePreferredBaseRoot(Transform rbTransform, Transform controllerTransform)
		{
			// Prefer the rigidbody subtree because it is guaranteed to move with the chassis.
			if (rbTransform != null && HasRenderableDescendant(rbTransform))
			{
				return rbTransform;
			}

			if (controllerTransform != null
				&& (rbTransform == null || controllerTransform == rbTransform || controllerTransform.IsChildOf(rbTransform))
				&& HasRenderableDescendant(controllerTransform))
			{
				return controllerTransform;
			}

			return rbTransform != null ? rbTransform : controllerTransform;
		}

		private void BuildShadowHierarchy()
		{
			EnsureShadowMaterial();
			_ignoreRaycastLayer = LayerMask.NameToLayer("Ignore Raycast");
			EnsureShadowContainer();
			ClearShadowNodes();
			_shadowArmMountTransform = null;
			Transform excludedBaseSubtreeRoot = GetExcludedBaseSubtreeRoot();

			if (_cachedBaseRoot != null)
			{
				BuildShadowNodeRecursive(_cachedBaseRoot, _shadowContainer, true, null, excludedBaseSubtreeRoot);
			}

			if (_cachedArmRoot != null)
			{
				bool useShadowArmTwinWorldPose = _manager != null && _manager.ShouldUseShadowArmTwinSource();
				_shadowArmMountTransform = useShadowArmTwinWorldPose ? null : TryMapShadowTransform(_sourceArmMountTransform);
				if (_shadowArmMountTransform == null && !useShadowArmTwinWorldPose)
				{
					_shadowArmMountTransform = TryMapShadowTransform(_cachedBaseRoot);
				}

				Transform armShadowParent = useShadowArmTwinWorldPose
					? _shadowContainer
					: (_shadowArmMountTransform != null ? _shadowArmMountTransform : _shadowContainer);
				Transform armRootReference = useShadowArmTwinWorldPose
					? null
					: (_sourceArmMountTransform != null ? _sourceArmMountTransform : null);
				BuildShadowNodeRecursive(_cachedArmRoot, armShadowParent, true, armRootReference, null);
			}

			ResolvePendingSkinnedBindings();
			bindingCount = _bindings.Count;
			_isBuilt = _bindings.Count > 0;
			if (_shadowContainer != null)
			{
				_shadowContainer.gameObject.SetActive(visualizationEnabled);
			}
		}

		private void EnsureShadowContainer()
		{
			PruneDuplicateShadowContainers();
			if (_shadowContainer != null)
			{
				ApplyShadowObjectHideFlags(_shadowContainer.gameObject);
				return;
			}

			GameObject containerObject = new GameObject(ShadowContainerName);
			ApplyShadowObjectHideFlags(containerObject);
			_shadowContainer = containerObject.transform;
			_shadowContainer.SetParent(transform, false);
			_shadowContainer.localPosition = Vector3.zero;
			_shadowContainer.localRotation = Quaternion.identity;
			_shadowContainer.localScale = Vector3.one;
		}

		private void PruneDuplicateShadowContainers()
		{
			List<GameObject> duplicates = new List<GameObject>();
			for (int childIndex = 0; childIndex < transform.childCount; childIndex++)
			{
				Transform child = transform.GetChild(childIndex);
				if (child == null || child.name != ShadowContainerName)
				{
					continue;
				}

				if (_shadowContainer == null)
				{
					_shadowContainer = child;
					continue;
				}

				if (child != _shadowContainer)
				{
					duplicates.Add(child.gameObject);
				}
			}

			for (int i = 0; i < duplicates.Count; i++)
			{
				DestroyGameObject(duplicates[i]);
			}
		}

		private Transform GetExcludedBaseSubtreeRoot()
		{
			if (_manager == null
				|| !_manager.ShouldUseShadowArmTwinSource()
				|| _manager.armBinder == null
				|| _manager.armBinder.armRoot == null)
			{
				return null;
			}

			Transform liveArmRoot = _manager.armBinder.armRoot.transform;
			return liveArmRoot != null && _cachedBaseRoot != null && liveArmRoot.IsChildOf(_cachedBaseRoot)
				? liveArmRoot
				: null;
		}

		private void BuildShadowNodeRecursive(Transform source, Transform parent, bool isRoot, Transform rootReference, Transform excludedSubtreeRoot)
		{
			if (source == null || parent == null)
			{
				return;
			}

			if (excludedSubtreeRoot != null && source == excludedSubtreeRoot)
			{
				return;
			}

			GameObject shadowObject = new GameObject($"{source.name}_Shadow");
			ApplyShadowObjectHideFlags(shadowObject);
			if (_ignoreRaycastLayer >= 0)
			{
				shadowObject.layer = _ignoreRaycastLayer;
			}

			Transform shadowTransform = shadowObject.transform;
			shadowTransform.SetParent(parent, false);
			if (isRoot)
			{
				if (rootReference != null)
				{
					shadowTransform.localPosition = rootReference.InverseTransformPoint(source.position);
					shadowTransform.localRotation = Quaternion.Inverse(rootReference.rotation) * source.rotation;
					shadowTransform.localScale = source.localScale;
				}
				else
				{
					shadowTransform.position = source.position;
					shadowTransform.rotation = source.rotation;
					shadowTransform.localScale = source.lossyScale;
				}
			}
			else
			{
				shadowTransform.localPosition = source.localPosition;
				shadowTransform.localRotation = source.localRotation;
				shadowTransform.localScale = source.localScale;
			}

			_bindings.Add(new TransformBinding
			{
				source = source,
				shadow = shadowTransform,
				isRoot = isRoot,
				rootUsesWorldSpace = rootReference == null,
				rootReference = rootReference
			});
			_sourceToShadowTransformMap[source] = shadowTransform;

			TryCopyRenderer(source, shadowObject);

			for (int i = 0; i < source.childCount; i++)
			{
				BuildShadowNodeRecursive(source.GetChild(i), shadowTransform, false, null, excludedSubtreeRoot);
			}
		}

		private void TryCopyRenderer(Transform source, GameObject shadowObject)
		{
			if (source == null || shadowObject == null)
			{
				return;
			}

			MeshFilter sourceMeshFilter = source.GetComponent<MeshFilter>();
			MeshRenderer sourceMeshRenderer = source.GetComponent<MeshRenderer>();
			if (sourceMeshFilter != null && sourceMeshRenderer != null && sourceMeshFilter.sharedMesh != null)
			{
				MeshFilter shadowMeshFilter = shadowObject.AddComponent<MeshFilter>();
				shadowMeshFilter.sharedMesh = sourceMeshFilter.sharedMesh;

				MeshRenderer shadowMeshRenderer = shadowObject.AddComponent<MeshRenderer>();
				ApplyShadowRendererSettings(
					shadowMeshRenderer,
					sourceMeshRenderer.sharedMaterials != null ? sourceMeshRenderer.sharedMaterials.Length : 1);
				return;
			}

			SkinnedMeshRenderer sourceSkinnedMeshRenderer = source.GetComponent<SkinnedMeshRenderer>();
			if (sourceSkinnedMeshRenderer != null && sourceSkinnedMeshRenderer.sharedMesh != null)
			{
				SkinnedMeshRenderer shadowSkinnedRenderer = shadowObject.AddComponent<SkinnedMeshRenderer>();
				shadowSkinnedRenderer.sharedMesh = sourceSkinnedMeshRenderer.sharedMesh;
				shadowSkinnedRenderer.localBounds = sourceSkinnedMeshRenderer.localBounds;
				shadowSkinnedRenderer.updateWhenOffscreen = sourceSkinnedMeshRenderer.updateWhenOffscreen;
				ApplyShadowRendererSettings(
					shadowSkinnedRenderer,
					sourceSkinnedMeshRenderer.sharedMaterials != null ? sourceSkinnedMeshRenderer.sharedMaterials.Length : 1);
				_pendingSkinnedBindings.Add(new PendingSkinnedBinding
				{
					sourceRenderer = sourceSkinnedMeshRenderer,
					shadowRenderer = shadowSkinnedRenderer
				});
			}
		}

		private void ResolvePendingSkinnedBindings()
		{
			for (int i = 0; i < _pendingSkinnedBindings.Count; i++)
			{
				PendingSkinnedBinding binding = _pendingSkinnedBindings[i];
				if (binding.sourceRenderer == null || binding.shadowRenderer == null)
				{
					continue;
				}

				Transform mappedRoot = TryMapShadowTransform(binding.sourceRenderer.rootBone);
				if (mappedRoot == null)
				{
					mappedRoot = TryMapShadowTransform(binding.sourceRenderer.transform);
				}

				binding.shadowRenderer.rootBone = mappedRoot;

				Transform[] sourceBones = binding.sourceRenderer.bones;
				if (sourceBones == null || sourceBones.Length == 0)
				{
					binding.shadowRenderer.bones = Array.Empty<Transform>();
					continue;
				}

				Transform[] mappedBones = new Transform[sourceBones.Length];
				for (int boneIndex = 0; boneIndex < sourceBones.Length; boneIndex++)
				{
					Transform mappedBone = TryMapShadowTransform(sourceBones[boneIndex]);
					mappedBones[boneIndex] = mappedBone != null ? mappedBone : mappedRoot;
				}

				binding.shadowRenderer.bones = mappedBones;
			}
		}

		private Transform TryMapShadowTransform(Transform source)
		{
			if (source == null)
			{
				return null;
			}

			return _sourceToShadowTransformMap.TryGetValue(source, out Transform mapped) ? mapped : null;
		}

		private void ApplyShadowRendererSettings(Renderer renderer, int materialSlots)
		{
			if (renderer == null)
			{
				return;
			}

			int slotCount = Mathf.Max(1, materialSlots);
			Material[] materials = new Material[slotCount];
			for (int i = 0; i < slotCount; i++)
			{
				materials[i] = _sharedShadowMaterial;
			}

			renderer.sharedMaterials = materials;
			renderer.shadowCastingMode = ShadowCastingMode.Off;
			renderer.receiveShadows = false;
			renderer.lightProbeUsage = LightProbeUsage.Off;
			renderer.reflectionProbeUsage = ReflectionProbeUsage.Off;
			renderer.motionVectorGenerationMode = MotionVectorGenerationMode.ForceNoMotion;
		}

		private void SyncShadowTransforms()
		{
			AdvanceSimulatedBasePose(Time.realtimeSinceStartup);

			for (int i = 0; i < _bindings.Count; i++)
			{
				TransformBinding binding = _bindings[i];
				if (binding.source == null || binding.shadow == null)
				{
					_isBuilt = false;
					return;
				}

				if (binding.isRoot)
				{
					if (binding.source == _cachedBaseRoot
						&& TryGetDesiredBasePose(out Vector3 desiredBasePosition, out Quaternion desiredBaseRotation))
					{
						binding.shadow.position = desiredBasePosition;
						binding.shadow.rotation = desiredBaseRotation;
						binding.shadow.localScale = binding.source.lossyScale;
					}
					else if (!binding.rootUsesWorldSpace && binding.rootReference != null)
					{
						binding.shadow.localPosition = binding.rootReference.InverseTransformPoint(binding.source.position);
						binding.shadow.localRotation = Quaternion.Inverse(binding.rootReference.rotation) * binding.source.rotation;
						binding.shadow.localScale = binding.source.localScale;
					}
					else
					{
						binding.shadow.position = binding.source.position;
						binding.shadow.rotation = binding.source.rotation;
						binding.shadow.localScale = binding.source.lossyScale;
					}
				}
				else
				{
					binding.shadow.localPosition = binding.source.localPosition;
					binding.shadow.localRotation = binding.source.localRotation;
					binding.shadow.localScale = binding.source.localScale;
				}
			}

			ApplyPredictedArmPoseToShadow();
		}

		private void SetSimulatedBasePose(Vector3 worldPosition, Quaternion worldRotation)
		{
			float nowRealtime = Time.realtimeSinceStartup;
			_simulatedBasePosition = worldPosition;
			_simulatedBaseRotation = ExtractPlanarRotation(worldRotation);
			_simulatedBaseLinearVelocity = 0f;
			_simulatedBaseAngularVelocity = 0f;
			_simulatedBaseCommandExpireRealtime = nowRealtime;
			_simulatedBaseLastUpdateRealtime = nowRealtime;
			_hasSimulatedBasePose = true;
		}

		private void SetSimulatedBaseCommand(float linearVelocity, float angularVelocity, float holdSeconds)
		{
			float nowRealtime = Time.realtimeSinceStartup;
			EnsureSimulatedBasePose(nowRealtime);
			AdvanceSimulatedBasePose(nowRealtime);
			_simulatedBaseLinearVelocity = linearVelocity;
			_simulatedBaseAngularVelocity = angularVelocity;
			_simulatedBaseCommandExpireRealtime = nowRealtime + Mathf.Max(0.05f, holdSeconds);
			_simulatedBaseLastUpdateRealtime = nowRealtime;
		}

		private void EnsureSimulatedBasePose(float nowRealtime)
		{
			if (_hasSimulatedBasePose)
			{
				return;
			}

			if (_cachedBaseRoot != null)
			{
				_simulatedBasePosition = _cachedBaseRoot.position;
				_simulatedBaseRotation = ExtractPlanarRotation(_cachedBaseRoot.rotation);
				_simulatedBaseLinearVelocity = 0f;
				_simulatedBaseAngularVelocity = 0f;
				_simulatedBaseCommandExpireRealtime = nowRealtime;
				_simulatedBaseLastUpdateRealtime = nowRealtime;
				_hasSimulatedBasePose = true;
			}
		}

		private void AdvanceSimulatedBasePose(float nowRealtime)
		{
			if (!_hasSimulatedBasePose)
			{
				return;
			}

			if (_simulatedBaseLastUpdateRealtime <= 0f)
			{
				_simulatedBaseLastUpdateRealtime = nowRealtime;
				return;
			}

			float targetRealtime = Mathf.Max(nowRealtime, _simulatedBaseLastUpdateRealtime);
			float activeRealtime = Mathf.Min(targetRealtime, Mathf.Max(_simulatedBaseLastUpdateRealtime, _simulatedBaseCommandExpireRealtime));
			float deltaTime = activeRealtime - _simulatedBaseLastUpdateRealtime;
			if (deltaTime > 1e-5f)
			{
				IntegrateBasePose(
					ref _simulatedBasePosition,
					ref _simulatedBaseRotation,
					_simulatedBaseLinearVelocity,
					_simulatedBaseAngularVelocity,
					deltaTime);
			}

			_simulatedBaseLastUpdateRealtime = targetRealtime;
			if (targetRealtime >= _simulatedBaseCommandExpireRealtime - 1e-5f)
			{
				_simulatedBaseLinearVelocity = 0f;
				_simulatedBaseAngularVelocity = 0f;
			}
		}

		private bool TryGetSimulatedBasePose(out Vector3 worldPosition, out Quaternion worldRotation)
		{
			float nowRealtime = Time.realtimeSinceStartup;
			if (!_hasSimulatedBasePose
				|| !IsPredictionSessionActive(nowRealtime))
			{
				worldPosition = default;
				worldRotation = Quaternion.identity;
				return false;
			}

			worldPosition = _simulatedBasePosition;
			worldRotation = _simulatedBaseRotation;
			return true;
		}

		private bool IsPredictionSessionActive(float nowRealtime)
		{
			if (_manager != null
				&& _manager.trajectoryPlanner != null
				&& _manager.trajectoryPlanner.IsPlanning)
			{
				return true;
			}

			return nowRealtime <= _simulatedBaseCommandExpireRealtime + 1e-5f;
		}

		private bool TryGetLiveBaseReferenceLeadPose(out Vector3 worldPosition, out Quaternion worldRotation)
		{
			worldPosition = default;
			worldRotation = Quaternion.identity;

			if (_manager == null || _manager.diffDriveController == null || _manager.diffDriveController.rb == null)
			{
				return false;
			}

			DiffDriveTwinController controller = _manager.diffDriveController;
			float linearVelocity = controller.CurrentLinearVelocity;
			float angularVelocity = controller.CurrentAngularVelocity;
			if (Mathf.Abs(linearVelocity) <= 1e-4f && Mathf.Abs(angularVelocity) <= 1e-4f)
			{
				return false;
			}

			worldPosition = controller.rb.position;
			worldRotation = ExtractPlanarRotation(controller.rb.rotation);
			IntegrateBasePose(
				ref worldPosition,
				ref worldRotation,
				linearVelocity,
				angularVelocity,
				Mathf.Max(0.05f, fallbackLeadSeconds));
			return true;
		}

		private bool TryGetPredictedBaseReferencePose(out Vector3 worldPosition, out Quaternion worldRotation)
		{
			if (usePredictedBasePose
				&& _hasPredictedBasePose
				&& Time.realtimeSinceStartup <= _predictedBasePoseExpireRealtime)
			{
				worldPosition = _predictedBasePosition;
				worldRotation = _predictedBaseRotation;
				return true;
			}

			if (usePredictedBasePose && TryGetSimulatedBasePose(out worldPosition, out worldRotation))
			{
				return true;
			}

			if (usePredictedBasePose && TryGetLiveBaseReferenceLeadPose(out worldPosition, out worldRotation))
			{
				return true;
			}

			worldPosition = default;
			worldRotation = Quaternion.identity;
			return false;
		}

		private static void IntegrateBasePose(
			ref Vector3 worldPosition,
			ref Quaternion worldRotation,
			float linearVelocity,
			float angularVelocity,
			float deltaTime)
		{
			if (deltaTime <= 1e-5f)
			{
				return;
			}

			Quaternion planarRotation = ExtractPlanarRotation(worldRotation);
			Vector3 startPosition = worldPosition;
			Vector3 planarForward = planarRotation * Vector3.forward;
			planarForward.y = 0f;
			if (planarForward.sqrMagnitude <= 1e-6f)
			{
				planarForward = Vector3.forward;
			}

			planarForward.Normalize();
			if (Mathf.Abs(angularVelocity) <= 1e-4f)
			{
				worldPosition = startPosition + (planarForward * linearVelocity * deltaTime);
				worldPosition.y = startPosition.y;
				worldRotation = planarRotation;
				return;
			}

			Vector3 planarRight = new Vector3(planarForward.z, 0f, -planarForward.x);
			float deltaYawRad = angularVelocity * deltaTime;
			float radius = linearVelocity / angularVelocity;
			Vector3 planarDisplacement =
				(planarForward * (radius * Mathf.Sin(deltaYawRad)))
				+ (planarRight * (radius * (1f - Mathf.Cos(deltaYawRad))));

			worldPosition = startPosition + planarDisplacement;
			worldPosition.y = startPosition.y;
			worldRotation = planarRotation * Quaternion.Euler(0f, deltaYawRad * Mathf.Rad2Deg, 0f);
		}

		private static Quaternion ExtractPlanarRotation(Quaternion worldRotation)
		{
			return Quaternion.Euler(0f, worldRotation.eulerAngles.y, 0f);
		}

		private float GetVisualizationLeadSeconds(float requestedLeadSeconds)
		{
			float minLead = 0.08f;
			float maxLead = Mathf.Max(minLead, maxVisualizationLeadSeconds);
			return Mathf.Clamp(requestedLeadSeconds > 0f ? requestedLeadSeconds : fallbackLeadSeconds, minLead, maxLead);
		}

		private bool TryPredictBasePoseFromCommand(SafetyGateTimelineCommand command, float visualizationLeadSeconds, out Vector3 worldPosition, out Quaternion worldRotation)
		{
			worldPosition = default;
			worldRotation = Quaternion.identity;

			if (_baseReferenceTransform == null)
			{
				return false;
			}

			float linearVelocity = command.baseLinearVelocity;
			float angularVelocity = command.baseAngularVelocity;
			float durationSeconds = Mathf.Max(0.02f, visualizationLeadSeconds);
			if (Mathf.Abs(linearVelocity) <= 1e-4f && Mathf.Abs(angularVelocity) <= 1e-4f)
			{
				return false;
			}

			Vector3 startPosition = _baseReferenceTransform.position;
			Quaternion startRotation = ExtractPlanarRotation(_baseReferenceTransform.rotation);
			if (Mathf.Abs(angularVelocity) <= 1e-4f)
			{
				worldRotation = startRotation;
				worldPosition = startPosition + (startRotation * Vector3.forward * linearVelocity * durationSeconds);
				return true;
			}

			float yawDeltaDeg = angularVelocity * Mathf.Rad2Deg * durationSeconds;
			worldRotation = startRotation * Quaternion.Euler(0f, yawDeltaDeg, 0f);

			Vector3 planarForward = startRotation * Vector3.forward;
			planarForward.y = 0f;
			if (planarForward.sqrMagnitude <= 1e-6f)
			{
				planarForward = Vector3.forward;
			}

			planarForward.Normalize();
			Vector3 planarRight = new Vector3(planarForward.z, 0f, -planarForward.x);
			float radius = linearVelocity / angularVelocity;
			float sinDelta = Mathf.Sin(angularVelocity * durationSeconds);
			float cosDelta = Mathf.Cos(angularVelocity * durationSeconds);
			Vector3 planarDisplacement = (planarForward * (radius * sinDelta)) + (planarRight * (radius * (1f - cosDelta)));
			worldPosition = startPosition + planarDisplacement;
			worldPosition.y = startPosition.y;
			return true;
		}

		private bool TryGetPredictedBasePose(out Vector3 worldPosition, out Quaternion worldRotation)
		{
			if (!TryGetPredictedBaseReferencePose(out Vector3 referencePosition, out Quaternion referenceRotation))
			{
				worldPosition = default;
				worldRotation = Quaternion.identity;
				return false;
			}

			return TryGetBaseRootPoseFromReference(referencePosition, referenceRotation, out worldPosition, out worldRotation);
		}

		private bool TryGetDesiredBasePose(out Vector3 worldPosition, out Quaternion worldRotation)
		{
			if (previewMode == PreviewMode.ArmPreview
				&& _hasArmPreviewBaseReferencePose)
			{
				return TryGetBaseRootPoseFromReference(
					_armPreviewBaseReferencePosition,
					_armPreviewBaseReferenceRotation,
					out worldPosition,
					out worldRotation);
			}

			if (previewMode == PreviewMode.BasePreview
				&& _manager != null
				&& _manager.TryGetShadowBaseTwinPose(out worldPosition, out worldRotation))
			{
				return true;
			}

			if (previewMode == PreviewMode.BasePreview
				&& TryGetExecutionBasePose(out worldPosition, out worldRotation))
			{
				return true;
			}

			if (_cachedBaseRoot != null)
			{
				worldPosition = _cachedBaseRoot.position;
				worldRotation = _cachedBaseRoot.rotation;
				return true;
			}

			worldPosition = default;
			worldRotation = Quaternion.identity;
			return false;
		}

		private bool TryGetBaseRootPoseFromReference(Vector3 referencePosition, Quaternion referenceRotation, out Vector3 worldPosition, out Quaternion worldRotation)
		{
			if (_cachedBaseRoot == null || _baseReferenceTransform == null)
			{
				worldPosition = referencePosition;
				worldRotation = referenceRotation;
				return true;
			}

			worldPosition = referencePosition + (referenceRotation * _baseRootLocalPositionFromReference);
			worldRotation = referenceRotation * _baseRootLocalRotationFromReference;
			return true;
		}

		private void SetPredictedArmAngles(float[] armAnglesDeg, float holdSeconds)
		{
			if (armAnglesDeg == null || armAnglesDeg.Length < 6)
			{
				ClearPredictedArmPose();
				return;
			}

			for (int i = 0; i < 6; i++)
			{
				_predictedArmAnglesDeg[i] = armAnglesDeg[i];
			}

			_predictedArmPoseExpireRealtime = Time.realtimeSinceStartup + Mathf.Max(0.05f, holdSeconds);
			_hasPredictedArmAngles = true;
		}

		private void ClearPredictedArmPose()
		{
			_predictedArmPoseExpireRealtime = 0f;
			_hasPredictedArmAngles = false;
		}

		private void ClearPredictedBasePose()
		{
			_predictedBasePoseExpireRealtime = 0f;
			_hasPredictedBasePose = false;
		}

		private void SetExecutionBasePose(Vector3 worldPosition, Quaternion worldRotation)
		{
			_executionBasePosition = worldPosition;
			_executionBaseRotation = worldRotation;
			_hasExecutionBasePose = true;
		}

		private void ClearExecutionBasePose()
		{
			_hasExecutionBasePose = false;
			_executionBasePosition = Vector3.zero;
			_executionBaseRotation = Quaternion.identity;
		}

		private bool TryGetExecutionBasePose(out Vector3 worldPosition, out Quaternion worldRotation)
		{
			if (_hasExecutionBasePose)
			{
				worldPosition = _executionBasePosition;
				worldRotation = _executionBaseRotation;
				return true;
			}

			worldPosition = default;
			worldRotation = Quaternion.identity;
			return false;
		}

		private void TraceArmHierarchySnapshotIfNeeded()
		{
			if (!_armDiagnosticsEnabled || _armHierarchySnapshotLogged)
			{
				return;
			}

			lastArmHierarchySnapshot = BuildArmHierarchySnapshotReport();
			_armHierarchySnapshotLogged = true;
			if (!string.IsNullOrEmpty(lastArmHierarchySnapshot))
			{
				Debug.Log($"[ArmTrace][Hierarchy]\n{lastArmHierarchySnapshot}");
			}
		}

		private string BuildArmHierarchySnapshotReport()
		{
			StringBuilder builder = new StringBuilder();
			builder.AppendLine(
				$"context: baseRoot={BuildTransformPath(_cachedBaseRoot)}, armRoot={BuildTransformPath(_cachedArmRoot)}, baseRef={BuildTransformPath(_baseReferenceTransform)}, armMount={BuildTransformPath(_sourceArmMountTransform)}, shadowArmMount={BuildTransformPath(_shadowArmMountTransform)}, sourceSummary={sourceSummary}");
			builder.AppendLine(_manager != null && _manager.ShouldUseShadowArmTwinSource()
				? "contract: active arm source is ShadowArmTwinRuntime; the visualizer mirrors the articulated shadow twin transforms instead of projecting FK angles."
				: "contract: shadow arm clones Transform/Mesh/SkinnedMeshRenderer/bone mapping only; it does not add ArticulationBody, OneJointTrapezoidController, Collider, or Rigidbody.");

			if (!TryGetArmJointDiagnosticInputs(out Transform[] armJoints, out _, out ArticulationBody[] articulationJoints, out OneJointTrapezoidController[] coordinatedControllers))
			{
				builder.Append("arm-diagnostics unavailable");
				return builder.ToString();
			}

			for (int jointIndex = 0; jointIndex < 6; jointIndex++)
			{
				Transform sourceJoint = armJoints[jointIndex];
				_sourceToShadowTransformMap.TryGetValue(sourceJoint, out Transform shadowJoint);
				bool liveBoundController = coordinatedControllers != null
					&& coordinatedControllers.Length > jointIndex
					&& coordinatedControllers[jointIndex] != null;
				bool shadowBoundController = false;
				builder.Append("J").Append(jointIndex + 1).Append(": ");
				builder.Append("src=").Append(BuildTransformPath(sourceJoint));
				builder.Append(", shadow=").Append(BuildTransformPath(shadowJoint));
				builder.Append(", srcParent=").Append(BuildTransformPath(sourceJoint != null ? sourceJoint.parent : null));
				builder.Append(", shadowParent=").Append(BuildTransformPath(shadowJoint != null ? shadowJoint.parent : null));
				builder.Append(", srcLocalPos=").Append(sourceJoint != null ? FormatVector3(sourceJoint.localPosition) : "null");
				builder.Append(", shadowLocalPos=").Append(shadowJoint != null ? FormatVector3(shadowJoint.localPosition) : "null");
				builder.Append(", srcLocalRot=").Append(sourceJoint != null ? FormatQuaternion(sourceJoint.localRotation) : "null");
				builder.Append(", shadowLocalRot=").Append(shadowJoint != null ? FormatQuaternion(shadowJoint.localRotation) : "null");
				builder.Append(", srcScale=").Append(sourceJoint != null ? FormatVector3(sourceJoint.localScale) : "null");
				builder.Append(", shadowScale=").Append(shadowJoint != null ? FormatVector3(shadowJoint.localScale) : "null");
				builder.Append(", srcComponents=[ArticulationBody:").Append(FormatBool(articulationJoints != null && articulationJoints.Length > jointIndex && articulationJoints[jointIndex] != null));
				builder.Append(", BoundTrapCtrl:").Append(FormatBool(liveBoundController));
				builder.Append(", DirectTrapCtrl:").Append(FormatBool(sourceJoint != null && sourceJoint.GetComponent<OneJointTrapezoidController>() != null));
				builder.Append(", Collider:").Append(FormatBool(sourceJoint != null && sourceJoint.GetComponent<Collider>() != null));
				builder.Append(", Rigidbody:").Append(FormatBool(sourceJoint != null && sourceJoint.GetComponent<Rigidbody>() != null));
				builder.Append("], shadowComponents=[ArticulationBody:").Append(FormatBool(shadowJoint != null && shadowJoint.GetComponent<ArticulationBody>() != null));
				builder.Append(", BoundTrapCtrl:").Append(FormatBool(shadowBoundController));
				builder.Append(", DirectTrapCtrl:").Append(FormatBool(shadowJoint != null && shadowJoint.GetComponent<OneJointTrapezoidController>() != null));
				builder.Append(", Collider:").Append(FormatBool(shadowJoint != null && shadowJoint.GetComponent<Collider>() != null));
				builder.Append(", Rigidbody:").Append(FormatBool(shadowJoint != null && shadowJoint.GetComponent<Rigidbody>() != null));
				builder.AppendLine("]");
			}

			return builder.ToString();
		}

		private string BuildArmRuntimeStateSummary(float[] targetAnglesDeg)
		{
			StringBuilder builder = new StringBuilder();
			if (!TryGetArmJointDiagnosticInputs(out Transform[] armJoints, out float[] measuredAngles, out ArticulationBody[] articulationJoints, out OneJointTrapezoidController[] coordinatedControllers))
			{
				builder.Append("arm-runtime unavailable");
				return builder.ToString();
			}

			bool hasTargetAngles = targetAnglesDeg != null && targetAnglesDeg.Length >= 6;
			bool noOpBaseline = hasTargetAngles && AreAnglesEquivalent(measuredAngles, targetAnglesDeg, 0.001f, out _);
			float maxRotationDeltaDeg = 0f;
			float maxLiveTargetErrorDeg = 0f;
			float maxShadowTargetErrorDeg = 0f;
			for (int jointIndex = 0; jointIndex < 6; jointIndex++)
			{
				Transform sourceJoint = armJoints[jointIndex];
				_sourceToShadowTransformMap.TryGetValue(sourceJoint, out Transform shadowJoint);
				float measuredDeg = measuredAngles != null && measuredAngles.Length > jointIndex ? measuredAngles[jointIndex] : 0f;
				float targetDeg = hasTargetAngles ? targetAnglesDeg[jointIndex] : measuredDeg;
				float liveDriveTargetDeg = articulationJoints != null && articulationJoints.Length > jointIndex && articulationJoints[jointIndex] != null
					? articulationJoints[jointIndex].xDrive.target
					: float.NaN;
				float trapGoalDeg = coordinatedControllers != null && coordinatedControllers.Length > jointIndex && coordinatedControllers[jointIndex] != null
					? coordinatedControllers[jointIndex].goalDeg
					: float.NaN;
				float liveTargetErrorDeg = Mathf.Abs(measuredDeg - targetDeg);
				if (liveTargetErrorDeg > maxLiveTargetErrorDeg)
				{
					maxLiveTargetErrorDeg = liveTargetErrorDeg;
				}

				float rotationDeltaDeg = sourceJoint != null && shadowJoint != null
					? Quaternion.Angle(sourceJoint.localRotation, shadowJoint.localRotation)
					: float.PositiveInfinity;
				if (!float.IsInfinity(rotationDeltaDeg) && rotationDeltaDeg > maxRotationDeltaDeg)
				{
					maxRotationDeltaDeg = rotationDeltaDeg;
				}

				float shadowDerivedDeg = float.NaN;
				float shadowAxisDotX = float.NaN;
				float shadowTargetErrorDeg = float.NaN;
				if (sourceJoint != null && shadowJoint != null)
				{
					Quaternion zeroLocalRotation = sourceJoint.localRotation * Quaternion.AngleAxis(-measuredDeg, Vector3.right);
					if (TryComputeSignedAngleAboutLocalXAxis(zeroLocalRotation, shadowJoint.localRotation, out shadowDerivedDeg, out shadowAxisDotX))
					{
						shadowTargetErrorDeg = Mathf.Abs(shadowDerivedDeg - targetDeg);
						if (shadowTargetErrorDeg > maxShadowTargetErrorDeg)
						{
							maxShadowTargetErrorDeg = shadowTargetErrorDeg;
						}
					}
				}

				builder.Append("J").Append(jointIndex + 1).Append(": ");
				builder.Append("measured=").Append(measuredDeg.ToString("F3"));
				builder.Append(", target=").Append(targetDeg.ToString("F3"));
				builder.Append(", shadowDerived=").Append(FormatOptionalFloat(shadowDerivedDeg));
				builder.Append(", shadowAxisDotX=").Append(FormatOptionalFloat(shadowAxisDotX));
				builder.Append(", liveXDriveTarget=").Append(FormatOptionalFloat(liveDriveTargetDeg));
				builder.Append(", liveTrapGoal=").Append(FormatOptionalFloat(trapGoalDeg));
				builder.Append(", liveTargetError=").Append(liveTargetErrorDeg.ToString("F3"));
				builder.Append(", shadowTargetError=").Append(FormatOptionalFloat(shadowTargetErrorDeg));
				builder.Append(", liveVsShadowRotDelta=").Append(FormatOptionalFloat(rotationDeltaDeg));
				builder.Append(", srcLocalRot=").Append(sourceJoint != null ? FormatQuaternion(sourceJoint.localRotation) : "null");
				builder.Append(", shadowLocalRot=").Append(shadowJoint != null ? FormatQuaternion(shadowJoint.localRotation) : "null");
				if (jointIndex < 5)
				{
					builder.AppendLine();
				}
			}

			string classification;
			if (noOpBaseline)
			{
				classification = maxRotationDeltaDeg <= 0.05f ? "NoOpAligned" : "Hierarchy/BasisMismatch";
				lastArmNoOpAlignmentSnapshot = $"classification={classification}, maxRotationDeltaDeg={maxRotationDeltaDeg:F4}\n{builder}";
			}
			else if (maxShadowTargetErrorDeg <= 0.25f && maxLiveTargetErrorDeg >= 0.5f)
			{
				classification = "ActuationMismatchCandidate";
			}
			else
			{
				classification = "RuntimeTrace";
			}

			return $"classification={classification}, noOpBaseline={FormatBool(noOpBaseline)}, maxRotationDeltaDeg={maxRotationDeltaDeg:F4}, maxLiveTargetErrorDeg={maxLiveTargetErrorDeg:F4}, maxShadowTargetErrorDeg={maxShadowTargetErrorDeg:F4}\n{builder}";
		}

		private bool TryGetArmJointDiagnosticInputs(
			out Transform[] armJoints,
			out float[] measuredAngles,
			out ArticulationBody[] articulationJoints,
			out OneJointTrapezoidController[] coordinatedControllers)
		{
			armJoints = null;
			measuredAngles = null;
			articulationJoints = null;
			coordinatedControllers = null;
			if (_manager == null || _manager.arm6DOFFKController == null)
			{
				return false;
			}

			armJoints = _manager.arm6DOFFKController.jointTransforms;
			if (armJoints == null || armJoints.Length < 6)
			{
				return false;
			}

			measuredAngles = _manager.arm6DOFFKController.CaptureMeasuredJointAngles();
			articulationJoints = _manager.arm6DOFFKController.joints;
			coordinatedControllers = _manager.arm6DOFFKController.coordinatedJointControllers;
			return true;
		}

		private static bool AreAnglesEquivalent(float[] expected, float[] actual, float toleranceDeg, out float maxDeltaDeg)
		{
			maxDeltaDeg = 0f;
			if (ReferenceEquals(expected, actual))
			{
				return true;
			}

			if (expected == null || actual == null || expected.Length < 6 || actual.Length < 6)
			{
				maxDeltaDeg = float.PositiveInfinity;
				return false;
			}

			for (int i = 0; i < 6; i++)
			{
				float delta = Mathf.Abs(expected[i] - actual[i]);
				if (delta > maxDeltaDeg)
				{
					maxDeltaDeg = delta;
				}

				if (delta > toleranceDeg)
				{
					return false;
				}
			}

			return true;
		}

		private static bool AreAnglesEquivalent(float[] expected, float[] actual, float toleranceDeg = 0.001f)
		{
			return AreAnglesEquivalent(expected, actual, toleranceDeg, out _);
		}

		private static string FormatAngles(float[] anglesDeg)
		{
			if (anglesDeg == null || anglesDeg.Length < 6)
			{
				return "j[n/a]";
			}

			StringBuilder builder = new StringBuilder("j[");
			for (int i = 0; i < 6; i++)
			{
				if (i > 0)
				{
					builder.Append(',');
				}

				builder.Append(anglesDeg[i].ToString("F3"));
			}

			builder.Append(']');
			return builder.ToString();
		}

		private static string BuildTransformPath(Transform transform)
		{
			if (transform == null)
			{
				return "null";
			}

			StringBuilder builder = new StringBuilder(transform.name);
			Transform current = transform.parent;
			while (current != null)
			{
				builder.Insert(0, '/');
				builder.Insert(0, current.name);
				current = current.parent;
			}

			return builder.ToString();
		}

		private static string FormatVector3(Vector3 value)
		{
			return $"({value.x:F4},{value.y:F4},{value.z:F4})";
		}

		private static string FormatQuaternion(Quaternion value)
		{
			return $"({value.x:F4},{value.y:F4},{value.z:F4},{value.w:F4})";
		}

		private static string FormatBool(bool value)
		{
			return value ? "Y" : "N";
		}

		private static bool TryComputeSignedAngleAboutLocalXAxis(
			Quaternion zeroLocalRotation,
			Quaternion currentLocalRotation,
			out float signedAngleDeg,
			out float axisAlignment)
		{
			Quaternion relative = Quaternion.Inverse(zeroLocalRotation) * currentLocalRotation;
			relative.ToAngleAxis(out float rawAngleDeg, out Vector3 rawAxis);
			if (rawAxis.sqrMagnitude <= 1e-8f)
			{
				signedAngleDeg = 0f;
				axisAlignment = 1f;
				return true;
			}

			Vector3 axis = rawAxis.normalized;
			float angleDeg = rawAngleDeg > 180f ? rawAngleDeg - 360f : rawAngleDeg;
			axisAlignment = Vector3.Dot(axis, Vector3.right);
			if (axisAlignment < 0f)
			{
				angleDeg = -angleDeg;
				axisAlignment = -axisAlignment;
			}

			signedAngleDeg = angleDeg;
			return true;
		}

		private static string FormatOptionalFloat(float value)
		{
			return float.IsNaN(value) || float.IsInfinity(value) ? "n/a" : value.ToString("F4");
		}

		private void EmitPendingArmRuntimeTraceIfNeeded(float[] measuredAngles)
		{
			if (!_armDiagnosticsEnabled || !_hasPendingArmTrace)
			{
				return;
			}

			lastArmRuntimeSnapshot = BuildArmRuntimeStateSummary(_pendingArmTrace.targetAnglesDeg);
			Debug.Log(
				$"[ArmTrace][ShadowApplied] seq={_pendingArmTrace.sequenceId}, sample={_pendingArmTrace.sampleIndex}, lead={_pendingArmTrace.leadSeconds:F3}s, throttle={_pendingArmTrace.throttleRatio:F2}, target={FormatAngles(_pendingArmTrace.targetAnglesDeg)}\n{lastArmRuntimeSnapshot}");
			if (AreAnglesEquivalent(measuredAngles, _pendingArmTrace.targetAnglesDeg, 0.001f))
			{
				Debug.Log($"[ArmTrace][NoOpBaseline]\n{lastArmNoOpAlignmentSnapshot}");
			}

			_hasPendingArmTrace = false;
		}

		private static Quaternion ComposeAbsoluteArmJointLocalRotation(Quaternion measuredLocalRotation, float measuredDeg, float targetDeg)
		{
			Quaternion zeroLocalRotation = measuredLocalRotation * Quaternion.AngleAxis(-measuredDeg, Vector3.right);
			return zeroLocalRotation * Quaternion.AngleAxis(targetDeg, Vector3.right);
		}

		private void ApplyPredictedArmPoseToShadow()
		{
			if (previewMode != PreviewMode.ArmPreview
				|| !_hasPredictedArmAngles
				|| _manager == null
				|| _manager.arm6DOFFKController == null
				|| _manager.arm6DOFFKController.jointTransforms == null)
			{
				return;
			}

			if (_manager.ShouldUseShadowArmTwinSource())
			{
				return;
			}

			if (Time.realtimeSinceStartup > _predictedArmPoseExpireRealtime)
			{
				ClearPredictedArmPose();
				return;
			}

			Transform[] armJoints = _manager.arm6DOFFKController.jointTransforms;
			float[] measuredAngles = _manager.arm6DOFFKController.CaptureMeasuredJointAngles();
			for (int jointIndex = 0; jointIndex < 6; jointIndex++)
			{
				if (armJoints.Length <= jointIndex || armJoints[jointIndex] == null)
				{
					continue;
				}

				if (!_sourceToShadowTransformMap.TryGetValue(armJoints[jointIndex], out Transform shadowJoint) || shadowJoint == null)
				{
					continue;
				}

				float measuredDeg = measuredAngles != null && measuredAngles.Length > jointIndex ? measuredAngles[jointIndex] : _predictedArmAnglesDeg[jointIndex];
				float predictedDeg = _predictedArmAnglesDeg[jointIndex];
				shadowJoint.localRotation = ComposeAbsoluteArmJointLocalRotation(
					armJoints[jointIndex].localRotation,
					measuredDeg,
					predictedDeg);
			}

			EmitPendingArmRuntimeTraceIfNeeded(measuredAngles);
		}

		private void ClearShadowHierarchy()
		{
			ClearShadowNodes();

			if (_shadowContainer == null)
			{
				return;
			}

			DestroyGameObject(_shadowContainer.gameObject);
			_shadowContainer = null;
		}

		private void ClearShadowNodes()
		{
			_bindings.Clear();
			_sourceToShadowTransformMap.Clear();
			_pendingSkinnedBindings.Clear();
			ClearExecutionBasePose();
			_hasPendingArmTrace = false;
			bindingCount = 0;
			_isBuilt = false;

			if (_shadowContainer == null)
			{
				return;
			}

			List<GameObject> children = new List<GameObject>();
			for (int childIndex = 0; childIndex < _shadowContainer.childCount; childIndex++)
			{
				Transform child = _shadowContainer.GetChild(childIndex);
				if (child != null)
				{
					children.Add(child.gameObject);
				}
			}

			for (int i = 0; i < children.Count; i++)
			{
				DestroyGameObject(children[i]);
			}
		}

		private static void DestroyGameObject(GameObject target)
		{
			if (target == null)
			{
				return;
			}

			if (Application.isPlaying)
			{
				Destroy(target);
			}
			else
			{
				DestroyImmediate(target);
			}
		}

		private static void ApplyShadowObjectHideFlags(GameObject target)
		{
			if (target == null)
			{
				return;
			}

			target.hideFlags = HideFlags.DontSaveInEditor | HideFlags.DontSaveInBuild;
		}

		private void EnsureShadowMaterial()
		{
			if (_sharedShadowMaterial != null)
			{
				UpdateShadowMaterialTint();
				return;
			}

			Shader shader = FindShadowShader();
			_sharedShadowMaterial = new Material(shader)
			{
				name = "ShadowRobotMaterial"
			};
			ConfigureTransparentMaterial(_sharedShadowMaterial);
			UpdateShadowMaterialTint();
		}

		private void DestroyShadowMaterial()
		{
			if (_sharedShadowMaterial == null)
			{
				return;
			}

			if (Application.isPlaying)
			{
				Destroy(_sharedShadowMaterial);
			}
			else
			{
				DestroyImmediate(_sharedShadowMaterial);
			}

			_sharedShadowMaterial = null;
		}

		private void UpdateShadowMaterialTint()
		{
			if (_sharedShadowMaterial == null)
			{
				return;
			}

			Color tint = shadowTint;
			tint.a = Mathf.Clamp01(shadowAlpha);
			if (_sharedShadowMaterial.HasProperty("_BaseColor"))
			{
				_sharedShadowMaterial.SetColor("_BaseColor", tint);
			}

			if (_sharedShadowMaterial.HasProperty("_Color"))
			{
				_sharedShadowMaterial.SetColor("_Color", tint);
			}
		}

		private static Shader FindShadowShader()
		{
			string[] shaderNames =
			{
				"Universal Render Pipeline/Lit",
				"Universal Render Pipeline/Simple Lit",
				"Standard",
				"Legacy Shaders/Transparent/Diffuse",
				"Sprites/Default"
			};

			for (int i = 0; i < shaderNames.Length; i++)
			{
				Shader shader = Shader.Find(shaderNames[i]);
				if (shader != null)
				{
					return shader;
				}
			}

			return Shader.Find("Standard");
		}

		private static void ConfigureTransparentMaterial(Material material)
		{
			if (material == null)
			{
				return;
			}

			if (material.HasProperty("_Surface"))
			{
				material.SetFloat("_Surface", 1f);
			}

			if (material.HasProperty("_Blend"))
			{
				material.SetFloat("_Blend", 0f);
			}

			if (material.HasProperty("_Mode"))
			{
				material.SetFloat("_Mode", 3f);
			}

			if (material.HasProperty("_SrcBlend"))
			{
				material.SetInt("_SrcBlend", (int)BlendMode.SrcAlpha);
			}

			if (material.HasProperty("_DstBlend"))
			{
				material.SetInt("_DstBlend", (int)BlendMode.OneMinusSrcAlpha);
			}

			if (material.HasProperty("_ZWrite"))
			{
				material.SetInt("_ZWrite", 0);
			}

			material.DisableKeyword("_ALPHATEST_ON");
			material.EnableKeyword("_ALPHABLEND_ON");
			material.DisableKeyword("_ALPHAPREMULTIPLY_ON");
			material.EnableKeyword("_SURFACE_TYPE_TRANSPARENT");
			material.renderQueue = (int)RenderQueue.Transparent;
		}

		private static bool HasRenderableDescendant(Transform root)
		{
			if (root == null)
			{
				return false;
			}

			if (root.GetComponentInChildren<MeshRenderer>(true) != null)
			{
				return true;
			}

			return root.GetComponentInChildren<SkinnedMeshRenderer>(true) != null;
		}
	}
}
