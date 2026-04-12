using System;
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
		[SerializeField] [Range(0.05f, 1f)] private float shadowAlpha = 0.35f;
		[SerializeField] private Color shadowTint = new Color(0.18f, 0.82f, 1f, 0.35f);
		[SerializeField] private bool usePredictedBasePose = true;
		[SerializeField] [Range(0.05f, 1f)] private float fallbackLeadSeconds = 0.20f;
		[SerializeField] [Range(0.05f, 0.50f)] private float maxVisualizationLeadSeconds = 0.25f;

		[Header("Debug")]
		[SerializeField] private int bindingCount;
		[SerializeField] private string sourceSummary = "Shadow source is not configured.";
		[SerializeField] private PreviewMode previewMode = PreviewMode.Mirror;

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
				SetPredictedArmAngles(command.armCommand.toAnglesDeg, GetVisualizationLeadSeconds(command.predictedLeadSeconds));
			}
		}

		public void ApplyManualBasePreviewStep(
			SafetyGateTimelineCommand command,
			Vector3 predictedBaseWorldPosition,
			Quaternion predictedBaseWorldRotation)
		{
			if (_activeInstance != null && _activeInstance != this)
			{
				_activeInstance.ApplyManualBasePreviewStep(command, predictedBaseWorldPosition, predictedBaseWorldRotation);
				return;
			}

			if (previewMode != PreviewMode.BasePreview)
			{
				return;
			}

			float holdSeconds = GetVisualizationLeadSeconds(command.predictedLeadSeconds);
			SetSimulatedBaseCommand(command.baseLinearVelocity, command.baseAngularVelocity, holdSeconds);
			SetSimulatedBasePose(predictedBaseWorldPosition, predictedBaseWorldRotation);
			PushPredictedBasePose(predictedBaseWorldPosition, predictedBaseWorldRotation, holdSeconds);
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

			if (_manager.armBinder != null && _manager.armBinder.armRoot != null)
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

			if (_cachedBaseRoot != null)
			{
				BuildShadowNodeRecursive(_cachedBaseRoot, _shadowContainer, true, null);
			}

			if (_cachedArmRoot != null)
			{
				_shadowArmMountTransform = TryMapShadowTransform(_sourceArmMountTransform);
				if (_shadowArmMountTransform == null)
				{
					_shadowArmMountTransform = TryMapShadowTransform(_cachedBaseRoot);
				}

				Transform armShadowParent = _shadowArmMountTransform != null ? _shadowArmMountTransform : _shadowContainer;
				Transform armRootReference = _sourceArmMountTransform != null ? _sourceArmMountTransform : null;
				BuildShadowNodeRecursive(_cachedArmRoot, armShadowParent, true, armRootReference);
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

		private void BuildShadowNodeRecursive(Transform source, Transform parent, bool isRoot, Transform rootReference)
		{
			if (source == null || parent == null)
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
				BuildShadowNodeRecursive(source.GetChild(i), shadowTransform, false, null);
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
				&& TryGetPredictedBasePose(out worldPosition, out worldRotation))
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
				float deltaDeg = Mathf.DeltaAngle(measuredDeg, predictedDeg);
				shadowJoint.localRotation = armJoints[jointIndex].localRotation * Quaternion.AngleAxis(deltaDeg, Vector3.right);
			}
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
