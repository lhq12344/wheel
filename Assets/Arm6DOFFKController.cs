using System.Collections.Generic;
using System.IO;
using UnityEngine;

namespace RobotSimulation
{
	/// <summary>
	/// 6-DOF articulated arm controller with URDF-driven PoE kinematics.
	/// </summary>
	public class Arm6DOFFKController : MonoBehaviour
	{
		public static readonly float[] DefaultConfiguredHomeJointAnglesDeg = { 0f, 0f, 150f, -30f, 90f, 0f };

		public static Arm6DOFFKController Instance { get; private set; }

		[Header("Joint Configuration")]
		public ArticulationBody[] joints = new ArticulationBody[6];
		public Transform[] jointTransforms = new Transform[6];
		public OneJointTrapezoidController[] coordinatedJointControllers = new OneJointTrapezoidController[6];
		public bool autoBindByJointName = true;
		public bool autoRebindOnMismatch = true;
		public bool preferLinkNameBinding = true;
		public Transform armHierarchyRoot;
		public bool searchWholeSceneIfLocalSearchFails = true;
		public string[] preferredLinkNames = new string[] { "Link_01", "Link_02", "Link_03", "Link_04", "Link_05", "Link_06" };
		public string[] expectedJointNames = new string[] { "Joint01", "Joint02", "Joint03", "Joint04", "Joint05", "Joint06" };

		[Header("URDF Kinematics")]
		public TextAsset urdfSource;
		public string urdfAssetPath;

		[Header("Joint Limits (degrees)")]
		public Vector2[] jointLimits = new Vector2[]
		{
			new Vector2(-360f, 360f),
			new Vector2(-85f, 265f),
			new Vector2(-175f, 175f),
			new Vector2(-85f, 265f),
			new Vector2(-360f, 360f),
			new Vector2(-360f, 360f)
		};

		[Header("End Effector")]
		public Transform endEffector;

		[Header("Home Pose")]
		public bool applyConfiguredHomeOnInitialize = true;
		public float[] configuredHomeJointAnglesDeg = CreateDefaultConfiguredHomeJointAnglesDeg();

		[Header("Status")]
		[SerializeField] private float[] _currentJointAngles = new float[6];
		[SerializeField] private Vector3 _measuredEndEffectorPosition;
		[SerializeField] private Quaternion _measuredEndEffectorRotation = Quaternion.identity;
		[SerializeField] private Vector3 _measuredEndEffectorWorldPosition;
		[SerializeField] private Quaternion _measuredEndEffectorWorldRotation = Quaternion.identity;
		[SerializeField] private Vector3 _modelEndEffectorPosition;
		[SerializeField] private Quaternion _modelEndEffectorRotation = Quaternion.identity;
		[SerializeField] private float _modelVsMeasuredPositionError;
		[SerializeField] private bool _kinematicsReady;
		[SerializeField] private bool _isInitialized;

		private Arm6DOFKinematicsModel _kinematicsModel = new Arm6DOFKinematicsModel();
		private ArmCollisionMonitor _collisionMonitor;
		private ArmCollisionGuardResult _lastCollisionGuardResult = new ArmCollisionGuardResult();

		public float[] CurrentJointAngles => _currentJointAngles;
		public Vector3 EndEffectorPosition => _measuredEndEffectorPosition;
		public Vector3 EndEffectorWorldPosition => _measuredEndEffectorWorldPosition;
		public Quaternion EndEffectorRotation => _measuredEndEffectorRotation;
		public Pose ModelEndEffectorPose => new Pose(_modelEndEffectorPosition, _modelEndEffectorRotation);
		public Pose MeasuredEndEffectorPose => new Pose(_measuredEndEffectorPosition, _measuredEndEffectorRotation);
		public Pose MeasuredEndEffectorWorldPose => new Pose(_measuredEndEffectorWorldPosition, _measuredEndEffectorWorldRotation);
		public float ModelVsMeasuredPositionError => _modelVsMeasuredPositionError;
		public bool KinematicsReady => _kinematicsReady;
		public bool IsInitialized => _isInitialized;
		public Arm6DOFKinematicsModel KinematicsModel => _kinematicsModel;
		public Transform BaseFrameTransform => GetBaseFrameTransform();
		public ArmCollisionGuardResult LastCollisionGuardResult => _lastCollisionGuardResult;

		public static float[] CreateDefaultConfiguredHomeJointAnglesDeg()
		{
			return (float[])DefaultConfiguredHomeJointAnglesDeg.Clone();
		}

		private void Awake()
		{
			if (Instance == null)
			{
				Instance = this;
			}
		}

		private void Start()
		{
			InitializeJoints();
		}

		private void FixedUpdate()
		{
			if (!_isInitialized)
			{
				return;
			}

			UpdateJointStates();
			ComputeForwardKinematics();
		}

		public void InitializeJoints()
		{
			if (!HasValidAssignedJointArray())
			{
				Debug.LogWarning("[Arm6DOF] Auto-detecting joints...");
				AutoDetectJoints();
			}

			EnsureJointTransformCache();

			if (autoRebindOnMismatch && !IsMappingConsistentWithExpectedNames())
			{
				Debug.LogWarning("[Arm6DOF] Assigned joint mapping is inconsistent. Rebinding joints automatically.");
				AutoDetectJoints();
				EnsureJointTransformCache();
			}

			for (int i = 0; i < 6; i++)
			{
				if (joints[i] == null)
				{
					Debug.LogError($"[Arm6DOF] Joint {i} is not assigned!");
					_isInitialized = false;
					return;
				}
			}

			ValidateJointMappingAgainstExpectedNames();
			EnsureEndEffectorReference();
			InitializeKinematicsModel();
			if (Application.isPlaying && applyConfiguredHomeOnInitialize)
			{
				ApplyAllJointTargetsRaw(configuredHomeJointAnglesDeg);
			}

			_isInitialized = true;
			UpdateJointStates();
			ComputeForwardKinematics();
			Debug.Log("[Arm6DOF] Initialized successfully");
		}

		public bool InitializeKinematicsModel()
		{
			SanitizeUrdfReferences();
			TryAutoAssignUrdfSource();
			_kinematicsReady = false;
			string urdfText = ResolveUrdfText();

			if (string.IsNullOrWhiteSpace(urdfText))
			{
				Debug.LogError("[Arm6DOF] URDF source is not assigned or could not be read. Kinematics model is unavailable.");
				return false;
			}

			_kinematicsModel = new Arm6DOFKinematicsModel();
			Transform baseTransform = GetBaseFrameTransform();
			_kinematicsReady = _kinematicsModel.Initialize(urdfText, baseTransform, jointTransforms, endEffector, configuredHomeJointAnglesDeg);
			if (!_kinematicsReady)
			{
				Debug.LogError($"[Arm6DOF] Failed to initialize kinematics model: {_kinematicsModel.ErrorMessage}");
				return false;
			}

			return true;
		}

		public void ComputeForwardKinematics()
		{
			UpdateMeasuredEndEffectorPose();

			if (!_kinematicsReady)
			{
				_modelEndEffectorPosition = _measuredEndEffectorPosition;
				_modelEndEffectorRotation = _measuredEndEffectorRotation;
				_modelVsMeasuredPositionError = 0f;
				return;
			}

			Pose modelPose = _kinematicsModel.ForwardPoe(_currentJointAngles);
			_modelEndEffectorPosition = modelPose.position;
			_modelEndEffectorRotation = modelPose.rotation;
			_modelVsMeasuredPositionError = Vector3.Distance(_modelEndEffectorPosition, _measuredEndEffectorPosition);
		}

		public Pose ForwardPoe(float[] jointAnglesDeg)
		{
			return _kinematicsModel.ForwardPoe(jointAnglesDeg);
		}

		public Pose ForwardUrdfPoe(float[] jointAnglesDeg)
		{
			return _kinematicsModel.ForwardUrdfPoe(jointAnglesDeg);
		}

		public Pose ForwardDh(float[] jointAnglesDeg)
		{
			return _kinematicsModel.ForwardDh(jointAnglesDeg);
		}

		public Pose ForwardUrdfChain(float[] jointAnglesDeg)
		{
			return _kinematicsModel.ForwardUrdfChain(jointAnglesDeg);
		}

		public Pose[] ComputeLinkPosesBase(float[] jointAnglesDeg)
		{
			return _kinematicsModel.ComputeLinkPosesBase(jointAnglesDeg);
		}

		public Pose[] ComputeLinkPosesWorld(float[] jointAnglesDeg)
		{
			Pose[] basePoses = ComputeLinkPosesBase(jointAnglesDeg);
			Transform baseTransform = GetBaseFrameTransform();
			if (baseTransform == null)
			{
				return basePoses;
			}

			Pose[] worldPoses = new Pose[basePoses.Length];
			for (int i = 0; i < basePoses.Length; i++)
			{
				worldPoses[i] = new Pose(
					baseTransform.TransformPoint(basePoses[i].position),
					baseTransform.rotation * basePoses[i].rotation);
			}

			return worldPoses;
		}

		public float[,] ComputeGeometricJacobian(float[] jointAnglesDeg)
		{
			return _kinematicsModel.ComputeSpaceJacobian(jointAnglesDeg);
		}

		public Vector3 WorldToBasePosition(Vector3 worldPosition)
		{
			return _kinematicsModel.WorldToBasePosition(GetBaseFrameTransform(), worldPosition);
		}

		public bool IsPositionReachable(Vector3 worldPosition, float tolerance = 0.01f)
		{
			return TryGetReachability(worldPosition, tolerance, out _);
		}

		public float GetMaxReach()
		{
			return _kinematicsReady ? _kinematicsModel.MaxReachDistance : 0f;
		}

		public float GetMinReach()
		{
			return _kinematicsReady ? _kinematicsModel.MinReachDistance : 0f;
		}

		public bool IsBasePositionReachable(Vector3 basePosition, float tolerance = 0.01f)
		{
			if (!_kinematicsReady)
			{
				return false;
			}

			return _kinematicsModel.IsWithinWorkspaceShell(basePosition, tolerance, out _);
		}

		public bool TryGetReachability(Vector3 worldPosition, float tolerance, out string reason)
		{
			reason = "Kinematics model is not initialized.";
			if (!_kinematicsReady)
			{
				return false;
			}

			Vector3 basePosition = WorldToBasePosition(worldPosition);
			return _kinematicsModel.IsWithinWorkspaceShell(basePosition, tolerance, out reason);
		}

		public float[] GetJointVelocities()
		{
			float[] velocities = new float[6];
			for (int i = 0; i < 6; i++)
			{
				if (joints[i] != null && joints[i].jointVelocity.dofCount > 0)
				{
					velocities[i] = joints[i].jointVelocity[0] * Mathf.Rad2Deg;
				}
			}

			return velocities;
		}

		public bool IsAtTarget(float toleranceDeg = 1f)
		{
			for (int i = 0; i < 6; i++)
			{
				if (joints[i] == null)
				{
					continue;
				}

				float error = Mathf.Abs((joints[i].jointPosition[0] * Mathf.Rad2Deg) - joints[i].xDrive.target);
				if (error > toleranceDeg)
				{
					return false;
				}
			}

			return true;
		}

		public bool AreJointAnglesNear(float[] targetAnglesDeg, float toleranceDeg = 1f)
		{
			if (targetAnglesDeg == null || targetAnglesDeg.Length < 6)
			{
				return false;
			}

			for (int i = 0; i < 6; i++)
			{
				if (joints[i] == null || joints[i].jointPosition.dofCount <= 0)
				{
					continue;
				}

				float measuredDeg = joints[i].jointPosition[0] * Mathf.Rad2Deg;
				if (Mathf.Abs(Mathf.DeltaAngle(measuredDeg, targetAnglesDeg[i])) > toleranceDeg)
				{
					return false;
				}
			}

			return true;
		}

		public float GetMaxJointAngleError(float[] targetAnglesDeg)
		{
			if (targetAnglesDeg == null || targetAnglesDeg.Length < 6)
			{
				return float.PositiveInfinity;
			}

			float maxError = 0f;
			for (int i = 0; i < 6; i++)
			{
				if (joints[i] == null || joints[i].jointPosition.dofCount <= 0)
				{
					continue;
				}

				float measuredDeg = joints[i].jointPosition[0] * Mathf.Rad2Deg;
				maxError = Mathf.Max(maxError, Mathf.Abs(Mathf.DeltaAngle(measuredDeg, targetAnglesDeg[i])));
			}

			return maxError;
		}

		public bool EvaluateMotionCollision(float[] startAnglesDeg, float[] targetAnglesDeg, out ArmCollisionGuardResult result)
		{
			result = new ArmCollisionGuardResult
			{
				allowed = true
			};

			RefreshCollisionMonitor();
			if (!_kinematicsReady || _collisionMonitor == null)
			{
				_lastCollisionGuardResult = result;
				return false;
			}

			float[] safeStart = startAnglesDeg != null ? (float[])startAnglesDeg.Clone() : CaptureMeasuredJointAngles();
			float[] safeTarget = targetAnglesDeg != null ? (float[])targetAnglesDeg.Clone() : safeStart;
			float maxDelta = 0f;
			for (int i = 0; i < 6; i++)
			{
				maxDelta = Mathf.Max(maxDelta, Mathf.Abs(safeTarget[i] - safeStart[i]));
			}

			int sampleCount = Mathf.Clamp(Mathf.CeilToInt(maxDelta / Mathf.Max(0.5f, _collisionMonitor.previewStepDegrees)), 1, Mathf.Max(1, _collisionMonitor.maxPreviewSamples));
			float[] sampleAngles = new float[6];
			for (int sampleIndex = 1; sampleIndex <= sampleCount; sampleIndex++)
			{
				float t = sampleIndex / (float)sampleCount;
				for (int i = 0; i < 6; i++)
				{
					sampleAngles[i] = Mathf.Lerp(safeStart[i], safeTarget[i], t);
				}

				if (_collisionMonitor.EvaluatePredictedCollision(ComputeLinkPosesWorld(sampleAngles), out result))
				{
					result.sampleIndex = sampleIndex;
					_lastCollisionGuardResult = result;
					return true;
				}
			}

			_lastCollisionGuardResult = result;
			return false;
		}

		public bool TrySetJointTarget(int jointIndex, float angleDeg)
		{
			if (jointIndex < 0 || jointIndex >= 6)
			{
				Debug.LogError($"[Arm6DOF] Invalid joint index: {jointIndex}");
				return false;
			}

			float[] targetAngles = CaptureMeasuredJointAngles();
			targetAngles[jointIndex] = Mathf.Clamp(angleDeg, jointLimits[jointIndex].x, jointLimits[jointIndex].y);
			if (EvaluateMotionCollision(CaptureMeasuredJointAngles(), targetAngles, out ArmCollisionGuardResult guardResult))
			{
				Debug.LogWarning($"[Arm6DOF] {guardResult.message}");
				return false;
			}

			ApplyJointTargetRaw(jointIndex, targetAngles[jointIndex]);
			return true;
		}

		public bool TrySetAllJointTargets(float[] angles)
		{
			if (angles == null || angles.Length < 6)
			{
				Debug.LogError("[Arm6DOF] Invalid angles array");
				return false;
			}

			float[] targetAngles = (float[])angles.Clone();
			for (int i = 0; i < 6; i++)
			{
				targetAngles[i] = Mathf.Clamp(targetAngles[i], jointLimits[i].x, jointLimits[i].y);
			}

			float[] startAngles = CaptureMeasuredJointAngles();
			if (EvaluateMotionCollision(startAngles, targetAngles, out ArmCollisionGuardResult guardResult))
			{
				Debug.LogWarning($"[Arm6DOF] {guardResult.message}");
				return false;
			}

			ApplyAllJointTargetsRaw(targetAngles);
			return true;
		}

		public bool TryGoHome()
		{
			return TrySetAllJointTargets(configuredHomeJointAnglesDeg);
		}

		public void SetJointTarget(int jointIndex, float angleDeg)
		{
			TrySetJointTarget(jointIndex, angleDeg);
		}

		public void SetAllJointTargets(float[] angles)
		{
			TrySetAllJointTargets(angles);
		}

		internal void ApplyJointTargetRaw(int jointIndex, float angleDeg)
		{
			if (jointIndex < 0 || jointIndex >= 6)
			{
				return;
			}

			angleDeg = Mathf.Clamp(angleDeg, jointLimits[jointIndex].x, jointLimits[jointIndex].y);
			ArticulationBody joint = joints[jointIndex];
			if (joint == null)
			{
				return;
			}

			ArticulationDrive drive = joint.xDrive;
			drive.target = angleDeg;
			joint.xDrive = drive;
			joint.WakeUp();

			if (coordinatedJointControllers != null && jointIndex < coordinatedJointControllers.Length && coordinatedJointControllers[jointIndex] != null)
			{
				coordinatedJointControllers[jointIndex].goalDeg = angleDeg;
			}
		}

		internal void ApplyAllJointTargetsRaw(float[] angles)
		{
			if (angles == null || angles.Length < 6)
			{
				return;
			}

			for (int i = 0; i < 6; i++)
			{
				ApplyJointTargetRaw(i, angles[i]);
			}
		}

		public Vector2 GetJointLimits(int jointIndex)
		{
			return jointIndex >= 0 && jointIndex < 6 ? jointLimits[jointIndex] : new Vector2(-180f, 180f);
		}

		public void EmergencyStop()
		{
			for (int i = 0; i < 6; i++)
			{
				if (joints[i] == null)
				{
					continue;
				}

				ArticulationDrive drive = joints[i].xDrive;
				drive.targetVelocity = 0f;
				joints[i].xDrive = drive;
			}
		}

		public float[] CaptureMeasuredJointAngles()
		{
			float[] measured = new float[6];
			for (int i = 0; i < 6; i++)
			{
				if (joints[i] == null || joints[i].jointPosition.dofCount <= 0)
				{
					continue;
				}

				measured[i] = joints[i].jointPosition[0] * Mathf.Rad2Deg;
			}

			return measured;
		}

		public void HoldCurrentPose()
		{
			float[] measured = CaptureMeasuredJointAngles();
			ApplyAllJointTargetsRaw(measured);

			for (int i = 0; i < 6; i++)
			{
				if (joints[i] == null)
				{
					continue;
				}

				ArticulationDrive drive = joints[i].xDrive;
				drive.target = measured[i];
				drive.targetVelocity = 0f;
				joints[i].xDrive = drive;
				joints[i].WakeUp();
			}
		}

		public void RefreshRuntimeState()
		{
			if (!_isInitialized)
			{
				return;
			}

			UpdateJointStates();
			ComputeForwardKinematics();
		}

		public void GoHome()
		{
			TryGoHome();
		}

		public string GetStatusReport()
		{
			string report = "=== Arm Status ===\n";
			report += $"Initialized: {_isInitialized}\n";
			report += $"Kinematics Ready: {_kinematicsReady}\n";
			report += $"Measured EE: ({_measuredEndEffectorPosition.x:F3}, {_measuredEndEffectorPosition.y:F3}, {_measuredEndEffectorPosition.z:F3})\n";
			report += $"Model EE: ({_modelEndEffectorPosition.x:F3}, {_modelEndEffectorPosition.y:F3}, {_modelEndEffectorPosition.z:F3})\n";
			report += $"Model Error: {_modelVsMeasuredPositionError:F4}m\n";
			for (int i = 0; i < 6; i++)
			{
				report += $"  J{i + 1}: {_currentJointAngles[i]:F2}\n";
			}

			return report;
		}

		public string GetKinematicsParameterReport()
		{
			if (!_kinematicsReady || _kinematicsModel == null)
			{
				return "Kinematics model is not ready.";
			}

			return _kinematicsModel.BuildParameterReport();
		}

		private void AutoDetectJoints()
		{
			List<ArticulationBody> dofJoints = CollectDofJointsForBinding();
			joints = new ArticulationBody[6];
			jointTransforms = new Transform[6];

			if (dofJoints.Count == 0)
			{
				Debug.LogError("[Arm6DOF] Auto-detect found 0 DOF joints. Set 'armHierarchyRoot' to the arm root object or assign joints[] manually.");
				return;
			}

			bool nameBindingSuccess = false;
			if (preferLinkNameBinding && preferredLinkNames != null && preferredLinkNames.Length >= 6)
			{
				nameBindingSuccess = TryBindJointsByNameList(dofJoints, preferredLinkNames, "preferred Link_01..Link_06 names");
			}

			if (autoBindByJointName && expectedJointNames != null && expectedJointNames.Length >= 6)
			{
				nameBindingSuccess = nameBindingSuccess || TryBindJointsByExpectedNames(dofJoints);
			}

			if (!nameBindingSuccess)
			{
				for (int i = 0; i < Mathf.Min(6, dofJoints.Count); i++)
				{
					joints[i] = dofJoints[i];
					jointTransforms[i] = dofJoints[i].transform;
				}

				Debug.LogWarning($"[Arm6DOF] Auto-detected DOF joints by hierarchy order: {dofJoints.Count}. Consider assigning joints[] manually or checking expectedJointNames.");
			}

			LogDetectedJointMapping();
		}

		private List<ArticulationBody> CollectDofJointsForBinding()
		{
			List<ArticulationBody> result = new List<ArticulationBody>();
			Transform root = armHierarchyRoot != null ? armHierarchyRoot : transform;
			AppendDofJointsFromRoot(root, result);

			if (searchWholeSceneIfLocalSearchFails && result.Count < 6)
			{
				ArticulationBody[] allBodies = FindObjectsOfType<ArticulationBody>(true);
				for (int i = 0; i < allBodies.Length; i++)
				{
					ArticulationBody body = allBodies[i];
					if (!HasDofBody(body) || result.Contains(body))
					{
						continue;
					}

					result.Add(body);
				}
			}

			LogDofJointCandidates(result);
			return result;
		}

		private void AppendDofJointsFromRoot(Transform root, List<ArticulationBody> output)
		{
			if (root == null)
			{
				return;
			}

			ArticulationBody[] bodies = root.GetComponentsInChildren<ArticulationBody>(true);
			for (int i = 0; i < bodies.Length; i++)
			{
				if (!HasDofBody(bodies[i]) || output.Contains(bodies[i]))
				{
					continue;
				}

				output.Add(bodies[i]);
			}
		}

		private static bool HasDofBody(ArticulationBody body)
		{
			return body != null && body.jointPosition.dofCount > 0;
		}

		private void LogDofJointCandidates(List<ArticulationBody> candidates)
		{
			if (candidates == null || candidates.Count == 0)
			{
				Debug.LogWarning("[Arm6DOF] No DOF candidates available for joint binding.");
				return;
			}

			string names = string.Empty;
			for (int i = 0; i < candidates.Count; i++)
			{
				names += i == 0 ? candidates[i].name : ", " + candidates[i].name;
			}

			Debug.Log($"[Arm6DOF] DOF candidates ({candidates.Count}): {names}");
		}

		private bool TryBindJointsByExpectedNames(List<ArticulationBody> dofJoints)
		{
			return TryBindJointsByNameList(dofJoints, expectedJointNames, "expected Joint01..Joint06 names");
		}

		private bool TryBindJointsByNameList(List<ArticulationBody> dofJoints, string[] targetNames, string label)
		{
			int boundCount = 0;
			for (int i = 0; i < 6; i++)
			{
				string expected = targetNames[i];
				ArticulationBody match = null;
				for (int j = 0; j < dofJoints.Count; j++)
				{
					if (dofJoints[j] != null && dofJoints[j].name == expected)
					{
						match = dofJoints[j];
						break;
					}
				}

				if (match == null)
				{
					for (int j = 0; j < dofJoints.Count; j++)
					{
						if (dofJoints[j] == null)
						{
							continue;
						}

						int expectedOrdinal = ExtractJointOrdinal(expected);
						int actualOrdinal = ExtractJointOrdinal(dofJoints[j].name);
						if (expectedOrdinal > 0 && actualOrdinal == expectedOrdinal)
						{
							match = dofJoints[j];
							break;
						}
					}
				}

				if (match != null)
				{
					joints[i] = match;
					jointTransforms[i] = match.transform;
					boundCount++;
				}
			}

			if (boundCount == 6)
			{
				Debug.Log($"[Arm6DOF] Bound joints by {label}.");
				return true;
			}

			Debug.LogWarning($"[Arm6DOF] Name-based binding incomplete for {label} ({boundCount}/6).");
			return false;
		}

		private void EnsureJointTransformCache()
		{
			if (jointTransforms == null || jointTransforms.Length < 6)
			{
				jointTransforms = new Transform[6];
			}

			for (int i = 0; i < 6; i++)
			{
				if (joints != null && i < joints.Length && joints[i] != null)
				{
					jointTransforms[i] = joints[i].transform;
				}
			}
		}

		private void ValidateJointMappingAgainstExpectedNames()
		{
			if (expectedJointNames == null || expectedJointNames.Length < 6)
			{
				return;
			}

			for (int i = 0; i < 6; i++)
			{
				if (joints[i] == null)
				{
					continue;
				}

				string expected = expectedJointNames[i];
				string actual = joints[i].name;
				int expectedOrdinal = ExtractJointOrdinal(expected);
				int actualOrdinal = ExtractJointOrdinal(actual);
				bool mismatch = expectedOrdinal > 0 && actualOrdinal > 0 ? expectedOrdinal != actualOrdinal : actual != expected;
				if (mismatch)
				{
					Debug.LogWarning($"[Arm6DOF] Joint mapping mismatch at J{i + 1}: expected '{expected}', actual '{actual}'.");
				}
			}
		}

		private bool HasValidAssignedJointArray()
		{
			if (joints == null || joints.Length < 6)
			{
				return false;
			}

			for (int i = 0; i < 6; i++)
			{
				if (joints[i] == null || joints[i].jointPosition.dofCount <= 0)
				{
					return false;
				}
			}

			return true;
		}

		private bool IsMappingConsistentWithExpectedNames()
		{
			if (expectedJointNames == null || expectedJointNames.Length < 6)
			{
				return true;
			}

			for (int i = 0; i < 6; i++)
			{
				if (joints == null || joints.Length <= i || joints[i] == null)
				{
					return false;
				}

				int expectedOrdinal = ExtractJointOrdinal(expectedJointNames[i]);
				int actualOrdinal = ExtractJointOrdinal(joints[i].name);
				if (expectedOrdinal > 0 && actualOrdinal > 0)
				{
					if (expectedOrdinal != actualOrdinal)
					{
						return false;
					}
				}
				else if (joints[i].name != expectedJointNames[i])
				{
					return false;
				}
			}

			return true;
		}

		private static int ExtractJointOrdinal(string name)
		{
			if (string.IsNullOrEmpty(name))
			{
				return -1;
			}

			int value = 0;
			int factor = 1;
			bool found = false;
			for (int i = name.Length - 1; i >= 0; i--)
			{
				char c = name[i];
				if (c >= '0' && c <= '9')
				{
					value += (c - '0') * factor;
					factor *= 10;
					found = true;
				}
				else if (found)
				{
					break;
				}
			}

			return found ? value : -1;
		}

		private void LogDetectedJointMapping()
		{
			for (int i = 0; i < 6; i++)
			{
				string jointName = joints != null && i < joints.Length && joints[i] != null ? joints[i].name : "null";
				Debug.Log($"[Arm6DOF] J{i + 1} -> {jointName}");
			}
		}

		private void UpdateJointStates()
		{
			for (int i = 0; i < 6; i++)
			{
				if (joints[i] != null && joints[i].jointPosition.dofCount > 0)
				{
					_currentJointAngles[i] = joints[i].jointPosition[0] * Mathf.Rad2Deg;
				}
			}
		}

		private void UpdateMeasuredEndEffectorPose()
		{
			if (endEffector != null)
			{
				_measuredEndEffectorWorldPosition = endEffector.position;
				_measuredEndEffectorWorldRotation = endEffector.rotation;
				Transform baseTransform = GetBaseFrameTransform();
				if (baseTransform != null)
				{
					_measuredEndEffectorPosition = baseTransform.InverseTransformPoint(_measuredEndEffectorWorldPosition);
					_measuredEndEffectorRotation = Quaternion.Inverse(baseTransform.rotation) * _measuredEndEffectorWorldRotation;
				}
				else
				{
					_measuredEndEffectorPosition = _measuredEndEffectorWorldPosition;
					_measuredEndEffectorRotation = _measuredEndEffectorWorldRotation;
				}
			}
		}

		private void EnsureEndEffectorReference()
		{
			Transform candidate = endEffector;
			if (jointTransforms.Length > 0 && jointTransforms[5] != null)
			{
				if (candidate == null || candidate == jointTransforms[5])
				{
					candidate = jointTransforms[5];
				}
			}

			endEffector = FindPreferredEndEffector(candidate);
		}

		private static Transform FindPreferredEndEffector(Transform rootCandidate)
		{
			if (rootCandidate == null)
			{
				return null;
			}

			Transform[] children = rootCandidate.GetComponentsInChildren<Transform>(true);
			for (int i = 0; i < children.Length; i++)
			{
				string name = children[i].name;
				if (name.IndexOf("tcp", System.StringComparison.OrdinalIgnoreCase) >= 0
					|| name.IndexOf("tool", System.StringComparison.OrdinalIgnoreCase) >= 0
					|| name.IndexOf("endeffector", System.StringComparison.OrdinalIgnoreCase) >= 0
					|| name.IndexOf("flange", System.StringComparison.OrdinalIgnoreCase) >= 0)
				{
					return children[i];
				}
			}

			return rootCandidate;
		}

		private Transform GetBaseFrameTransform()
		{
			if (jointTransforms != null && jointTransforms.Length > 0 && jointTransforms[0] != null && jointTransforms[0].parent != null)
			{
				return jointTransforms[0].parent;
			}

			return armHierarchyRoot != null ? armHierarchyRoot : transform;
		}

		private void RefreshCollisionMonitor()
		{
			if (_collisionMonitor == null)
			{
				_collisionMonitor = ArmCollisionMonitor.Instance;
			}

			if (_collisionMonitor == null)
			{
				_collisionMonitor = FindObjectOfType<ArmCollisionMonitor>();
			}
		}

		private void TryAutoAssignUrdfSource()
		{
			if (HasValidUrdfReference())
			{
				return;
			}

			#if UNITY_EDITOR
			string[] guids = UnityEditor.AssetDatabase.FindAssets("Zu5_LDASM_unity_fixed");
			for (int i = 0; i < guids.Length; i++)
			{
				string path = UnityEditor.AssetDatabase.GUIDToAssetPath(guids[i]);
				if (!path.EndsWith(".urdf", System.StringComparison.OrdinalIgnoreCase))
				{
					continue;
				}

				if (path.IndexOf("Zone.Identifier", System.StringComparison.OrdinalIgnoreCase) >= 0)
				{
					continue;
				}

				urdfAssetPath = path;
				urdfSource = null;
				break;
			}
			#endif
		}

		private string ResolveUrdfText()
		{
			#if UNITY_EDITOR
			if (IsValidUrdfPath(urdfAssetPath))
			{
				string fullPath = urdfAssetPath.Replace("Assets", Application.dataPath);
				if (File.Exists(fullPath))
				{
					return File.ReadAllText(fullPath);
				}
			}
			#endif

			if (urdfSource != null)
			{
				return urdfSource.text;
			}

			return null;
		}

		private void SanitizeUrdfReferences()
		{
			#if UNITY_EDITOR
			if (urdfSource != null)
			{
				string sourcePath = UnityEditor.AssetDatabase.GetAssetPath(urdfSource);
				if (!IsValidUrdfPath(sourcePath))
				{
					urdfSource = null;
				}
				else if (string.IsNullOrEmpty(urdfAssetPath))
				{
					urdfAssetPath = sourcePath;
				}
			}
			#endif

			if (!IsValidUrdfPath(urdfAssetPath))
			{
				urdfAssetPath = string.Empty;
			}
		}

		private bool HasValidUrdfReference()
		{
			if (IsValidUrdfPath(urdfAssetPath))
			{
				return true;
			}

			#if UNITY_EDITOR
			if (urdfSource != null)
			{
				return IsValidUrdfPath(UnityEditor.AssetDatabase.GetAssetPath(urdfSource));
			}
			#endif

			return false;
		}

		private static bool IsValidUrdfPath(string path)
		{
			if (string.IsNullOrEmpty(path))
			{
				return false;
			}

			if (!path.EndsWith(".urdf", System.StringComparison.OrdinalIgnoreCase))
			{
				return false;
			}

			return path.IndexOf("Zone.Identifier", System.StringComparison.OrdinalIgnoreCase) < 0;
		}
	}
}
