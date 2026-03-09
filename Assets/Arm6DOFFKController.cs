using UnityEngine;
using System.Collections.Generic;

namespace RobotSimulation
{
	/// <summary>
	/// 6-DOF Articulated Arm Controller
	/// Manages all 6 joints and provides forward kinematics
	/// </summary>
	public class Arm6DOFFKController : MonoBehaviour
	{
		public static Arm6DOFFKController Instance { get; private set; }

		[Header("Joint Configuration")]
		public ArticulationBody[] joints = new ArticulationBody[6];
		public Transform[] jointTransforms = new Transform[6];
		public bool autoBindByJointName = true;
		public bool autoRebindOnMismatch = true;
		public bool preferLinkNameBinding = true;
		public Transform armHierarchyRoot;
		public bool searchWholeSceneIfLocalSearchFails = true;
		public string[] preferredLinkNames = new string[] { "Link_01", "Link_02", "Link_03", "Link_04", "Link_05", "Link_06" };
		public string[] expectedJointNames = new string[] { "Joint01", "Joint02", "Joint03", "Joint04", "Joint05", "Joint06" };

		[Header("Link Lengths (DH Parameters)")]
		public float[] linkLengths = new float[] { 0.175f, 0.475f, 0.425f, 0.0f, 0.0f, 0.0f }; // Adjust based on actual robot

		[Header("Joint Limits (degrees)")]
		public Vector2[] jointLimits = new Vector2[]
		{
			new Vector2(-180, 180),  // J1: Base rotation
			new Vector2(-90, 90),     // J2: Shoulder
			new Vector2(-150, 150),   // J3: Elbow
			new Vector2(-180, 180),   // J4: Wrist 1
			new Vector2(-120, 120),   // J5: Wrist 2
			new Vector2(-180, 180)    // J6: Wrist 3
		};

		[Header("End Effector")]
		public Transform endEffector; // Tip of the arm

		[Header("Status")]
		[SerializeField] private float[] _currentJointAngles = new float[6];
		[SerializeField] private Vector3 _endEffectorPosition;
		[SerializeField] private Quaternion _endEffectorRotation;
		[SerializeField] private bool _isInitialized = false;

		public float[] CurrentJointAngles => _currentJointAngles;
		public Vector3 EndEffectorPosition => _endEffectorPosition;
		public Quaternion EndEffectorRotation => _endEffectorRotation;
		public bool IsInitialized => _isInitialized;

		void Awake()
		{
			if (Instance == null)
			{
				Instance = this;
			}
		}

		void Start()
		{
			InitializeJoints();
		}

		void FixedUpdate()
		{
			if (!_isInitialized) return;

			UpdateJointStates();
			ComputeForwardKinematics();
		}

		/// <summary>
		/// Initialize and validate joint references
		/// </summary>
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

			// Validate all joints are assigned
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

			// Auto-find end effector if not assigned
			if (endEffector == null && jointTransforms.Length > 0 && jointTransforms[5] != null)
			{
				endEffector = jointTransforms[5];
				Transform[] children = endEffector.GetComponentsInChildren<Transform>();
				if (children.Length > 1)
				{
					endEffector = children[children.Length - 1];
				}
			}

			_isInitialized = true;
			Debug.Log("[Arm6DOF] Initialized successfully");
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
					if (!HasDofBody(body)) continue;
					if (!result.Contains(body))
					{
						result.Add(body);
					}
				}
			}

			LogDofJointCandidates(result);
			return result;
		}

		private void AppendDofJointsFromRoot(Transform root, List<ArticulationBody> output)
		{
			if (root == null) return;

			ArticulationBody[] bodies = root.GetComponentsInChildren<ArticulationBody>(true);
			for (int i = 0; i < bodies.Length; i++)
			{
				ArticulationBody body = bodies[i];
				if (!HasDofBody(body)) continue;
				if (!output.Contains(body))
				{
					output.Add(body);
				}
			}
		}

		private bool HasDofBody(ArticulationBody body)
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

			string names = "";
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
						if (dofJoints[j] == null) continue;

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
			if (expectedJointNames == null || expectedJointNames.Length < 6) return;

			for (int i = 0; i < 6; i++)
			{
				if (joints[i] == null) continue;

				string expected = expectedJointNames[i];
				string actual = joints[i].name;
				int expectedOrdinal = ExtractJointOrdinal(expected);
				int actualOrdinal = ExtractJointOrdinal(actual);

				bool mismatch = expectedOrdinal > 0 && actualOrdinal > 0
					? expectedOrdinal != actualOrdinal
					: actual != expected;

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
				if (joints[i] == null)
				{
					return false;
				}

				if (joints[i].jointPosition.dofCount <= 0)
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

		private int ExtractJointOrdinal(string name)
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

			if (!found)
			{
				return -1;
			}

			return value;
		}

		private void LogDetectedJointMapping()
		{
			for (int i = 0; i < 6; i++)
			{
				string jointName = joints != null && i < joints.Length && joints[i] != null ? joints[i].name : "null";
				Debug.Log($"[Arm6DOF] J{i + 1} -> {jointName}");
			}
		}

		/// <summary>
		/// Update current joint angle states
		/// </summary>
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

		/// <summary>
		/// Compute forward kinematics - get end effector pose from joint angles
		/// </summary>
		public void ComputeForwardKinematics()
		{
			if (endEffector != null)
			{
				_endEffectorPosition = endEffector.position;
				_endEffectorRotation = endEffector.rotation;
			}
		}

		/// <summary>
		/// Set target angle for a specific joint
		/// </summary>
		public void SetJointTarget(int jointIndex, float angleDeg)
		{
			if (jointIndex < 0 || jointIndex >= 6)
			{
				Debug.LogError($"[Arm6DOF] Invalid joint index: {jointIndex}");
				return;
			}

			// Apply joint limits
			angleDeg = Mathf.Clamp(angleDeg, jointLimits[jointIndex].x, jointLimits[jointIndex].y);

			var joint = joints[jointIndex];
			if (joint != null)
			{
				var drive = joint.xDrive;
				drive.target = angleDeg;
				joint.xDrive = drive;
				joint.WakeUp();
			}
		}

		/// <summary>
		/// Set all joint targets at once
		/// </summary>
		public void SetAllJointTargets(float[] angles)
		{
			if (angles == null || angles.Length < 6)
			{
				Debug.LogError("[Arm6DOF] Invalid angles array");
				return;
			}

			for (int i = 0; i < 6; i++)
			{
				SetJointTarget(i, angles[i]);
			}
		}

		/// <summary>
		/// Get joint limits for a specific joint
		/// </summary>
		public Vector2 GetJointLimits(int jointIndex)
		{
			if (jointIndex >= 0 && jointIndex < 6)
			{
				return jointLimits[jointIndex];
			}
			return new Vector2(-180, 180);
		}

		/// <summary>
		/// Check if a position is reachable (simplified)
		/// </summary>
		public bool IsPositionReachable(Vector3 position, float tolerance = 0.01f)
		{
			// Calculate maximum reach based on link lengths
			float maxReach = 0f;
			for (int i = 0; i < linkLengths.Length; i++)
			{
				maxReach += Mathf.Abs(linkLengths[i]);
			}

			// Minimum reach (arm cannot fold completely to 0)
			float minReach = maxReach * 0.1f;

			float distance = Vector3.Distance(transform.position, position);

			return distance >= minReach && distance <= maxReach + tolerance;
		}

		/// <summary>
		/// Get maximum reach distance
		/// </summary>
		public float GetMaxReach()
		{
			float maxReach = 0f;
			for (int i = 0; i < linkLengths.Length; i++)
			{
				maxReach += Mathf.Abs(linkLengths[i]);
			}
			return maxReach;
		}

		/// <summary>
		/// Get current joint velocities
		/// </summary>
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

		/// <summary>
		/// Check if all joints are at target (within tolerance)
		/// </summary>
		public bool IsAtTarget(float toleranceDeg = 1f)
		{
			for (int i = 0; i < 6; i++)
			{
				if (joints[i] != null)
				{
					var drive = joints[i].xDrive;
					float error = Mathf.Abs(joints[i].jointPosition[0] * Mathf.Rad2Deg - drive.target);
					if (error > toleranceDeg)
					{
						return false;
					}
				}
			}
			return true;
		}

		/// <summary>
		/// Emergency stop - zero all joint velocities
		/// </summary>
		public void EmergencyStop()
		{
			for (int i = 0; i < 6; i++)
			{
				if (joints[i] != null)
				{
					var drive = joints[i].xDrive;
					drive.targetVelocity = 0f;
					joints[i].xDrive = drive;
				}
			}
		}

		/// <summary>
		/// Reset arm to home position
		/// </summary>
		public void GoHome()
		{
			SetAllJointTargets(new float[] { 0, 0, 0, 0, 0, 0 });
		}

		/// <summary>
		/// Get arm status report
		/// </summary>
		public string GetStatusReport()
		{
			string report = "=== Arm Status ===\n";
			report += $"Initialized: {_isInitialized}\n";
			report += $"End Effector: ({_endEffectorPosition.x:F3}, {_endEffectorPosition.y:F3}, {_endEffectorPosition.z:F3})\n";
			report += "Joint Angles:\n";
			for (int i = 0; i < 6; i++)
			{
				report += $"  J{i + 1}: {_currentJointAngles[i]:F2}°\n";
			}
			return report;
		}
	}
}
