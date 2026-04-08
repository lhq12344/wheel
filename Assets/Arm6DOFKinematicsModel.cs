using System;
using System.Collections.Generic;
using System.Globalization;
using System.Text;
using System.Xml.Linq;
using UnityEngine;

namespace RobotSimulation
{
	[Serializable]
	public struct ScrewAxis
	{
		public Vector3 angular;
		public Vector3 linear;

		public ScrewAxis(Vector3 angular, Vector3 linear)
		{
			this.angular = angular;
			this.linear = linear;
		}
	}

	[Serializable]
	public struct DhRow
	{
		public string jointName;
		public float a;
		public float alphaDeg;
		public float d;
		public float thetaOffsetDeg;
		public int axisSign;
		public Vector3 origin;
		public Vector3 rpyDeg;
		public Vector3 axisLocal;
	}

	public sealed class Arm6DOFKinematicsModel
	{
		private const int JointCount = 6;
		private const float JacobianEpsilonDeg = 0.25f;
		private const float WorkspaceOuterRadiusMeters = 0.8066f;
		private const float WorkspaceInnerRadiusMeters = 0.24338f;
		private const float WorkspaceVerticalSpanMeters = 1.59701f;
		private const float WorkspaceBaseDiameterMeters = 0.228f;
		private const float WorkspaceVolumeCubicMeters = 0.002138f;

		private struct JointDefinition
		{
			public string name;
			public string parentLink;
			public string childLink;
			public Vector3 origin;
			public Vector3 rpyRad;
			public Vector3 axisLocal;
		}

		public bool IsInitialized { get; private set; }
		public bool HasSceneCalibration { get; private set; }
		public string ErrorMessage { get; private set; }
		public Matrix4x4 HomeEndEffectorBase => HasSceneCalibration ? _referenceEndEffectorBase : _homeEndEffectorUrdf;
		public float MaxReachDistance => WorkspaceOuterRadiusMeters;
		public float MinReachDistance => WorkspaceInnerRadiusMeters;
		public float WorkspaceVerticalSpan => WorkspaceVerticalSpanMeters;
		public float WorkspaceBaseDiameter => WorkspaceBaseDiameterMeters;
		public float WorkspaceVolumeCubicMetersValue => WorkspaceVolumeCubicMeters;

		private readonly JointDefinition[] _joints = new JointDefinition[JointCount];
		private readonly DhRow[] _dhRows = new DhRow[JointCount];
		private readonly ScrewAxis[] _poeAxesUrdf = new ScrewAxis[JointCount];
		private readonly ScrewAxis[] _poeAxesBase = new ScrewAxis[JointCount];
		private readonly Vector3[] _homeJointPositionsUrdf = new Vector3[JointCount];
		private readonly float[] _segmentLengths = new float[JointCount + 1];
		private readonly float[] _referenceJointAnglesDeg = new float[JointCount];
		private readonly float[] _configuredHomeAnglesDeg = new float[JointCount];

		private Matrix4x4 _homeEndEffectorUrdf = Matrix4x4.identity;
		private Matrix4x4 _referenceEndEffectorBase = Matrix4x4.identity;
		private float _segmentReachUpperBound;

		public DhRow[] GetDhRows()
		{
			DhRow[] result = new DhRow[JointCount];
			Array.Copy(_dhRows, result, JointCount);
			return result;
		}

		public string BuildParameterReport()
		{
			StringBuilder builder = new StringBuilder();
			builder.AppendLine("=== Arm6DOF Kinematics Parameters ===");
			builder.AppendLine($"Initialized: {IsInitialized}");
			builder.AppendLine($"Scene Calibration: {HasSceneCalibration}");
			builder.AppendLine($"Workspace Min Reach: {MinReachDistance:F6} m");
			builder.AppendLine($"Workspace Max Reach: {MaxReachDistance:F6} m");
			builder.AppendLine($"Workspace Vertical Span: {WorkspaceVerticalSpan:F6} m");
			builder.AppendLine($"Workspace Base Diameter: {WorkspaceBaseDiameter:F6} m");
			builder.AppendLine($"Workspace Volume: {WorkspaceVolumeCubicMetersValue:F6} m^3");
			builder.AppendLine($"Segment Reach Upper Bound: {_segmentReachUpperBound:F6} m");
			builder.AppendLine($"Configured Home Angles: {FormatFloatArray(_configuredHomeAnglesDeg)}");
			builder.AppendLine($"Home M (active): {FormatMatrix(HomeEndEffectorBase)}");
			builder.AppendLine($"Home M (URDF): {FormatMatrix(_homeEndEffectorUrdf)}");

			for (int i = 0; i < JointCount; i++)
			{
				ScrewAxis activeAxis = HasSceneCalibration ? _poeAxesBase[i] : _poeAxesUrdf[i];
				builder.AppendLine($"J{i + 1} {_joints[i].name}");
				builder.AppendLine($"  parent -> child: {_joints[i].parentLink} -> {_joints[i].childLink}");
				builder.AppendLine($"  urdf origin xyz: {FormatVector(_joints[i].origin)}");
				builder.AppendLine($"  urdf origin rpy deg: {FormatVector(_joints[i].rpyRad * Mathf.Rad2Deg)}");
				builder.AppendLine($"  urdf axis local: {FormatVector(_joints[i].axisLocal)}");
				builder.AppendLine($"  urdf screw w: {FormatVector(_poeAxesUrdf[i].angular)}");
				builder.AppendLine($"  urdf screw v: {FormatVector(_poeAxesUrdf[i].linear)}");
				if (HasSceneCalibration)
				{
					builder.AppendLine($"  reference angle deg: {_referenceJointAnglesDeg[i]:F6}");
					builder.AppendLine($"  calibrated screw w: {FormatVector(activeAxis.angular)}");
					builder.AppendLine($"  calibrated screw v: {FormatVector(activeAxis.linear)}");
				}

				builder.AppendLine($"  mdh a_(i-1): {_dhRows[i].a:F6}");
				builder.AppendLine($"  mdh alpha_(i-1) deg: {_dhRows[i].alphaDeg:F6}");
				builder.AppendLine($"  mdh d_i: {_dhRows[i].d:F6}");
				builder.AppendLine($"  mdh theta_offset deg: {_dhRows[i].thetaOffsetDeg:F6}");
				builder.AppendLine($"  mdh sign: {_dhRows[i].axisSign}");
			}

			builder.AppendLine($"Segment Lengths: {FormatFloatArray(_segmentLengths)}");
			return builder.ToString();
		}

		public bool Initialize(TextAsset urdfSource, Transform baseTransform = null, Transform[] jointTransforms = null, Transform endEffector = null)
		{
			return Initialize(urdfSource != null ? urdfSource.text : null, baseTransform, jointTransforms, endEffector, null);
		}

		public bool Initialize(TextAsset urdfSource, Transform baseTransform, Transform[] jointTransforms, Transform endEffector, float[] configuredHomeAnglesDeg)
		{
			return Initialize(urdfSource != null ? urdfSource.text : null, baseTransform, jointTransforms, endEffector, configuredHomeAnglesDeg);
		}

		public bool Initialize(string urdfText, Transform baseTransform = null, Transform[] jointTransforms = null, Transform endEffector = null, float[] configuredHomeAnglesDeg = null)
		{
			ErrorMessage = string.Empty;
			IsInitialized = false;
			HasSceneCalibration = false;
			_referenceEndEffectorBase = Matrix4x4.identity;
			Array.Clear(_referenceJointAnglesDeg, 0, _referenceJointAnglesDeg.Length);
			CopyAngles(configuredHomeAnglesDeg, _configuredHomeAnglesDeg);

			if (string.IsNullOrWhiteSpace(urdfText))
			{
				ErrorMessage = "URDF source is not assigned.";
				return false;
			}

			if (!TryParseUrdf(urdfText, out string parseError))
			{
				ErrorMessage = parseError;
				return false;
			}

			BuildHomeData();

			if (baseTransform != null && jointTransforms != null && jointTransforms.Length >= JointCount)
			{
				HasSceneCalibration = TryCalibrateToScene(baseTransform, jointTransforms, endEffector);
			}

			IsInitialized = true;
			return true;
		}

		public Pose ForwardPoe(float[] jointAnglesDeg)
		{
			return ForwardPoseInternal(EnsureAngles(jointAnglesDeg), HasSceneCalibration);
		}

		public Pose ForwardUrdfPoe(float[] jointAnglesDeg)
		{
			return ForwardPoseInternal(EnsureAngles(jointAnglesDeg), false);
		}

		public Pose ForwardDh(float[] jointAnglesDeg)
		{
			float[] safeAngles = EnsureAngles(jointAnglesDeg);
			Matrix4x4 transform = Matrix4x4.identity;

			for (int i = 0; i < JointCount; i++)
			{
				transform *= BuildDhStepTransform(_dhRows[i], safeAngles[i], _configuredHomeAnglesDeg[i]);
			}

			return MatrixToPose(transform);
		}

		public Pose ForwardUrdfChain(float[] jointAnglesDeg)
		{
			return MatrixToPose(ComputeUrdfChainTransform(EnsureAngles(jointAnglesDeg)));
		}

		public Pose[] ComputeLinkPosesBase(float[] jointAnglesDeg)
		{
			Matrix4x4[] transforms = ComputeUrdfLinkTransforms(EnsureAngles(jointAnglesDeg));
			Pose[] poses = new Pose[transforms.Length];
			for (int i = 0; i < transforms.Length; i++)
			{
				poses[i] = MatrixToPose(transforms[i]);
			}

			return poses;
		}

		public float[,] ComputeSpaceJacobian(float[] jointAnglesDeg)
		{
			float[] safeAngles = EnsureAngles(jointAnglesDeg);
			float[,] jacobian = new float[6, JointCount];
			float epsilonRad = JacobianEpsilonDeg * Mathf.Deg2Rad;
			Pose basePose = ForwardPoe(safeAngles);
			ScrewAxis[] axes = HasSceneCalibration ? _poeAxesBase : _poeAxesUrdf;

			for (int i = 0; i < JointCount; i++)
			{
				float[] perturbed = (float[])safeAngles.Clone();
				perturbed[i] += JacobianEpsilonDeg;
				Pose offsetPose = ForwardPoe(perturbed);
				Vector3 dp = (offsetPose.position - basePose.position) / epsilonRad;
				Vector3 angular = axes[i].angular;

				jacobian[0, i] = angular.x;
				jacobian[1, i] = angular.y;
				jacobian[2, i] = angular.z;
				jacobian[3, i] = dp.x;
				jacobian[4, i] = dp.y;
				jacobian[5, i] = dp.z;
			}

			return jacobian;
		}

		public Vector3 WorldToBasePosition(Transform armBase, Vector3 worldPosition)
		{
			return armBase != null ? armBase.InverseTransformPoint(worldPosition) : worldPosition;
		}

		public bool IsWithinWorkspaceShell(Vector3 basePosition, float tolerance, out string reason)
		{
			float distance = basePosition.magnitude;
			float minReach = Mathf.Max(0f, MinReachDistance - tolerance);
			float maxReach = MaxReachDistance + tolerance;

			if (distance < minReach)
			{
				reason = $"Target is inside the Zu5 dead-zone sphere (distance {distance:F3}m < min {MinReachDistance:F3}m).";
				return false;
			}

			if (distance > maxReach)
			{
				reason = $"Target is outside the Zu5 outer workspace sphere (distance {distance:F3}m > max {MaxReachDistance:F3}m).";
				return false;
			}

			reason = "Target is inside the Zu5 spherical-shell workspace.";
			return true;
		}

		private Pose ForwardPoseInternal(float[] jointAnglesDeg, bool useSceneCalibration)
		{
			Matrix4x4 transform = Matrix4x4.identity;
			ScrewAxis[] axes = useSceneCalibration ? _poeAxesBase : _poeAxesUrdf;
			Matrix4x4 home = useSceneCalibration ? _referenceEndEffectorBase : _homeEndEffectorUrdf;

			for (int i = 0; i < JointCount; i++)
			{
				float deltaDeg = jointAnglesDeg[i];
				if (useSceneCalibration)
				{
					deltaDeg -= _referenceJointAnglesDeg[i];
				}
				else
				{
					deltaDeg -= _configuredHomeAnglesDeg[i];
				}

				transform *= MatrixExponential(axes[i], deltaDeg * Mathf.Deg2Rad);
			}

			transform *= home;
			return MatrixToPose(transform);
		}

		private bool TryParseUrdf(string urdfText, out string error)
		{
			error = string.Empty;
			try
			{
				XDocument document = XDocument.Parse(urdfText);
				XElement robot = document.Root;
				if (robot == null || robot.Name != "robot")
				{
					error = "URDF robot root is missing.";
					return false;
				}

				Dictionary<string, XElement> jointElements = new Dictionary<string, XElement>(StringComparer.Ordinal);
				foreach (XElement jointElement in robot.Elements("joint"))
				{
					XAttribute nameAttr = jointElement.Attribute("name");
					XAttribute typeAttr = jointElement.Attribute("type");
					if (nameAttr == null || typeAttr == null || typeAttr.Value != "revolute")
					{
						continue;
					}

					jointElements[nameAttr.Value] = jointElement;
				}

				for (int i = 0; i < JointCount; i++)
				{
					string jointName = $"Joint{i + 1:00}";
					if (!jointElements.TryGetValue(jointName, out XElement jointElement))
					{
						error = $"Missing revolute joint '{jointName}' in URDF.";
						return false;
					}

					XElement originElement = jointElement.Element("origin");
					XElement parentElement = jointElement.Element("parent");
					XElement childElement = jointElement.Element("child");
					XElement axisElement = jointElement.Element("axis");
					if (originElement == null || parentElement == null || childElement == null || axisElement == null)
					{
						error = $"Joint '{jointName}' is missing origin/parent/child/axis data.";
						return false;
					}

					_joints[i] = new JointDefinition
					{
						name = jointName,
						parentLink = parentElement.Attribute("link")?.Value ?? string.Empty,
						childLink = childElement.Attribute("link")?.Value ?? string.Empty,
						origin = ParseVector3(originElement.Attribute("xyz")?.Value),
						rpyRad = ParseVector3(originElement.Attribute("rpy")?.Value),
						axisLocal = ParseVector3(axisElement.Attribute("xyz")?.Value).normalized
					};

					if (i > 0 && _joints[i - 1].childLink != _joints[i].parentLink)
					{
						error = $"Joint chain is broken between '{_joints[i - 1].name}' and '{jointName}'.";
						return false;
					}
				}
			}
			catch (Exception ex)
			{
				error = $"Failed to parse URDF: {ex.Message}";
				return false;
			}

			return true;
		}

		private void BuildHomeData()
		{
			Matrix4x4 current = Matrix4x4.identity;

			for (int i = 0; i < JointCount; i++)
			{
				current *= BuildOriginTransform(_joints[i].origin, _joints[i].rpyRad * Mathf.Rad2Deg);
				Vector3 jointPosition = ExtractPosition(current);
				Vector3 axisWorld = current.MultiplyVector(_joints[i].axisLocal).normalized;

				_homeJointPositionsUrdf[i] = jointPosition;
				_poeAxesUrdf[i] = new ScrewAxis(axisWorld, -Vector3.Cross(axisWorld, jointPosition));
				float alphaDeg = _joints[i].rpyRad.x * Mathf.Rad2Deg;
				float d = DeriveModifiedDhOffset(_joints[i].origin, alphaDeg);
				int axisSign = DetermineAxisSign(_joints[i].axisLocal);
				_dhRows[i] = new DhRow
				{
					jointName = _joints[i].name,
					a = _joints[i].origin.x,
					alphaDeg = alphaDeg,
					d = d,
					thetaOffsetDeg = (_joints[i].rpyRad.z * Mathf.Rad2Deg) + (axisSign * _configuredHomeAnglesDeg[i]),
					axisSign = axisSign,
					origin = _joints[i].origin,
					rpyDeg = _joints[i].rpyRad * Mathf.Rad2Deg,
					axisLocal = _joints[i].axisLocal
				};
			}

			_homeEndEffectorUrdf = ComputeUrdfChainTransform(_configuredHomeAnglesDeg);

			Vector3 previous = Vector3.zero;
			for (int i = 0; i < JointCount; i++)
			{
				_segmentLengths[i] = Vector3.Distance(previous, _homeJointPositionsUrdf[i]);
				previous = _homeJointPositionsUrdf[i];
			}

			_segmentLengths[JointCount] = Vector3.Distance(previous, ExtractPosition(_homeEndEffectorUrdf));
			_segmentReachUpperBound = SumAbsoluteSegments(_segmentLengths);
		}

		private bool TryCalibrateToScene(Transform baseTransform, Transform[] jointTransforms, Transform endEffector)
		{
			if (baseTransform == null || jointTransforms == null || jointTransforms.Length < JointCount)
			{
				return false;
			}

			Transform measuredEndEffector = endEffector != null ? endEffector : jointTransforms[JointCount - 1];
			if (measuredEndEffector == null)
			{
				return false;
			}

			_referenceEndEffectorBase = BuildRelativeTransform(baseTransform, measuredEndEffector);

			Vector3 previous = Vector3.zero;
			float sceneReach = 0f;
			for (int i = 0; i < JointCount; i++)
			{
				Transform jointTransform = jointTransforms[i];
				if (jointTransform == null)
				{
					return false;
				}

				ArticulationBody body = jointTransform.GetComponent<ArticulationBody>();
				if (body != null && body.jointPosition.dofCount > 0)
				{
					_referenceJointAnglesDeg[i] = body.jointPosition[0] * Mathf.Rad2Deg;
				}
				else
				{
					_referenceJointAnglesDeg[i] = 0f;
				}

				Vector3 jointPositionBase = ResolveSceneJointPosition(baseTransform, jointTransform, body);
				Vector3 jointAxisBase = ResolveSceneAxis(baseTransform, jointTransform, body).normalized;
				if (jointAxisBase.sqrMagnitude < 1e-8f)
				{
					return false;
				}

				_poeAxesBase[i] = new ScrewAxis(jointAxisBase, -Vector3.Cross(jointAxisBase, jointPositionBase));
				sceneReach += Vector3.Distance(previous, jointPositionBase);
				previous = jointPositionBase;
			}

			sceneReach += Vector3.Distance(previous, ExtractPosition(_referenceEndEffectorBase));
			_segmentReachUpperBound = Mathf.Max(_segmentReachUpperBound, sceneReach);
			return true;
		}

		private static float[] EnsureAngles(float[] jointAnglesDeg)
		{
			float[] safe = new float[JointCount];
			if (jointAnglesDeg == null)
			{
				return safe;
			}

			int count = Mathf.Min(JointCount, jointAnglesDeg.Length);
			for (int i = 0; i < count; i++)
			{
				safe[i] = jointAnglesDeg[i];
			}

			return safe;
		}

		private static void CopyAngles(float[] source, float[] destination)
		{
			Array.Clear(destination, 0, destination.Length);
			if (source == null)
			{
				return;
			}

			int count = Mathf.Min(source.Length, destination.Length);
			for (int i = 0; i < count; i++)
			{
				destination[i] = source[i];
			}
		}

		private Matrix4x4 ComputeUrdfChainTransform(float[] jointAnglesDeg)
		{
			Matrix4x4[] transforms = ComputeUrdfLinkTransforms(jointAnglesDeg);
			return transforms[JointCount];
		}

		private Matrix4x4[] ComputeUrdfLinkTransforms(float[] jointAnglesDeg)
		{
			Matrix4x4[] transforms = new Matrix4x4[JointCount + 1];
			Matrix4x4 transform = Matrix4x4.identity;
			transforms[0] = transform;
			for (int i = 0; i < JointCount; i++)
			{
				transform *= BuildOriginTransform(_joints[i].origin, _joints[i].rpyRad * Mathf.Rad2Deg);
				Vector3 axisLocal = _joints[i].axisLocal.sqrMagnitude > 1e-8f ? _joints[i].axisLocal.normalized : Vector3.forward;
				transform *= Matrix4x4.Rotate(Quaternion.AngleAxis(jointAnglesDeg[i], axisLocal));
				transforms[i + 1] = transform;
			}

			return transforms;
		}

		private static Matrix4x4 BuildOriginTransform(DhRow row)
		{
			return BuildOriginTransform(row.origin, row.rpyDeg);
		}

		private static Matrix4x4 BuildOriginTransform(Vector3 origin, Vector3 rpyDeg)
		{
			return Matrix4x4.TRS(origin, QuaternionFromRpyDeg(rpyDeg), Vector3.one);
		}

		private static Matrix4x4 BuildDhStepTransform(DhRow row, float jointAngleDeg, float referenceHomeAngleDeg)
		{
			float thetaDeg = (row.axisSign * (jointAngleDeg - referenceHomeAngleDeg)) + row.thetaOffsetDeg;
			return Matrix4x4.Translate(new Vector3(row.a, 0f, 0f))
				* Matrix4x4.Rotate(Quaternion.AngleAxis(row.alphaDeg, Vector3.right))
				* Matrix4x4.Rotate(Quaternion.AngleAxis(thetaDeg, Vector3.forward))
				* Matrix4x4.Translate(new Vector3(0f, 0f, row.d));
		}

		private static Matrix4x4 BuildRelativeTransform(Transform origin, Transform target)
		{
			Vector3 localPosition = origin.InverseTransformPoint(target.position);
			Quaternion localRotation = Quaternion.Inverse(origin.rotation) * target.rotation;
			return Matrix4x4.TRS(localPosition, localRotation, Vector3.one);
		}

		private static Quaternion QuaternionFromRpyDeg(Vector3 rpyDeg)
		{
			return Quaternion.AngleAxis(rpyDeg.z, Vector3.forward)
				 * Quaternion.AngleAxis(rpyDeg.y, Vector3.up)
				 * Quaternion.AngleAxis(rpyDeg.x, Vector3.right);
		}

		private static Vector3 ResolveSceneJointPosition(Transform baseTransform, Transform jointTransform, ArticulationBody body)
		{
			if (body != null && jointTransform.parent != null)
			{
				Vector3 anchorWorld = jointTransform.parent.TransformPoint(body.parentAnchorPosition);
				return baseTransform.InverseTransformPoint(anchorWorld);
			}

			return baseTransform.InverseTransformPoint(jointTransform.position);
		}

		private static Vector3 ResolveSceneAxis(Transform baseTransform, Transform jointTransform, ArticulationBody body)
		{
			Vector3 worldAxis = jointTransform.right;
			if (body != null && jointTransform.parent != null)
			{
				Quaternion anchorWorldRotation = jointTransform.parent.rotation * body.parentAnchorRotation;
				worldAxis = anchorWorldRotation * Vector3.right;
			}

			Vector3 baseAxis = baseTransform.InverseTransformDirection(worldAxis);
			return baseAxis.normalized;
		}

		private static Matrix4x4 MatrixExponential(ScrewAxis axis, float thetaRad)
		{
			Vector3 w = axis.angular;
			Vector3 v = axis.linear;
			if (w.sqrMagnitude < 1e-8f)
			{
				return Matrix4x4.TRS(v * thetaRad, Quaternion.identity, Vector3.one);
			}

			Quaternion rotation = Quaternion.AngleAxis(thetaRad * Mathf.Rad2Deg, w.normalized);
			Vector3 translation = ((Matrix3x3.identity - RotationMatrix3x3(rotation)) * Vector3.Cross(w, v))
				+ (Vector3.Dot(w, v) * w * thetaRad);
			return Matrix4x4.TRS(translation, rotation, Vector3.one);
		}

		private static Matrix3x3 RotationMatrix3x3(Quaternion rotation)
		{
			Matrix4x4 matrix = Matrix4x4.Rotate(rotation);
			return new Matrix3x3(matrix.GetColumn(0), matrix.GetColumn(1), matrix.GetColumn(2));
		}

		private static Pose MatrixToPose(Matrix4x4 matrix)
		{
			return new Pose(ExtractPosition(matrix), matrix.rotation);
		}

		private static Vector3 ExtractPosition(Matrix4x4 matrix)
		{
			return matrix.MultiplyPoint3x4(Vector3.zero);
		}

		private static Vector3 ParseVector3(string value)
		{
			if (string.IsNullOrWhiteSpace(value))
			{
				return Vector3.zero;
			}

			string[] parts = value.Split(new[] { ' ' }, StringSplitOptions.RemoveEmptyEntries);
			if (parts.Length != 3)
			{
				return Vector3.zero;
			}

			return new Vector3(ParseFloat(parts[0]), ParseFloat(parts[1]), ParseFloat(parts[2]));
		}

		private static float ParseFloat(string value)
		{
			return float.Parse(value, CultureInfo.InvariantCulture);
		}

		private static int DetermineAxisSign(Vector3 axis)
		{
			Vector3 normalized = axis.normalized;
			float max = Mathf.Max(Mathf.Abs(normalized.x), Mathf.Abs(normalized.y), Mathf.Abs(normalized.z));
			if (Mathf.Approximately(max, Mathf.Abs(normalized.x)))
			{
				return normalized.x < 0f ? -1 : 1;
			}

			if (Mathf.Approximately(max, Mathf.Abs(normalized.y)))
			{
				return normalized.y < 0f ? -1 : 1;
			}

			return normalized.z < 0f ? -1 : 1;
		}

		private static float DeriveModifiedDhOffset(Vector3 origin, float alphaDeg)
		{
			float alphaRad = alphaDeg * Mathf.Deg2Rad;
			float sin = Mathf.Sin(alphaRad);
			float cos = Mathf.Cos(alphaRad);

			if (Mathf.Abs(cos) >= Mathf.Abs(sin))
			{
				return Mathf.Abs(cos) > 1e-6f ? origin.z / cos : 0f;
			}

			return Mathf.Abs(sin) > 1e-6f ? -origin.y / sin : 0f;
		}

		private static float SumAbsoluteSegments(float[] segments)
		{
			float total = 0f;
			for (int i = 0; i < segments.Length; i++)
			{
				total += Mathf.Abs(segments[i]);
			}

			return total;
		}

		private static string FormatVector(Vector3 value)
		{
			return $"({value.x:F6}, {value.y:F6}, {value.z:F6})";
		}

		private static string FormatFloatArray(float[] values)
		{
			StringBuilder builder = new StringBuilder();
			for (int i = 0; i < values.Length; i++)
			{
				if (i > 0)
				{
					builder.Append(", ");
				}

				builder.Append(values[i].ToString("F6", CultureInfo.InvariantCulture));
			}

			return builder.ToString();
		}

		private static string FormatMatrix(Matrix4x4 matrix)
		{
			return $"[{FormatRow(matrix, 0)}; {FormatRow(matrix, 1)}; {FormatRow(matrix, 2)}; {FormatRow(matrix, 3)}]";
		}

		private static string FormatRow(Matrix4x4 matrix, int row)
		{
			return $"{matrix[row, 0]:F6}, {matrix[row, 1]:F6}, {matrix[row, 2]:F6}, {matrix[row, 3]:F6}";
		}

		private readonly struct Matrix3x3
		{
			public static Matrix3x3 identity => new Matrix3x3(new Vector3(1f, 0f, 0f), new Vector3(0f, 1f, 0f), new Vector3(0f, 0f, 1f));

			private readonly Vector3 _c0;
			private readonly Vector3 _c1;
			private readonly Vector3 _c2;

			public Matrix3x3(Vector4 c0, Vector4 c1, Vector4 c2)
			{
				_c0 = c0;
				_c1 = c1;
				_c2 = c2;
			}

			public Matrix3x3(Vector3 c0, Vector3 c1, Vector3 c2)
			{
				_c0 = c0;
				_c1 = c1;
				_c2 = c2;
			}

			public static Vector3 operator *(Matrix3x3 matrix, Vector3 vector)
			{
				return (matrix._c0 * vector.x) + (matrix._c1 * vector.y) + (matrix._c2 * vector.z);
			}

			public static Matrix3x3 operator -(Matrix3x3 lhs, Matrix3x3 rhs)
			{
				return new Matrix3x3(lhs._c0 - rhs._c0, lhs._c1 - rhs._c1, lhs._c2 - rhs._c2);
			}
		}
	}
}
