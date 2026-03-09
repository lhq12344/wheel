using UnityEngine;
using System.Collections;

namespace RobotSimulation
{
	/// <summary>
	/// Inverse Kinematics Controller for 6-DOF Arm.
	/// Iterative Jacobian-transpose solver running on physics steps.
	/// </summary>
	public class Arm6DOFIKController : MonoBehaviour
	{
		public static Arm6DOFIKController Instance { get; private set; }

		[Header("References")]
		public Arm6DOFFKController armController;

		[Header("IK Settings")]
		public int maxIterations = 40;
		public float tolerance = 0.01f;           // meters
		public float angleTolerance = 1f;         // degrees (reserved)
		public float damping = 0.5f;              // Jacobian transpose gain
		public float maxDeltaDegPerIteration = 5f;
		public float dlsLambda = 0.05f;           // Damped least-squares regularization
		public bool useMeasuredJointState = true; // Use articulation measured angles each iteration

		[Header("Joint Limits")]
		public bool enforceJointLimits = true;

		[Header("Status")]
		[SerializeField] private bool _isSolving = false;
		[SerializeField] private bool _lastSolveSuccess = false;
		[SerializeField] private float _lastSolveError = float.MaxValue;
		[SerializeField] private Vector3 _targetPosition;
		[SerializeField] private Quaternion _targetRotation = Quaternion.identity;

		public bool IsSolving => _isSolving;
		public bool LastSolveSuccess => _lastSolveSuccess;
		public float LastSolveError => _lastSolveError;
		public Vector3 TargetPosition => _targetPosition;

		private ArticulationBody[] _joints;
		private Transform[] _jointTransforms;
		private Transform _endEffector;
		private float[] _currentAngles = new float[6];
		private Vector2[] _jointLimits;
		private Coroutine _solveRoutine;

		void Awake()
		{
			if (Instance == null)
			{
				Instance = this;
			}
		}

		void Start()
		{
			RefreshReferences();
		}

		public bool MoveToPosition(Vector3 targetPosition)
		{
			return MoveToPosition(targetPosition, Quaternion.identity, false);
		}

		public bool MoveToPosition(Vector3 targetPosition, Quaternion targetRotation, bool withRotation = true)
		{
			RefreshReferences();

			if (armController == null || !armController.IsInitialized)
			{
				Debug.LogError("[Arm6DOF IK] Arm controller not initialized!");
				return false;
			}

			if (!IsPositionReachable(targetPosition))
			{
				float maxReach = armController.GetMaxReach();
				float distance = Vector3.Distance(armController.transform.position, targetPosition);
				_lastSolveSuccess = false;
				_lastSolveError = distance - maxReach;

				Debug.LogError("[Arm6DOF IK] ERROR: Target position is OUT OF REACH!");
				Debug.LogError($"  Target: ({targetPosition.x:F3}, {targetPosition.y:F3}, {targetPosition.z:F3})");
				Debug.LogError($"  Distance from base: {distance:F3}m");
				Debug.LogError($"  Max reach: {maxReach:F3}m");
				Debug.LogError($"  Shortfall: {_lastSolveError:F3}m");
				return false;
			}

			if (_isSolving && _solveRoutine != null)
			{
				StopCoroutine(_solveRoutine);
				_solveRoutine = null;
				_isSolving = false;
			}

			_targetPosition = targetPosition;
			_targetRotation = targetRotation;
			_currentAngles = (float[])armController.CurrentJointAngles.Clone();

			_lastSolveSuccess = false;
			_lastSolveError = CalculatePositionError(targetPosition);

			_solveRoutine = StartCoroutine(SolveIKCoroutine(targetPosition, targetRotation, withRotation));
			return true;
		}

		public bool IsPositionReachable(Vector3 position)
		{
			if (armController == null) return false;
			return armController.IsPositionReachable(position);
		}

		private IEnumerator SolveIKCoroutine(Vector3 targetPos, Quaternion targetRot, bool withRotation)
		{
			_isSolving = true;
			RefreshReferences();

			if (_jointTransforms == null || _jointTransforms.Length < 6)
			{
				_isSolving = false;
				_lastSolveSuccess = false;
				Debug.LogError("[Arm6DOF IK] Joint transforms are not ready. Please initialize FK controller joints.");
				yield break;
			}

			bool success = false;

			for (int iteration = 0; iteration < maxIterations; iteration++)
			{
				if (useMeasuredJointState)
				{
					SyncCurrentAnglesFromMeasuredState();
				}

				Vector3 current = GetEndEffectorPosition();
				float error = Vector3.Distance(current, targetPos);
				if (error < tolerance)
				{
					success = true;
					Debug.Log($"[Arm6DOF IK] Solved in {iteration + 1} iterations. Error: {error:F4}m");
					break;
				}

				Vector3 posError = targetPos - current;

				float[,] jacobian = CalculateJacobian();
				float[] deltaThetaRad = CalculateDampedLeastSquaresStep(jacobian, posError);

				for (int i = 0; i < 6; i++)
				{
					float deltaDeg = Mathf.Clamp(deltaThetaRad[i] * Mathf.Rad2Deg, -maxDeltaDegPerIteration, maxDeltaDegPerIteration);
					_currentAngles[i] += deltaDeg;

					if (enforceJointLimits && _jointLimits != null && i < _jointLimits.Length)
					{
						_currentAngles[i] = Mathf.Clamp(_currentAngles[i], _jointLimits[i].x, _jointLimits[i].y);
					}
				}

				armController.SetAllJointTargets(_currentAngles);
				yield return new WaitForFixedUpdate();
			}

			_lastSolveError = Vector3.Distance(GetEndEffectorPosition(), targetPos);
			_lastSolveSuccess = success && _lastSolveError <= tolerance;
			_isSolving = false;
			_solveRoutine = null;

			if (!_lastSolveSuccess)
			{
				Debug.LogWarning($"[Arm6DOF IK] Failed to converge. Final error: {_lastSolveError:F4}m");
			}
		}

		private float[,] CalculateJacobian()
		{
			float[,] jacobian = new float[3, 6];
			if (_jointTransforms == null) return jacobian;

			Vector3 endPos = GetEndEffectorPosition();

			for (int i = 0; i < 6; i++)
			{
				if (i >= _jointTransforms.Length || _jointTransforms[i] == null) continue;

				// This project drives articulation xDrive, so joint axis is local X.
				Vector3 jointAxis = _jointTransforms[i].right;
				Vector3 fromJointToEnd = endPos - _jointTransforms[i].position;
				Vector3 col = Vector3.Cross(jointAxis, fromJointToEnd);

				jacobian[0, i] = col.x;
				jacobian[1, i] = col.y;
				jacobian[2, i] = col.z;
			}

			return jacobian;
		}

		private float[] CalculateDampedLeastSquaresStep(float[,] jacobian, Vector3 posError)
		{
			// deltaTheta = J^T * (J*J^T + lambda^2*I)^-1 * error
			float[,] jjt = new float[3, 3];

			for (int r = 0; r < 3; r++)
			{
				for (int c = 0; c < 3; c++)
				{
					float sum = 0f;
					for (int k = 0; k < 6; k++)
					{
						sum += jacobian[r, k] * jacobian[c, k];
					}
					jjt[r, c] = sum;
				}
			}

			float lambda2 = Mathf.Max(1e-6f, dlsLambda * dlsLambda);
			jjt[0, 0] += lambda2;
			jjt[1, 1] += lambda2;
			jjt[2, 2] += lambda2;

			if (!TryInvert3x3(jjt, out float[,] inv))
			{
				// Fallback to Jacobian transpose if matrix inversion fails.
				float[] fallback = new float[6];
				for (int i = 0; i < 6; i++)
				{
					float v = 0f;
					for (int j = 0; j < 3; j++)
					{
						v += jacobian[j, i] * posError[j];
					}
					fallback[i] = v * damping;
				}
				return fallback;
			}

			float[] weightedError = new float[3];
			for (int r = 0; r < 3; r++)
			{
				weightedError[r] = inv[r, 0] * posError[0]
								 + inv[r, 1] * posError[1]
								 + inv[r, 2] * posError[2];
			}

			float[] deltaThetaRad = new float[6];
			for (int i = 0; i < 6; i++)
			{
				float v = 0f;
				for (int j = 0; j < 3; j++)
				{
					v += jacobian[j, i] * weightedError[j];
				}
				deltaThetaRad[i] = v * damping;
			}

			return deltaThetaRad;
		}

		private bool TryInvert3x3(float[,] m, out float[,] inv)
		{
			inv = new float[3, 3];

			float a = m[0, 0]; float b = m[0, 1]; float c = m[0, 2];
			float d = m[1, 0]; float e = m[1, 1]; float f = m[1, 2];
			float g = m[2, 0]; float h = m[2, 1]; float i = m[2, 2];

			float A = (e * i) - (f * h);
			float B = -((d * i) - (f * g));
			float C = (d * h) - (e * g);
			float D = -((b * i) - (c * h));
			float E = (a * i) - (c * g);
			float F = -((a * h) - (b * g));
			float G = (b * f) - (c * e);
			float H = -((a * f) - (c * d));
			float I = (a * e) - (b * d);

			float det = a * A + b * B + c * C;
			if (Mathf.Abs(det) < 1e-8f)
			{
				return false;
			}

			float invDet = 1f / det;
			inv[0, 0] = A * invDet; inv[0, 1] = D * invDet; inv[0, 2] = G * invDet;
			inv[1, 0] = B * invDet; inv[1, 1] = E * invDet; inv[1, 2] = H * invDet;
			inv[2, 0] = C * invDet; inv[2, 1] = F * invDet; inv[2, 2] = I * invDet;

			return true;
		}

		private void SyncCurrentAnglesFromMeasuredState()
		{
			if (armController == null || armController.CurrentJointAngles == null) return;
			if (_currentAngles == null || _currentAngles.Length < 6)
			{
				_currentAngles = new float[6];
			}

			for (int i = 0; i < 6; i++)
			{
				_currentAngles[i] = armController.CurrentJointAngles[i];
			}
		}

		private Vector3 GetEndEffectorPosition()
		{
			if (_endEffector != null) return _endEffector.position;
			if (_jointTransforms != null && _jointTransforms.Length > 0 && _jointTransforms[5] != null)
			{
				return _jointTransforms[5].position;
			}
			return armController != null ? armController.EndEffectorPosition : Vector3.zero;
		}

		private float CalculatePositionError(Vector3 target)
		{
			return Vector3.Distance(GetEndEffectorPosition(), target);
		}

		public string GetReachabilityInfo(Vector3 position)
		{
			if (armController == null) return "Arm controller not initialized";

			float maxReach = armController.GetMaxReach();
			float distance = Vector3.Distance(armController.transform.position, position);
			float shortfall = maxReach - distance;

			string info = "=== Reachability Info ===\n";
			info += $"Target Position: ({position.x:F3}, {position.y:F3}, {position.z:F3})\n";
			info += $"Distance from base: {distance:F3}m\n";
			info += $"Max reach: {maxReach:F3}m\n";

			if (shortfall > 0)
			{
				info += $"Status: REACHABLE (margin: {shortfall:F3}m)\n";
			}
			else
			{
				info += $"Status: OUT OF REACH (shortfall: {Mathf.Abs(shortfall):F3}m)\n";
			}

			return info;
		}

		public Coroutine MoveToPositionAsync(Vector3 targetPosition, System.Action<bool> onComplete = null)
		{
			return StartCoroutine(MoveToPositionCoroutine(targetPosition, Quaternion.identity, false, onComplete));
		}

		public Coroutine RunRegressionTest(System.Action<string> onComplete = null)
		{
			return StartCoroutine(RunRegressionTestCoroutine(onComplete));
		}

		private IEnumerator RunRegressionTestCoroutine(System.Action<string> onComplete)
		{
			RefreshReferences();
			if (armController == null || !armController.IsInitialized)
			{
				string failed = "IK regression aborted: FK controller is not initialized.";
				Debug.LogError($"[Arm6DOF IK] {failed}");
				onComplete?.Invoke(failed);
				yield break;
			}

			Vector3 seed = GetEndEffectorPosition();
			Vector3[] targets = new Vector3[]
			{
				seed + new Vector3(0.04f, 0.00f, 0.00f),
				seed + new Vector3(-0.04f, 0.00f, 0.00f),
				seed + new Vector3(0.00f, 0.04f, 0.00f),
				seed + new Vector3(0.00f, -0.04f, 0.00f),
				seed + new Vector3(0.00f, 0.00f, 0.04f),
				seed + new Vector3(0.00f, 0.00f, -0.04f),
				seed + new Vector3(0.03f, 0.02f, -0.02f),
				seed + new Vector3(-0.03f, 0.02f, 0.02f)
			};

			int passed = 0;
			float worstError = 0f;
			for (int i = 0; i < targets.Length; i++)
			{
				bool done = false;
				bool success = false;

				MoveToPositionAsync(targets[i], result =>
				{
					success = result;
					done = true;
				});

				while (!done)
				{
					yield return null;
				}

				if (success)
				{
					passed++;
				}

				worstError = Mathf.Max(worstError, _lastSolveError);
				Debug.Log($"[Arm6DOF IK][Regression] Case {i + 1}/{targets.Length}: success={success}, error={_lastSolveError:F4}m, target=({targets[i].x:F3},{targets[i].y:F3},{targets[i].z:F3})");
			}

			string summary = $"IK regression finished: {passed}/{targets.Length} passed, worstError={worstError:F4}m, tolerance={tolerance:F4}m";
			if (passed == targets.Length)
			{
				Debug.Log($"[Arm6DOF IK][Regression] {summary}");
			}
			else
			{
				Debug.LogWarning($"[Arm6DOF IK][Regression] {summary}");
			}

			onComplete?.Invoke(summary);
		}

		private IEnumerator MoveToPositionCoroutine(Vector3 targetPos, Quaternion targetRot, bool withRotation, System.Action<bool> onComplete)
		{
			bool accepted = MoveToPosition(targetPos, targetRot, withRotation);
			if (!accepted)
			{
				onComplete?.Invoke(false);
				yield break;
			}

			while (_isSolving)
			{
				yield return null;
			}

			onComplete?.Invoke(_lastSolveSuccess);
		}

		private void RefreshReferences()
		{
			if (armController == null)
			{
				armController = Arm6DOFFKController.Instance;
			}

			if (armController == null) return;

			if (_joints == null || _joints.Length < 6)
			{
				_joints = armController.joints;
			}

			if (_jointTransforms == null || _jointTransforms.Length < 6)
			{
				_jointTransforms = armController.jointTransforms;
			}

			if (_jointLimits == null || _jointLimits.Length < 6)
			{
				_jointLimits = armController.jointLimits;
			}

			if (_endEffector == null)
			{
				_endEffector = armController.endEffector;
			}
		}
	}
}
