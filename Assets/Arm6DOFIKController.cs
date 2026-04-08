using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace RobotSimulation
{
	/// <summary>
	/// Inverse kinematics controller using the URDF-driven kinematics model.
	/// </summary>
	public class Arm6DOFIKController : MonoBehaviour
	{
		public static Arm6DOFIKController Instance { get; private set; }

		[Header("References")]
		public Arm6DOFFKController armController;

		[Header("IK Settings")]
		public int maxIterations = 120;
		public float tolerance = 0.01f;
		public float damping = 1.0f;
		public float maxDeltaDegPerIteration = 8f;
		public float dlsLambda = 0.02f;
		public bool useMeasuredJointState = true;
		public bool resyncMeasuredStateEachIteration = false;

		[Header("Joint Limits")]
		public bool enforceJointLimits = true;

		[Header("Status")]
		[SerializeField] private bool _isSolving;
		[SerializeField] private bool _isMoveInProgress;
		[SerializeField] private bool _lastSolveSuccess;
		[SerializeField] private float _lastSolveError = float.MaxValue;
		[SerializeField] private int _lastSolveIterations;
		[SerializeField] private Vector3 _targetWorldPosition;
		[SerializeField] private Vector3 _targetBasePosition;
		[SerializeField] private float _activeMoveSpeedScale = 1f;

		private float[] _currentAngles = new float[6];
		private Vector2[] _jointLimits;
		private Coroutine _solveRoutine;
		private Coroutine _moveWaitRoutine;
		private ArmMoveResult _lastMoveResult = new ArmMoveResult();
		private ArmCollisionMonitor _collisionMonitor;
		private ArmCollisionGuardResult _lastCollisionGuardResult = new ArmCollisionGuardResult();
		private bool _solveBlockedByCollisionGuard;
		private readonly ArmMotionPlanner _motionPlanner = new ArmMotionPlanner();

		public bool IsSolving => _isSolving;
		public bool IsMoveInProgress => _isMoveInProgress;
		public bool LastSolveSuccess => _lastSolveSuccess;
		public float LastSolveError => _lastSolveError;
		public int LastSolveIterations => _lastSolveIterations;
		public Vector3 TargetWorldPosition => _targetWorldPosition;
		public ArmMoveResult LastMoveResult => _lastMoveResult;
		public ArmCollisionGuardResult LastCollisionGuardResult => _lastCollisionGuardResult;

		private void Awake()
		{
			if (Instance == null)
			{
				Instance = this;
			}
		}

		private void Start()
		{
			RefreshReferences();
		}

		public bool TryStartMoveToWorldPosition(Vector3 worldPosition)
		{
			return MoveToPosition(worldPosition);
		}

		public bool MoveToPosition(Vector3 targetWorldPosition)
		{
			return MoveToPosition(targetWorldPosition, Quaternion.identity, false);
		}

		public bool MoveToPosition(Vector3 targetWorldPosition, Quaternion targetRotation, bool withRotation = false)
		{
			StopActiveCoroutines();
			ArmMoveRequest request = new ArmMoveRequest
			{
				worldPosition = targetWorldPosition,
				positionToleranceMeters = tolerance,
				stableFixedFrames = 1,
				timeoutSeconds = Mathf.Max(1f, maxIterations * Time.fixedDeltaTime * 2f),
				speedScale = 1f
			};
			return MoveToWorldPositionAndWait(request, null) != null;
		}

		public Coroutine MoveToWorldPositionAndWait(ArmMoveRequest request, Action<ArmMoveResult> onComplete = null)
		{
			if (_moveWaitRoutine != null)
			{
				StopCoroutine(_moveWaitRoutine);
			}

			_moveWaitRoutine = StartCoroutine(MoveToWorldPositionAndWaitCoroutine(request, onComplete));
			return _moveWaitRoutine;
		}

		public void StopCurrentMove(bool emergencyStop = true)
		{
			StopActiveCoroutines();
			if (emergencyStop)
			{
				RefreshReferences();
				if (armController != null)
				{
					armController.EmergencyStop();
				}
			}

			Vector3 currentWorld = GetCurrentWorldEndEffectorPosition();
			_lastMoveResult = new ArmMoveResult
			{
				accepted = true,
				success = false,
				targetWorldPosition = _targetWorldPosition,
				finalWorldPosition = currentWorld,
				finalPositionError = Vector3.Distance(currentWorld, _targetWorldPosition),
				iterations = _lastSolveIterations,
				collisionGuardResult = _lastCollisionGuardResult,
				summary = "Stopped by user."
			};
		}

		public bool IsPositionReachable(Vector3 worldPosition)
		{
			RefreshReferences();
			return armController != null && armController.IsPositionReachable(worldPosition);
		}

		public string GetReachabilityInfo(Vector3 worldPosition)
		{
			RefreshReferences();
			if (armController == null || !armController.KinematicsReady)
			{
				return "Arm controller not initialized";
			}

			Vector3 targetBase = armController.WorldToBasePosition(worldPosition);
			float distance = targetBase.magnitude;
			float minReach = armController.GetMinReach();
			float maxReach = armController.GetMaxReach();
			bool reachable = armController.TryGetReachability(worldPosition, tolerance, out string reason);
			float outerMargin = maxReach - distance;
			float innerMargin = distance - minReach;

			string info = "=== Reachability Info ===\n";
			info += $"Target World Position: ({worldPosition.x:F3}, {worldPosition.y:F3}, {worldPosition.z:F3})\n";
			info += $"Target Base Position: ({targetBase.x:F3}, {targetBase.y:F3}, {targetBase.z:F3})\n";
			info += $"Distance from base: {distance:F3}m\n";
			info += $"Min reach (dead-zone radius): {minReach:F3}m\n";
			info += $"Max reach: {maxReach:F3}m\n";
			info += reachable
				? $"Status: REACHABLE (inner margin: {innerMargin:F3}m, outer margin: {outerMargin:F3}m)\n"
				: distance < minReach
					? $"Status: OUT OF REACH - INSIDE DEAD ZONE (shortfall: {minReach - distance:F3}m)\n"
					: $"Status: OUT OF REACH - OUTSIDE OUTER SPHERE (shortfall: {distance - maxReach:F3}m)\n";
			info += $"Reason: {reason}\n";
			return info;
		}

		public Coroutine MoveToPositionAsync(Vector3 targetPosition, Action<bool> onComplete = null)
		{
			return MoveToWorldPositionAndWait(new ArmMoveRequest
			{
				worldPosition = targetPosition,
				positionToleranceMeters = tolerance,
				stableFixedFrames = 1,
				timeoutSeconds = Mathf.Max(1f, maxIterations * Time.fixedDeltaTime * 2f)
			}, result => onComplete?.Invoke(result.success));
		}

		public Coroutine RunRegressionTest(Action<string> onComplete = null)
		{
			return StartCoroutine(RunRegressionTestCoroutine(onComplete));
		}

		private bool TryBeginSolve(Vector3 targetWorldPosition, out ArmMoveResult immediateResult)
		{
			immediateResult = new ArmMoveResult
			{
				targetWorldPosition = targetWorldPosition
			};

			RefreshReferences();
			RefreshCollisionMonitor();
			if (armController == null || !armController.IsInitialized || !armController.KinematicsReady)
			{
				Debug.LogError("[Arm6DOF IK] Arm controller or kinematics model is not initialized.");
				immediateResult.summary = "Arm controller is not initialized.";
				return false;
			}

			if (!armController.TryGetReachability(targetWorldPosition, tolerance, out string reachabilityReason))
			{
				Vector3 currentWorld = GetCurrentWorldEndEffectorPosition();
				immediateResult.accepted = false;
				immediateResult.unreachable = true;
				immediateResult.finalWorldPosition = currentWorld;
				immediateResult.finalPositionError = Vector3.Distance(currentWorld, targetWorldPosition);
				immediateResult.summary = reachabilityReason;
				_lastMoveResult = immediateResult;
				_lastSolveSuccess = false;
				_lastSolveError = immediateResult.finalPositionError;
				Debug.LogError($"[Arm6DOF IK] Unreachable target: {immediateResult.summary}");
				return false;
			}

			_targetWorldPosition = targetWorldPosition;
			_targetBasePosition = armController.WorldToBasePosition(targetWorldPosition);
			if (useMeasuredJointState || _currentAngles == null || _currentAngles.Length < 6)
			{
				_currentAngles = (float[])armController.CurrentJointAngles.Clone();
			}
			_jointLimits = armController.jointLimits;
			_lastSolveSuccess = false;
			_lastSolveIterations = 0;
			_lastSolveError = Vector3.Distance(armController.ModelEndEffectorPose.position, _targetBasePosition);
			_solveBlockedByCollisionGuard = false;
			_lastCollisionGuardResult = new ArmCollisionGuardResult { allowed = true };

			StartSolveRoutine();
			immediateResult.accepted = true;
			immediateResult.summary = "IK solve started.";
			return true;
		}

		private bool TryBeginPlannedMove(
			Vector3 targetWorldPosition,
			out ArmMoveResult immediateResult,
			out List<RobotPlanJointSample> plannedSamples)
		{
			immediateResult = new ArmMoveResult
			{
				targetWorldPosition = targetWorldPosition
			};
			plannedSamples = null;

			RefreshReferences();
			RefreshCollisionMonitor();
			if (armController == null || !armController.IsInitialized || !armController.KinematicsReady)
			{
				immediateResult.summary = "Arm controller is not initialized.";
				_lastMoveResult = immediateResult;
				_lastSolveSuccess = false;
				_lastSolveError = float.MaxValue;
				return false;
			}

			_targetWorldPosition = targetWorldPosition;
			_targetBasePosition = armController.WorldToBasePosition(targetWorldPosition);
			_jointLimits = armController.jointLimits;
			_lastSolveIterations = 0;
			_lastSolveSuccess = false;
			_solveBlockedByCollisionGuard = false;
			_lastCollisionGuardResult = new ArmCollisionGuardResult { allowed = true };
			ConfigureMotionPlannerFromIkSettings();

			if (!_motionPlanner.TryPlanToWorldPosition(
				armController,
				targetWorldPosition,
				null,
				out plannedSamples,
				out string failureReason,
				null,
				tolerance))
			{
				Vector3 currentWorld = GetCurrentWorldEndEffectorPosition();
				immediateResult.accepted = false;
				immediateResult.success = false;
				immediateResult.unreachable = failureReason != null
					&& (failureReason.IndexOf("outside", StringComparison.OrdinalIgnoreCase) >= 0
						|| failureReason.IndexOf("dead zone", StringComparison.OrdinalIgnoreCase) >= 0
						|| failureReason.IndexOf("工作空间", StringComparison.OrdinalIgnoreCase) >= 0);
				immediateResult.blockedByCollisionGuard = failureReason != null
					&& (failureReason.IndexOf("collision guard", StringComparison.OrdinalIgnoreCase) >= 0
						|| failureReason.IndexOf("碰撞守卫", StringComparison.OrdinalIgnoreCase) >= 0
						|| failureReason.IndexOf("forbidden", StringComparison.OrdinalIgnoreCase) >= 0);
				immediateResult.finalWorldPosition = currentWorld;
				immediateResult.finalPositionError = Vector3.Distance(currentWorld, targetWorldPosition);
				immediateResult.summary = string.IsNullOrEmpty(failureReason)
					? "Arm planner failed to produce a valid trajectory."
					: failureReason;
				_lastMoveResult = immediateResult;
				_lastSolveError = immediateResult.finalPositionError;
				return false;
			}

			_lastSolveIterations = plannedSamples != null ? plannedSamples.Count : 0;
			_lastSolveError = Vector3.Distance(armController.EndEffectorWorldPosition, targetWorldPosition);
			_isSolving = false;
			immediateResult.accepted = true;
			immediateResult.summary = "Arm motion plan generated.";
			return true;
		}

		private void ConfigureMotionPlannerFromIkSettings()
		{
			_motionPlanner.settings.maxIterations = Mathf.Max(16, maxIterations);
			_motionPlanner.settings.toleranceMeters = Mathf.Max(0.001f, tolerance);
			_motionPlanner.settings.dlsLambda = Mathf.Max(1e-4f, dlsLambda);
			_motionPlanner.settings.maxDeltaDegPerIteration = Mathf.Max(0.5f, maxDeltaDegPerIteration);
			_motionPlanner.settings.sampleTimeStepSeconds = Mathf.Max(0.02f, 0.08f / Mathf.Max(0.1f, _activeMoveSpeedScale));
		}

		private void StartSolveRoutine()
		{
			if (_solveRoutine != null)
			{
				StopCoroutine(_solveRoutine);
				_solveRoutine = null;
			}

			_solveRoutine = StartCoroutine(SolveIKCoroutine(_targetBasePosition));
		}

		private IEnumerator SolveIKCoroutine(Vector3 targetBasePosition)
		{
			_isSolving = true;
			bool converged = false;

			for (int iteration = 0; iteration < maxIterations; iteration++)
			{
				_lastSolveIterations = iteration + 1;

				if (resyncMeasuredStateEachIteration)
				{
					SyncCurrentAnglesFromMeasuredState();
				}

				Pose currentPose = armController.ForwardPoe(_currentAngles);
				Vector3 currentPosition = currentPose.position;
				Vector3 positionError = targetBasePosition - currentPosition;
				float errorMagnitude = positionError.magnitude;
				if (errorMagnitude <= tolerance)
				{
					converged = true;
					_lastSolveError = errorMagnitude;
					break;
				}

				float[,] jacobian = armController.ComputeGeometricJacobian(_currentAngles);
				float[] deltaThetaRad = CalculateDampedLeastSquaresStep(jacobian, positionError);

				for (int i = 0; i < 6; i++)
				{
					float maxStepDeg = maxDeltaDegPerIteration * Mathf.Max(0.1f, _activeMoveSpeedScale);
					float deltaDeg = Mathf.Clamp(deltaThetaRad[i] * Mathf.Rad2Deg, -maxStepDeg, maxStepDeg);
					_currentAngles[i] += deltaDeg;
					if (enforceJointLimits && _jointLimits != null && i < _jointLimits.Length)
					{
						_currentAngles[i] = Mathf.Clamp(_currentAngles[i], _jointLimits[i].x, _jointLimits[i].y);
					}
				}

				float[] stepStartAngles = armController.CaptureMeasuredJointAngles();
				float[] stepTargetAngles = (float[])_currentAngles.Clone();
				if (armController.EvaluateMotionCollision(stepStartAngles, stepTargetAngles, out ArmCollisionGuardResult guardResult))
				{
					_lastCollisionGuardResult = guardResult;
					_solveBlockedByCollisionGuard = true;
					_lastSolveSuccess = false;
					_lastSolveError = Vector3.Distance(armController.ForwardPoe(stepStartAngles).position, targetBasePosition);
					_lastMoveResult = new ArmMoveResult
					{
						accepted = true,
						success = false,
						targetWorldPosition = _targetWorldPosition,
						finalWorldPosition = GetCurrentWorldEndEffectorPosition(),
						finalPositionError = Vector3.Distance(GetCurrentWorldEndEffectorPosition(), _targetWorldPosition),
						iterations = _lastSolveIterations,
						blockedByCollisionGuard = true,
						collisionGuardResult = guardResult,
						summary = guardResult.message
					};
					armController.HoldCurrentPose();
					_isSolving = false;
					_solveRoutine = null;
					if (!_isMoveInProgress)
					{
						Debug.LogWarning($"[Arm6DOF IK] {guardResult.message}");
					}
					yield break;
				}

				armController.ApplyAllJointTargetsRaw(stepTargetAngles);
				yield return new WaitForFixedUpdate();
			}

			_lastSolveError = Vector3.Distance(armController.ForwardPoe(_currentAngles).position, targetBasePosition);
			_lastSolveSuccess = converged || _lastSolveError <= tolerance;
			_isSolving = false;
			_solveRoutine = null;

			if (!_lastSolveSuccess && !_isMoveInProgress)
			{
				Debug.LogWarning($"[Arm6DOF IK] Solve ended without convergence. Error={_lastSolveError:F4}m after {_lastSolveIterations} iterations.");
			}
		}

		private IEnumerator MoveToWorldPositionAndWaitCoroutine(ArmMoveRequest request, Action<ArmMoveResult> onComplete)
		{
			ArmMoveRequest safeRequest = request ?? new ArmMoveRequest();
			if (safeRequest.stableFixedFrames <= 0)
			{
				safeRequest.stableFixedFrames = 1;
			}

			if (safeRequest.timeoutSeconds <= 0f)
			{
				safeRequest.timeoutSeconds = 5f;
			}

			safeRequest.speedScale = Mathf.Clamp(safeRequest.speedScale, 0.1f, 3f);
			_activeMoveSpeedScale = safeRequest.speedScale;

			_isMoveInProgress = true;
			StopSolveRoutine();
			if (!TryBeginPlannedMove(safeRequest.worldPosition, out ArmMoveResult startResult, out List<RobotPlanJointSample> plannedSamples))
			{
				_isMoveInProgress = false;
				startResult.timedOut = false;
				startResult.success = false;
				_moveWaitRoutine = null;
				_activeMoveSpeedScale = 1f;
				_lastMoveResult = startResult;
				onComplete?.Invoke(startResult);
				yield break;
			}

			int stableFrames = 0;
			float elapsed = 0f;
			bool arrivalLockActive = false;
			float trajectoryElapsed = 0f;
			int sampleIndex = 0;
			float[] lastCommandedAngles = armController != null ? armController.CaptureMeasuredJointAngles() : null;
			ArmMoveResult result = new ArmMoveResult
			{
				accepted = true,
				targetWorldPosition = safeRequest.worldPosition
			};

			while (elapsed < safeRequest.timeoutSeconds)
			{
				yield return new WaitForFixedUpdate();
				elapsed += Time.fixedDeltaTime;
				trajectoryElapsed += Time.fixedDeltaTime * Mathf.Max(0.1f, safeRequest.speedScale);

				if (armController != null && plannedSamples != null)
				{
					while (sampleIndex < plannedSamples.Count && trajectoryElapsed + 1e-4f >= plannedSamples[sampleIndex].timeSeconds)
					{
						float[] nextAngles = plannedSamples[sampleIndex].jointAnglesDeg;
						if (armController.EvaluateMotionCollision(lastCommandedAngles, nextAngles, out ArmCollisionGuardResult guardResult))
						{
							_lastCollisionGuardResult = guardResult;
							_solveBlockedByCollisionGuard = true;
							result.success = false;
							result.blockedByCollisionGuard = true;
							result.collisionGuardResult = guardResult;
							result.finalWorldPosition = GetCurrentWorldEndEffectorPosition();
							result.finalPositionError = Vector3.Distance(result.finalWorldPosition, safeRequest.worldPosition);
							result.iterations = sampleIndex + 1;
							result.summary = string.IsNullOrEmpty(guardResult?.message)
								? "Arm motion blocked by forbidden collision guard."
								: guardResult.message;
							armController.HoldCurrentPose();
							break;
						}

						armController.ApplyAllJointTargetsRaw(nextAngles);
						lastCommandedAngles = (float[])nextAngles.Clone();
						_lastSolveIterations = sampleIndex + 1;
						sampleIndex++;
					}
				}

				Vector3 currentWorldPosition = GetCurrentWorldEndEffectorPosition();
				float error = Vector3.Distance(currentWorldPosition, safeRequest.worldPosition);
				if (_collisionMonitor != null && _collisionMonitor.EvaluateCollisionState())
				{
					result.collided = true;
					result.success = false;
					result.finalWorldPosition = currentWorldPosition;
					result.finalPositionError = error;
					result.iterations = _lastSolveIterations;
					result.collisionMessage = _collisionMonitor.ActiveCollisionMessage;
					result.summary = string.IsNullOrEmpty(result.collisionMessage)
						? "Arm collision detected."
						: result.collisionMessage;
					break;
				}

				if (_solveBlockedByCollisionGuard)
				{
					result.success = false;
					result.blockedByCollisionGuard = true;
					result.collisionGuardResult = _lastCollisionGuardResult;
					result.finalWorldPosition = currentWorldPosition;
					result.finalPositionError = error;
					result.iterations = _lastSolveIterations;
					result.summary = string.IsNullOrEmpty(_lastCollisionGuardResult?.message)
						? "Arm motion blocked by forbidden collision guard."
						: _lastCollisionGuardResult.message;
					break;
				}

				if (error <= safeRequest.positionToleranceMeters)
				{
					if (!arrivalLockActive && armController != null)
					{
						arrivalLockActive = true;
						armController.HoldCurrentPose();
					}

					stableFrames++;
					if (stableFrames >= safeRequest.stableFixedFrames)
					{
						result.success = true;
						result.finalWorldPosition = currentWorldPosition;
						result.finalPositionError = error;
						result.iterations = Mathf.Max(_lastSolveIterations, plannedSamples != null ? plannedSamples.Count : 0);
						result.summary = $"Reached target in {elapsed:F2}s.";
						_lastSolveSuccess = true;
						_lastSolveError = error;
						if (armController != null)
						{
							armController.HoldCurrentPose();
						}
						break;
					}
				}
				else
				{
					stableFrames = 0;
				}
			}

			if (!result.success)
			{
				_lastSolveSuccess = false;
				if (!result.collided)
				{
					if (!result.blockedByCollisionGuard)
					{
						result.finalWorldPosition = GetCurrentWorldEndEffectorPosition();
						result.finalPositionError = Vector3.Distance(result.finalWorldPosition, safeRequest.worldPosition);
						result.iterations = Mathf.Max(_lastSolveIterations, plannedSamples != null ? plannedSamples.Count : 0);
						result.timedOut = true;
						result.summary = "Timed out while waiting for the end effector to settle at the target position.";
					}
				}

				_lastSolveError = result.finalPositionError;
			}

			StopSolveRoutine();
			_activeMoveSpeedScale = 1f;
			_lastMoveResult = result;
			_isMoveInProgress = false;
			_moveWaitRoutine = null;
			onComplete?.Invoke(result);
		}

		private float[] CalculateDampedLeastSquaresStep(float[,] jacobian, Vector3 positionError)
		{
			float[,] linearJacobian = new float[3, 6];
			for (int c = 0; c < 6; c++)
			{
				linearJacobian[0, c] = jacobian[3, c];
				linearJacobian[1, c] = jacobian[4, c];
				linearJacobian[2, c] = jacobian[5, c];
			}

			float[,] jjt = new float[3, 3];
			for (int r = 0; r < 3; r++)
			{
				for (int c = 0; c < 3; c++)
				{
					float sum = 0f;
					for (int k = 0; k < 6; k++)
					{
						sum += linearJacobian[r, k] * linearJacobian[c, k];
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
				float[] fallback = new float[6];
				for (int i = 0; i < 6; i++)
				{
					float value = 0f;
					value += linearJacobian[0, i] * positionError.x;
					value += linearJacobian[1, i] * positionError.y;
					value += linearJacobian[2, i] * positionError.z;
					fallback[i] = value * damping;
				}

				return fallback;
			}

			float[] weightedError = new float[3];
			weightedError[0] = inv[0, 0] * positionError.x + inv[0, 1] * positionError.y + inv[0, 2] * positionError.z;
			weightedError[1] = inv[1, 0] * positionError.x + inv[1, 1] * positionError.y + inv[1, 2] * positionError.z;
			weightedError[2] = inv[2, 0] * positionError.x + inv[2, 1] * positionError.y + inv[2, 2] * positionError.z;

			float[] deltaTheta = new float[6];
			for (int i = 0; i < 6; i++)
			{
				float value = 0f;
				value += linearJacobian[0, i] * weightedError[0];
				value += linearJacobian[1, i] * weightedError[1];
				value += linearJacobian[2, i] * weightedError[2];
				deltaTheta[i] = value * damping;
			}

			return deltaTheta;
		}

		private static bool TryInvert3x3(float[,] matrix, out float[,] inverse)
		{
			inverse = new float[3, 3];
			float a = matrix[0, 0];
			float b = matrix[0, 1];
			float c = matrix[0, 2];
			float d = matrix[1, 0];
			float e = matrix[1, 1];
			float f = matrix[1, 2];
			float g = matrix[2, 0];
			float h = matrix[2, 1];
			float i = matrix[2, 2];

			float A = (e * i) - (f * h);
			float B = -((d * i) - (f * g));
			float C = (d * h) - (e * g);
			float D = -((b * i) - (c * h));
			float E = (a * i) - (c * g);
			float F = -((a * h) - (b * g));
			float G = (b * f) - (c * e);
			float H = -((a * f) - (c * d));
			float I = (a * e) - (b * d);

			float determinant = (a * A) + (b * B) + (c * C);
			if (Mathf.Abs(determinant) < 1e-8f)
			{
				return false;
			}

			float invDet = 1f / determinant;
			inverse[0, 0] = A * invDet; inverse[0, 1] = D * invDet; inverse[0, 2] = G * invDet;
			inverse[1, 0] = B * invDet; inverse[1, 1] = E * invDet; inverse[1, 2] = H * invDet;
			inverse[2, 0] = C * invDet; inverse[2, 1] = F * invDet; inverse[2, 2] = I * invDet;
			return true;
		}

		private void SyncCurrentAnglesFromMeasuredState()
		{
			if (armController == null || armController.CurrentJointAngles == null)
			{
				return;
			}

			for (int i = 0; i < 6; i++)
			{
				_currentAngles[i] = armController.CurrentJointAngles[i];
			}
		}

		private Vector3 GetCurrentWorldEndEffectorPosition()
		{
			RefreshReferences();
			if (armController == null)
			{
				return Vector3.zero;
			}

			return armController.EndEffectorWorldPosition;
		}

		private IEnumerator RunRegressionTestCoroutine(Action<string> onComplete)
		{
			RefreshReferences();
			if (armController == null || !armController.IsInitialized || !armController.KinematicsReady)
			{
				string failed = "IK regression aborted: FK controller or kinematics model is not initialized.";
				Debug.LogError($"[Arm6DOF IK] {failed}");
				onComplete?.Invoke(failed);
				yield break;
			}

			Vector3 seed = GetCurrentWorldEndEffectorPosition();
			Vector3[] targets = new Vector3[]
			{
				seed + new Vector3(0.03f, 0.02f, -0.02f),
				seed + new Vector3(-0.03f, 0.02f, 0.02f),
				seed + new Vector3(0.02f, -0.02f, 0.02f)
			};

			int passed = 0;
			float worstError = 0f;
			for (int i = 0; i < targets.Length; i++)
			{
				bool done = false;
				ArmMoveResult result = null;
				MoveToWorldPositionAndWait(new ArmMoveRequest
				{
					worldPosition = targets[i],
					positionToleranceMeters = tolerance,
					stableFixedFrames = 3,
					timeoutSeconds = 4f
				}, moveResult =>
				{
					result = moveResult;
					done = true;
				});

				while (!done)
				{
					yield return null;
				}

				if (result != null && result.success)
				{
					passed++;
				}

				float caseError = result != null ? result.finalPositionError : float.MaxValue;
				worstError = Mathf.Max(worstError, caseError);
				Debug.Log($"[Arm6DOF IK][Regression] Case {i + 1}/{targets.Length}: success={result?.success}, error={caseError:F4}m");
			}

			string summary = $"IK regression finished: {passed}/{targets.Length} passed, worstError={worstError:F4}m";
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

		private void RefreshReferences()
		{
			if (armController == null)
			{
				armController = Arm6DOFFKController.Instance;
			}

			if (armController == null)
			{
				armController = FindObjectOfType<Arm6DOFFKController>();
			}
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

		private void StopSolveRoutine()
		{
			if (_solveRoutine != null)
			{
				StopCoroutine(_solveRoutine);
				_solveRoutine = null;
			}

			_isSolving = false;
		}

		private void StopActiveCoroutines()
		{
			StopSolveRoutine();

			if (_moveWaitRoutine != null)
			{
				StopCoroutine(_moveWaitRoutine);
				_moveWaitRoutine = null;
			}

			_isSolving = false;
			_isMoveInProgress = false;
		}
	}
}
