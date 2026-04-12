using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class DiffDriveTwinController : MonoBehaviour
{
	public enum PointGoalPhase
	{
		Idle,
		RotateInPlace,
		Cruise,
		Brake,
		Arrived
	}

	public enum ControlMode
	{
		TargetPoint,   // 点到点
		TargetYaw      // 只对准角度
	}

	[System.Serializable]
	public struct DrivePredictionState
	{
		public Vector3 worldPosition;
		public Quaternion worldRotation;
		public ControlMode mode;
		public Vector3 targetPointWorld;
		public bool hasTargetPoint;
		public float targetYawDeg;
		public Vector3 targetDirectionWorld;
		public bool useVelocityCommandOverride;
		public float overrideLinearVelocityTarget;
		public float overrideAngularVelocityTarget;
		public float currentLinearVelocity;
		public float currentAngularVelocity;
		public float lastLeftVelocity;
		public float lastRightVelocity;
		public bool goalReachedLatched;
		public bool finalYawAlignmentActive;
		public PointGoalPhase pointGoalPhase;
	}

	[System.Serializable]
	public struct DrivePredictionStep
	{
		public DrivePredictionState nextState;
		public float targetLinearVelocity;
		public float targetAngularVelocity;
		public float commandedLinearVelocity;
		public float commandedAngularVelocity;
		public float leftVelocity;
		public float rightVelocity;
		public float leftAcceleration;
		public float rightAcceleration;
		public Vector3 predictedWorldPosition;
		public Quaternion predictedWorldRotation;
		public bool applyPositionCorrection;
		public Vector3 correctedWorldPosition;
		public bool applyRotationCorrection;
		public Quaternion correctedWorldRotation;
		public bool hardStopped;
		public bool arrived;
	}

	[Header("References")]
	public Rigidbody rb;

	[Header("Wheel Transforms (visual only)")]
	public Transform wheelLF;
	public Transform wheelLR;
	public Transform wheelRF;
	public Transform wheelRR;

	[Header("Geometry Auto-Detect")]
	public bool autoDetectGeometryOnStart = true;

	[Tooltip("左右方向轴：通常差速左右分布在 CarRoot 的本地 X 轴（右正），因此选 LocalX")]
	public LateralAxis lateralAxis = LateralAxis.LocalX;

	[Tooltip("轮子滚动轴：大多数圆柱轮子绕本地 X 或 Z 轴滚动，按你的模型选择")]
	public WheelRollAxis wheelRollAxis = WheelRollAxis.LocalX;

	public enum LateralAxis { LocalX, LocalZ }
	public enum WheelRollAxis { LocalX, LocalY, LocalZ }

	[Header("Computed Geometry (Read-Only)")]
	[SerializeField] private float trackWidth_b = 0.0f;   // m
	[SerializeField] private float wheelRadius_r = 0.0f;  // m

	public float TrackWidth => trackWidth_b;
	public float WheelRadius => wheelRadius_r;
	public bool HasValidDriveGeometry => trackWidth_b > 1e-4f && wheelRadius_r > 1e-4f;

	[Header("Control Mode")]
	public ControlMode mode = ControlMode.TargetPoint;

	[Header("Target Point Input (Mouse Click)")]
	public bool enableMouseClickToSetTargetPoint = true;
	public LayerMask groundMask = ~0; // default: everything
	public Vector3 targetPointWorld;
	public bool hasTargetPoint = false;

	[Header("Target Yaw Input (Degrees)")]
	public float targetYawDeg = 0f;
	public bool enableKeyToApplyTargetYaw = true;
	public KeyCode applyYawKey = KeyCode.Y; // 按Y：切换到角度模式并应用 targetYawDeg

	[Header("Target Direction Input")]
	[Tooltip("是否在到达目标点后使用目标方向对齐（优先于 targetYawDeg）")]
	public bool useTargetDirectionAtGoal = true;

	[Tooltip("世界坐标下的目标方向向量（只使用XZ平面）")]
	public Vector3 targetDirectionWorld = Vector3.forward;

	[Header("Speed / Acc Limits")]
	public float vMax = 0.8f;               // m/s  最大线速度
	public float aMax = 1.0f;               // m/s^2 最大线加速度（斜坡）
	public float wMax = 2.5f;               // rad/s 最大角速度
	public float alphaMax = 6.0f;           // rad/s^2 最大角加速度（斜坡）

	[Header("Go-to-Goal Gains")]
	public float kDist = 1.5f;              // 距离到速度的比例（点到点）
	public float kYaw = 4.0f;               // 航向误差到角速度的比例（P 控制）

	[Header("Behavior")]
	[Tooltip("航向误差超过该角度（度）先原地转向，v=0")]
	public float rotateInPlaceAngleDeg = 25f;

	[Tooltip("接近目标点时的停止距离阈值（米）")]
	public float posTolerance = 0.03f;

	[Tooltip("进入该距离后直接判定到位，并把底盘吸附到目标点，避免最后几厘米缓慢蹭行")]
	public float arrivalSnapDistance = 0.03f;

	[Tooltip("Brake phase starts when remaining distance falls below the current stopping distance plus this padding.")]
	public float brakingDistancePadding = 0.04f;

	[Tooltip("到点后小角速度直接清零阈值（rad/s）")]
	public float yawRateStopTolerance = 0.05f;

	[Tooltip("对准角度的停止阈值（度）")]
	public float yawToleranceDeg = 2.0f;

	[Tooltip("车体朝向修正（度）。若模型朝前与控制前向不一致，可设为180")]
	public float headingOffsetDeg = 0f;

	[Tooltip("到点后释放锁定的距离阈值（米），用于避免在阈值边缘抖动")]
	public float goalReleaseDistance = 0.15f;

	[Tooltip("到达目标点后，是否再对准 targetYawDeg（或朝向目标点方向）")]
	public bool alignYawAtGoal = false;

	[Tooltip("到达目标点后是否允许继续原地转向；关闭时会强制轮速为0")]
	public bool rotateAtGoal = false;

	[Tooltip("点到点到位后保持当前行进方向，不再额外转向对齐")]
	public bool preserveArrivalHeading = true;

	[Tooltip("点到点到位后把底盘 XZ 位置直接收敛到目标点，消除残余距离误差")]
	public bool snapPositionToGoalOnArrival = true;

	[Tooltip("Rotate-in-place exits when yaw error falls below this angle, avoiding chatter near the threshold.")]
	public float rotateExitAngleDeg = 10f;

	[Tooltip("Cruise phase keeps at least this heading scale, so the chassis advances decisively instead of crawling.")]
	public float cruiseHeadingFloor = 0.55f;

	[Tooltip("Brake phase keeps at least this heading scale while still correcting yaw.")]
	public float brakeHeadingFloor = 0.40f;

	[Tooltip("Brake phase keeps at least this speed until the chassis is very close to the goal.")]
	public float brakeMinSpeed = 0.12f;

	[Header("Final Yaw Alignment")]
	[Tooltip("到点后的最终对齐阶段使用更强的偏航增益，减少慢吞吞地挪角度")]
	public float finalYawGain = 7.5f;

	[Tooltip("到点后的最终对齐阶段允许的最小角速度（rad/s），避免快到位时轮子慢速空转")]
	public float finalYawMinRate = 0.35f;

	[Tooltip("到点后的最终对齐阶段角加速度放大倍数")]
	public float finalYawAccelerationScale = 2.0f;

	[Tooltip("偏航误差进入容差后，直接把车体朝向收敛到目标角，避免剩余角速度拖尾")]
	public bool snapYawToTargetWhenAligned = true;

	[Header("Twin Visuals")]
	public bool animateWheelRoll = true;

	[Header("Outputs (Wheel Commands)")]
	public float vLeft;   // m/s  左轮组线速度
	public float vRight;  // m/s  右轮组线速度
	public float aLeft;   // m/s^2 左轮组线加速度
	public float aRight;  // m/s^2 右轮组线加速度

	public float wLF, wLR, wRF, wRR;        // rad/s 轮角速度（用于下发/动画）
	public float alphaLF, alphaLR, alphaRF, alphaRR; // rad/s^2 轮角加速度

	// internal state (current commanded chassis velocities)
	private float vCmd = 0f;   // m/s
	private float wCmd = 0f;   // rad/s

	public float CurrentLinearVelocity => vCmd;
	public float CurrentAngularVelocity => wCmd;
	public float CurrentPlanarSpeedMeasured => rb != null ? new Vector3(rb.velocity.x, 0f, rb.velocity.z).magnitude : Mathf.Abs(vCmd);
	public float CurrentYawRateMeasured => rb != null ? Mathf.Abs(rb.angularVelocity.y) : Mathf.Abs(wCmd);

	private float lastVLeft = 0f;
	private float lastVRight = 0f;

	// wheel roll accumulators (visual)
	private float rollLF, rollLR, rollRF, rollRR;
	private bool goalReachedLatched = false;
	private bool finalYawAlignmentActive = false;
	private bool useVelocityCommandOverride = false;
	private float overrideLinearVelocityTarget = 0f;
	private float overrideAngularVelocityTarget = 0f;
	[SerializeField] private PointGoalPhase pointGoalPhase = PointGoalPhase.Idle;

	private struct PoseCorrection
	{
		public bool applyPosition;
		public Vector3 correctedWorldPosition;
		public bool applyRotation;
		public Quaternion correctedWorldRotation;
	}

	void Reset()
	{
		rb = GetComponent<Rigidbody>();
	}

	void Start()
	{
		if (rb == null) rb = GetComponent<Rigidbody>();

		if (rb != null)
		{
			// This controller expects dynamic Rigidbody so gravity can work.
			if (rb.isKinematic) rb.isKinematic = false;
			if (!rb.useGravity) rb.useGravity = true;
		}

		if (autoDetectGeometryOnStart)
		{
			DetectGeometry();
		}
	}

	void Update()
	{
		// 鼠标点击设置目标点
		if (enableMouseClickToSetTargetPoint && Input.GetMouseButtonDown(0))
		{
			if (TryPickGroundPoint(out Vector3 p))
			{
				SetTargetPointGoal(p);
			}
		}

		// 按键应用目标Yaw
		if (enableKeyToApplyTargetYaw && Input.GetKeyDown(applyYawKey))
		{
			SetTargetYawGoal(targetYawDeg);
		}
	}

	void FixedUpdate()
	{
		float dt = Time.fixedDeltaTime;

		if (trackWidth_b <= 1e-4f || wheelRadius_r <= 1e-4f)
		{
			// 几何取不到就不运行（避免除0）
			vCmd = 0f;
			wCmd = 0f;
			ApplyChassisMotion(0f, 0f, dt);
			UpdateWheelOutputs(0f, 0f, dt);
			return;
		}

		// 1) 根据模式计算“目标”底盘速度 vTarget / wTarget
		DrivePredictionState predictionState = CaptureDrivePredictionState();
		if (TrySimulateDrivePredictionStep(ref predictionState, dt, out DrivePredictionStep predictionStep))
		{
			if (predictionStep.hardStopped)
			{
				HardStopAtGoal();
			}
			else
			{
				ApplyChassisMotion(predictionStep.commandedLinearVelocity, predictionStep.commandedAngularVelocity, dt);
			}

			if (rb != null)
			{
				if (predictionStep.applyPositionCorrection)
				{
					rb.MovePosition(predictionStep.correctedWorldPosition);
				}

				if (predictionStep.applyRotationCorrection)
				{
					rb.MoveRotation(predictionStep.correctedWorldRotation);
				}
			}

			ApplyPredictedStateToController(predictionStep.nextState, predictionStep);
			UpdateWheelOutputs(vLeft, vRight, dt);
			return;
		}

		float vTarget = 0f;
		float wTarget = 0f;

		if (useVelocityCommandOverride)
		{
			vTarget = Mathf.Clamp(overrideLinearVelocityTarget, -vMax, vMax);
			wTarget = Mathf.Clamp(overrideAngularVelocityTarget, -wMax, wMax);
			finalYawAlignmentActive = false;
		}
		else if (mode == ControlMode.TargetPoint)
		{
			if (!hasTargetPoint)
			{
				// No goal: force-stop to avoid micro oscillation near final point.
				pointGoalPhase = goalReachedLatched ? PointGoalPhase.Arrived : PointGoalPhase.Idle;
				HardStopAtGoal();
				UpdateWheelOutputs(0f, 0f, dt);
				return;
			}
			else
			{
				ComputePointToPointTargets(out vTarget, out wTarget, dt);
			}
		}
		else // TargetYaw
		{
			ComputeYawOnlyTargets(out vTarget, out wTarget);
		}

		// 2) 速度/角速度做加速度限制（斜坡）
		vCmd = Ramp(vCmd, vTarget, aMax, dt);
		float angularAccelerationLimit = finalYawAlignmentActive
			? alphaMax * Mathf.Max(1f, finalYawAccelerationScale)
			: alphaMax;
		wCmd = Ramp(wCmd, wTarget, angularAccelerationLimit, dt);

		// 3) 由 (vCmd, wCmd) 解算左右轮线速度，再映射到四轮
		vLeft = vCmd - wCmd * (trackWidth_b * 0.5f);
		vRight = vCmd + wCmd * (trackWidth_b * 0.5f);

		// 线加速度（轮组）用差分得到（也可用 vTarget/wTarget 导出）
		aLeft = (vLeft - lastVLeft) / Mathf.Max(1e-5f, dt);
		aRight = (vRight - lastVRight) / Mathf.Max(1e-5f, dt);

		lastVLeft = vLeft;
		lastVRight = vRight;

		// 4) 更新孪生体底盘运动（先转向再走由上面控制策略自然实现）
		ApplyChassisMotion(vCmd, wCmd, dt);

		// 5) 更新轮子输出（角速度/角加速度）+ 可视化滚动动画
		UpdateWheelOutputs(vLeft, vRight, dt);
	}

	public DrivePredictionState CaptureDrivePredictionState()
	{
		return new DrivePredictionState
		{
			worldPosition = rb != null ? rb.position : transform.position,
			worldRotation = rb != null ? rb.rotation : transform.rotation,
			mode = mode,
			targetPointWorld = targetPointWorld,
			hasTargetPoint = hasTargetPoint,
			targetYawDeg = targetYawDeg,
			targetDirectionWorld = targetDirectionWorld,
			useVelocityCommandOverride = useVelocityCommandOverride,
			overrideLinearVelocityTarget = overrideLinearVelocityTarget,
			overrideAngularVelocityTarget = overrideAngularVelocityTarget,
			currentLinearVelocity = vCmd,
			currentAngularVelocity = wCmd,
			lastLeftVelocity = lastVLeft,
			lastRightVelocity = lastVRight,
			goalReachedLatched = goalReachedLatched,
			finalYawAlignmentActive = finalYawAlignmentActive,
			pointGoalPhase = pointGoalPhase
		};
	}

	public DrivePredictionState CreateDrivePreviewStateForTargetPoint(Vector3 point)
	{
		DrivePredictionState state = CaptureDrivePredictionState();
		PrepareDrivePredictionStateForTargetPoint(ref state, point);
		return state;
	}

	public DrivePredictionState CreateDrivePreviewStateForTargetYaw(float yawDegrees)
	{
		DrivePredictionState state = CaptureDrivePredictionState();
		PrepareDrivePredictionStateForTargetYaw(ref state, yawDegrees);
		return state;
	}

	public bool TrySimulateDrivePredictionStep(ref DrivePredictionState state, float dt, out DrivePredictionStep step)
	{
		step = new DrivePredictionStep
		{
			nextState = state,
			predictedWorldPosition = state.worldPosition,
			predictedWorldRotation = state.worldRotation,
			correctedWorldPosition = state.worldPosition,
			correctedWorldRotation = state.worldRotation
		};

		if (dt <= 1e-5f || !HasValidDriveGeometry)
		{
			return false;
		}

		float previousLeftVelocity = state.lastLeftVelocity;
		float previousRightVelocity = state.lastRightVelocity;
		EvaluatePredictionControlTargets(ref state, out float vTarget, out float wTarget, out PoseCorrection correction, out bool hardStop);
		ApplyPoseCorrectionToState(ref state, correction);

		if (hardStop)
		{
			state.currentLinearVelocity = 0f;
			state.currentAngularVelocity = 0f;
			state.lastLeftVelocity = 0f;
			state.lastRightVelocity = 0f;
			step.nextState = state;
			step.targetLinearVelocity = vTarget;
			step.targetAngularVelocity = wTarget;
			step.commandedLinearVelocity = 0f;
			step.commandedAngularVelocity = 0f;
			step.leftVelocity = 0f;
			step.rightVelocity = 0f;
			step.leftAcceleration = (0f - previousLeftVelocity) / Mathf.Max(1e-5f, dt);
			step.rightAcceleration = (0f - previousRightVelocity) / Mathf.Max(1e-5f, dt);
			step.predictedWorldPosition = state.worldPosition;
			step.predictedWorldRotation = state.worldRotation;
			step.applyPositionCorrection = correction.applyPosition;
			step.correctedWorldPosition = state.worldPosition;
			step.applyRotationCorrection = correction.applyRotation;
			step.correctedWorldRotation = state.worldRotation;
			step.hardStopped = true;
			step.arrived = true;
			return true;
		}

		state.currentLinearVelocity = Ramp(state.currentLinearVelocity, vTarget, aMax, dt);
		float angularAccelerationLimit = state.finalYawAlignmentActive
			? alphaMax * Mathf.Max(1f, finalYawAccelerationScale)
			: alphaMax;
		state.currentAngularVelocity = Ramp(state.currentAngularVelocity, wTarget, angularAccelerationLimit, dt);

		float leftVelocity = state.currentLinearVelocity - state.currentAngularVelocity * (trackWidth_b * 0.5f);
		float rightVelocity = state.currentLinearVelocity + state.currentAngularVelocity * (trackWidth_b * 0.5f);
		float leftAcceleration = (leftVelocity - previousLeftVelocity) / Mathf.Max(1e-5f, dt);
		float rightAcceleration = (rightVelocity - previousRightVelocity) / Mathf.Max(1e-5f, dt);
		state.lastLeftVelocity = leftVelocity;
		state.lastRightVelocity = rightVelocity;
		IntegratePredictedBasePose(ref state.worldPosition, ref state.worldRotation, state.currentLinearVelocity, state.currentAngularVelocity, dt);

		step.nextState = state;
		step.targetLinearVelocity = vTarget;
		step.targetAngularVelocity = wTarget;
		step.commandedLinearVelocity = state.currentLinearVelocity;
		step.commandedAngularVelocity = state.currentAngularVelocity;
		step.leftVelocity = leftVelocity;
		step.rightVelocity = rightVelocity;
		step.leftAcceleration = leftAcceleration;
		step.rightAcceleration = rightAcceleration;
		step.predictedWorldPosition = state.worldPosition;
		step.predictedWorldRotation = state.worldRotation;
		step.applyPositionCorrection = correction.applyPosition;
		step.correctedWorldPosition = correction.applyPosition ? correction.correctedWorldPosition : state.worldPosition;
		step.applyRotationCorrection = correction.applyRotation;
		step.correctedWorldRotation = correction.applyRotation ? correction.correctedWorldRotation : state.worldRotation;
		step.hardStopped = false;
		step.arrived = state.mode == ControlMode.TargetPoint
			? !state.hasTargetPoint && state.pointGoalPhase == PointGoalPhase.Arrived
			: false;
		return true;
	}

	private void ApplyPredictedStateToController(DrivePredictionState state, DrivePredictionStep step)
	{
		mode = state.mode;
		targetPointWorld = state.targetPointWorld;
		hasTargetPoint = state.hasTargetPoint;
		targetYawDeg = state.targetYawDeg;
		targetDirectionWorld = state.targetDirectionWorld;
		useVelocityCommandOverride = state.useVelocityCommandOverride;
		overrideLinearVelocityTarget = state.overrideLinearVelocityTarget;
		overrideAngularVelocityTarget = state.overrideAngularVelocityTarget;
		goalReachedLatched = state.goalReachedLatched;
		finalYawAlignmentActive = state.finalYawAlignmentActive;
		pointGoalPhase = state.pointGoalPhase;
		vCmd = step.commandedLinearVelocity;
		wCmd = step.commandedAngularVelocity;
		vLeft = step.leftVelocity;
		vRight = step.rightVelocity;
		aLeft = step.leftAcceleration;
		aRight = step.rightAcceleration;
		lastVLeft = state.lastLeftVelocity;
		lastVRight = state.lastRightVelocity;
	}

	private void PrepareDrivePredictionStateForTargetPoint(ref DrivePredictionState state, Vector3 point)
	{
		state.targetPointWorld = point;
		state.hasTargetPoint = true;
		state.goalReachedLatched = false;
		state.finalYawAlignmentActive = false;
		state.useVelocityCommandOverride = false;
		state.overrideLinearVelocityTarget = 0f;
		state.overrideAngularVelocityTarget = 0f;
		state.pointGoalPhase = PointGoalPhase.Idle;
		state.mode = ControlMode.TargetPoint;
	}

	private void PrepareDrivePredictionStateForTargetYaw(ref DrivePredictionState state, float yawDegrees)
	{
		state.targetYawDeg = yawDegrees;
		state.hasTargetPoint = false;
		state.goalReachedLatched = false;
		state.finalYawAlignmentActive = false;
		state.useVelocityCommandOverride = false;
		state.overrideLinearVelocityTarget = 0f;
		state.overrideAngularVelocityTarget = 0f;
		state.pointGoalPhase = PointGoalPhase.Idle;
		state.mode = ControlMode.TargetYaw;
	}

	// -------------------------
	// Control: Point-to-Point
	// -------------------------
	private void EvaluatePredictionControlTargets(
		ref DrivePredictionState state,
		out float vTarget,
		out float wTarget,
		out PoseCorrection correction,
		out bool hardStop)
	{
		correction = default;
		hardStop = false;
		vTarget = 0f;
		wTarget = 0f;

		if (state.useVelocityCommandOverride)
		{
			vTarget = Mathf.Clamp(state.overrideLinearVelocityTarget, -vMax, vMax);
			wTarget = Mathf.Clamp(state.overrideAngularVelocityTarget, -wMax, wMax);
			state.finalYawAlignmentActive = false;
			return;
		}

		if (state.mode == ControlMode.TargetPoint)
		{
			if (!state.hasTargetPoint)
			{
				state.pointGoalPhase = state.goalReachedLatched ? PointGoalPhase.Arrived : PointGoalPhase.Idle;
				hardStop = true;
				return;
			}

			ComputePredictionPointToPointTargets(ref state, out vTarget, out wTarget, out correction, out hardStop);
			return;
		}

		ComputePredictionYawOnlyTargets(ref state, out vTarget, out wTarget, out correction, out hardStop);
	}

	private void ComputePredictionPointToPointTargets(
		ref DrivePredictionState state,
		out float vTarget,
		out float wTarget,
		out PoseCorrection correction,
		out bool hardStop)
	{
		correction = default;
		hardStop = false;
		Vector3 toGoal = state.targetPointWorld - state.worldPosition;
		toGoal.y = 0f;

		float dist = toGoal.magnitude;
		float arrivalDistance = Mathf.Max(posTolerance, arrivalSnapDistance);
		float desiredYaw = Mathf.Atan2(toGoal.x, toGoal.z);
		float yaw = CurrentYawRad(state.worldRotation);
		float yawError = WrapPi(desiredYaw - yaw);
		float yawErrorDeg = Mathf.Abs(yawError) * Mathf.Rad2Deg;

		if (state.goalReachedLatched && dist <= Mathf.Max(goalReleaseDistance, arrivalDistance))
		{
			state.pointGoalPhase = PointGoalPhase.Arrived;
			vTarget = 0f;
			wTarget = 0f;
			return;
		}

		if (state.goalReachedLatched && dist > Mathf.Max(goalReleaseDistance, arrivalDistance))
		{
			state.goalReachedLatched = false;
		}

		if (dist <= arrivalDistance)
		{
			FinishPredictionPointGoal(ref state, out vTarget, out wTarget, out correction, out hardStop);
			return;
		}

		UpdatePredictionPointGoalPhase(ref state, dist, arrivalDistance, yawErrorDeg);
		wTarget = Mathf.Clamp(kYaw * yawError, -wMax, wMax);

		switch (state.pointGoalPhase)
		{
			case PointGoalPhase.RotateInPlace:
				vTarget = 0f;
				break;

			case PointGoalPhase.Cruise:
				vTarget = vMax * Mathf.Max(cruiseHeadingFloor, Mathf.Clamp01(Mathf.Cos(yawError)));
				break;

			case PointGoalPhase.Brake:
			{
				float remainingForBrake = Mathf.Max(0f, dist - arrivalDistance);
				float brakeEnvelope = Mathf.Sqrt(Mathf.Max(0f, 2f * aMax * remainingForBrake));
				float headingScale = Mathf.Max(brakeHeadingFloor, Mathf.Clamp01(Mathf.Cos(yawError)));
				vTarget = Mathf.Min(vMax, brakeEnvelope) * headingScale;
				if (remainingForBrake > 0.15f && headingScale > 0.25f)
				{
					vTarget = Mathf.Max(vTarget, brakeMinSpeed);
				}
				break;
			}

			case PointGoalPhase.Arrived:
				FinishPredictionPointGoal(ref state, out vTarget, out wTarget, out correction, out hardStop);
				return;

			default:
				vTarget = 0f;
				break;
		}
	}

	private void UpdatePredictionPointGoalPhase(ref DrivePredictionState state, float dist, float arrivalDistance, float yawErrorDeg)
	{
		float rotateEnter = Mathf.Max(rotateInPlaceAngleDeg, rotateExitAngleDeg);
		float rotateExit = Mathf.Min(rotateEnter - 1f, rotateExitAngleDeg);
		float stopDistance = (state.currentLinearVelocity * state.currentLinearVelocity) / (2f * Mathf.Max(aMax, 1e-4f));
		float brakeDistance = Mathf.Max(arrivalDistance + brakingDistancePadding, stopDistance + brakingDistancePadding);

		switch (state.pointGoalPhase)
		{
			case PointGoalPhase.Idle:
				state.pointGoalPhase = yawErrorDeg > rotateEnter ? PointGoalPhase.RotateInPlace : PointGoalPhase.Cruise;
				break;

			case PointGoalPhase.RotateInPlace:
				if (yawErrorDeg <= rotateExit)
				{
					state.pointGoalPhase = PointGoalPhase.Cruise;
				}
				break;

			case PointGoalPhase.Cruise:
				if (yawErrorDeg > rotateEnter)
				{
					state.pointGoalPhase = PointGoalPhase.RotateInPlace;
				}
				else if (dist <= brakeDistance)
				{
					state.pointGoalPhase = PointGoalPhase.Brake;
				}
				break;

			case PointGoalPhase.Brake:
				if (dist <= arrivalDistance)
				{
					state.pointGoalPhase = PointGoalPhase.Arrived;
				}
				break;

			case PointGoalPhase.Arrived:
				if (dist > Mathf.Max(goalReleaseDistance, arrivalDistance))
				{
					state.pointGoalPhase = PointGoalPhase.Idle;
				}
				break;
		}
	}

	private void FinishPredictionPointGoal(
		ref DrivePredictionState state,
		out float vTarget,
		out float wTarget,
		out PoseCorrection correction,
		out bool hardStop)
	{
		correction = default;
		hardStop = false;
		state.goalReachedLatched = true;
		state.pointGoalPhase = PointGoalPhase.Arrived;
		state.hasTargetPoint = false;
		state.mode = ControlMode.TargetPoint;
		state.finalYawAlignmentActive = false;

		if (!preserveArrivalHeading && alignYawAtGoal && rotateAtGoal)
		{
			state.mode = ControlMode.TargetYaw;
			state.finalYawAlignmentActive = true;
			vTarget = 0f;
			if (useTargetDirectionAtGoal)
			{
				state.targetYawDeg = DirectionToYawDegFromVector(state.targetDirectionWorld, state.targetYawDeg);
			}

			wTarget = Mathf.Clamp(ComputeYawRateToTargetYawFromRotation(state.worldRotation, state.targetYawDeg), -wMax, wMax);
			return;
		}

		if (snapPositionToGoalOnArrival)
		{
			Vector3 snappedPosition = state.worldPosition;
			snappedPosition.x = state.targetPointWorld.x;
			snappedPosition.z = state.targetPointWorld.z;
			correction.applyPosition = true;
			correction.correctedWorldPosition = snappedPosition;
		}

		vTarget = 0f;
		wTarget = 0f;
		hardStop = true;
	}

	void ComputePointToPointTargets(out float vTarget, out float wTarget, float dt)
	{
		Vector3 pos = rb.position;
		Vector3 toGoal = targetPointWorld - pos;
		toGoal.y = 0f;

		float dist = toGoal.magnitude;
		float arrivalDistance = Mathf.Max(posTolerance, arrivalSnapDistance);
		float desiredYaw = Mathf.Atan2(toGoal.x, toGoal.z);
		float yaw = CurrentYawRad();
		float eYaw = WrapPi(desiredYaw - yaw);
		float yawErrorDeg = Mathf.Abs(eYaw) * Mathf.Rad2Deg;

		// Arrival hysteresis: once reached, keep stop unless target is clearly far again.
		if (goalReachedLatched && dist <= Mathf.Max(goalReleaseDistance, arrivalDistance))
		{
			pointGoalPhase = PointGoalPhase.Arrived;
			vTarget = 0f;
			wTarget = 0f;
			return;
		}
		if (goalReachedLatched && dist > Mathf.Max(goalReleaseDistance, arrivalDistance))
		{
			goalReachedLatched = false;
		}

		if (dist <= arrivalDistance)
		{
			FinishPointGoal(out vTarget, out wTarget);
			return;
		}

		UpdatePointGoalPhase(dist, arrivalDistance, yawErrorDeg);
		wTarget = Mathf.Clamp(kYaw * eYaw, -wMax, wMax);

		switch (pointGoalPhase)
		{
			case PointGoalPhase.RotateInPlace:
				vTarget = 0f;
				break;

			case PointGoalPhase.Cruise:
			{
				float headingScale = Mathf.Max(cruiseHeadingFloor, Mathf.Clamp01(Mathf.Cos(eYaw)));
				vTarget = vMax * headingScale;
				break;
			}

			case PointGoalPhase.Brake:
			{
				float remainingForBrake = Mathf.Max(0f, dist - arrivalDistance);
				float brakeEnvelope = Mathf.Sqrt(Mathf.Max(0f, 2f * aMax * remainingForBrake));
				float headingScale = Mathf.Max(brakeHeadingFloor, Mathf.Clamp01(Mathf.Cos(eYaw)));
				vTarget = Mathf.Min(vMax, brakeEnvelope) * headingScale;
				if (remainingForBrake > 0.15f && headingScale > 0.25f)
				{
					vTarget = Mathf.Max(vTarget, brakeMinSpeed);
				}
				break;
			}

			case PointGoalPhase.Arrived:
				FinishPointGoal(out vTarget, out wTarget);
				return;

			default:
				vTarget = 0f;
				break;
		}
	}

	void UpdatePointGoalPhase(float dist, float arrivalDistance, float yawErrorDeg)
	{
		float rotateEnter = Mathf.Max(rotateInPlaceAngleDeg, rotateExitAngleDeg);
		float rotateExit = Mathf.Min(rotateEnter - 1f, rotateExitAngleDeg);
		float stopDistance = (vCmd * vCmd) / (2f * Mathf.Max(aMax, 1e-4f));
		float brakeDistance = Mathf.Max(arrivalDistance + brakingDistancePadding, stopDistance + brakingDistancePadding);

		switch (pointGoalPhase)
		{
			case PointGoalPhase.Idle:
				pointGoalPhase = yawErrorDeg > rotateEnter ? PointGoalPhase.RotateInPlace : PointGoalPhase.Cruise;
				break;

			case PointGoalPhase.RotateInPlace:
				if (yawErrorDeg <= rotateExit)
				{
					pointGoalPhase = PointGoalPhase.Cruise;
				}
				break;

			case PointGoalPhase.Cruise:
				if (yawErrorDeg > rotateEnter)
				{
					pointGoalPhase = PointGoalPhase.RotateInPlace;
				}
				else if (dist <= brakeDistance)
				{
					pointGoalPhase = PointGoalPhase.Brake;
				}
				break;

			case PointGoalPhase.Brake:
				if (dist <= arrivalDistance)
				{
					pointGoalPhase = PointGoalPhase.Arrived;
				}
				break;

			case PointGoalPhase.Arrived:
				if (dist > Mathf.Max(goalReleaseDistance, arrivalDistance))
				{
					pointGoalPhase = PointGoalPhase.Idle;
				}
				break;
		}
	}

	void FinishPointGoal(out float vTarget, out float wTarget)
	{
		goalReachedLatched = true;
		pointGoalPhase = PointGoalPhase.Arrived;
		hasTargetPoint = false;
		mode = ControlMode.TargetPoint;
		finalYawAlignmentActive = false;
		if (!preserveArrivalHeading && alignYawAtGoal && rotateAtGoal)
		{
			mode = ControlMode.TargetYaw;
			finalYawAlignmentActive = true;
			vTarget = 0f;

			if (useTargetDirectionAtGoal)
			{
				targetYawDeg = DirectionToYawDeg(targetDirectionWorld);
			}

			wTarget = Mathf.Clamp(ComputeYawRateToTargetYaw(targetYawDeg), -wMax, wMax);
			return;
		}

		HardStopAtGoal();
		vTarget = 0f;
		wTarget = 0f;
	}

	// -------------------------
	// Control: Yaw Only
	// -------------------------
	private void ComputePredictionYawOnlyTargets(
		ref DrivePredictionState state,
		out float vTarget,
		out float wTarget,
		out PoseCorrection correction,
		out bool hardStop)
	{
		correction = default;
		hardStop = false;
		vTarget = 0f;
		float yawErrorRad = WrapPi(TargetYawRad(state.targetYawDeg) - CurrentYawRad(state.worldRotation));
		float yawErrorDeg = Mathf.Abs(yawErrorRad) * Mathf.Rad2Deg;
		if (yawErrorDeg <= yawToleranceDeg)
		{
			if (snapYawToTargetWhenAligned)
			{
				correction.applyRotation = true;
				correction.correctedWorldRotation = Quaternion.Euler(0f, state.targetYawDeg - headingOffsetDeg, 0f);
			}

			state.finalYawAlignmentActive = false;
			wTarget = 0f;
			hardStop = true;
			return;
		}

		float yawGain = state.finalYawAlignmentActive ? Mathf.Max(kYaw, finalYawGain) : kYaw;
		wTarget = Mathf.Clamp(yawGain * yawErrorRad, -wMax, wMax);
		if (state.finalYawAlignmentActive && Mathf.Abs(wTarget) < finalYawMinRate)
		{
			wTarget = finalYawMinRate * Mathf.Sign(yawErrorRad);
		}
	}

	private static void ApplyPoseCorrectionToState(ref DrivePredictionState state, PoseCorrection correction)
	{
		if (correction.applyPosition)
		{
			state.worldPosition = correction.correctedWorldPosition;
		}

		if (correction.applyRotation)
		{
			state.worldRotation = correction.correctedWorldRotation;
		}
	}

	private static void IntegratePredictedBasePose(
		ref Vector3 worldPosition,
		ref Quaternion worldRotation,
		float linearVelocity,
		float angularVelocity,
		float dt)
	{
		if (dt <= 1e-5f)
		{
			return;
		}

		Quaternion planarRotation = Quaternion.Euler(0f, worldRotation.eulerAngles.y, 0f);
		Vector3 startPosition = worldPosition;
		Vector3 planarForward = planarRotation * Vector3.forward;
		planarForward.y = 0f;
		if (planarForward.sqrMagnitude <= 1e-6f)
		{
			planarForward = Vector3.forward;
		}
		else
		{
			planarForward.Normalize();
		}

		if (Mathf.Abs(angularVelocity) <= 1e-4f)
		{
			worldPosition = startPosition + planarForward * linearVelocity * dt;
			worldPosition.y = startPosition.y;
			worldRotation = planarRotation;
			return;
		}

		Vector3 planarRight = new Vector3(planarForward.z, 0f, -planarForward.x);
		float deltaYaw = angularVelocity * dt;
		float radius = linearVelocity / angularVelocity;
		Vector3 planarDisplacement =
			planarForward * (radius * Mathf.Sin(deltaYaw))
			+ planarRight * (radius * (1f - Mathf.Cos(deltaYaw)));
		worldPosition = startPosition + planarDisplacement;
		worldPosition.y = startPosition.y;
		worldRotation = planarRotation * Quaternion.Euler(0f, deltaYaw * Mathf.Rad2Deg, 0f);
	}

	private float ComputeYawRateToTargetYawFromRotation(Quaternion worldRotation, float yawDeg)
	{
		float yaw = CurrentYawRad(worldRotation);
		float yawT = TargetYawRad(yawDeg);
		float e = WrapPi(yawT - yaw);
		return kYaw * e;
	}

	private float CurrentYawRad(Quaternion worldRotation)
	{
		Vector3 forward = worldRotation * Vector3.forward;
		forward.y = 0f;
		if (forward.sqrMagnitude <= 1e-8f)
		{
			forward = Vector3.forward;
		}
		else
		{
			forward.Normalize();
		}

		return WrapPi(Mathf.Atan2(forward.x, forward.z) + headingOffsetDeg * Mathf.Deg2Rad);
	}

	private float DirectionToYawDegFromVector(Vector3 dirWorld, float fallbackYawDeg)
	{
		dirWorld.y = 0f;
		if (dirWorld.sqrMagnitude <= 1e-8f)
		{
			return fallbackYawDeg;
		}

		dirWorld.Normalize();
		return Mathf.Atan2(dirWorld.x, dirWorld.z) * Mathf.Rad2Deg;
	}

	void ComputeYawOnlyTargets(out float vTarget, out float wTarget)
	{
		vTarget = 0f;
		float yawErrorRad = WrapPi(TargetYawRad(targetYawDeg) - CurrentYawRad());
		float yawErrDeg = Mathf.Abs(yawErrorRad) * Mathf.Rad2Deg;
		if (yawErrDeg <= yawToleranceDeg)
		{
			if (snapYawToTargetWhenAligned)
			{
				SnapYawToTarget(targetYawDeg);
			}

			finalYawAlignmentActive = false;
			HardStopAtGoal();
			wTarget = 0f;
			return;
		}

		float yawGain = finalYawAlignmentActive ? Mathf.Max(kYaw, finalYawGain) : kYaw;
		wTarget = Mathf.Clamp(yawGain * yawErrorRad, -wMax, wMax);
		if (finalYawAlignmentActive && Mathf.Abs(wTarget) < finalYawMinRate)
		{
			wTarget = finalYawMinRate * Mathf.Sign(yawErrorRad);
		}
	}

	float ComputeYawRateToTargetYaw(float yawDeg)
	{
		float yaw = CurrentYawRad();
		float yawT = TargetYawRad(yawDeg);
		float e = WrapPi(yawT - yaw);
		return kYaw * e;
	}

	float CurrentYawRad()
	{
		Vector3 f = rb.rotation * Vector3.forward;
		f.y = 0f;
		f.Normalize();
		return WrapPi(Mathf.Atan2(f.x, f.z) + headingOffsetDeg * Mathf.Deg2Rad);
	}

	float TargetYawRad(float yawDeg)
	{
		return WrapPi(yawDeg * Mathf.Deg2Rad);
	}

	float DirectionToYawDeg(Vector3 dirWorld)
	{
		dirWorld.y = 0f;
		if (dirWorld.sqrMagnitude <= 1e-8f)
		{
			return targetYawDeg;
		}

		dirWorld.Normalize();
		return Mathf.Atan2(dirWorld.x, dirWorld.z) * Mathf.Rad2Deg;
	}

	// -------------------------
	// Apply Motion to Twin Chassis
	// -------------------------
	void ApplyChassisMotion(float v, float w, float dt)
	{
		if (rb == null) return;

		// Control only horizontal motion and keep physics-driven vertical speed (gravity/fall).
		Vector3 planarVel = transform.forward * v;
		Vector3 vel = rb.velocity;
		vel.x = planarVel.x;
		vel.z = planarVel.z;
		rb.velocity = vel;

		Quaternion newRot = rb.rotation * Quaternion.Euler(0f, w * Mathf.Rad2Deg * dt, 0f);
		rb.MoveRotation(newRot);
	}

	public void SetTargetPointGoal(Vector3 point)
	{
		targetPointWorld = point;
		hasTargetPoint = true;
		goalReachedLatched = false;
		finalYawAlignmentActive = false;
		useVelocityCommandOverride = false;
		overrideLinearVelocityTarget = 0f;
		overrideAngularVelocityTarget = 0f;
		pointGoalPhase = PointGoalPhase.Idle;
		mode = ControlMode.TargetPoint;
	}

	public void UpdateTrackingGoal(Vector3 point, bool keepGoalActive = true)
	{
		targetPointWorld = point;
		hasTargetPoint = keepGoalActive;
	}

	public void SetVelocityCommand(float linearVelocityTarget, float angularVelocityTarget, Vector3? trackingGoalPoint = null)
	{
		useVelocityCommandOverride = true;
		overrideLinearVelocityTarget = Mathf.Clamp(linearVelocityTarget, -vMax, vMax);
		overrideAngularVelocityTarget = Mathf.Clamp(angularVelocityTarget, -wMax, wMax);
		goalReachedLatched = false;
		finalYawAlignmentActive = false;
		pointGoalPhase = PointGoalPhase.Idle;
		mode = ControlMode.TargetPoint;
		if (trackingGoalPoint.HasValue)
		{
			targetPointWorld = trackingGoalPoint.Value;
			hasTargetPoint = true;
		}
	}

	public void ClearVelocityCommand(bool stopImmediately)
	{
		useVelocityCommandOverride = false;
		overrideLinearVelocityTarget = 0f;
		overrideAngularVelocityTarget = 0f;
		if (stopImmediately)
		{
			HardStopAtGoal();
		}
	}

	public void CompletePointGoal(Vector3 point)
	{
		targetPointWorld = point;
		hasTargetPoint = false;
		goalReachedLatched = true;
		finalYawAlignmentActive = false;
		pointGoalPhase = PointGoalPhase.Arrived;
		ClearVelocityCommand(true);
	}

	public bool TrySnapPositionToPoint(Vector3 point, float maxPlanarCorrectionMeters)
	{
		if (rb == null)
		{
			return false;
		}

		Vector3 current = rb.position;
		Vector3 delta = point - current;
		delta.y = 0f;
		float maxCorrection = Mathf.Max(0.001f, maxPlanarCorrectionMeters);
		if (delta.sqrMagnitude > maxCorrection * maxCorrection)
		{
			return false;
		}

		if (delta.sqrMagnitude <= 1e-8f)
		{
			return true;
		}

		Vector3 snappedPosition = current;
		snappedPosition.x = point.x;
		snappedPosition.z = point.z;
		rb.MovePosition(snappedPosition);
		Physics.SyncTransforms();
		return true;
	}

	public bool EnsureDriveGeometryReady()
	{
		if (HasValidDriveGeometry)
		{
			return true;
		}

		DetectGeometry();
		return HasValidDriveGeometry;
	}

	public void SetTargetYawGoal(float yawDegrees)
	{
		targetYawDeg = yawDegrees;
		hasTargetPoint = false;
		goalReachedLatched = false;
		finalYawAlignmentActive = false;
		useVelocityCommandOverride = false;
		overrideLinearVelocityTarget = 0f;
		overrideAngularVelocityTarget = 0f;
		pointGoalPhase = PointGoalPhase.Idle;
		mode = ControlMode.TargetYaw;
	}

	public void HardStopAtGoal()
	{
		useVelocityCommandOverride = false;
		overrideLinearVelocityTarget = 0f;
		overrideAngularVelocityTarget = 0f;
		vCmd = 0f;
		wCmd = 0f;
		vLeft = 0f;
		vRight = 0f;
		aLeft = 0f;
		aRight = 0f;
		lastVLeft = 0f;
		lastVRight = 0f;
		alphaLF = 0f;
		alphaLR = 0f;
		alphaRF = 0f;
		alphaRR = 0f;
		wLF = 0f;
		wLR = 0f;
		wRF = 0f;
		wRR = 0f;

		if (rb == null) return;

		Vector3 vel = rb.velocity;
		vel.x = 0f;
		vel.z = 0f;
		rb.velocity = vel;

		Vector3 angular = rb.angularVelocity;
		angular.y = 0f;
		rb.angularVelocity = angular;
	}

	void SnapYawToTarget(float yawDeg)
	{
		if (rb == null)
		{
			return;
		}

		float snappedYawDeg = yawDeg - headingOffsetDeg;
		Quaternion targetRotation = Quaternion.Euler(0f, snappedYawDeg, 0f);
		rb.MoveRotation(targetRotation);
	}

	void SnapPositionToTargetPoint()
	{
		if (rb == null)
		{
			return;
		}

		Vector3 snappedPosition = rb.position;
		snappedPosition.x = targetPointWorld.x;
		snappedPosition.z = targetPointWorld.z;
		rb.MovePosition(snappedPosition);
	}

	private static Vector3 ProjectXZ(Vector3 value)
	{
		return new Vector3(value.x, 0f, value.z);
	}

	// -------------------------
	// Wheel Outputs + Visual Roll
	// -------------------------
	void UpdateWheelOutputs(float vL, float vR, float dt)
	{
		// 四轮差速：左两轮同速，右两轮同速
		float wL = vL / wheelRadius_r; // rad/s
		float wR = vR / wheelRadius_r;

		// 角加速度：差分（用于下发或显示）
		// 这里用上一帧的轮角速度差分，为简洁直接。
		// 你也可用 aLeft/aRight 直接 / r 得到 alpha。
		alphaLF = (wL - wLF) / Mathf.Max(1e-5f, dt);
		alphaLR = (wL - wLR) / Mathf.Max(1e-5f, dt);
		alphaRF = (wR - wRF) / Mathf.Max(1e-5f, dt);
		alphaRR = (wR - wRR) / Mathf.Max(1e-5f, dt);

		wLF = wL; wLR = wL; wRF = wR; wRR = wR;

		// 轮组线加速度转角加速度（更“物理一致”）
		// （若你下位机更需要这个，请用它；上面差分 alpha 也可以）
		float alphaL_fromA = aLeft / wheelRadius_r;
		float alphaR_fromA = aRight / wheelRadius_r;

		// 你可以选择下发 alphaL_fromA / alphaR_fromA
		// 这里保留差分 alphaLF.. 作为输出展示

		if (!animateWheelRoll) return;

		// 滚动动画：roll += omega * dt
		rollLF += wL * dt;
		rollLR += wL * dt;
		rollRF += wR * dt;
		rollRR += wR * dt;

		ApplyWheelRoll(wheelLF, rollLF);
		ApplyWheelRoll(wheelLR, rollLR);
		ApplyWheelRoll(wheelRF, rollRF);
		ApplyWheelRoll(wheelRR, rollRR);
	}

	void ApplyWheelRoll(Transform wheel, float rollRad)
	{
		if (wheel == null) return;

		Quaternion q;
		float deg = rollRad * Mathf.Rad2Deg;

		switch (wheelRollAxis)
		{
			case WheelRollAxis.LocalX: q = Quaternion.Euler(deg, 0f, 0f); break;
			case WheelRollAxis.LocalY: q = Quaternion.Euler(0f, deg, 0f); break;
			case WheelRollAxis.LocalZ: q = Quaternion.Euler(0f, 0f, deg); break;
			default: q = Quaternion.Euler(deg, 0f, 0f); break;
		}

		// 注意：这里用 localRotation 覆盖滚动；如果你轮子还有造型旋转偏置，可改为叠加
		wheel.localRotation = q;
	}

	// -------------------------
	// Geometry Detection
	// -------------------------
	public void DetectGeometry()
	{
		// 轮距：用左右轮中心的本地坐标差
		Vector3 lf = LocalWheelPos(wheelLF);
		Vector3 lr = LocalWheelPos(wheelLR);
		Vector3 rf = LocalWheelPos(wheelRF);
		Vector3 rr = LocalWheelPos(wheelRR);

		float leftLat = 0f;
		float rightLat = 0f;

		int leftCount = 0, rightCount = 0;

		// 以本地横向坐标的正负来区分左右（健壮性更好）
		AccumulateSide(lf, ref leftLat, ref rightLat, ref leftCount, ref rightCount);
		AccumulateSide(lr, ref leftLat, ref rightLat, ref leftCount, ref rightCount);
		AccumulateSide(rf, ref leftLat, ref rightLat, ref leftCount, ref rightCount);
		AccumulateSide(rr, ref leftLat, ref rightLat, ref leftCount, ref rightCount);

		if (leftCount > 0) leftLat /= leftCount;
		if (rightCount > 0) rightLat /= rightCount;

		trackWidth_b = Mathf.Abs(rightLat - leftLat);

		// 轮半径：优先 Collider，其次 Mesh bounds
		wheelRadius_r = EstimateWheelRadius(wheelLF);
		if (wheelRadius_r <= 1e-4f) wheelRadius_r = EstimateWheelRadius(wheelRF);

		Debug.Log($"[DiffDrive] Detected trackWidth_b={trackWidth_b:F4} m, wheelRadius_r={wheelRadius_r:F4} m");
	}

	Vector3 LocalWheelPos(Transform w)
	{
		if (w == null) return Vector3.zero;
		return transform.InverseTransformPoint(w.position);
	}

	void AccumulateSide(Vector3 localPos, ref float leftSum, ref float rightSum, ref int leftCount, ref int rightCount)
	{
		float lat = (lateralAxis == LateralAxis.LocalX) ? localPos.x : localPos.z;
		if (lat < 0f) { leftSum += lat; leftCount++; }
		else { rightSum += lat; rightCount++; }
	}

	float EstimateWheelRadius(Transform wheel)
	{
		if (wheel == null) return 0f;

		// 1) Sphere/Capsule 取 radius
		var sc = wheel.GetComponent<SphereCollider>();
		if (sc != null) return sc.radius * MaxAbsScale(wheel);

		var cc = wheel.GetComponent<CapsuleCollider>();
		if (cc != null) return cc.radius * MaxAbsScale(wheel);

		// 2) Mesh bounds 推半径
		var mf = wheel.GetComponent<MeshFilter>();
		if (mf != null && mf.sharedMesh != null)
		{
			var ext = mf.sharedMesh.bounds.extents;
			Vector3 s = wheel.lossyScale;

			// 假设轮子“滚动轴”方向就是轮轴方向，半径取垂直于该轴的最大 extents
			float r = 0f;
			switch (wheelRollAxis)
			{
				case WheelRollAxis.LocalX:
					r = Mathf.Max(Mathf.Abs(ext.y * s.y), Mathf.Abs(ext.z * s.z));
					break;
				case WheelRollAxis.LocalY:
					r = Mathf.Max(Mathf.Abs(ext.x * s.x), Mathf.Abs(ext.z * s.z));
					break;
				case WheelRollAxis.LocalZ:
					r = Mathf.Max(Mathf.Abs(ext.x * s.x), Mathf.Abs(ext.y * s.y));
					break;
			}
			return r;
		}

		return 0f;
	}

	float MaxAbsScale(Transform t)
	{
		Vector3 s = t.lossyScale;
		return Mathf.Max(Mathf.Abs(s.x), Mathf.Abs(s.y), Mathf.Abs(s.z));
	}

	// -------------------------
	// Utils
	// -------------------------
	float Ramp(float current, float target, float rateLimit, float dt)
	{
		float maxDelta = Mathf.Max(0f, rateLimit) * dt;
		return Mathf.MoveTowards(current, target, maxDelta);
	}

	float WrapPi(float a)
	{
		while (a > Mathf.PI) a -= 2f * Mathf.PI;
		while (a < -Mathf.PI) a += 2f * Mathf.PI;
		return a;
	}

	bool TryPickGroundPoint(out Vector3 point)
	{
		point = Vector3.zero;
		var cam = Camera.main;
		if (cam == null) return false;

		Ray ray = cam.ScreenPointToRay(Input.mousePosition);
		if (Physics.Raycast(ray, out RaycastHit hit, 500f, groundMask, QueryTriggerInteraction.Ignore))
		{
			point = hit.point;
			return true;
		}
		return false;
	}
}
