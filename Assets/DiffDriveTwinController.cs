using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class DiffDriveTwinController : MonoBehaviour
{
	public enum ControlMode
	{
		TargetPoint,   // 点到点
		TargetYaw      // 只对准角度
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
	public float posTolerance = 0.05f;

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

	private float lastVLeft = 0f;
	private float lastVRight = 0f;

	// wheel roll accumulators (visual)
	private float rollLF, rollLR, rollRF, rollRR;
	private bool goalReachedLatched = false;

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
				targetPointWorld = p;
				hasTargetPoint = true;
				goalReachedLatched = false;
				mode = ControlMode.TargetPoint;
			}
		}

		// 按键应用目标Yaw
		if (enableKeyToApplyTargetYaw && Input.GetKeyDown(applyYawKey))
		{
			mode = ControlMode.TargetYaw;
			// targetYawDeg 直接用 Inspector 中的值
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
		float vTarget = 0f;
		float wTarget = 0f;

		if (mode == ControlMode.TargetPoint)
		{
			if (!hasTargetPoint)
			{
				// No goal: force-stop to avoid micro oscillation near final point.
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
		wCmd = Ramp(wCmd, wTarget, alphaMax, dt);

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

	// -------------------------
	// Control: Point-to-Point
	// -------------------------
	void ComputePointToPointTargets(out float vTarget, out float wTarget, float dt)
	{
		Vector3 pos = rb.position;
		Vector3 toGoal = targetPointWorld - pos;
		toGoal.y = 0f;

		float dist = toGoal.magnitude;

		// Arrival hysteresis: once reached, keep stop unless target is clearly far again.
		if (goalReachedLatched && dist <= Mathf.Max(goalReleaseDistance, posTolerance))
		{
			vTarget = 0f;
			wTarget = 0f;
			return;
		}
		if (goalReachedLatched && dist > Mathf.Max(goalReleaseDistance, posTolerance))
		{
			goalReachedLatched = false;
		}

		// 到点停止
		if (dist <= posTolerance)
		{
			goalReachedLatched = true;
			hasTargetPoint = false;

			if (alignYawAtGoal && rotateAtGoal)
			{
				// 到点后对准 targetYawDeg
				mode = ControlMode.TargetYaw;
				vTarget = 0f;

				if (useTargetDirectionAtGoal)
				{
					targetYawDeg = DirectionToYawDeg(targetDirectionWorld);
				}

				wTarget = ComputeYawRateToTargetYaw(targetYawDeg);
				wTarget = Mathf.Clamp(wTarget, -wMax, wMax);
				return;
			}

			mode = ControlMode.TargetPoint;
			HardStopAtGoal();

			vTarget = 0f;
			wTarget = 0f;
			return;
		}

		// 目标航向：朝向目标点
		float desiredYaw = Mathf.Atan2(toGoal.x, toGoal.z); // Unity: yaw around Y, forward is +Z
		float yaw = CurrentYawRad();

		float eYaw = WrapPi(desiredYaw - yaw);

		// 角速度：P 控制 + 限幅
		wTarget = Mathf.Clamp(kYaw * eYaw, -wMax, wMax);

		// Near goal, decay yaw command to avoid in-place slip/spin on threshold boundary.
		float nearGoalScale = Mathf.Clamp01(dist / Mathf.Max(2f * posTolerance, 1e-3f));
		wTarget *= nearGoalScale;

		// 线速度：距离比例 + 限幅 + 近点刹车约束（避免冲过头）
		float vRaw = Mathf.Clamp(kDist * dist, 0f, vMax);

		// “能停住”的速度上限：v <= sqrt(2*aMax*dist)
		float vStop = Mathf.Sqrt(Mathf.Max(0f, 2f * aMax * dist));
		vRaw = Mathf.Min(vRaw, vStop);

		// 大角度误差先原地转向
		float rotateDeg = Mathf.Abs(eYaw) * Mathf.Rad2Deg;
		if (rotateDeg > rotateInPlaceAngleDeg)
		{
			vTarget = 0f;
		}
		else
		{
			// 航向误差越大，前进越慢（避免画大弧线）
			float headingScale = Mathf.Clamp01(Mathf.Cos(eYaw));
			vTarget = vRaw * headingScale;
		}
	}

	// -------------------------
	// Control: Yaw Only
	// -------------------------
	void ComputeYawOnlyTargets(out float vTarget, out float wTarget)
	{
		vTarget = 0f;
		wTarget = Mathf.Clamp(ComputeYawRateToTargetYaw(targetYawDeg), -wMax, wMax);

		// 接近目标角度就停
		float yawErrDeg = Mathf.Abs(WrapPi(TargetYawRad(targetYawDeg) - CurrentYawRad())) * Mathf.Rad2Deg;
		if (yawErrDeg <= yawToleranceDeg)
		{
			wTarget = 0f;
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

	public void HardStopAtGoal()
	{
		vCmd = 0f;
		wCmd = 0f;
		vLeft = 0f;
		vRight = 0f;
		aLeft = 0f;
		aRight = 0f;
		lastVLeft = 0f;
		lastVRight = 0f;

		if (rb == null) return;

		Vector3 vel = rb.velocity;
		vel.x = 0f;
		vel.z = 0f;
		rb.velocity = vel;

		Vector3 w = rb.angularVelocity;
		if (Mathf.Abs(w.y) <= yawRateStopTolerance)
		{
			w.y = 0f;
			rb.angularVelocity = w;
		}
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
