using UnityEngine;
using System.Collections.Generic;

public class OneJointTrapezoidController : MonoBehaviour
{
	public ArticulationBody joint;
	[Header("Auto-Bind")]
	public string expectedJointName = "";

	[Header("Goal")]
	public float goalDeg = 0f;
	public float stopToleranceDeg = 0.1f;

	[Header("Limits")]
	public float vMaxDeg = 60f;
	public float aMaxDeg = 180f;

	[Header("Drive Gains")]
	public float stiffness = 50000f;
	public float damping = 4000f;
	public float forceLimit = 1000f;
	public bool wakeUpEachStep = true;

	private float qCmdDeg;
	private float vCmdDeg;
	private float lastGoalDeg;

	void Awake()
	{
		joint = ResolveJointWithDof(joint);
		if (joint == null)
		{
			Debug.LogError("[OneJointTrapezoidController] No ArticulationBody with DOF found. Assign a revolute joint body to 'joint'.");
			enabled = false;
			return;
		}

		qCmdDeg = joint.jointPosition[0] * Mathf.Rad2Deg;
		lastGoalDeg = goalDeg;

		var d = joint.xDrive;
		d.stiffness = stiffness;
		d.damping = damping;
		d.forceLimit = forceLimit;
		d.target = qCmdDeg;
		d.targetVelocity = 0f;
		joint.xDrive = d;
	}

	void FixedUpdate()
	{
		if (joint == null || joint.jointPosition.dofCount <= 0) return;

		float qActDeg = joint.jointPosition[0] * Mathf.Rad2Deg;

		// If target changes at runtime/Inspector, restart profile from measured state.
		if (!Mathf.Approximately(lastGoalDeg, goalDeg))
		{
			qCmdDeg = qActDeg;
			vCmdDeg = 0f;
			lastGoalDeg = goalDeg;
			joint.WakeUp();
		}

		float dt = Time.fixedDeltaTime;
		float e = goalDeg - qActDeg;

		if (Mathf.Abs(e) <= Mathf.Max(stopToleranceDeg, 1e-4f))
		{
			qCmdDeg = goalDeg;
			vCmdDeg = 0f;

			var stopDrive = joint.xDrive;
			stopDrive.stiffness = stiffness;
			stopDrive.damping = damping;
			stopDrive.forceLimit = forceLimit;
			stopDrive.target = qCmdDeg;
			stopDrive.targetVelocity = 0f;
			joint.xDrive = stopDrive;
			if (wakeUpEachStep) joint.WakeUp();
			return;
		}

		float dBrake = (vCmdDeg * vCmdDeg) / (2f * Mathf.Max(aMaxDeg, 1e-6f));
		float vTarget = Mathf.Sign(e) * vMaxDeg;
		if (Mathf.Abs(e) <= dBrake) vTarget = 0f;

		vCmdDeg = Mathf.MoveTowards(vCmdDeg, vTarget, aMaxDeg * dt);

		float qNext = qCmdDeg + vCmdDeg * dt;
		if (Mathf.Sign(goalDeg - qNext) != Mathf.Sign(e))
		{
			qNext = goalDeg;
			vCmdDeg = 0f;
		}

		qCmdDeg = qNext;

		var d = joint.xDrive;
		d.stiffness = stiffness;
		d.damping = damping;
		d.forceLimit = forceLimit;
		d.target = qCmdDeg;
		d.targetVelocity = vCmdDeg;
		joint.xDrive = d;
		if (wakeUpEachStep) joint.WakeUp();
	}

	ArticulationBody ResolveJointWithDof(ArticulationBody preferred)
	{
		if (HasDof(preferred)) return preferred;

		ArticulationBody self = GetComponent<ArticulationBody>();
		if (HasDof(self)) return self;

		ArticulationBody[] bodies = GetComponentsInChildren<ArticulationBody>(true);
		List<ArticulationBody> candidates = new List<ArticulationBody>();
		for (int i = 0; i < bodies.Length; i++)
		{
			if (HasDof(bodies[i]))
			{
				candidates.Add(bodies[i]);
			}
		}

		if (candidates.Count == 1)
		{
			Debug.Log($"[OneJointTrapezoidController] Auto-selected joint: {candidates[0].name}");
			return candidates[0];
		}

		if (!string.IsNullOrEmpty(expectedJointName))
		{
			for (int i = 0; i < candidates.Count; i++)
			{
				if (candidates[i].name == expectedJointName)
				{
					Debug.Log($"[OneJointTrapezoidController] Auto-selected expected joint: {candidates[i].name}");
					return candidates[i];
				}
			}
		}

		int controllerOrdinal = ExtractTrailingNumber(gameObject.name);
		if (controllerOrdinal > 0)
		{
			ArticulationBody ordinalMatch = null;
			for (int i = 0; i < candidates.Count; i++)
			{
				if (ExtractTrailingNumber(candidates[i].name) == controllerOrdinal)
				{
					if (ordinalMatch != null)
					{
						ordinalMatch = null;
						break;
					}
					ordinalMatch = candidates[i];
				}
			}

			if (ordinalMatch != null)
			{
				Debug.Log($"[OneJointTrapezoidController] Auto-selected joint by ordinal: {ordinalMatch.name}");
				return ordinalMatch;
			}
		}

		if (candidates.Count > 1)
		{
			string candidateNames = "";
			for (int i = 0; i < candidates.Count; i++)
			{
				candidateNames += i == 0 ? candidates[i].name : ", " + candidates[i].name;
			}

			Debug.LogError($"[OneJointTrapezoidController] Ambiguous auto-bind on '{gameObject.name}'. Found {candidates.Count} DOF joints under this object: {candidateNames}. Assign 'joint' or set 'expectedJointName' (e.g. Link_05).");
		}

		return null;
	}

	int ExtractTrailingNumber(string value)
	{
		if (string.IsNullOrEmpty(value))
		{
			return -1;
		}

		int number = 0;
		int factor = 1;
		bool foundDigit = false;
		for (int i = value.Length - 1; i >= 0; i--)
		{
			char c = value[i];
			if (c >= '0' && c <= '9')
			{
				number += (c - '0') * factor;
				factor *= 10;
				foundDigit = true;
			}
			else if (foundDigit)
			{
				break;
			}
		}

		return foundDigit ? number : -1;
	}

	bool HasDof(ArticulationBody body)
	{
		return body != null && body.jointPosition.dofCount > 0;
	}
}
