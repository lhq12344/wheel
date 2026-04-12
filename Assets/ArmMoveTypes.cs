using System;
using System.Collections.Generic;
using UnityEngine;

namespace RobotSimulation
{
	[Serializable]
	public enum ManualPreviewSessionKind
	{
		None,
		BaseYaw,
		BasePoint,
		ArmWorldMove
	}

	[Serializable]
	public enum ManualPreviewSessionState
	{
		Idle,
		PreviewPending,
		Executing,
		Completed,
		Cancelled
	}

	[Serializable]
	public class ArmCollisionGuardResult
	{
		public bool allowed = true;
		public bool blockedByForbiddenCollision;
		public string message;
		public string armColliderName;
		public string forbiddenColliderName;
		public int sampleIndex = -1;
	}

	[Serializable]
	public class ArmMoveRequest
	{
		public Vector3 worldPosition;
		public float positionToleranceMeters = 0.01f;
		public int stableFixedFrames = 5;
		public float timeoutSeconds = 5f;
		public float speedScale = 1f;
	}

	[Serializable]
	public class ArmMoveResult
	{
		public bool accepted;
		public bool success;
		public bool timedOut;
		public bool unreachable;
		public bool collided;
		public Vector3 targetWorldPosition;
		public Vector3 finalWorldPosition;
		public float finalPositionError;
		public int iterations;
		public string collisionMessage;
		public bool blockedByCollisionGuard;
		public ArmCollisionGuardResult collisionGuardResult;
		public string summary;
	}

	[Serializable]
	public class ArmTrajectoryPreviewPlan
	{
		public ArmMoveRequest request = new ArmMoveRequest();
		public List<RobotPlanJointSample> samples = new List<RobotPlanJointSample>();
		public ArmMoveResult planningResult = new ArmMoveResult();
		public float totalDurationSeconds;
	}
}
