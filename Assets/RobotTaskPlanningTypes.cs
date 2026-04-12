using System;
using System.Collections.Generic;
using UnityEngine;

namespace RobotSimulation
{
	[Serializable]
	public enum RobotPlanningStage
	{
		None,
		TargetCheck,
		CurrentBaseReachabilityCheck,
		DockingSearch,
		BasePlanning,
		BaseShadowValidation,
		BaseExecution,
		BaseSettling,
		ArmPlanning,
		ArmShadowValidation,
		ArmExecution,
		LocalReplan,
		Completed,
		Failed
	}

	[Serializable]
	public class SafetyGateRequestSettings
	{
		public bool enabled = true;
		public float commLatencySeconds = 0.04f;
		public float computeBudgetSeconds = 0.02f;
		public float minLookaheadSeconds = 0.08f;
		public float baseInflationMeters = 0.02f;
		public float armInflationMeters = 0.015f;
		public bool throttleOnRisk = true;
		public float queueLeadTimeSeconds = 0f;
		public int maxRecoveryReplans = 3;
		public bool dynamicBaseLockWhenEeWithinTolerance = true;
		public int decelFramesBeforeStop = 2;
		public bool enableStartupCommandTrace = false;
		public int startupPreviewMaxItems = 8;
	}

	[Serializable]
	public class RobotPlanRequest
	{
		public Vector3 baseTargetWorldPosition;
		public float baseTargetYawDeg;
		public Vector3 armTargetWorldPosition;
		public bool autoResolveBaseDockingPose = true;
		public float eePositionToleranceMeters = 0.03f;
		public bool requireBaseMove = true;
		public bool requireArmMove = true;
		public bool allowReplan = true;
		public float planningTimeoutSeconds = 10f;
		public SafetyGateRequestSettings safetyGate = new SafetyGateRequestSettings();
	}

	[Serializable]
	public class RobotPlanJointSample
	{
		public float timeSeconds;
		public float[] jointAnglesDeg = new float[6];
		public float clearance;
		public float singularityPenalty;
	}

	[Serializable]
	public class ShadowValidationResult
	{
		public bool passed = true;
		public bool collisionDetected;
		public bool singularityRisk;
		public int failedSampleIndex = -1;
		public string message = string.Empty;
	}

	[Serializable]
	public class RobotPlanResult
	{
		public bool accepted;
		public bool success;
		public bool dockingPoseFound;
		public bool baseMoveRequired;
		public bool armReachableFromCurrentBase;
		public bool armReachabilityIsLoosePrecheck;
		public int coarseCandidateCount;
		public int fineCandidateCount;
		public int ikSolveCount;
		public int basePathCheckCount;
		public string dockingFailureCategory = string.Empty;
		public RobotPlanningStage failedAtStage = RobotPlanningStage.None;
		public string failureReason = string.Empty;
		public List<Vector3> baseWaypoints = new List<Vector3>();
		public List<RobotPlanJointSample> armTrajectorySamples = new List<RobotPlanJointSample>();
		public Vector3 resolvedBaseStopWorldPosition;
		public float resolvedBaseStopYawDeg;
		public string dockingSummary = string.Empty;
		public bool shadowValidationPassed;
		public int replanCount;
		public int safetyGateInterceptCount;
		public float safetyGateMinLeadTimeMs = float.PositiveInfinity;
		public float safetyGateMinPredictedClearanceMeters = float.PositiveInfinity;
		public string safetyGateLastDecision = string.Empty;
		public string safetyGateLastReason = string.Empty;
		public int safetyGateQueueFlushCount;
		public float safetyGateLastDeltaTSeconds;
		public int safetyGateRecoveryAttemptCount;
		public long safetyGateLastBlockSequenceId;
		public int startupCommandCount;
		public List<SafetyGateStartupPreviewItem> startupCommandPreview = new List<SafetyGateStartupPreviewItem>();
		public float effectiveEePositionToleranceMeters;
		public string summary = string.Empty;
	}

	[Serializable]
	public class SafetyGateStartupPreviewItem
	{
		public long sequenceId;
		public string kind = string.Empty;
		public float plannedTimeSeconds;
		public float leadTimeSeconds;
		public float throttleRatio = 1f;
		public string fromSummary = string.Empty;
		public string toSummary = string.Empty;
	}
}
