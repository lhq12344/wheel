using System;
using System.Collections.Generic;
using UnityEngine;

namespace RobotSimulation
{
	public enum SafetyGateDecisionType
	{
		Allow,
		Throttle,
		Block
	}

	public enum SafetyGateBlockCategory
	{
		None,
		GateBlockedByCollision,
		GateBlockedByDeadZone,
		GateBlockedByClearance,
		GateBlockedBySingularity
	}

	public struct MirrorSnapshot
	{
		public Vector3 baseWorldPosition;
		public Quaternion baseWorldRotation;
		public float basePlanarSpeed;
		public float baseYawRate;
		public float[] armJointAnglesDeg;
		public float[] armJointVelocitiesDegPerSecond;
		public Vector3 armEndEffectorWorldPosition;
	}

	public struct ShadowPredictorState
	{
		public Vector3 predictedBaseWorldPosition;
		public Quaternion predictedBaseWorldRotation;
		public float[] predictedArmJointAnglesDeg;
		public float predictedLeadTimeSeconds;
		public float minPredictedClearanceMeters;
	}

	public sealed class SafetyGateDecision
	{
		public SafetyGateDecisionType type = SafetyGateDecisionType.Allow;
		public SafetyGateBlockCategory blockCategory = SafetyGateBlockCategory.None;
		public string reason = string.Empty;
		public float throttleRatio = 1f;
		public float leadTimeMs;
		public float minPredictedClearanceMeters = float.PositiveInfinity;
		public ShadowPredictorState predictorState;
	}

	public interface IMirrorStateProvider
	{
		MirrorSnapshot Capture(
			DiffDriveTwinController baseController,
			Arm6DOFFKController armController);
	}

	public sealed class UnityMirrorStateProvider : IMirrorStateProvider
	{
		public MirrorSnapshot Capture(
			DiffDriveTwinController baseController,
			Arm6DOFFKController armController)
		{
			MirrorSnapshot snapshot = new MirrorSnapshot
			{
				baseWorldPosition = baseController != null && baseController.rb != null ? baseController.rb.position : Vector3.zero,
				baseWorldRotation = baseController != null && baseController.rb != null ? baseController.rb.rotation : Quaternion.identity,
				basePlanarSpeed = baseController != null ? baseController.CurrentPlanarSpeedMeasured : 0f,
				baseYawRate = baseController != null ? baseController.CurrentYawRateMeasured : 0f,
				armJointAnglesDeg = armController != null ? armController.CaptureMeasuredJointAngles() : null,
				armJointVelocitiesDegPerSecond = armController != null ? armController.GetJointVelocities() : null,
				armEndEffectorWorldPosition = armController != null ? armController.EndEffectorWorldPosition : Vector3.zero
			};
			return snapshot;
		}
	}

	public sealed class SafetyGateRuntimeContext
	{
		public bool enabled = true;
		public bool throttleOnRisk = true;
		public float commLatencySeconds = 0.04f;
		public float computeBudgetSeconds = 0.02f;
		public float minLookaheadSeconds = 0.08f;
		public float baseInflationMeters = 0.02f;
		public float armInflationMeters = 0.015f;
		public float baseRadiusMeters = 0.35f;
		public float loadFactor;
		public float loadInflationGain = 0.35f;
		public float baseBrakeDecelMetersPerSecond2 = 0.9f;
		public float armBrakeDecelDegPerSecond2 = 120f;
		public float gateBlockClearanceMeters = 0.01f;
		public float gateThrottleClearanceMeters = 0.05f;
		public float deadZoneMarginMeters = 0.01f;
		public float queueLeadTimeSeconds;
		public int maxRecoveryReplans = 3;
		public bool dynamicBaseLockWhenEeWithinTolerance = true;
		public int decelFramesBeforeStop = 2;
		public IReadOnlyList<Collider> obstacles;
		public ISet<Collider> obstacleLookup;
	}

	public sealed class SafetyGateCommandBuffer<T>
	{
		private readonly Queue<T> _queue = new Queue<T>();
		public int Count => _queue.Count;
		public void Clear() => _queue.Clear();
		public void Enqueue(T item) => _queue.Enqueue(item);
		public bool TryPeek(out T item)
		{
			if (_queue.Count > 0)
			{
				item = _queue.Peek();
				return true;
			}

			item = default;
			return false;
		}

		public bool TryDequeue(out T item)
		{
			if (_queue.Count > 0)
			{
				item = _queue.Dequeue();
				return true;
			}

			item = default;
			return false;
		}
	}

	public struct BaseGateCommand
	{
		public Vector3 fromWorldPosition;
		public Vector3 toWorldPosition;
		public float targetYawDeg;
	}

	public struct ArmGateCommand
	{
		public float[] fromAnglesDeg;
		public float[] toAnglesDeg;
	}

	public enum SafetyGateTimelineCommandKind
	{
		Base,
		Arm
	}

	public struct SafetyGateTimelineCommand
	{
		public long sequenceId;
		public float enqueueRealtime;
		public float readyRealtime;
		public SafetyGateTimelineCommandKind kind;
		public BaseGateCommand baseCommand;
		public float baseLinearVelocity;
		public float baseAngularVelocity;
		public ArmGateCommand armCommand;
		public Vector3 predictedBaseWorldPosition;
		public Quaternion predictedBaseWorldRotation;
		public float throttleRatio;
		public float predictedLeadSeconds;
		public string predictedRiskSummary;
	}

	public sealed class SafetyGateTimelineBuffer
	{
		private readonly Queue<SafetyGateTimelineCommand> _queue = new Queue<SafetyGateTimelineCommand>();
		private readonly object _sync = new object();

		public int Count
		{
			get
			{
				lock (_sync)
				{
					return _queue.Count;
				}
			}
		}

		public void Clear()
		{
			lock (_sync)
			{
				_queue.Clear();
			}
		}

		public int Flush()
		{
			lock (_sync)
			{
				int count = _queue.Count;
				_queue.Clear();
				return count;
			}
		}

		public void Enqueue(SafetyGateTimelineCommand command)
		{
			lock (_sync)
			{
				_queue.Enqueue(command);
			}
		}

		public bool TryPeek(out SafetyGateTimelineCommand command)
		{
			lock (_sync)
			{
				if (_queue.Count > 0)
				{
					command = _queue.Peek();
					return true;
				}

				command = default;
				return false;
			}
		}

		public bool TryDequeueReady(float nowRealtime, float requiredLeadSeconds, out SafetyGateTimelineCommand command)
		{
			lock (_sync)
			{
				if (_queue.Count <= 0)
				{
					command = default;
					return false;
				}

				SafetyGateTimelineCommand head = _queue.Peek();
				float requiredReadyRealtime = head.enqueueRealtime + Mathf.Max(0f, requiredLeadSeconds);
				float headReadyRealtime = head.readyRealtime > 0f
					? Mathf.Max(requiredReadyRealtime, head.readyRealtime)
					: requiredReadyRealtime;
				if (nowRealtime + 1e-5f < headReadyRealtime)
				{
					command = default;
					return false;
				}

				command = _queue.Dequeue();
				return true;
			}
		}
	}

	public sealed class ShadowSimulationGate
	{
		private const float MinLookaheadFloorSeconds = 0.02f;

		public float requiredBaseClearance = 0.05f;
		public float armSingularityPenaltyThreshold = 60f;
		public float armSingularityPenaltyBlockingThreshold = 72f;
		public float armSingularityPenaltyHardThreshold = 95f;
		public int armConsecutiveBlockingSingularitySamplesToReject = 5;
		public float basePreviewSampleSpacingMeters = 0.08f;
		public float armPreviewSampleSpacingDeg = 4f;
		public float armSingularityDeterminantThrottleThreshold = 2.5e-4f;
		public float armSingularityDeterminantHardThreshold = 1.0e-4f;
		private static readonly Collider[] OverlapBuffer = new Collider[64];

		public SafetyGateDecision EvaluateBaseCommand(
			MirrorSnapshot snapshot,
			BaseGateCommand command,
			SafetyGateRuntimeContext context)
		{
			SafetyGateDecision decision = CreateDefaultGateDecision(context, snapshot, armMode: false);
			if (context == null || !context.enabled)
			{
				return decision;
			}

			float inflatedBaseRadius = Mathf.Max(0.02f, context.baseRadiusMeters + ComputeInflation(context.baseInflationMeters, context));
			float previewWorstClearance = float.PositiveInfinity;
			int previewSampleCount = EstimateBasePreviewSampleCount(
				command.fromWorldPosition,
				command.toWorldPosition,
				basePreviewSampleSpacingMeters,
				decision.predictorState.predictedLeadTimeSeconds);
			int previewDisplaySampleIndex = ResolveBasePreviewDisplaySampleIndex(
				command.fromWorldPosition,
				command.toWorldPosition,
				snapshot,
				context,
				decision.predictorState.predictedLeadTimeSeconds,
				previewSampleCount);
			Vector3 predictedPosition = command.fromWorldPosition;
			Quaternion predictedRotation = snapshot.baseWorldRotation;
			bool predictedPoseAssigned = false;
			bool collisionDetected = false;
			Collider blockingCollider = null;
			for (int sampleIndex = 0; sampleIndex <= previewSampleCount; sampleIndex++)
			{
				float t = previewSampleCount <= 0 ? 1f : sampleIndex / (float)previewSampleCount;
				Vector3 samplePosition = Vector3.Lerp(command.fromWorldPosition, command.toWorldPosition, t);
				if (!predictedPoseAssigned && sampleIndex >= previewDisplaySampleIndex)
				{
					predictedPosition = samplePosition;
					predictedRotation = Quaternion.Slerp(snapshot.baseWorldRotation, Quaternion.Euler(0f, command.targetYawDeg, 0f), t);
					predictedPoseAssigned = true;
				}

				if (TryOverlapObstacle(samplePosition, inflatedBaseRadius, context.obstacles, context.obstacleLookup, out Collider sampleHitCollider))
				{
					collisionDetected = true;
					blockingCollider = sampleHitCollider;
					if (!predictedPoseAssigned || sampleIndex <= previewDisplaySampleIndex)
					{
						predictedPosition = samplePosition;
						predictedRotation = Quaternion.Slerp(snapshot.baseWorldRotation, Quaternion.Euler(0f, command.targetYawDeg, 0f), t);
						predictedPoseAssigned = true;
					}
					break;
				}

				float sampleClearance = ComputeSphereClearance(samplePosition, inflatedBaseRadius, context.obstacles);
				if (sampleClearance < previewWorstClearance)
				{
					previewWorstClearance = sampleClearance;
				}
			}

			if (!predictedPoseAssigned)
			{
				predictedPosition = command.toWorldPosition;
				predictedRotation = Quaternion.Euler(0f, command.targetYawDeg, 0f);
			}

			decision.predictorState.predictedBaseWorldPosition = predictedPosition;
			decision.predictorState.predictedBaseWorldRotation = predictedRotation;
			Collider hitCollider = blockingCollider;

			if (collisionDetected)
			{
				decision.type = SafetyGateDecisionType.Block;
				decision.blockCategory = SafetyGateBlockCategory.GateBlockedByCollision;
				decision.reason = L(
					$"SafetyGate 阻止底盘命令：预测位姿与障碍物 '{hitCollider?.name}' 重叠。",
					$"SafetyGate blocked the base command because the predicted pose overlaps obstacle '{hitCollider?.name}'.");
				return decision;
			}

			float minClearance = float.IsPositiveInfinity(previewWorstClearance)
				? ComputeSphereClearance(predictedPosition, inflatedBaseRadius, context.obstacles)
				: previewWorstClearance;
			decision.minPredictedClearanceMeters = minClearance;
			decision.predictorState.minPredictedClearanceMeters = minClearance;
			float blockClearance = Mathf.Max(0.001f, context.gateBlockClearanceMeters);
			float throttleClearance = Mathf.Max(blockClearance, context.gateThrottleClearanceMeters);

			if (minClearance <= blockClearance)
			{
				decision.type = SafetyGateDecisionType.Block;
				decision.blockCategory = SafetyGateBlockCategory.GateBlockedByClearance;
				decision.reason = L(
					$"SafetyGate 阻止底盘命令：预测净空 {minClearance:F3}m 低于阻断阈值 {blockClearance:F3}m。",
					$"SafetyGate blocked the base command because predicted clearance {minClearance:F3}m is below blocking threshold {blockClearance:F3}m.");
				return decision;
			}

			if (minClearance <= throttleClearance)
			{
				if (!context.throttleOnRisk)
				{
					decision.type = SafetyGateDecisionType.Block;
					decision.blockCategory = SafetyGateBlockCategory.GateBlockedByClearance;
					decision.reason = L(
						$"SafetyGate 阻止底盘命令：预测净空 {minClearance:F3}m 低于限速阈值且禁用降速放行。",
						$"SafetyGate blocked the base command because predicted clearance {minClearance:F3}m is below throttle threshold while throttle-on-risk is disabled.");
					return decision;
				}

				decision.type = SafetyGateDecisionType.Throttle;
				decision.throttleRatio = Mathf.Min(decision.throttleRatio, ComputeThrottleRatio(minClearance, blockClearance, throttleClearance));
				decision.reason = L(
					$"SafetyGate 对底盘命令降速：预测净空 {minClearance:F3}m，限速比例 {decision.throttleRatio:F2}。",
					$"SafetyGate throttled the base command: predicted clearance {minClearance:F3}m, throttle ratio {decision.throttleRatio:F2}.");
			}

			return decision;
		}

		public SafetyGateDecision EvaluateArmCommand(
			MirrorSnapshot snapshot,
			ArmGateCommand command,
			SafetyGateRuntimeContext context,
			Arm6DOFFKController armController)
		{
			SafetyGateDecision decision = CreateDefaultGateDecision(context, snapshot, armMode: true);
			decision.predictorState.predictedBaseWorldPosition = snapshot.baseWorldPosition;
			decision.predictorState.predictedBaseWorldRotation = snapshot.baseWorldRotation;
			if (context == null || !context.enabled)
			{
				return decision;
			}

			if (armController == null || command.toAnglesDeg == null || command.toAnglesDeg.Length < 6)
			{
				decision.type = SafetyGateDecisionType.Block;
				decision.blockCategory = SafetyGateBlockCategory.GateBlockedByCollision;
				decision.reason = L(
					"SafetyGate 阻止机械臂命令：机械臂控制器或目标关节数据不可用。",
					"SafetyGate blocked the arm command because arm controller or target joint data is unavailable.");
				return decision;
			}

			float[] fromAngles = command.fromAnglesDeg != null && command.fromAnglesDeg.Length >= 6
				? (float[])command.fromAnglesDeg.Clone()
				: snapshot.armJointAnglesDeg != null && snapshot.armJointAnglesDeg.Length >= 6
					? (float[])snapshot.armJointAnglesDeg.Clone()
					: armController.CaptureMeasuredJointAngles();
			float[] predictedAngles = (float[])command.toAnglesDeg.Clone();
			decision.predictorState.predictedArmJointAnglesDeg = predictedAngles;

			if (armController.EvaluateMotionCollision(fromAngles, command.toAnglesDeg, out ArmCollisionGuardResult guardResult))
			{
				decision.type = SafetyGateDecisionType.Block;
				decision.blockCategory = SafetyGateBlockCategory.GateBlockedByCollision;
				decision.reason = string.IsNullOrEmpty(guardResult?.message)
					? L("SafetyGate 阻止机械臂命令：预测关节段存在禁碰。", "SafetyGate blocked the arm command because the predicted segment has a forbidden collision.")
					: guardResult.message;
				return decision;
			}

			float inflatedArmRadius = Mathf.Max(0.005f, ComputeInflation(context.armInflationMeters, context));
			float minPreviewDeterminant = float.PositiveInfinity;
			float minPreviewClearance = float.PositiveInfinity;
			float previewDeadZoneThreshold = Mathf.Max(0.001f, armController.GetMinReach() + Mathf.Max(0f, context.deadZoneMarginMeters));
			int previewSampleCount = EstimateArmPreviewSampleCount(
				fromAngles,
				command.toAnglesDeg,
				armPreviewSampleSpacingDeg,
				decision.predictorState.predictedLeadTimeSeconds);
			for (int sampleIndex = 0; sampleIndex <= previewSampleCount; sampleIndex++)
			{
				float t = previewSampleCount <= 0 ? 1f : sampleIndex / (float)previewSampleCount;
				float[] sampleAngles = LerpAngles(fromAngles, command.toAnglesDeg, t);
				float[,] sampleJacobian = armController.ComputeGeometricJacobian(sampleAngles);
				float sampleDeterminant = ComputeLinearJacobianGramDeterminant(sampleJacobian);
				if (sampleDeterminant < minPreviewDeterminant)
				{
					minPreviewDeterminant = sampleDeterminant;
				}

				if (sampleDeterminant <= armSingularityDeterminantHardThreshold)
				{
					decision.type = SafetyGateDecisionType.Block;
					decision.blockCategory = SafetyGateBlockCategory.GateBlockedBySingularity;
					decision.reason = L(
						$"SafetyGate blocked arm command: preview sample {sampleIndex + 1}/{previewSampleCount + 1} det(JvJv^T)={sampleDeterminant:E3} is below hard threshold {armSingularityDeterminantHardThreshold:E3}.",
						$"SafetyGate blocked arm command: preview sample {sampleIndex + 1}/{previewSampleCount + 1} det(JvJv^T)={sampleDeterminant:E3} is below hard threshold {armSingularityDeterminantHardThreshold:E3}.");
					decision.predictorState.predictedArmJointAnglesDeg = sampleAngles;
					return decision;
				}

				Pose sampleEeBase = armController.ForwardPoe(sampleAngles);
				if (sampleEeBase.position.magnitude < previewDeadZoneThreshold)
				{
					predictedAngles = sampleAngles;
					break;
				}

				Pose[] sampleLinkPoses = armController.ComputeLinkPosesWorld(sampleAngles);
				bool sampleCollision = false;
				float sampleMinClearance = float.PositiveInfinity;
				if (sampleLinkPoses != null && sampleLinkPoses.Length > 0)
				{
					for (int poseIndex = 0; poseIndex < sampleLinkPoses.Length; poseIndex++)
					{
						Vector3 point = sampleLinkPoses[poseIndex].position;
						if (TryOverlapObstacle(point, inflatedArmRadius, context.obstacles, context.obstacleLookup, out _))
						{
							sampleCollision = true;
							break;
						}

						float pointClearance = ComputeSphereClearance(point, inflatedArmRadius, context.obstacles);
						if (pointClearance < sampleMinClearance)
						{
							sampleMinClearance = pointClearance;
						}
					}
				}

				if (sampleCollision)
				{
					predictedAngles = sampleAngles;
					break;
				}

				if (sampleMinClearance < minPreviewClearance)
				{
					minPreviewClearance = sampleMinClearance;
					predictedAngles = sampleAngles;
				}
			}

			if (decision.type == SafetyGateDecisionType.Allow
				&& minPreviewDeterminant <= armSingularityDeterminantThrottleThreshold)
			{
				if (!context.throttleOnRisk)
				{
					decision.type = SafetyGateDecisionType.Block;
					decision.blockCategory = SafetyGateBlockCategory.GateBlockedBySingularity;
					decision.reason = L(
						$"SafetyGate blocked arm command: preview singularity determinant {minPreviewDeterminant:E3} is below throttle threshold while throttle-on-risk is disabled.",
						$"SafetyGate blocked arm command: preview singularity determinant {minPreviewDeterminant:E3} is below throttle threshold while throttle-on-risk is disabled.");
					decision.predictorState.predictedArmJointAnglesDeg = (float[])predictedAngles.Clone();
					return decision;
				}

				decision.type = SafetyGateDecisionType.Throttle;
				decision.throttleRatio = ComputeThrottleRatioBySingularity(
					minPreviewDeterminant,
					armSingularityDeterminantHardThreshold,
					armSingularityDeterminantThrottleThreshold);
				decision.reason = L(
					$"SafetyGate throttled arm command: preview singularity determinant {minPreviewDeterminant:E3}, throttle ratio {decision.throttleRatio:F2}.",
					$"SafetyGate throttled arm command: preview singularity determinant {minPreviewDeterminant:E3}, throttle ratio {decision.throttleRatio:F2}.");
			}

			decision.predictorState.predictedArmJointAnglesDeg = (float[])predictedAngles.Clone();
			Pose[] linkWorldPoses = armController.ComputeLinkPosesWorld(predictedAngles);
			float minClearance = float.PositiveInfinity;
			if (linkWorldPoses != null && linkWorldPoses.Length > 0)
			{
				for (int poseIndex = 0; poseIndex < linkWorldPoses.Length; poseIndex++)
				{
					Vector3 point = linkWorldPoses[poseIndex].position;
					if (TryOverlapObstacle(point, inflatedArmRadius, context.obstacles, context.obstacleLookup, out Collider hitCollider))
					{
						decision.type = SafetyGateDecisionType.Block;
						decision.blockCategory = SafetyGateBlockCategory.GateBlockedByCollision;
						decision.reason = L(
							$"SafetyGate 阻止机械臂命令：预测连杆关键点与障碍物 '{hitCollider?.name}' 重叠。",
							$"SafetyGate blocked the arm command because a predicted link point overlaps obstacle '{hitCollider?.name}'.");
						return decision;
					}

					minClearance = Mathf.Min(minClearance, ComputeSphereClearance(point, inflatedArmRadius, context.obstacles));
				}
			}

			Transform armBase = armController.BaseFrameTransform;
			Pose predictedEeBase = armController.ForwardPoe(predictedAngles);
			Vector3 predictedEeWorld = armBase != null
				? armBase.TransformPoint(predictedEeBase.position)
				: predictedEeBase.position;
			float predictedDeadZoneDistance = armBase != null
				? armBase.InverseTransformPoint(predictedEeWorld).magnitude
				: predictedEeBase.position.magnitude;
			float deadZoneThreshold = Mathf.Max(0.001f, armController.GetMinReach() + Mathf.Max(0f, context.deadZoneMarginMeters));
			if (predictedDeadZoneDistance < deadZoneThreshold)
			{
				decision.type = SafetyGateDecisionType.Block;
				decision.blockCategory = SafetyGateBlockCategory.GateBlockedByDeadZone;
				decision.reason = L(
					$"SafetyGate 阻止机械臂命令：预测末端半径 {predictedDeadZoneDistance:F3}m 小于死区阈值 {deadZoneThreshold:F3}m。",
					$"SafetyGate blocked the arm command because predicted EE radius {predictedDeadZoneDistance:F3}m is inside dead-zone threshold {deadZoneThreshold:F3}m.");
				return decision;
			}

			decision.minPredictedClearanceMeters = minClearance;
			decision.predictorState.minPredictedClearanceMeters = minClearance;
			float blockClearance = Mathf.Max(0.001f, context.gateBlockClearanceMeters);
			float throttleClearance = Mathf.Max(blockClearance, context.gateThrottleClearanceMeters);
			if (minClearance <= blockClearance)
			{
				decision.type = SafetyGateDecisionType.Block;
				decision.blockCategory = SafetyGateBlockCategory.GateBlockedByClearance;
				decision.reason = L(
					$"SafetyGate 阻止机械臂命令：预测净空 {minClearance:F3}m 低于阻断阈值 {blockClearance:F3}m。",
					$"SafetyGate blocked the arm command because predicted clearance {minClearance:F3}m is below blocking threshold {blockClearance:F3}m.");
				return decision;
			}

			if (minClearance <= throttleClearance)
			{
				if (!context.throttleOnRisk)
				{
					decision.type = SafetyGateDecisionType.Block;
					decision.blockCategory = SafetyGateBlockCategory.GateBlockedByClearance;
					decision.reason = L(
						$"SafetyGate 阻止机械臂命令：预测净空 {minClearance:F3}m 低于限速阈值且禁用降速放行。",
						$"SafetyGate blocked the arm command because predicted clearance {minClearance:F3}m is below throttle threshold while throttle-on-risk is disabled.");
					return decision;
				}

				decision.type = SafetyGateDecisionType.Throttle;
				decision.throttleRatio = Mathf.Min(decision.throttleRatio, ComputeThrottleRatio(minClearance, blockClearance, throttleClearance));
				decision.reason = L(
					$"SafetyGate 对机械臂命令降速：预测净空 {minClearance:F3}m，限速比例 {decision.throttleRatio:F2}。",
					$"SafetyGate throttled the arm command: predicted clearance {minClearance:F3}m, throttle ratio {decision.throttleRatio:F2}.");
			}

			return decision;
		}

		public ShadowValidationResult ValidateBasePath(
			IReadOnlyList<Vector3> waypoints,
			float baseRadius,
			PlannerPhysicsQueries physicsQueries,
			SceneDistanceFieldSampler sampler,
			IReadOnlyList<Collider> obstacles,
			bool allowInitialPoseOccupied = false)
		{
			ShadowValidationResult result = new ShadowValidationResult();
			if (waypoints == null || waypoints.Count == 0)
			{
				return result;
			}

			for (int i = 0; i < waypoints.Count; i++)
			{
				Vector3 point = waypoints[i];
				if (allowInitialPoseOccupied && i == 0)
				{
					continue;
				}

				if (!physicsQueries.IsBasePoseCollisionFree(point, baseRadius, obstacles, out Collider hitCollider))
				{
					result.passed = false;
					result.collisionDetected = true;
					result.failedSampleIndex = i;
					result.message = $"Shadow validation blocked base waypoint {i + 1} by '{hitCollider?.name}'.";
					return result;
				}

				if (sampler != null && sampler.IsReady)
				{
					float clearance = sampler.SampleDistance(point);
					if (clearance < requiredBaseClearance)
					{
						result.passed = false;
						result.collisionDetected = true;
						result.failedSampleIndex = i;
						result.message = $"Shadow validation rejected base waypoint {i + 1}: clearance {clearance:F3}m is below the safety threshold.";
						return result;
					}
				}
			}

			return result;
		}

		public ShadowValidationResult ValidateBaseSegment(
			Vector3 from,
			Vector3 to,
			float baseRadius,
			PlannerPhysicsQueries physicsQueries,
			SceneDistanceFieldSampler sampler,
			IReadOnlyList<Collider> obstacles,
			bool allowInitialPoseOccupied = false)
		{
			ShadowValidationResult result = new ShadowValidationResult();
			if (physicsQueries == null)
			{
				return result;
			}

			if (!allowInitialPoseOccupied && !physicsQueries.IsBasePoseCollisionFree(from, baseRadius, obstacles, out Collider blockedStart))
			{
				result.passed = false;
				result.collisionDetected = true;
				result.failedSampleIndex = 0;
				result.message = $"Shadow validation blocked base waypoint 1 by '{blockedStart?.name}'.";
				return result;
			}

			if (!physicsQueries.IsBasePoseCollisionFree(to, baseRadius, obstacles, out Collider blockedEnd))
			{
				result.passed = false;
				result.collisionDetected = true;
				result.failedSampleIndex = 1;
				result.message = $"Shadow validation blocked base waypoint 2 by '{blockedEnd?.name}'.";
				return result;
			}

			if (sampler != null && sampler.IsReady)
			{
				float fromClearance = sampler.SampleDistance(from);
				if (!allowInitialPoseOccupied && fromClearance < requiredBaseClearance)
				{
					result.passed = false;
					result.collisionDetected = true;
					result.failedSampleIndex = 0;
					result.message = $"Shadow validation rejected base waypoint 1: clearance {fromClearance:F3}m is below the safety threshold.";
					return result;
				}

				float toClearance = sampler.SampleDistance(to);
				if (toClearance < requiredBaseClearance)
				{
					result.passed = false;
					result.collisionDetected = true;
					result.failedSampleIndex = 1;
					result.message = $"Shadow validation rejected base waypoint 2: clearance {toClearance:F3}m is below the safety threshold.";
					return result;
				}
			}

			return result;
		}

		public ShadowValidationResult ValidateArmTrajectory(Arm6DOFFKController armController, IReadOnlyList<RobotPlanJointSample> samples)
		{
			ShadowValidationResult result = new ShadowValidationResult();
			if (armController == null || samples == null || samples.Count == 0)
			{
				return result;
			}

			float[] previousAngles = armController.CaptureMeasuredJointAngles();
			int advisorySampleCount = 0;
			int firstAdvisorySampleIndex = -1;
			float advisoryPeakSingularityPenalty = 0f;
			int consecutiveBlockingSingularitySamples = 0;
			int firstBlockingSingularitySampleIndex = -1;
			float blockingPeakSingularityPenalty = 0f;
			for (int i = 0; i < samples.Count; i++)
			{
				RobotPlanJointSample sample = samples[i];
				if (sample == null || sample.jointAnglesDeg == null || sample.jointAnglesDeg.Length < 6)
				{
					continue;
				}

				if (armController.EvaluateMotionCollision(previousAngles, sample.jointAnglesDeg, out ArmCollisionGuardResult guardResult))
				{
					result.passed = false;
					result.collisionDetected = true;
					result.failedSampleIndex = i;
					result.message = string.IsNullOrEmpty(guardResult?.message)
						? L($"影子验证拒绝了机械臂采样点 {i + 1}，原因是发生禁碰碰撞。", $"Shadow validation rejected arm sample {i + 1} because of a forbidden collision.")
						: guardResult.message;
					return result;
				}

				if (sample.singularityPenalty >= armSingularityPenaltyHardThreshold)
				{
					result.passed = false;
					result.singularityRisk = true;
					result.failedSampleIndex = i;
					result.message = L(
						$"影子验证拒绝了机械臂采样点 {i + 1}：奇异性惩罚 {sample.singularityPenalty:F2} 已超过硬阈值。",
						$"Shadow validation rejected arm sample {i + 1}: singularity penalty {sample.singularityPenalty:F2} exceeded the hard threshold.");
					return result;
				}

				if (sample.singularityPenalty >= armSingularityPenaltyThreshold)
				{
					if (advisorySampleCount == 0)
					{
						firstAdvisorySampleIndex = i;
					}

					advisorySampleCount++;
					advisoryPeakSingularityPenalty = Mathf.Max(advisoryPeakSingularityPenalty, sample.singularityPenalty);
				}

				if (sample.singularityPenalty >= armSingularityPenaltyBlockingThreshold)
				{
					if (consecutiveBlockingSingularitySamples == 0)
					{
						firstBlockingSingularitySampleIndex = i;
						blockingPeakSingularityPenalty = sample.singularityPenalty;
					}

					consecutiveBlockingSingularitySamples++;
					blockingPeakSingularityPenalty = Mathf.Max(blockingPeakSingularityPenalty, sample.singularityPenalty);
					if (consecutiveBlockingSingularitySamples >= Mathf.Max(1, armConsecutiveBlockingSingularitySamplesToReject))
					{
						result.passed = false;
						result.singularityRisk = true;
						result.failedSampleIndex = firstBlockingSingularitySampleIndex;
						result.message = L(
							$"影子验证拒绝了机械臂轨迹：从采样点 {firstBlockingSingularitySampleIndex + 1} 开始连续 {consecutiveBlockingSingularitySamples} 个点的奇异性惩罚超过阻断阈值，峰值为 {blockingPeakSingularityPenalty:F2}。",
							$"Shadow validation rejected the arm trajectory: starting at sample {firstBlockingSingularitySampleIndex + 1}, {consecutiveBlockingSingularitySamples} consecutive samples exceeded the blocking singularity threshold, peaking at {blockingPeakSingularityPenalty:F2}.");
						return result;
					}
				}
				else
				{
					consecutiveBlockingSingularitySamples = 0;
					firstBlockingSingularitySampleIndex = -1;
					blockingPeakSingularityPenalty = 0f;
				}

				previousAngles = sample.jointAnglesDeg;
			}

			if (advisorySampleCount > 0)
			{
				result.singularityRisk = true;
				result.message = L(
					$"机械臂轨迹靠近奇异区：从采样点 {firstAdvisorySampleIndex + 1} 开始共检测到 {advisorySampleCount} 个高奇异性采样点，峰值为 {advisoryPeakSingularityPenalty:F2}；本次作为告警继续执行。",
					$"Arm trajectory approached a singular region: starting at sample {firstAdvisorySampleIndex + 1}, {advisorySampleCount} elevated-singularity samples were observed, peaking at {advisoryPeakSingularityPenalty:F2}; this run is allowed as a warning.");
			}

			return result;
		}

		private static string L(string chinese, string english)
		{
			return RobotSimulationLocalization.Text(chinese, english);
		}

		private static SafetyGateDecision CreateDefaultGateDecision(SafetyGateRuntimeContext context, MirrorSnapshot snapshot, bool armMode)
		{
			SafetyGateDecision decision = new SafetyGateDecision();
			float lookahead = ComputeLookaheadSeconds(context, snapshot, armMode);
			decision.leadTimeMs = lookahead * 1000f;
			decision.predictorState = new ShadowPredictorState
			{
				predictedLeadTimeSeconds = lookahead,
				minPredictedClearanceMeters = float.PositiveInfinity
			};
			return decision;
		}

		private static int EstimateBasePreviewSampleCount(Vector3 fromWorldPosition, Vector3 toWorldPosition, float spacingMeters, float lookaheadSeconds)
		{
			float planarDistance = Vector3.Distance(new Vector3(fromWorldPosition.x, 0f, fromWorldPosition.z), new Vector3(toWorldPosition.x, 0f, toWorldPosition.z));
			float effectiveSpacing = Mathf.Max(0.02f, spacingMeters);
			int distanceSamples = Mathf.CeilToInt(planarDistance / effectiveSpacing);
			int leadSamples = Mathf.CeilToInt(Mathf.Max(MinLookaheadFloorSeconds, lookaheadSeconds) / Mathf.Max(0.02f, Time.fixedDeltaTime));
			return Mathf.Clamp(Mathf.Max(distanceSamples, leadSamples), 2, 64);
		}

		private static int ResolveBasePreviewDisplaySampleIndex(
			Vector3 fromWorldPosition,
			Vector3 toWorldPosition,
			MirrorSnapshot snapshot,
			SafetyGateRuntimeContext context,
			float lookaheadSeconds,
			int previewSampleCount)
		{
			if (previewSampleCount <= 0)
			{
				return 0;
			}

			Vector3 planarDelta = new Vector3(toWorldPosition.x - fromWorldPosition.x, 0f, toWorldPosition.z - fromWorldPosition.z);
			float planarDistance = planarDelta.magnitude;
			if (planarDistance <= 1e-4f)
			{
				return previewSampleCount;
			}

			float minPreviewSpeed = context != null ? Mathf.Max(0.25f, context.baseRadiusMeters * 1.5f) : 0.4f;
			float effectiveSpeed = Mathf.Max(snapshot.basePlanarSpeed, minPreviewSpeed);
			float lookaheadDistance = Mathf.Min(planarDistance, effectiveSpeed * Mathf.Max(MinLookaheadFloorSeconds, lookaheadSeconds));
			float previewT = Mathf.Clamp01(lookaheadDistance / planarDistance);
			return Mathf.Clamp(Mathf.CeilToInt(previewT * previewSampleCount), 1, previewSampleCount);
		}

		private static int EstimateArmPreviewSampleCount(float[] fromAnglesDeg, float[] toAnglesDeg, float spacingDeg, float lookaheadSeconds)
		{
			if (fromAnglesDeg == null || toAnglesDeg == null || fromAnglesDeg.Length < 6 || toAnglesDeg.Length < 6)
			{
				return 2;
			}

			float maxDelta = 0f;
			for (int i = 0; i < 6; i++)
			{
				maxDelta = Mathf.Max(maxDelta, Mathf.Abs(toAnglesDeg[i] - fromAnglesDeg[i]));
			}

			float effectiveSpacing = Mathf.Max(1f, spacingDeg);
			int distanceSamples = Mathf.CeilToInt(maxDelta / effectiveSpacing);
			int leadSamples = Mathf.CeilToInt(Mathf.Max(MinLookaheadFloorSeconds, lookaheadSeconds) / Mathf.Max(0.02f, Time.fixedDeltaTime));
			return Mathf.Clamp(Mathf.Max(distanceSamples, leadSamples), 2, 64);
		}

		private static float[] LerpAngles(float[] fromAnglesDeg, float[] toAnglesDeg, float t)
		{
			float[] result = new float[6];
			for (int i = 0; i < 6; i++)
			{
				float from = fromAnglesDeg != null && fromAnglesDeg.Length > i ? fromAnglesDeg[i] : 0f;
				float to = toAnglesDeg != null && toAnglesDeg.Length > i ? toAnglesDeg[i] : from;
				result[i] = Mathf.Lerp(from, to, t);
			}

			return result;
		}

		private static float ComputeLinearJacobianGramDeterminant(float[,] jacobian)
		{
			if (jacobian == null || jacobian.GetLength(0) < 6 || jacobian.GetLength(1) < 6)
			{
				return 0f;
			}

			float[,] jjt = new float[3, 3];
			for (int row = 0; row < 3; row++)
			{
				for (int column = 0; column < 3; column++)
				{
					float sum = 0f;
					for (int jointIndex = 0; jointIndex < 6; jointIndex++)
					{
						sum += jacobian[3 + row, jointIndex] * jacobian[3 + column, jointIndex];
					}

					jjt[row, column] = sum;
				}
			}

			float determinant =
				(jjt[0, 0] * ((jjt[1, 1] * jjt[2, 2]) - (jjt[1, 2] * jjt[2, 1])))
				- (jjt[0, 1] * ((jjt[1, 0] * jjt[2, 2]) - (jjt[1, 2] * jjt[2, 0])))
				+ (jjt[0, 2] * ((jjt[1, 0] * jjt[2, 1]) - (jjt[1, 1] * jjt[2, 0])));
			return Mathf.Max(0f, determinant);
		}

		private static float ComputeLookaheadSeconds(SafetyGateRuntimeContext context, MirrorSnapshot snapshot, bool armMode)
		{
			if (context == null)
			{
				return 0.08f;
			}

			float baseWindow = Mathf.Max(Mathf.Max(context.minLookaheadSeconds, MinLookaheadFloorSeconds), context.commLatencySeconds + context.computeBudgetSeconds);
			if (!armMode)
			{
				float baseBrake = snapshot.basePlanarSpeed / Mathf.Max(0.05f, context.baseBrakeDecelMetersPerSecond2);
				return Mathf.Max(baseWindow, baseBrake);
			}

			float maxJointSpeed = 0f;
			if (snapshot.armJointVelocitiesDegPerSecond != null)
			{
				for (int i = 0; i < snapshot.armJointVelocitiesDegPerSecond.Length; i++)
				{
					maxJointSpeed = Mathf.Max(maxJointSpeed, Mathf.Abs(snapshot.armJointVelocitiesDegPerSecond[i]));
				}
			}

			float armBrake = maxJointSpeed / Mathf.Max(1f, context.armBrakeDecelDegPerSecond2);
			return Mathf.Max(baseWindow, armBrake);
		}

		private static float ComputeInflation(float baseInflation, SafetyGateRuntimeContext context)
		{
			if (context == null)
			{
				return Mathf.Max(0f, baseInflation);
			}

			float safeBaseInflation = Mathf.Max(0f, baseInflation);
			float scale = 1f + Mathf.Max(0f, context.loadFactor) * Mathf.Max(0f, context.loadInflationGain);
			return safeBaseInflation * scale;
		}

		private static float ComputeThrottleRatio(float minClearance, float blockThreshold, float throttleThreshold)
		{
			if (throttleThreshold <= blockThreshold + 1e-5f)
			{
				return 0.25f;
			}

			float normalized = Mathf.InverseLerp(blockThreshold, throttleThreshold, minClearance);
			return Mathf.Clamp(Mathf.Lerp(0.25f, 0.85f, normalized), 0.2f, 0.9f);
		}

		private static float ComputeThrottleRatioBySingularity(float determinant, float hardThreshold, float throttleThreshold)
		{
			if (throttleThreshold <= hardThreshold + 1e-8f)
			{
				return 0.25f;
			}

			float normalized = Mathf.InverseLerp(hardThreshold, throttleThreshold, determinant);
			return Mathf.Clamp(Mathf.Lerp(0.2f, 0.85f, normalized), 0.2f, 0.9f);
		}

		private static HashSet<Collider> BuildObstacleLookup(IReadOnlyList<Collider> obstacles)
		{
			HashSet<Collider> lookup = new HashSet<Collider>();
			if (obstacles == null)
			{
				return lookup;
			}

			for (int i = 0; i < obstacles.Count; i++)
			{
				Collider obstacle = obstacles[i];
				if (obstacle != null && obstacle.enabled && obstacle.gameObject.activeInHierarchy)
				{
					lookup.Add(obstacle);
				}
			}

			return lookup;
		}

		private static bool TryOverlapObstacle(
			Vector3 center,
			float radius,
			IReadOnlyList<Collider> obstacles,
			ISet<Collider> obstacleLookup,
			out Collider hitCollider)
		{
			hitCollider = null;
			if (obstacles == null || obstacles.Count == 0)
			{
				return false;
			}

			ISet<Collider> effectiveLookup = obstacleLookup;
			if (effectiveLookup == null || effectiveLookup.Count == 0)
			{
				effectiveLookup = BuildObstacleLookup(obstacles);
			}

			int hitCount = Physics.OverlapSphereNonAlloc(center, Mathf.Max(0.001f, radius), OverlapBuffer, ~0, QueryTriggerInteraction.Ignore);
			for (int hitIndex = 0; hitIndex < hitCount; hitIndex++)
			{
				Collider collider = OverlapBuffer[hitIndex];
				if (collider == null)
				{
					continue;
				}

				if (effectiveLookup.Contains(collider))
				{
					hitCollider = collider;
					return true;
				}
			}

			return false;
		}

		private static float ComputeSphereClearance(
			Vector3 center,
			float radius,
			IReadOnlyList<Collider> obstacles)
		{
			if (obstacles == null || obstacles.Count == 0)
			{
				return float.PositiveInfinity;
			}

			float minClearance = float.PositiveInfinity;
			for (int i = 0; i < obstacles.Count; i++)
			{
				Collider obstacle = obstacles[i];
				if (obstacle == null || !obstacle.enabled || !obstacle.gameObject.activeInHierarchy)
				{
					continue;
				}

				Vector3 closest = obstacle.ClosestPoint(center);
				float distance = Vector3.Distance(center, closest) - radius;
				minClearance = Mathf.Min(minClearance, distance);
			}

			return minClearance;
		}
	}

	public sealed class ExecutionSafetyFilter
	{
		private readonly ShadowSimulationGate _gate;

		public ExecutionSafetyFilter(ShadowSimulationGate gate)
		{
			_gate = gate;
		}

		public bool ValidateBaseSegment(
			Vector3 from,
			Vector3 to,
			float baseRadius,
			PlannerPhysicsQueries physicsQueries,
			SceneDistanceFieldSampler sampler,
			IReadOnlyList<Collider> obstacles,
			out ShadowValidationResult result)
		{
			bool startOccupied = physicsQueries != null && !physicsQueries.IsBasePoseCollisionFree(from, baseRadius, obstacles, out _);
			result = _gate.ValidateBaseSegment(from, to, baseRadius, physicsQueries, sampler, obstacles, startOccupied);
			return result == null || result.passed;
		}

		public bool ValidateArmSegment(Arm6DOFFKController armController, float[] fromAngles, float[] toAngles, out ShadowValidationResult result)
		{
			result = new ShadowValidationResult();
			if (armController == null)
			{
				result.passed = false;
				result.message = "Arm controller is missing.";
				return false;
			}

			if (armController.EvaluateMotionCollision(fromAngles, toAngles, out ArmCollisionGuardResult guardResult))
			{
				result.passed = false;
				result.collisionDetected = true;
				result.message = string.IsNullOrEmpty(guardResult?.message)
					? "Execution safety filter blocked an arm segment."
					: guardResult.message;
				return false;
			}

			return true;
		}
	}

	public sealed class LocalReplanner
	{
		public bool TryReplanBase(
			BaseRrtStarPlanner planner,
			Vector3 startWorld,
			float startYawDeg,
			Vector3 goalWorld,
			float goalYawDeg,
			float baseRadius,
			SceneDistanceFieldSampler sampler,
			PlannerPhysicsQueries physicsQueries,
			IReadOnlyList<Collider> obstacles,
			out List<Vector3> waypoints,
			out string failureReason)
		{
			return planner.TryPlan(startWorld, startYawDeg, goalWorld, goalYawDeg, baseRadius, sampler, physicsQueries, obstacles, out waypoints, out failureReason);
		}

		public bool TryReplanArm(
			ArmMotionPlanner planner,
			Arm6DOFFKController armController,
			Vector3 targetWorldPosition,
			SceneDistanceFieldSampler sampler,
			out List<RobotPlanJointSample> samples,
			out string failureReason)
		{
			return planner.TryPlanToWorldPosition(armController, targetWorldPosition, sampler, out samples, out failureReason);
		}
	}
}
