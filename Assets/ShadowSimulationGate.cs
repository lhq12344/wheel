using System.Collections.Generic;
using UnityEngine;

namespace RobotSimulation
{
	public sealed class ShadowSimulationGate
	{
		public float requiredBaseClearance = 0.05f;
		public float armSingularityPenaltyThreshold = 60f;
		public float armSingularityPenaltyBlockingThreshold = 72f;
		public float armSingularityPenaltyHardThreshold = 95f;
		public int armConsecutiveBlockingSingularitySamplesToReject = 5;

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
			List<Vector3> preview = new List<Vector3> { from, to };
			bool startOccupied = physicsQueries != null && !physicsQueries.IsBasePoseCollisionFree(from, baseRadius, obstacles, out _);
			result = _gate.ValidateBasePath(preview, baseRadius, physicsQueries, sampler, obstacles, startOccupied);
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
