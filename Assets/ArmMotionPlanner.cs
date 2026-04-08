using System.Collections.Generic;
using UnityEngine;

namespace RobotSimulation
{
	[System.Serializable]
	public sealed class ArmMotionPlannerSettings
	{
		public int maxIterations = 180;
		public float toleranceMeters = 0.02f;
		public float dlsLambda = 0.02f;
		public float maxDeltaDegPerIteration = 6f;
		public float compoundNudgeDeg = 10f;
		public float sampleSpacingDeg = 4f;
		public float sampleTimeStepSeconds = 0.08f;
		public float preferredTrajectorySingularityPenalty = 42f;
		public float trajectorySingularityPenaltyWeight = 0.75f;
		public float finalSingularityPenaltyWeight = 0.25f;
		public float nearbyTargetProbeFactor = 0.75f;
	}

	public sealed class ArmMotionPlanner
	{
		public ArmMotionPlannerSettings settings = new ArmMotionPlannerSettings();

		public bool TryPlanToWorldPosition(
			Arm6DOFFKController armController,
			Vector3 targetWorldPosition,
			SceneDistanceFieldSampler sampler,
			out List<RobotPlanJointSample> samples,
			out string failureReason,
			float[] preferredSolveSeedAnglesDeg = null,
			float targetToleranceMeters = -1f)
		{
			samples = new List<RobotPlanJointSample>();
			failureReason = string.Empty;
			float singularityPenalty;
			float solveToleranceMeters = ResolveSolveTolerance(targetToleranceMeters);

			if (armController == null || !armController.IsInitialized || !armController.KinematicsReady)
			{
				failureReason = L("机械臂控制器或运动学模型尚未就绪。", "Arm controller or kinematics model is not ready.");
				return false;
			}

			float[] startAngles = armController.CaptureMeasuredJointAngles();
			Vector3 targetBasePosition = armController.WorldToBasePosition(targetWorldPosition);
			float[] workingAngles = ClampAnglesToJointLimits(armController, preferredSolveSeedAnglesDeg);
			bool preferredSeedIsUsable = workingAngles != null
				&& workingAngles.Length >= 6
				&& ComputeResidualToBaseTarget(armController, workingAngles, targetBasePosition) <= solveToleranceMeters;
			if (!preferredSeedIsUsable
				&& !TrySolveToBasePosition(
					armController,
					targetBasePosition,
					out workingAngles,
					out singularityPenalty,
					out failureReason,
					preferredSolveSeedAnglesDeg ?? startAngles,
					startAngles,
					solveToleranceMeters))
			{
				return false;
			}
			else if (preferredSeedIsUsable)
			{
				singularityPenalty = ComputeSingularityPenalty(armController.ComputeGeometricJacobian(workingAngles));
			}

			int sampleCount = EstimateSampleCountInternal(startAngles, workingAngles);
			for (int sampleIndex = 1; sampleIndex <= sampleCount; sampleIndex++)
			{
				float t = sampleIndex / (float)sampleCount;
				float[] jointAngles = new float[6];
				for (int i = 0; i < 6; i++)
				{
					jointAngles[i] = Mathf.Lerp(startAngles[i], workingAngles[i], t);
				}

				if (armController.EvaluateMotionCollision(sampleIndex == 1 ? startAngles : samples[sampleIndex - 2].jointAnglesDeg, jointAngles, out ArmCollisionGuardResult guardResult))
				{
					failureReason = string.IsNullOrEmpty(guardResult?.message)
						? L($"机械臂轨迹采样点 {sampleIndex} 被碰撞守卫阻止。", $"Arm trajectory sample {sampleIndex} is blocked by the collision guard.")
						: guardResult.message;
					return false;
				}

				Pose poseBase = armController.ForwardPoe(jointAngles);
				Vector3 poseWorld = armController.BaseFrameTransform != null
					? armController.BaseFrameTransform.TransformPoint(poseBase.position)
					: poseBase.position;
				float[,] jacobian = armController.ComputeGeometricJacobian(jointAngles);
				samples.Add(new RobotPlanJointSample
				{
					timeSeconds = sampleIndex * Mathf.Max(0.02f, settings.sampleTimeStepSeconds),
					jointAnglesDeg = jointAngles,
					clearance = sampler != null && sampler.IsReady ? sampler.SampleDistance(poseWorld) : 1f,
					singularityPenalty = ComputeSingularityPenalty(jacobian)
				});
			}

			return samples.Count > 0;
		}

		public bool TrySolveToBasePosition(
			Arm6DOFFKController armController,
			Vector3 targetBasePosition,
			out float[] solvedAnglesDeg,
			out float singularityPenalty,
			out string failureReason,
			float[] startAnglesDeg = null,
			float[] trajectoryStartAnglesDeg = null,
			float targetToleranceMeters = -1f)
		{
			solvedAnglesDeg = new float[6];
			singularityPenalty = float.PositiveInfinity;
			failureReason = string.Empty;
			float solveToleranceMeters = ResolveSolveTolerance(targetToleranceMeters);

			if (armController == null || !armController.IsInitialized || !armController.KinematicsReady)
			{
				failureReason = L("机械臂控制器或运动学模型尚未就绪。", "Arm controller or kinematics model is not ready.");
				return false;
			}

			List<float[]> solveSeeds = BuildSolveSeeds(armController, startAnglesDeg);
			float[] evaluationStartAngles = trajectoryStartAnglesDeg != null && trajectoryStartAnglesDeg.Length >= 6
				? ClampAnglesToJointLimits(armController, trajectoryStartAnglesDeg)
				: ClampAnglesToJointLimits(armController, armController.CaptureMeasuredJointAngles());
			bool foundConvergedSolution = false;
			float bestResidual = float.MaxValue;
			float bestSingularityPenalty = float.PositiveInfinity;
			float bestTrajectoryPeakSingularity = float.PositiveInfinity;
			float bestCandidateScore = float.PositiveInfinity;
			float[] bestAngles = null;

			for (int i = 0; i < solveSeeds.Count; i++)
			{
				if (!TrySolveSingleSeed(
					armController,
					targetBasePosition,
					solveSeeds[i],
					out float[] candidateAngles,
					out float candidateResidual,
					out float candidateSingularityPenalty,
					out bool converged,
					solveToleranceMeters))
				{
					continue;
				}

				float candidateTrajectoryPeakSingularity = EstimateTrajectoryPeakSingularity(
					armController,
					evaluationStartAngles,
					candidateAngles);
				float candidateScore = ComputeCandidateScore(
					candidateResidual,
					candidateSingularityPenalty,
					candidateTrajectoryPeakSingularity);
				bool candidateInPreferredBand = candidateTrajectoryPeakSingularity <= settings.preferredTrajectorySingularityPenalty;
				bool bestInPreferredBand = bestTrajectoryPeakSingularity <= settings.preferredTrajectorySingularityPenalty;
				bool candidateIsBetter =
					(converged && !foundConvergedSolution) ||
					(converged == foundConvergedSolution &&
						((candidateInPreferredBand && !bestInPreferredBand) ||
						(candidateInPreferredBand == bestInPreferredBand && candidateScore + 1e-4f < bestCandidateScore)));
				if (!candidateIsBetter)
				{
					continue;
				}

				foundConvergedSolution = converged;
				bestResidual = candidateResidual;
				bestSingularityPenalty = candidateSingularityPenalty;
				bestTrajectoryPeakSingularity = candidateTrajectoryPeakSingularity;
				bestCandidateScore = candidateScore;
				bestAngles = candidateAngles;
			}

			if (!foundConvergedSolution || bestAngles == null)
			{
				if (bestAngles != null
					&& TrySolveNearbyTargetOffsets(
						armController,
						targetBasePosition,
						solveToleranceMeters,
						bestAngles,
						evaluationStartAngles,
						out float[] nearbySolvedAngles,
						out float nearbyResidual,
						out float nearbySingularityPenalty,
						out float nearbyTrajectoryPeakSingularity))
				{
					solvedAnglesDeg = nearbySolvedAngles;
					singularityPenalty = nearbySingularityPenalty;
					return true;
				}

				failureReason = L(
					$"机械臂规划未收敛，最佳残差为 {bestResidual:F4}m。",
					$"Arm planner did not converge. Best residual={bestResidual:F4}m, tolerance={solveToleranceMeters:F4}m.");
				return false;
			}

			solvedAnglesDeg = bestAngles;
			singularityPenalty = bestSingularityPenalty;
			return true;
		}

		private bool TrySolveSingleSeed(
			Arm6DOFFKController armController,
			Vector3 targetBasePosition,
			float[] seedAnglesDeg,
			out float[] solvedAnglesDeg,
			out float residualMeters,
			out float singularityPenalty,
			out bool converged,
			float solveToleranceMeters)
		{
			solvedAnglesDeg = ClampAnglesToJointLimits(armController, seedAnglesDeg);
			singularityPenalty = float.PositiveInfinity;
			converged = false;

			if (solvedAnglesDeg == null || solvedAnglesDeg.Length < 6)
			{
				residualMeters = float.MaxValue;
				return false;
			}

			float[] workingAngles = (float[])solvedAnglesDeg.Clone();
			float[] bestAngles = (float[])workingAngles.Clone();
			float bestResidual = Vector3.Distance(armController.ForwardPoe(workingAngles).position, targetBasePosition);
			float adaptiveLambda = Mathf.Max(1e-4f, settings.dlsLambda);
			float[] stepScales = { 1f, 0.5f, 0.25f, 0.125f };

			for (int iteration = 0; iteration < settings.maxIterations; iteration++)
			{
				Pose currentPose = armController.ForwardPoe(workingAngles);
				Vector3 error = targetBasePosition - currentPose.position;
				float currentResidual = error.magnitude;
				if (currentResidual < bestResidual)
				{
					bestResidual = currentResidual;
					bestAngles = (float[])workingAngles.Clone();
				}

				if (currentResidual <= solveToleranceMeters)
				{
					converged = true;
					bestResidual = currentResidual;
					bestAngles = (float[])workingAngles.Clone();
					break;
				}

				float[,] jacobian = armController.ComputeGeometricJacobian(workingAngles);
				bool acceptedStep = false;
				float[] acceptedAngles = null;
				float acceptedResidual = currentResidual;

				for (int dampingAttempt = 0; dampingAttempt < 4 && !acceptedStep; dampingAttempt++)
				{
					float lambda = adaptiveLambda * Mathf.Pow(4f, dampingAttempt);
					float[] deltaTheta = CalculateDampedLeastSquaresStep(jacobian, error, lambda);
					for (int scaleIndex = 0; scaleIndex < stepScales.Length; scaleIndex++)
					{
						float[] candidateAngles = ApplyDeltaStep(armController, workingAngles, deltaTheta, stepScales[scaleIndex]);
						if (candidateAngles == null)
						{
							continue;
						}

						float candidateResidual = Vector3.Distance(armController.ForwardPoe(candidateAngles).position, targetBasePosition);
						if (candidateResidual < bestResidual)
						{
							bestResidual = candidateResidual;
							bestAngles = candidateAngles;
						}

						if (candidateResidual + 1e-5f < currentResidual)
						{
							acceptedStep = true;
							acceptedAngles = candidateAngles;
							acceptedResidual = candidateResidual;
							adaptiveLambda = Mathf.Max(settings.dlsLambda * 0.5f, lambda * 0.5f);
							break;
						}
					}
				}

				if (!acceptedStep || acceptedAngles == null)
				{
				if (!TryFindNudgeStep(
						armController,
						targetBasePosition,
						workingAngles,
						currentResidual,
						out acceptedAngles,
						out acceptedResidual))
					{
						if (!TryFindCompoundNudgeStep(
							armController,
							targetBasePosition,
							workingAngles,
							currentResidual,
							out acceptedAngles,
							out acceptedResidual))
						{
							break;
						}
					}
				}

				workingAngles = acceptedAngles;
				if (acceptedResidual <= solveToleranceMeters)
				{
					converged = true;
					bestResidual = acceptedResidual;
					bestAngles = (float[])workingAngles.Clone();
					break;
				}
			}

			solvedAnglesDeg = bestAngles;
			residualMeters = bestResidual;
			singularityPenalty = ComputeSingularityPenalty(armController.ComputeGeometricJacobian(bestAngles));
			return true;
		}

		private bool TrySolveNearbyTargetOffsets(
			Arm6DOFFKController armController,
			Vector3 originalTargetBasePosition,
			float solveToleranceMeters,
			float[] primarySeedAnglesDeg,
			float[] secondarySeedAnglesDeg,
			out float[] solvedAnglesDeg,
			out float residualMeters,
			out float singularityPenalty,
			out float trajectoryPeakSingularity)
		{
			solvedAnglesDeg = null;
			residualMeters = float.PositiveInfinity;
			singularityPenalty = float.PositiveInfinity;
			trajectoryPeakSingularity = float.PositiveInfinity;
			if (armController == null || primarySeedAnglesDeg == null || primarySeedAnglesDeg.Length < 6)
			{
				return false;
			}

			List<float[]> probeSeeds = new List<float[]>();
			AddSolveSeed(probeSeeds, ClampAnglesToJointLimits(armController, primarySeedAnglesDeg));
			AddSolveSeed(probeSeeds, ClampAnglesToJointLimits(armController, secondarySeedAnglesDeg));
			AddSolveSeed(probeSeeds, ClampAnglesToJointLimits(armController, armController.CaptureMeasuredJointAngles()));
			AddSolveSeed(probeSeeds, ClampAnglesToJointLimits(armController, armController.configuredHomeJointAnglesDeg));

			float probeDistance = Mathf.Max(0.002f, solveToleranceMeters * Mathf.Max(0.25f, settings.nearbyTargetProbeFactor));
			float[] probeScales = { 0.5f, 1f };
			Vector3[] probeDirections =
			{
				Vector3.right,
				Vector3.left,
				Vector3.up,
				Vector3.down,
				Vector3.forward,
				Vector3.back,
				new Vector3(1f, 0f, 1f).normalized,
				new Vector3(-1f, 0f, 1f).normalized,
				new Vector3(1f, 0f, -1f).normalized,
				new Vector3(-1f, 0f, -1f).normalized
			};

			float bestScore = float.PositiveInfinity;
			for (int scaleIndex = 0; scaleIndex < probeScales.Length; scaleIndex++)
			{
				float offsetDistance = probeDistance * probeScales[scaleIndex];
				for (int directionIndex = 0; directionIndex < probeDirections.Length; directionIndex++)
				{
					Vector3 nearbyTarget = originalTargetBasePosition + (probeDirections[directionIndex] * offsetDistance);
					for (int seedIndex = 0; seedIndex < probeSeeds.Count; seedIndex++)
					{
						float[] probeSeed = probeSeeds[seedIndex];
						if (probeSeed == null || probeSeed.Length < 6)
						{
							continue;
						}

						if (!TrySolveSingleSeed(
							armController,
							nearbyTarget,
							probeSeed,
							out float[] candidateAngles,
							out float candidateResidualToNearbyTarget,
							out float candidateSingularityPenalty,
							out bool converged,
							solveToleranceMeters))
						{
							continue;
						}

						if (!converged || candidateAngles == null)
						{
							continue;
						}

						float actualResidualToOriginal = ComputeResidualToBaseTarget(armController, candidateAngles, originalTargetBasePosition);
						if (actualResidualToOriginal > solveToleranceMeters)
						{
							continue;
						}

						float candidateTrajectoryPeakSingularity = EstimateTrajectoryPeakSingularity(
							armController,
							probeSeed,
							candidateAngles);
						float candidateScore = ComputeCandidateScore(
							actualResidualToOriginal,
							candidateSingularityPenalty,
							candidateTrajectoryPeakSingularity);
						if (candidateScore + 1e-4f >= bestScore)
						{
							continue;
						}

						bestScore = candidateScore;
						solvedAnglesDeg = candidateAngles;
						residualMeters = actualResidualToOriginal;
						singularityPenalty = candidateSingularityPenalty;
						trajectoryPeakSingularity = candidateTrajectoryPeakSingularity;
					}
				}
			}

			return solvedAnglesDeg != null;
		}

		private List<float[]> BuildSolveSeeds(Arm6DOFFKController armController, float[] primarySeedAnglesDeg)
		{
			List<float[]> seeds = new List<float[]>();
			AddSeedFamily(seeds, armController, primarySeedAnglesDeg);

			float[] measuredAngles = armController.CaptureMeasuredJointAngles();
			AddSeedFamily(seeds, armController, measuredAngles);
			AddSeedFamily(seeds, armController, armController.configuredHomeJointAnglesDeg);

			if (measuredAngles != null && measuredAngles.Length >= 6 && armController.configuredHomeJointAnglesDeg != null && armController.configuredHomeJointAnglesDeg.Length >= 6)
			{
				float[] blendedSeed = new float[6];
				for (int i = 0; i < 6; i++)
				{
					blendedSeed[i] = Mathf.Lerp(measuredAngles[i], armController.configuredHomeJointAnglesDeg[i], 0.5f);
				}

				AddSeedFamily(seeds, armController, blendedSeed);
			}

			float[] midRangeSeed = new float[6];
			for (int i = 0; i < 6; i++)
			{
				Vector2 limits = armController.GetJointLimits(i);
				midRangeSeed[i] = 0.5f * (limits.x + limits.y);
			}

			AddSeedFamily(seeds, armController, midRangeSeed);
			return seeds;
		}

		private void AddSeedFamily(List<float[]> seeds, Arm6DOFFKController armController, float[] baseSeed)
		{
			float[] clampedSeed = ClampAnglesToJointLimits(armController, baseSeed);
			AddSolveSeed(seeds, clampedSeed);
			AddWristBiasSeeds(seeds, armController, clampedSeed);
			AddShoulderElbowBiasSeeds(seeds, armController, clampedSeed);
		}

		private void AddWristBiasSeeds(List<float[]> seeds, Arm6DOFFKController armController, float[] baseSeed)
		{
			if (seeds == null || armController == null || baseSeed == null || baseSeed.Length < 6)
			{
				return;
			}

			float[][] wristOffsets =
			{
				new[] { 0f, 0f, 0f, 45f, 25f, -45f },
				new[] { 0f, 0f, 0f, -45f, -25f, 45f },
				new[] { 0f, 0f, 0f, 90f, 35f, 0f },
				new[] { 0f, 0f, 0f, -90f, -35f, 0f },
				new[] { 0f, 0f, 0f, 0f, 35f, 45f },
				new[] { 0f, 0f, 0f, 0f, -35f, -45f },
			};

			for (int variantIndex = 0; variantIndex < wristOffsets.Length; variantIndex++)
			{
				float[] variant = (float[])baseSeed.Clone();
				for (int jointIndex = 0; jointIndex < 6; jointIndex++)
				{
					variant[jointIndex] += wristOffsets[variantIndex][jointIndex];
				}

				AddSolveSeed(seeds, ClampAnglesToJointLimits(armController, variant));
			}

			if (Mathf.Abs(baseSeed[4]) <= 20f)
			{
				float[] positiveBias = (float[])baseSeed.Clone();
				positiveBias[3] += 60f;
				positiveBias[4] = 45f;
				positiveBias[5] -= 60f;
				AddSolveSeed(seeds, ClampAnglesToJointLimits(armController, positiveBias));

				float[] negativeBias = (float[])baseSeed.Clone();
				negativeBias[3] -= 60f;
				negativeBias[4] = -45f;
				negativeBias[5] += 60f;
				AddSolveSeed(seeds, ClampAnglesToJointLimits(armController, negativeBias));
			}
		}

		private void AddShoulderElbowBiasSeeds(List<float[]> seeds, Arm6DOFFKController armController, float[] baseSeed)
		{
			if (seeds == null || armController == null || baseSeed == null || baseSeed.Length < 6)
			{
				return;
			}

			float[][] reachOffsets =
			{
				new[] { 0f, 25f, -35f, 15f, 0f, 0f },
				new[] { 0f, -25f, 35f, -15f, 0f, 0f },
				new[] { 20f, 18f, -28f, 10f, 0f, 0f },
				new[] { -20f, -18f, 28f, -10f, 0f, 0f },
				new[] { 35f, 0f, 0f, 0f, 0f, 0f },
				new[] { -35f, 0f, 0f, 0f, 0f, 0f }
			};

			for (int variantIndex = 0; variantIndex < reachOffsets.Length; variantIndex++)
			{
				float[] variant = (float[])baseSeed.Clone();
				for (int jointIndex = 0; jointIndex < 6; jointIndex++)
				{
					variant[jointIndex] += reachOffsets[variantIndex][jointIndex];
				}

				AddSolveSeed(seeds, ClampAnglesToJointLimits(armController, variant));
			}
		}

		private static void AddSolveSeed(List<float[]> seeds, float[] candidate)
		{
			if (seeds == null || candidate == null || candidate.Length < 6)
			{
				return;
			}

			for (int i = 0; i < seeds.Count; i++)
			{
				if (AreSeedsSimilar(seeds[i], candidate))
				{
					return;
				}
			}

			seeds.Add((float[])candidate.Clone());
		}

		private static bool AreSeedsSimilar(float[] left, float[] right)
		{
			if (left == null || right == null || left.Length < 6 || right.Length < 6)
			{
				return false;
			}

			for (int i = 0; i < 6; i++)
			{
				if (Mathf.Abs(left[i] - right[i]) > 1f)
				{
					return false;
				}
			}

			return true;
		}

		private float[] ApplyDeltaStep(Arm6DOFFKController armController, float[] workingAnglesDeg, float[] deltaThetaRad, float stepScale)
		{
			if (armController == null || workingAnglesDeg == null || deltaThetaRad == null || workingAnglesDeg.Length < 6 || deltaThetaRad.Length < 6)
			{
				return null;
			}

			float[] nextAngles = (float[])workingAnglesDeg.Clone();
			bool changed = false;
			for (int i = 0; i < 6; i++)
			{
				float deltaDeg = Mathf.Clamp(deltaThetaRad[i] * Mathf.Rad2Deg * stepScale, -settings.maxDeltaDegPerIteration, settings.maxDeltaDegPerIteration);
				Vector2 limits = armController.GetJointLimits(i);
				float clamped = Mathf.Clamp(nextAngles[i] + deltaDeg, limits.x, limits.y);
				if (Mathf.Abs(clamped - nextAngles[i]) > 1e-4f)
				{
					changed = true;
				}

				nextAngles[i] = clamped;
			}

			return changed ? nextAngles : null;
		}

		private bool TryFindNudgeStep(
			Arm6DOFFKController armController,
			Vector3 targetBasePosition,
			float[] workingAnglesDeg,
			float currentResidual,
			out float[] acceptedAngles,
			out float acceptedResidual)
		{
			acceptedAngles = null;
			acceptedResidual = currentResidual;
			if (armController == null || workingAnglesDeg == null || workingAnglesDeg.Length < 6)
			{
				return false;
			}

			int[] jointPriority = { 2, 1, 3, 0, 4, 5 };
			float[] nudgeMagnitudes = { settings.maxDeltaDegPerIteration, settings.maxDeltaDegPerIteration * 0.5f };
			float bestResidual = currentResidual;
			float[] bestAngles = null;
			for (int magnitudeIndex = 0; magnitudeIndex < nudgeMagnitudes.Length; magnitudeIndex++)
			{
				float nudgeMagnitude = Mathf.Max(1f, nudgeMagnitudes[magnitudeIndex]);
				for (int priorityIndex = 0; priorityIndex < jointPriority.Length; priorityIndex++)
				{
					int jointIndex = jointPriority[priorityIndex];
					for (int signIndex = 0; signIndex < 2; signIndex++)
					{
						float sign = signIndex == 0 ? -1f : 1f;
						float[] candidateAngles = (float[])workingAnglesDeg.Clone();
						Vector2 limits = armController.GetJointLimits(jointIndex);
						candidateAngles[jointIndex] = Mathf.Clamp(candidateAngles[jointIndex] + sign * nudgeMagnitude, limits.x, limits.y);
						float candidateResidual = ComputeResidualToBaseTarget(armController, candidateAngles, targetBasePosition);
						if (candidateResidual + 1e-5f < bestResidual)
						{
							bestResidual = candidateResidual;
							bestAngles = candidateAngles;
						}
					}
				}
			}

			if (bestAngles == null)
			{
				return false;
			}

			acceptedAngles = bestAngles;
			acceptedResidual = bestResidual;
			return true;
		}

		private bool TryFindCompoundNudgeStep(
			Arm6DOFFKController armController,
			Vector3 targetBasePosition,
			float[] workingAnglesDeg,
			float currentResidual,
			out float[] acceptedAngles,
			out float acceptedResidual)
		{
			acceptedAngles = null;
			acceptedResidual = currentResidual;
			if (armController == null || workingAnglesDeg == null || workingAnglesDeg.Length < 6)
			{
				return false;
			}

			int[,] jointPairs =
			{
				{ 1, 2 },
				{ 1, 3 },
				{ 2, 3 },
				{ 0, 1 },
				{ 2, 4 },
				{ 3, 4 }
			};
			float[] nudgeMagnitudes =
			{
				Mathf.Max(1f, settings.compoundNudgeDeg),
				Mathf.Max(0.5f, settings.compoundNudgeDeg * 0.5f)
			};
			float bestResidual = currentResidual;
			float[] bestAngles = null;
			for (int magnitudeIndex = 0; magnitudeIndex < nudgeMagnitudes.Length; magnitudeIndex++)
			{
				float magnitude = nudgeMagnitudes[magnitudeIndex];
				for (int pairIndex = 0; pairIndex < jointPairs.GetLength(0); pairIndex++)
				{
					int firstJoint = jointPairs[pairIndex, 0];
					int secondJoint = jointPairs[pairIndex, 1];
					for (int firstSignIndex = 0; firstSignIndex < 2; firstSignIndex++)
					{
						float firstSign = firstSignIndex == 0 ? -1f : 1f;
						for (int secondSignIndex = 0; secondSignIndex < 2; secondSignIndex++)
						{
							float secondSign = secondSignIndex == 0 ? -1f : 1f;
							float[] candidateAngles = (float[])workingAnglesDeg.Clone();
							Vector2 firstLimits = armController.GetJointLimits(firstJoint);
							Vector2 secondLimits = armController.GetJointLimits(secondJoint);
							candidateAngles[firstJoint] = Mathf.Clamp(candidateAngles[firstJoint] + firstSign * magnitude, firstLimits.x, firstLimits.y);
							candidateAngles[secondJoint] = Mathf.Clamp(candidateAngles[secondJoint] + secondSign * magnitude, secondLimits.x, secondLimits.y);
							float candidateResidual = ComputeResidualToBaseTarget(armController, candidateAngles, targetBasePosition);
							if (candidateResidual + 1e-5f < bestResidual)
							{
								bestResidual = candidateResidual;
								bestAngles = candidateAngles;
							}
						}
					}
				}
			}

			if (bestAngles == null)
			{
				return false;
			}

			acceptedAngles = bestAngles;
			acceptedResidual = bestResidual;
			return true;
		}

		private float[] ClampAnglesToJointLimits(Arm6DOFFKController armController, float[] sourceAnglesDeg)
		{
			if (armController == null)
			{
				return null;
			}

			float[] clampedAngles = new float[6];
			for (int i = 0; i < 6; i++)
			{
				Vector2 limits = armController.GetJointLimits(i);
				float source = sourceAnglesDeg != null && sourceAnglesDeg.Length > i ? sourceAnglesDeg[i] : 0f;
				clampedAngles[i] = Mathf.Clamp(source, limits.x, limits.y);
			}

			return clampedAngles;
		}

		private int EstimateSampleCountInternal(float[] startAnglesDeg, float[] targetAnglesDeg)
		{
			float maxDelta = 0f;
			for (int i = 0; i < 6; i++)
			{
				maxDelta = Mathf.Max(maxDelta, Mathf.Abs(targetAnglesDeg[i] - startAnglesDeg[i]));
			}

			return Mathf.Clamp(Mathf.CeilToInt(maxDelta / Mathf.Max(1f, settings.sampleSpacingDeg)), 2, 64);
		}

		private float EstimateTrajectoryPeakSingularity(Arm6DOFFKController armController, float[] startAnglesDeg, float[] targetAnglesDeg)
		{
			if (armController == null || startAnglesDeg == null || targetAnglesDeg == null || startAnglesDeg.Length < 6 || targetAnglesDeg.Length < 6)
			{
				return float.PositiveInfinity;
			}

			int sampleCount = EstimateSampleCountInternal(startAnglesDeg, targetAnglesDeg);
			float peakPenalty = 0f;
			for (int sampleIndex = 0; sampleIndex <= sampleCount; sampleIndex++)
			{
				float t = sampleCount <= 0 ? 1f : sampleIndex / (float)sampleCount;
				float[] sampleAngles = new float[6];
				for (int jointIndex = 0; jointIndex < 6; jointIndex++)
				{
					sampleAngles[jointIndex] = Mathf.Lerp(startAnglesDeg[jointIndex], targetAnglesDeg[jointIndex], t);
				}

				float samplePenalty = ComputeSingularityPenalty(armController.ComputeGeometricJacobian(sampleAngles));
				peakPenalty = Mathf.Max(peakPenalty, samplePenalty);
			}

			return peakPenalty;
		}

		private float ComputeCandidateScore(float residualMeters, float finalSingularityPenalty, float trajectoryPeakSingularity)
		{
			float residualCost = residualMeters * 1000f;
			float softExcessPenalty = Mathf.Max(0f, trajectoryPeakSingularity - settings.preferredTrajectorySingularityPenalty);
			return residualCost
				+ (trajectoryPeakSingularity * settings.trajectorySingularityPenaltyWeight)
				+ (finalSingularityPenalty * settings.finalSingularityPenaltyWeight)
				+ (softExcessPenalty * 4f);
		}

		private float ResolveSolveTolerance(float targetToleranceMeters)
		{
			return Mathf.Max(0.001f, targetToleranceMeters > 0f ? targetToleranceMeters : settings.toleranceMeters);
		}

		private float ComputeResidualToBaseTarget(Arm6DOFFKController armController, float[] jointAnglesDeg, Vector3 targetBasePosition)
		{
			if (armController == null || jointAnglesDeg == null || jointAnglesDeg.Length < 6)
			{
				return float.PositiveInfinity;
			}

			return Vector3.Distance(armController.ForwardPoe(jointAnglesDeg).position, targetBasePosition);
		}

		private static float[] CalculateDampedLeastSquaresStep(float[,] jacobian, Vector3 positionError, float lambda)
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

			float lambda2 = Mathf.Max(1e-6f, lambda * lambda);
			jjt[0, 0] += lambda2;
			jjt[1, 1] += lambda2;
			jjt[2, 2] += lambda2;

			if (!TryInvert3x3(jjt, out float[,] inverse))
			{
				return new float[6];
			}

			float[] weightedError = new float[3];
			weightedError[0] = inverse[0, 0] * positionError.x + inverse[0, 1] * positionError.y + inverse[0, 2] * positionError.z;
			weightedError[1] = inverse[1, 0] * positionError.x + inverse[1, 1] * positionError.y + inverse[1, 2] * positionError.z;
			weightedError[2] = inverse[2, 0] * positionError.x + inverse[2, 1] * positionError.y + inverse[2, 2] * positionError.z;

			float[] delta = new float[6];
			for (int i = 0; i < 6; i++)
			{
				delta[i] =
					(linearJacobian[0, i] * weightedError[0]) +
					(linearJacobian[1, i] * weightedError[1]) +
					(linearJacobian[2, i] * weightedError[2]);
			}

			return delta;
		}

		private static float ComputeSingularityPenalty(float[,] jacobian)
		{
			float[,] jjt = new float[3, 3];
			for (int r = 0; r < 3; r++)
			{
				for (int c = 0; c < 3; c++)
				{
					float sum = 0f;
					for (int k = 0; k < 6; k++)
					{
						sum += jacobian[3 + r, k] * jacobian[3 + c, k];
					}

					jjt[r, c] = sum;
				}
			}

			float determinant =
				(jjt[0, 0] * ((jjt[1, 1] * jjt[2, 2]) - (jjt[1, 2] * jjt[2, 1])))
				- (jjt[0, 1] * ((jjt[1, 0] * jjt[2, 2]) - (jjt[1, 2] * jjt[2, 0])))
				+ (jjt[0, 2] * ((jjt[1, 0] * jjt[2, 1]) - (jjt[1, 1] * jjt[2, 0])));
			float manipulability = Mathf.Sqrt(Mathf.Max(1e-8f, determinant));
			return 1f / manipulability;
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

		private static string L(string chinese, string english)
		{
			return RobotSimulationLocalization.Text(chinese, english);
		}
	}
}
