using System.Text;
using UnityEngine;

namespace RobotSimulation
{
	public class RobotModelResidualTracker : MonoBehaviour
	{
		[SerializeField] private RobotSimulationManager manager;
		[SerializeField] private float basePositionResidualMeters;
		[SerializeField] private float baseYawResidualDeg;
		[SerializeField] private float armPositionResidualMeters;
		[SerializeField] private string residualSummary = "Residual tracker idle.";

		public float BasePositionResidualMeters => basePositionResidualMeters;
		public float BaseYawResidualDeg => baseYawResidualDeg;
		public float ArmPositionResidualMeters => armPositionResidualMeters;
		public string ResidualSummary => residualSummary;

		public void Configure(RobotSimulationManager robotManager)
		{
			manager = robotManager;
		}

		public void Tick()
		{
			if (manager == null)
			{
				manager = GetComponent<RobotSimulationManager>();
			}

			if (manager == null)
			{
				residualSummary = "Residual tracker cannot find RobotSimulationManager.";
				return;
			}

			DiffDriveTwinController baseController = manager.diffDriveController;
			if (baseController != null && baseController.rb != null)
			{
				if (baseController.hasTargetPoint)
				{
					basePositionResidualMeters = Vector3.Distance(
						new Vector3(baseController.rb.position.x, 0f, baseController.rb.position.z),
						new Vector3(baseController.targetPointWorld.x, 0f, baseController.targetPointWorld.z));
				}
				else
				{
					basePositionResidualMeters = 0f;
				}

				baseYawResidualDeg = Mathf.Abs(Mathf.DeltaAngle(baseController.rb.rotation.eulerAngles.y + baseController.headingOffsetDeg, baseController.targetYawDeg));
			}

			Arm6DOFFKController armController = manager.arm6DOFFKController;
			armPositionResidualMeters = armController != null ? armController.ModelVsMeasuredPositionError : 0f;

			StringBuilder builder = new StringBuilder();
			builder.Append("Base pos=").Append(basePositionResidualMeters.ToString("F3")).Append("m, ");
			builder.Append("base yaw=").Append(baseYawResidualDeg.ToString("F2")).Append("deg, ");
			builder.Append("arm=").Append(armPositionResidualMeters.ToString("F4")).Append("m");
			residualSummary = builder.ToString();
		}
	}

	public class OnlineModelCalibration : MonoBehaviour
	{
		[SerializeField] private RobotModelResidualTracker residualTracker;
		[SerializeField] private float baseLinearScale = 1f;
		[SerializeField] private float baseAngularScale = 1f;
		[SerializeField] private float armZeroOffsetBlend;
		[SerializeField] private string calibrationSummary = "Calibration idle.";

		public float BaseLinearScale => baseLinearScale;
		public float BaseAngularScale => baseAngularScale;
		public float ArmZeroOffsetBlend => armZeroOffsetBlend;
		public string CalibrationSummary => calibrationSummary;

		public void Configure(RobotModelResidualTracker tracker)
		{
			residualTracker = tracker;
		}

		public void Tick()
		{
			if (residualTracker == null)
			{
				residualTracker = GetComponent<RobotModelResidualTracker>();
			}

			if (residualTracker == null)
			{
				calibrationSummary = "Calibration tracker not configured.";
				return;
			}

			float baseResidual = residualTracker.BasePositionResidualMeters;
			float yawResidual = residualTracker.BaseYawResidualDeg;
			float armResidual = residualTracker.ArmPositionResidualMeters;

			baseLinearScale = Mathf.Clamp(1f + Mathf.Clamp(baseResidual, -0.2f, 0.2f) * 0.05f, 0.9f, 1.1f);
			baseAngularScale = Mathf.Clamp(1f + Mathf.Clamp(yawResidual / 45f, -0.2f, 0.2f) * 0.05f, 0.9f, 1.1f);
			armZeroOffsetBlend = Mathf.Clamp01(Mathf.Lerp(armZeroOffsetBlend, armResidual * 2f, 0.05f));
			calibrationSummary = $"BaseScale=({baseLinearScale:F3}, {baseAngularScale:F3}) ArmBlend={armZeroOffsetBlend:F3}";
		}
	}
}
