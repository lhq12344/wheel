using System.Collections.Generic;
using System.Text;
using UnityEngine;

namespace RobotSimulation
{
	public static class RobotSimulationBootstrapContract
	{
		public static bool TryValidateExplicitContract(RobotSimulationManager manager, out string report)
		{
			List<string> issues = new List<string>();
			CollectExplicitContractIssues(manager, issues);
			if (issues.Count == 0)
			{
				report = string.Empty;
				return true;
			}

			StringBuilder builder = new StringBuilder();
			builder.AppendLine("[RobotSimulation] Explicit bootstrap contract validation failed:");
			for (int i = 0; i < issues.Count; i++)
			{
				builder.Append("- ").AppendLine(issues[i]);
			}

			report = builder.ToString().TrimEnd();
			return false;
		}

		public static void CollectExplicitContractIssues(RobotSimulationManager manager, List<string> issues)
		{
			if (issues == null)
			{
				return;
			}

			if (manager == null)
			{
				issues.Add("RobotSimulationManager is missing.");
				return;
			}

			if (manager.diffDriveController == null)
			{
				issues.Add("diffDriveController reference is missing.");
			}
			else if (manager.diffDriveController.rb == null)
			{
				issues.Add("DiffDriveTwinController.rb must be assigned.");
			}

			if (manager.armJointControllers == null || manager.armJointControllers.Length < 6)
			{
				issues.Add("armJointControllers must contain six ordered joint controllers.");
			}
			else
			{
				for (int i = 0; i < 6; i++)
				{
					OneJointTrapezoidController controller = manager.armJointControllers[i];
					if (controller == null)
					{
						issues.Add($"armJointControllers[{i}] is missing.");
						continue;
					}

					if (controller.joint == null || controller.joint.jointPosition.dofCount <= 0)
					{
						issues.Add($"armJointControllers[{i}] must reference a valid articulation joint.");
					}
				}
			}

			if (manager.armBinder == null)
			{
				issues.Add("armBinder reference is missing.");
			}
			else
			{
				if (manager.armBinder.carMount == null)
				{
					issues.Add("ArticulationArmBindToCar.carMount must be assigned.");
				}

				if (manager.armBinder.armRoot == null)
				{
					issues.Add("ArticulationArmBindToCar.armRoot must be assigned.");
				}
			}

			if (manager.arm6DOFFKController == null)
			{
				issues.Add("arm6DOFFKController reference is missing.");
			}

			if (manager.arm6DOFIKController == null)
			{
				issues.Add("arm6DOFIKController reference is missing.");
			}
			else if (manager.arm6DOFFKController != null && manager.arm6DOFIKController.armController != manager.arm6DOFFKController)
			{
				issues.Add("Arm6DOFIKController.armController must point to arm6DOFFKController.");
			}

			if (manager.armCollisionMonitor == null)
			{
				issues.Add("armCollisionMonitor reference is missing.");
			}
			else
			{
				if (manager.armCollisionMonitor.armRoot == null)
				{
					issues.Add("ArmCollisionMonitor.armRoot must be assigned.");
				}

				if (manager.armCollisionMonitor.forbiddenRoot == null)
				{
					issues.Add("ArmCollisionMonitor.forbiddenRoot must be assigned.");
				}
			}

			ValidateManagerComponent(manager.trajectoryPlanner, manager, "trajectoryPlanner", issues);
			ValidateManagerComponent(manager.residualTracker, manager, "residualTracker", issues);
			ValidateManagerComponent(manager.onlineCalibration, manager, "onlineCalibration", issues);
			ValidateManagerComponent(manager.shadowRobotVisualizer, manager, "shadowRobotVisualizer", issues);
		}

		private static void ValidateManagerComponent(Component component, RobotSimulationManager manager, string name, List<string> issues)
		{
			if (component == null)
			{
				issues.Add($"{name} reference is missing.");
				return;
			}

			if (manager != null && component.gameObject != manager.gameObject)
			{
				issues.Add($"{name} must live on the RobotSimulationManager GameObject.");
			}
		}
	}
}
