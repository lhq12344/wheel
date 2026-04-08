using System;
using UnityEngine;

namespace RobotSimulation
{
	public enum RobotSimulationLanguage
	{
		Chinese,
		English
	}

	public static class RobotSimulationLocalization
	{
		public static RobotSimulationLanguage CurrentLanguage { get; private set; } = RobotSimulationLanguage.Chinese;

		public static void SetLanguage(RobotSimulationLanguage language)
		{
			CurrentLanguage = language;
		}

		public static string Text(string chinese, string english)
		{
			return CurrentLanguage == RobotSimulationLanguage.Chinese ? chinese : english;
		}

		public static string ControlMode(string mode)
		{
			if (string.IsNullOrEmpty(mode))
			{
				return Text("未知", "Unknown");
			}

			switch (mode)
			{
				case "TargetPoint":
					return Text("目标点", "Target Point");
				case "TargetYaw":
					return Text("目标朝向", "Target Yaw");
				default:
					return mode;
			}
		}

		public static string Found(bool found)
		{
			return found ? Text("已找到", "Found") : Text("未找到", "Not Found");
		}

		public static string PlanningStage(RobotPlanningStage stage)
		{
			switch (stage)
			{
				case RobotPlanningStage.None:
					return Text("无", "None");
				case RobotPlanningStage.TargetCheck:
					return Text("目标检查", "TargetCheck");
				case RobotPlanningStage.CurrentBaseReachabilityCheck:
					return Text("当前底盘可达性检查", "CurrentBaseReachabilityCheck");
				case RobotPlanningStage.DockingSearch:
					return Text("停靠位搜索", "DockingSearch");
				case RobotPlanningStage.BasePlanning:
					return Text("底盘规划", "BasePlanning");
				case RobotPlanningStage.BaseShadowValidation:
					return Text("底盘影子验证", "BaseShadowValidation");
				case RobotPlanningStage.BaseExecution:
					return Text("底盘执行", "BaseExecution");
				case RobotPlanningStage.BaseSettling:
					return Text("底盘停稳", "BaseSettling");
				case RobotPlanningStage.ArmPlanning:
					return Text("机械臂规划", "ArmPlanning");
				case RobotPlanningStage.ArmShadowValidation:
					return Text("机械臂影子验证", "ArmShadowValidation");
				case RobotPlanningStage.ArmExecution:
					return Text("机械臂执行", "ArmExecution");
				case RobotPlanningStage.LocalReplan:
					return Text("局部重规划", "LocalReplan");
				case RobotPlanningStage.Completed:
					return Text("已完成", "Completed");
				case RobotPlanningStage.Failed:
					return Text("失败", "Failed");
				default:
					return stage.ToString();
			}
		}

		public static string PlanningStage(string stageName)
		{
			if (string.IsNullOrEmpty(stageName))
			{
				return PlanningStage(RobotPlanningStage.None);
			}

			return Enum.TryParse(stageName, out RobotPlanningStage stage)
				? PlanningStage(stage)
				: stageName;
		}
	}
}
