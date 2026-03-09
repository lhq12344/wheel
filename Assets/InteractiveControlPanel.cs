using UnityEngine;
using UnityEngine.UI;
using TMPro;

namespace RobotSimulation
{
    /// <summary>
    /// Interactive control panel for robot manipulation
    /// </summary>
    public class InteractiveControlPanel : MonoBehaviour
    {
        [Header("Manager Reference")]
        public RobotSimulationManager manager;

        [Header("Navigation Controls")]
        public Slider targetYawSlider;
        public TextMeshProUGUI targetYawValueText;
        public Button setYawButton;
        public Button clearTargetButton;
        public Button emergencyStopButton;

        [Header("Movement Controls")]
        public Slider velocitySlider;
        public Slider angularVelocitySlider;
        public TextMeshProUGUI velocityValueText;
        public TextMeshProUGUI angularValueText;

        [Header("Joint Controls")]
        public Slider[] jointSliders;
        public TextMeshProUGUI[] jointValueTexts;
        public Button[] jointSetButtons;

        [Header("Simulation Controls")]
        public Slider simulationSpeedSlider;
        public TextMeshProUGUI simulationSpeedText;
        public Toggle pauseToggle;

        private float[] _jointTargetAngles;

        void Start()
        {
            if (manager == null)
            {
                manager = RobotSimulationManager.Instance;
            }

            SetupControls();
            _jointTargetAngles = new float[6];
        }

        void Update()
        {
            UpdateJointControlDisplay();
        }

        private void SetupControls()
        {
            // Target Yaw Slider
            if (targetYawSlider != null)
            {
                targetYawSlider.minValue = -180f;
                targetYawSlider.maxValue = 180f;
                targetYawSlider.value = 0f;
                targetYawSlider.onValueChanged.AddListener(OnTargetYawChanged);
            }

            // Set Yaw Button
            if (setYawButton != null)
            {
                setYawButton.onClick.AddListener(OnSetYawClicked);
            }

            // Clear Target Button
            if (clearTargetButton != null)
            {
                clearTargetButton.onClick.AddListener(OnClearTargetClicked);
            }

            // Emergency Stop Button
            if (emergencyStopButton != null)
            {
                emergencyStopButton.onClick.AddListener(OnEmergencyStopClicked);
            }

            // Velocity Sliders
            if (velocitySlider != null)
            {
                velocitySlider.minValue = 0f;
                velocitySlider.maxValue = 2.0f;
                velocitySlider.value = 0.5f;
                velocitySlider.onValueChanged.AddListener(OnVelocityChanged);
            }

            if (angularVelocitySlider != null)
            {
                angularVelocitySlider.minValue = 0f;
                angularVelocitySlider.maxValue = 5.0f;
                angularVelocitySlider.value = 2.0f;
                angularVelocitySlider.onValueChanged.AddListener(OnAngularVelocityChanged);
            }

            // Simulation Speed Slider
            if (simulationSpeedSlider != null)
            {
                simulationSpeedSlider.minValue = 0.1f;
                simulationSpeedSlider.maxValue = 3.0f;
                simulationSpeedSlider.value = 1.0f;
                simulationSpeedSlider.onValueChanged.AddListener(OnSimulationSpeedChanged);
            }

            // Pause Toggle
            if (pauseToggle != null)
            {
                pauseToggle.onValueChanged.AddListener(OnPauseToggled);
            }

            // Joint Controls
            if (jointSliders != null)
            {
                for (int i = 0; i < jointSliders.Length; i++)
                {
                    int index = i;
                    if (jointSliders[i] != null)
                    {
                        jointSliders[i].minValue = -180f;
                        jointSliders[i].maxValue = 180f;
                        jointSliders[i].value = 0f;
                        jointSliders[i].onValueChanged.AddListener((val) => OnJointSliderChanged(index, val));
                    }
                }
            }

            if (jointSetButtons != null)
            {
                for (int i = 0; i < jointSetButtons.Length; i++)
                {
                    int index = i;
                    if (jointSetButtons[i] != null)
                    {
                        jointSetButtons[i].onClick.AddListener(() => OnJointSetClicked(index));
                    }
                }
            }
        }

        private void OnTargetYawChanged(float value)
        {
            if (targetYawValueText != null)
            {
                targetYawValueText.text = $"{value:F0}°";
            }
        }

        private void OnSetYawClicked()
        {
            if (manager != null && targetYawSlider != null)
            {
                manager.SetTargetYaw(targetYawSlider.value);
            }
        }

        private void OnClearTargetClicked()
        {
            if (manager != null)
            {
                manager.ClearTarget();
            }
        }

        private void OnEmergencyStopClicked()
        {
            if (manager != null)
            {
                manager.EmergencyStop();
            }
        }

        private void OnVelocityChanged(float value)
        {
            if (manager != null && manager.diffDriveController != null)
            {
                manager.diffDriveController.vMax = value;
            }

            if (velocityValueText != null)
            {
                velocityValueText.text = $"{value:F2} m/s";
            }
        }

        private void OnAngularVelocityChanged(float value)
        {
            if (manager != null && manager.diffDriveController != null)
            {
                manager.diffDriveController.wMax = value;
            }

            if (angularValueText != null)
            {
                angularValueText.text = $"{value:F2} rad/s";
            }
        }

        private void OnSimulationSpeedChanged(float value)
        {
            if (manager != null)
            {
                manager.simulationSpeed = value;
            }

            if (simulationSpeedText != null)
            {
                simulationSpeedText.text = $"{value:F1}x";
            }
        }

        private void OnPauseToggled(bool isPaused)
        {
            if (manager != null)
            {
                manager.enableSimulation = !isPaused;
            }
        }

        private void OnJointSliderChanged(int jointIndex, float value)
        {
            _jointTargetAngles[jointIndex] = value;

            if (jointValueTexts != null && jointIndex < jointValueTexts.Length && jointValueTexts[jointIndex] != null)
            {
                jointValueTexts[jointIndex].text = $"{value:F0}°";
            }
        }

        private void OnJointSetClicked(int jointIndex)
        {
            if (manager != null)
            {
                manager.SetJointTarget(jointIndex, _jointTargetAngles[jointIndex]);
            }
        }

        private void UpdateJointControlDisplay()
        {
            if (manager == null || manager.RobotState == null) return;

            var state = manager.RobotState;
            if (state.jointAngles != null && jointValueTexts != null)
            {
                for (int i = 0; i < Mathf.Min(state.jointAngles.Length, jointValueTexts.Length); i++)
                {
                    if (jointValueTexts[i] != null)
                    {
                        jointValueTexts[i].text = $"{state.jointAngles[i]:F1}°";
                    }
                }
            }
        }

        /// <summary>
        /// Programmatic control - set target yaw
        /// </summary>
        public void SetTargetYaw(float yawDegrees)
        {
            if (targetYawSlider != null)
            {
                targetYawSlider.value = yawDegrees;
            }
            if (manager != null)
            {
                manager.SetTargetYaw(yawDegrees);
            }
        }

        /// <summary>
        /// Programmatic control - set velocity limits
        /// </summary>
        public void SetVelocityLimits(float vMax, float wMax)
        {
            if (velocitySlider != null)
            {
                velocitySlider.value = vMax;
            }
            if (angularVelocitySlider != null)
            {
                angularVelocitySlider.value = wMax;
            }
        }

        /// <summary>
        /// Programmatic control - set all joint targets
        /// </summary>
        public void SetAllJointTargets(float[] angles)
        {
            if (manager == null || angles == null) return;

            for (int i = 0; i < Mathf.Min(angles.Length, jointSliders?.Length ?? 0); i++)
            {
                manager.SetJointTarget(i, angles[i]);
            }
        }
    }
}
