using UnityEngine;
using UnityEngine.UI;
using TMPro;
using System.Text;

namespace RobotSimulation
{
    /// <summary>
    /// Status display panel showing robot telemetry and state information
    /// </summary>
    public class StatusDisplayPanel : MonoBehaviour
    {
        [Header("Manager Reference")]
        public RobotSimulationManager manager;

        [Header("Position Display")]
        public TextMeshProUGUI positionText;
        public TextMeshProUGUI rotationText;
        public RectTransform positionIndicator; // Visual indicator on mini-map

        [Header("Velocity Display")]
        public TextMeshProUGUI linearVelocityText;
        public TextMeshProUGUI angularVelocityText;
        public Image velocityBar; // Linear velocity progress bar
        public Image angularVelocityBar; // Angular velocity progress bar

        [Header("Wheel Display")]
        public TextMeshProUGUI leftWheelText;
        public TextMeshProUGUI rightWheelText;
        public Image leftWheelBar;
        public Image rightWheelBar;

        [Header("Target Display")]
        public TextMeshProUGUI targetInfoText;
        public Image targetDistanceBar;
        public RectTransform targetDirectionIndicator;

        [Header("Joint Display")]
        public TextMeshProUGUI[] jointStatusTexts;
        public Image[] jointProgressBars;
        public float[] jointDisplayRanges = new float[] { 180f, 180f, 180f, 180f, 180f, 180f };

        [Header("Simulation Display")]
        public TextMeshProUGUI simulationTimeText;
        public TextMeshProUGUI fpsText;
        public TextMeshProUGUI controlModeText;

        [Header("Alert Display")]
        public TextMeshProUGUI alertText;
        public Image alertPanel;

        [Header("Display Settings")]
        public float maxLinearVelocity = 1.0f;
        public float maxAngularVelocity = 3.0f;
        public float maxWheelVelocity = 1.0f;
        public Color normalColor = Color.white;
        public Color warningColor = Color.yellow;
        public Color errorColor = Color.red;

        private float _lastFpsUpdate;
        private int _frameCount;
        private float _currentFps;
        private StringBuilder _sb = new StringBuilder();

        void Start()
        {
            if (manager == null)
            {
                manager = RobotSimulationManager.Instance;
            }
        }

        void Update()
        {
            UpdateFPS();
            UpdateDisplay();
        }

        private void UpdateFPS()
        {
            _frameCount++;
            float time = Time.realtimeSinceStartup;

            if (time >= _lastFpsUpdate + 0.5f)
            {
                _currentFps = _frameCount / (time - _lastFpsUpdate);
                _frameCount = 0;
                _lastFpsUpdate = time;
            }
        }

        private void UpdateDisplay()
        {
            if (manager == null || manager.RobotState == null) return;

            var state = manager.RobotState;

            // Position & Rotation
            if (positionText != null)
            {
                positionText.text = $"X: {state.position.x:F2}  Z: {state.position.z:F2}";
            }

            if (rotationText != null)
            {
                rotationText.text = $"Yaw: {state.rotation:F1}°";
            }

            // Velocity
            if (linearVelocityText != null)
            {
                linearVelocityText.text = $"V: {state.linearVelocity:F3} m/s";
            }
            if (angularVelocityText != null)
            {
                angularVelocityText.text = $"ω: {state.angularVelocity:F3} rad/s";
            }

            // Velocity bars
            if (velocityBar != null)
            {
                velocityBar.fillAmount = Mathf.Abs(state.linearVelocity) / maxLinearVelocity;
                velocityBar.color = GetVelocityColor(Mathf.Abs(state.linearVelocity), maxLinearVelocity);
            }

            if (angularVelocityBar != null)
            {
                angularVelocityBar.fillAmount = Mathf.Abs(state.angularVelocity) / maxAngularVelocity;
                angularVelocityBar.color = GetVelocityColor(Mathf.Abs(state.angularVelocity), maxAngularVelocity);
            }

            // Wheels
            if (leftWheelText != null)
            {
                leftWheelText.text = $"L: {state.leftWheelVelocity:F3} m/s";
            }
            if (rightWheelText != null)
            {
                rightWheelText.text = $"R: {state.rightWheelVelocity:F3} m/s";
            }

            if (leftWheelBar != null)
            {
                leftWheelBar.fillAmount = Mathf.Abs(state.leftWheelVelocity) / maxWheelVelocity;
            }

            if (rightWheelBar != null)
            {
                rightWheelBar.fillAmount = Mathf.Abs(state.rightWheelVelocity) / maxWheelVelocity;
            }

            // Target info
            if (targetInfoText != null)
            {
                if (state.hasTargetPoint)
                {
                    float dist = Vector3.Distance(state.position, state.targetPoint);
                    targetInfoText.text = $"Target: {dist:F2}m away";
                    targetInfoText.color = normalColor;
                }
                else
                {
                    targetInfoText.text = "Target: None";
                    targetInfoText.color = warningColor;
                }
            }

            // Target direction indicator
            if (targetDirectionIndicator != null && state.hasTargetPoint)
            {
                Vector3 dir = state.targetPoint - state.position;
                dir.y = 0;
                float angle = Vector3.SignedAngle(Vector3.forward, dir, Vector3.up);
                targetDirectionIndicator.localRotation = Quaternion.Euler(0, 0, -angle);
            }

            // Joint status
            if (jointStatusTexts != null && state.jointAngles != null)
            {
                for (int i = 0; i < Mathf.Min(state.jointAngles.Length, jointStatusTexts.Length); i++)
                {
                    if (jointStatusTexts[i] != null)
                    {
                        float angle = state.jointAngles[i];
                        float range = i < jointDisplayRanges.Length ? jointDisplayRanges[i] : 180f;
                        jointStatusTexts[i].text = $"J{i}: {angle:F1}°";

                        if (jointProgressBars != null && i < jointProgressBars.Length && jointProgressBars[i] != null)
                        {
                            jointProgressBars[i].fillAmount = Mathf.Abs(angle) / range;
                        }
                    }
                }
            }

            // Simulation info
            if (simulationTimeText != null)
            {
                simulationTimeText.text = $"Time: {state.simulationTime:F1}s";
            }

            if (fpsText != null)
            {
                fpsText.text = $"FPS: {_currentFps:F0}";
                fpsText.color = _currentFps < 30 ? warningColor : normalColor;
            }

            if (controlModeText != null)
            {
                controlModeText.text = $"Mode: {state.controlMode}";
            }
        }

        private Color GetVelocityColor(float current, float max)
        {
            float ratio = current / max;
            if (ratio < 0.5f) return normalColor;
            if (ratio < 0.8f) return warningColor;
            return errorColor;
        }

        /// <summary>
        /// Show an alert message
        /// </summary>
        public void ShowAlert(string message, float duration = 2f)
        {
            if (alertText != null)
            {
                alertText.text = message;
                alertText.gameObject.SetActive(true);
                StartCoroutine(HideAlertAfter(duration));
            }
        }

        private System.Collections.IEnumerator HideAlertAfter(float duration)
        {
            yield return new WaitForSeconds(duration);
            if (alertText != null)
            {
                alertText.gameObject.SetActive(false);
            }
        }

        /// <summary>
        /// Set velocity display range
        /// </summary>
        public void SetVelocityRange(float linearMax, float angularMax)
        {
            maxLinearVelocity = linearMax;
            maxAngularVelocity = angularMax;
        }

        /// <summary>
        /// Update mini-map position indicator
        /// </summary>
        public void UpdatePositionIndicator(Vector3 worldPos, float mapSize)
        {
            if (positionIndicator != null)
            {
                // Assuming map is centered at origin, Z-forward
                positionIndicator.anchoredPosition = new Vector2(
                    worldPos.x * mapSize,
                    worldPos.z * mapSize
                );
            }
        }

        /// <summary>
        /// Get formatted status report
        /// </summary>
        public string GetStatusReport()
        {
            if (manager == null || manager.RobotState == null) return "No robot connected";

            var state = manager.RobotState;
            _sb.Clear();
            _sb.AppendLine("=== Robot Status Report ===");
            _sb.AppendLine($"Time: {state.simulationTime:F2}s");
            _sb.AppendLine($"Mode: {state.controlMode}");
            _sb.AppendLine($"Position: ({state.position.x:F3}, {state.position.z:F3})");
            _sb.AppendLine($"Rotation: {state.rotation:F1}°");
            _sb.AppendLine($"Linear Velocity: {state.linearVelocity:F4} m/s");
            _sb.AppendLine($"Angular Velocity: {state.angularVelocity:F4} rad/s");
            _sb.AppendLine($"Left Wheel: {state.leftWheelVelocity:F4} m/s");
            _sb.AppendLine($"Right Wheel: {state.rightWheelVelocity:F4} m/s");
            _sb.AppendLine($"Target: {(state.hasTargetPoint ? "Set" : "None")}");

            if (state.jointAngles != null && state.jointAngles.Length > 0)
            {
                _sb.AppendLine("Joint Angles:");
                for (int i = 0; i < state.jointAngles.Length; i++)
                {
                    _sb.AppendLine($"  J{i}: {state.jointAngles[i]:F2}°");
                }
            }

            return _sb.ToString();
        }
    }
}
