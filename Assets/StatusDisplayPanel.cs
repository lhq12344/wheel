using System.Collections;
using System.Text;
using TMPro;
using UnityEngine;
using UnityEngine.UI;

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
        public RectTransform positionIndicator;

        [Header("Velocity Display")]
        public TextMeshProUGUI linearVelocityText;
        public TextMeshProUGUI angularVelocityText;
        public Image velocityBar;
        public Image angularVelocityBar;

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
        private readonly StringBuilder _sb = new StringBuilder();

        void Start()
        {
            if (manager == null)
            {
                manager = RobotSimulationManager.Instance;
            }
        }

        void Update()
        {
            if (manager != null)
            {
                RobotSimulationLocalization.SetLanguage(manager.displayLanguage);
            }

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

            if (positionText != null)
            {
                positionText.text = $"X: {state.position.x:F2}  Z: {state.position.z:F2}";
            }

            if (rotationText != null)
            {
                rotationText.text = $"{RobotSimulationLocalization.Text("偏航", "Yaw")}: {state.rotation:F1}deg";
            }

            if (linearVelocityText != null)
            {
                linearVelocityText.text = $"{RobotSimulationLocalization.Text("线速度", "V")}: {state.linearVelocity:F3} m/s";
            }

            if (angularVelocityText != null)
            {
                angularVelocityText.text = $"{RobotSimulationLocalization.Text("角速度", "W")}: {state.angularVelocity:F3} rad/s";
            }

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

            if (leftWheelText != null)
            {
                leftWheelText.text = $"{RobotSimulationLocalization.Text("左轮", "L")}: {state.leftWheelVelocity:F3} m/s";
            }

            if (rightWheelText != null)
            {
                rightWheelText.text = $"{RobotSimulationLocalization.Text("右轮", "R")}: {state.rightWheelVelocity:F3} m/s";
            }

            if (leftWheelBar != null)
            {
                leftWheelBar.fillAmount = Mathf.Abs(state.leftWheelVelocity) / maxWheelVelocity;
            }

            if (rightWheelBar != null)
            {
                rightWheelBar.fillAmount = Mathf.Abs(state.rightWheelVelocity) / maxWheelVelocity;
            }

            if (targetInfoText != null)
            {
                if (state.hasTargetPoint)
                {
                    float dist = Vector3.Distance(state.position, state.targetPoint);
                    targetInfoText.text = RobotSimulationLocalization.Text($"目标距离: {dist:F2}m", $"Target: {dist:F2}m away");
                    targetInfoText.color = normalColor;
                }
                else
                {
                    targetInfoText.text = RobotSimulationLocalization.Text("目标: 无", "Target: None");
                    targetInfoText.color = warningColor;
                }
            }

            if (targetDirectionIndicator != null && state.hasTargetPoint)
            {
                Vector3 dir = state.targetPoint - state.position;
                dir.y = 0f;
                float angle = Vector3.SignedAngle(Vector3.forward, dir, Vector3.up);
                targetDirectionIndicator.localRotation = Quaternion.Euler(0f, 0f, -angle);
            }

            if (jointStatusTexts != null && state.jointAngles != null)
            {
                for (int i = 0; i < Mathf.Min(state.jointAngles.Length, jointStatusTexts.Length); i++)
                {
                    if (jointStatusTexts[i] == null)
                    {
                        continue;
                    }

                    float angle = state.jointAngles[i];
                    float range = i < jointDisplayRanges.Length ? jointDisplayRanges[i] : 180f;
                    jointStatusTexts[i].text = $"{RobotSimulationLocalization.Text("关节", "J")}{i}: {angle:F1}deg";

                    if (jointProgressBars != null && i < jointProgressBars.Length && jointProgressBars[i] != null)
                    {
                        jointProgressBars[i].fillAmount = Mathf.Abs(angle) / range;
                    }
                }
            }

            if (simulationTimeText != null)
            {
                simulationTimeText.text = $"{RobotSimulationLocalization.Text("时间", "Time")}: {state.simulationTime:F1}s";
            }

            if (fpsText != null)
            {
                fpsText.text = $"FPS: {_currentFps:F0}";
                fpsText.color = _currentFps < 30 ? warningColor : normalColor;
            }

            if (controlModeText != null)
            {
                controlModeText.text = $"{RobotSimulationLocalization.Text("模式", "Mode")}: {RobotSimulationLocalization.ControlMode(state.controlMode)}";
            }
        }

        private Color GetVelocityColor(float current, float max)
        {
            float ratio = current / max;
            if (ratio < 0.5f) return normalColor;
            if (ratio < 0.8f) return warningColor;
            return errorColor;
        }

        public void ShowAlert(string message, float duration = 2f)
        {
            if (alertText != null)
            {
                alertText.text = message;
                alertText.gameObject.SetActive(true);
                StartCoroutine(HideAlertAfter(duration));
            }
        }

        private IEnumerator HideAlertAfter(float duration)
        {
            yield return new WaitForSeconds(duration);
            if (alertText != null)
            {
                alertText.gameObject.SetActive(false);
            }
        }

        public void SetVelocityRange(float linearMax, float angularMax)
        {
            maxLinearVelocity = linearMax;
            maxAngularVelocity = angularMax;
        }

        public void UpdatePositionIndicator(Vector3 worldPos, float mapSize)
        {
            if (positionIndicator != null)
            {
                float x = Mathf.Clamp(worldPos.x / mapSize, -0.5f, 0.5f);
                float y = Mathf.Clamp(worldPos.z / mapSize, -0.5f, 0.5f);
                positionIndicator.anchorMin = new Vector2(0.5f + x, 0.5f + y);
                positionIndicator.anchorMax = positionIndicator.anchorMin;
            }
        }
    }
}
