using UnityEngine;
using UnityEngine.UI;
using TMPro;
using System.IO;
using System.Text;

namespace RobotSimulation
{
	/// <summary>
	/// Configuration manager for saving, loading, and managing robot configurations
	/// </summary>
	public class ConfigManager : MonoBehaviour
	{
		public static ConfigManager Instance { get; private set; }

		[Header("Manager Reference")]
		public RobotSimulationManager robotManager;

		[Header("UI References")]
		public TMP_InputField configNameInput;
		public TextMeshProUGUI configListText;
		public Button saveButton;
		public Button loadButton;
		public Button deleteButton;
		public Button refreshButton;
		public Button exportButton;
		public Button importButton;

		[Header("Settings")]
		public string configDirectory = "RobotConfigs";
		public string defaultConfigName = "default";

		private string _configFolderPath;
		private string[] _availableConfigs;
		private string _selectedConfig;

		public string ConfigFolderPath => _configFolderPath;

		void Awake()
		{
			if (Instance == null)
			{
				Instance = this;
			}
			else
			{
				Destroy(gameObject);
				return;
			}
		}

		void Start()
		{
			if (robotManager == null)
			{
				robotManager = RobotSimulationManager.Instance;
			}

			SetupConfigDirectory();
			SetupButtonListeners();
			RefreshConfigList();
		}

		private void SetupConfigDirectory()
		{
			_configFolderPath = Path.Combine(Application.persistentDataPath, configDirectory);

			if (!Directory.Exists(_configFolderPath))
			{
				Directory.CreateDirectory(_configFolderPath);
				Debug.Log($"[ConfigManager] Created config directory: {_configFolderPath}");
			}
		}

		private void SetupButtonListeners()
		{
			if (saveButton != null)
			{
				saveButton.onClick.AddListener(SaveCurrentConfig);
			}

			if (loadButton != null)
			{
				loadButton.onClick.AddListener(LoadSelectedConfig);
			}

			if (deleteButton != null)
			{
				deleteButton.onClick.AddListener(DeleteSelectedConfig);
			}

			if (refreshButton != null)
			{
				refreshButton.onClick.AddListener(RefreshConfigList);
			}

			if (exportButton != null)
			{
				exportButton.onClick.AddListener(ExportConfig);
			}

			if (importButton != null)
			{
				importButton.onClick.AddListener(ImportConfig);
			}
		}

		/// <summary>
		/// Get list of available configuration files
		/// </summary>
		public void RefreshConfigList()
		{
			if (!Directory.Exists(_configFolderPath))
			{
				SetupConfigDirectory();
			}

			string[] jsonFiles = Directory.GetFiles(_configFolderPath, "*.json");
			_availableConfigs = new string[jsonFiles.Length];

			for (int i = 0; i < jsonFiles.Length; i++)
			{
				_availableConfigs[i] = Path.GetFileNameWithoutExtension(jsonFiles[i]);
			}

			UpdateConfigListDisplay();
		}

		private void UpdateConfigListDisplay()
		{
			if (configListText == null) return;

			if (_availableConfigs == null || _availableConfigs.Length == 0)
			{
				configListText.text = "No saved configurations";
				return;
			}

			_sb.Clear();
			for (int i = 0; i < _availableConfigs.Length; i++)
			{
				string marker = _availableConfigs[i] == _selectedConfig ? "► " : "  ";
				_sb.AppendLine($"{marker}{_availableConfigs[i]}");
			}
			configListText.text = _sb.ToString();
		}

		private StringBuilder _sb = new StringBuilder();

		/// <summary>
		/// Save current robot configuration
		/// </summary>
		public void SaveCurrentConfig()
		{
			string configName = defaultConfigName;

			if (configNameInput != null && !string.IsNullOrWhiteSpace(configNameInput.text))
			{
				configName = SanitizeFileName(configNameInput.text);
			}

			if (robotManager == null)
			{
				Debug.LogError("[ConfigManager] No robot manager reference!");
				return;
			}

			RobotConfig config = robotManager.GetRobotConfig();
			string filePath = Path.Combine(_configFolderPath, $"{configName}.json");

			try
			{
				string json = JsonUtility.ToJson(config, true);
				File.WriteAllText(filePath, json);

				Debug.Log($"[ConfigManager] Saved config to: {filePath}");
				RefreshConfigList();
				SelectConfig(configName);
			}
			catch (System.Exception e)
			{
				Debug.LogError($"[ConfigManager] Failed to save config: {e.Message}");
			}
		}

		/// <summary>
		/// Load selected configuration
		/// </summary>
		public void LoadSelectedConfig()
		{
			if (string.IsNullOrEmpty(_selectedConfig))
			{
				Debug.LogWarning("[ConfigManager] No config selected!");
				return;
			}

			string filePath = Path.Combine(_configFolderPath, $"{_selectedConfig}.json");

			if (!File.Exists(filePath))
			{
				Debug.LogError($"[ConfigManager] Config file not found: {filePath}");
				return;
			}

			try
			{
				string json = File.ReadAllText(filePath);
				RobotConfig config = JsonUtility.FromJson<RobotConfig>(json);

				if (config != null && robotManager != null)
				{
					robotManager.ApplyRobotConfig(config);
					Debug.Log($"[ConfigManager] Loaded config: {_selectedConfig}");
				}
			}
			catch (System.Exception e)
			{
				Debug.LogError($"[ConfigManager] Failed to load config: {e.Message}");
			}
		}

		/// <summary>
		/// Delete selected configuration
		/// </summary>
		public void DeleteSelectedConfig()
		{
			if (string.IsNullOrEmpty(_selectedConfig))
			{
				Debug.LogWarning("[ConfigManager] No config selected!");
				return;
			}

			string filePath = Path.Combine(_configFolderPath, $"{_selectedConfig}.json");

			if (File.Exists(filePath))
			{
				try
				{
					File.Delete(filePath);
					Debug.Log($"[ConfigManager] Deleted config: {_selectedConfig}");
					_selectedConfig = null;
					RefreshConfigList();
				}
				catch (System.Exception e)
				{
					Debug.LogError($"[ConfigManager] Failed to delete config: {e.Message}");
				}
			}
		}

		/// <summary>
		/// Export configuration to external file
		/// </summary>
		public void ExportConfig()
		{
			if (robotManager == null)
			{
				Debug.LogError("[ConfigManager] No robot manager reference!");
				return;
			}

			RobotConfig config = robotManager.GetRobotConfig();
			string json = JsonUtility.ToJson(config, true);

			// Use a temporary path for export
			string exportPath = Path.Combine(Application.persistentDataPath, $"export_{System.DateTime.Now:yyyyMMdd_HHmmss}.json");
			File.WriteAllText(exportPath, json);

			Debug.Log($"[ConfigManager] Exported config to: {exportPath}");

			// Copy to Downloads on desktop platforms
#if UNITY_STANDALONE_WIN || UNITY_EDITOR_WIN
			try
			{
				string downloadsPath = Path.Combine(System.Environment.GetFolderPath(System.Environment.SpecialFolder.UserProfile), "Downloads");
				string destPath = Path.Combine(downloadsPath, Path.GetFileName(exportPath));
				File.Copy(exportPath, destPath, true);
				Debug.Log($"[ConfigManager] Copied to: {destPath}");
			}
			catch (System.Exception e)
			{
				Debug.LogWarning($"[ConfigManager] Could not copy to Downloads: {e.Message}");
			}
#endif
		}

		/// <summary>
		/// Import configuration from external file
		/// </summary>
		public void ImportConfig()
		{
#if UNITY_STANDALONE_WIN || UNITY_EDITOR_WIN
			string downloadsPath = Path.Combine(System.Environment.GetFolderPath(System.Environment.SpecialFolder.UserProfile), "Downloads");
			string[] files = Directory.GetFiles(downloadsPath, "*.json");

			if (files.Length > 0)
			{
				// Import the most recent json file
				string latestFile = files[files.Length - 1];
				try
				{
					string json = File.ReadAllText(latestFile);
					RobotConfig config = JsonUtility.FromJson<RobotConfig>(json);

					if (config != null && robotManager != null)
					{
						robotManager.ApplyRobotConfig(config);
						Debug.Log($"[ConfigManager] Imported config from: {latestFile}");
					}
				}
				catch (System.Exception e)
				{
					Debug.LogError($"[ConfigManager] Failed to import config: {e.Message}");
				}
			}
			else
			{
				Debug.LogWarning("[ConfigManager] No json files found in Downloads");
			}
#else
            Debug.Log("[ConfigManager] Import only available on Windows");
#endif
		}

		/// <summary>
		/// Select a configuration from the list
		/// </summary>
		public void SelectConfig(string configName)
		{
			_selectedConfig = configName;
			UpdateConfigListDisplay();
		}

		/// <summary>
		/// Get configuration by name
		/// </summary>
		public RobotConfig GetConfig(string configName)
		{
			string filePath = Path.Combine(_configFolderPath, $"{configName}.json");

			if (!File.Exists(filePath))
			{
				return null;
			}

			try
			{
				string json = File.ReadAllText(filePath);
				return JsonUtility.FromJson<RobotConfig>(json);
			}
			catch
			{
				return null;
			}
		}

		/// <summary>
		/// Quick save with auto-naming
		/// </summary>
		public void QuickSave()
		{
			string quickName = $"quicksave_{System.DateTime.Now:yyyyMMdd_HHmmss}";

			if (configNameInput != null)
			{
				configNameInput.text = quickName;
			}

			SaveCurrentConfig();
		}

		/// <summary>
		/// Quick load the most recent save
		/// </summary>
		public void QuickLoad()
		{
			RefreshConfigList();

			if (_availableConfigs != null && _availableConfigs.Length > 0)
			{
				SelectConfig(_availableConfigs[_availableConfigs.Length - 1]);
				LoadSelectedConfig();
			}
		}

		private string SanitizeFileName(string fileName)
		{
			char[] invalidChars = Path.GetInvalidFileNameChars();
			foreach (char c in invalidChars)
			{
				fileName = fileName.Replace(c, '_');
			}
			return fileName;
		}
	}
}
