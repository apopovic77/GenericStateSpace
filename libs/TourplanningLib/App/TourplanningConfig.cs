using System;
using System.ComponentModel;
using Logicx.Utility;
using MathLib;


namespace Logicx.Optimization.Tourplanning.App
{
    /// <summary>
    /// If you want to create a new Configuration-File
    /// this class can be copied and renamed. It has
    /// all neccessary methods and tags that are used
    /// within the SettingsManagerApplication
    /// </summary>
    public class TourplanningConfig : SystemConfig
    {
        private TourplanningConfig(string path_to_config_file)
            : base(path_to_config_file)
        {
        }

        protected override void RegisterConfigProperties()
        {
            //settingsmanager: register begin


            AcsPopulationSize.PropertyChanged += Config_PropertyChanged;
			ConfigProperties.Add("AcsPopulationSize", AcsPopulationSize);
			AcsIterations.PropertyChanged += Config_PropertyChanged;
			ConfigProperties.Add("AcsIterations", AcsIterations);
			AcsEvaporationCoefficient.PropertyChanged += Config_PropertyChanged;
			ConfigProperties.Add("AcsEvaporationCoefficient", AcsEvaporationCoefficient);
			AcsPheromoneInfluence.PropertyChanged += Config_PropertyChanged;
			ConfigProperties.Add("AcsPheromoneInfluence", AcsPheromoneInfluence);
			AcsAttractionInfluence.PropertyChanged += Config_PropertyChanged;
			ConfigProperties.Add("AcsAttractionInfluence", AcsAttractionInfluence);
			AcsQValue.PropertyChanged += Config_PropertyChanged;
			ConfigProperties.Add("AcsQValue", AcsQValue);
			//settingsmanager: register end
        }

        protected override void ExecLoadValues()
        {
            //settingsmanager: construction begin

            //SettingsManager-Kommentar: no comment
			if (_config.AppSettings.Settings["AcsPopulationSize_value"] != null)
				AcsPopulationSize.Value = Convert.ToInt32(_config.AppSettings.Settings["AcsPopulationSize_value"].Value);
			if (_config.AppSettings.Settings["AcsPopulationSize_bez"] != null)
				AcsPopulationSize.Bezeichnung = _config.AppSettings.Settings["AcsPopulationSize_bez"].Value;
			if (_config.AppSettings.Settings["AcsPopulationSize_min"] != null)
				AcsPopulationSize.Minimum = Convert.ToInt32(_config.AppSettings.Settings["AcsPopulationSize_min"].Value);
			if (_config.AppSettings.Settings["AcsPopulationSize_max"] != null)
				AcsPopulationSize.Maximum = Convert.ToInt32(_config.AppSettings.Settings["AcsPopulationSize_max"].Value);
			//SettingsManager-Kommentar: AcsIterations
			if (_config.AppSettings.Settings["AcsIterations_value"] != null)
				AcsIterations.Value = Convert.ToInt32(_config.AppSettings.Settings["AcsIterations_value"].Value);
			if (_config.AppSettings.Settings["AcsIterations_bez"] != null)
				AcsIterations.Bezeichnung = _config.AppSettings.Settings["AcsIterations_bez"].Value;
			if (_config.AppSettings.Settings["AcsIterations_min"] != null)
				AcsIterations.Minimum = Convert.ToInt32(_config.AppSettings.Settings["AcsIterations_min"].Value);
			if (_config.AppSettings.Settings["AcsIterations_max"] != null)
				AcsIterations.Maximum = Convert.ToInt32(_config.AppSettings.Settings["AcsIterations_max"].Value);
			//SettingsManager-Kommentar: AcsEvaporationCoefficient
			if (_config.AppSettings.Settings["AcsEvaporationCoefficient_value"] != null)
				AcsEvaporationCoefficient.Value = Convert.ToSingle(_config.AppSettings.Settings["AcsEvaporationCoefficient_value"].Value);
			if (_config.AppSettings.Settings["AcsEvaporationCoefficient_bez"] != null)
				AcsEvaporationCoefficient.Bezeichnung = _config.AppSettings.Settings["AcsEvaporationCoefficient_bez"].Value;
			if (_config.AppSettings.Settings["AcsEvaporationCoefficient_min"] != null)
				AcsEvaporationCoefficient.Minimum = Convert.ToSingle(_config.AppSettings.Settings["AcsEvaporationCoefficient_min"].Value);
			if (_config.AppSettings.Settings["AcsEvaporationCoefficient_max"] != null)
				AcsEvaporationCoefficient.Maximum = Convert.ToSingle(_config.AppSettings.Settings["AcsEvaporationCoefficient_max"].Value);
			//SettingsManager-Kommentar: AcsPheromoneInfluence
			if (_config.AppSettings.Settings["AcsPheromoneInfluence_value"] != null)
				AcsPheromoneInfluence.Value = Convert.ToSingle(_config.AppSettings.Settings["AcsPheromoneInfluence_value"].Value);
			if (_config.AppSettings.Settings["AcsPheromoneInfluence_bez"] != null)
				AcsPheromoneInfluence.Bezeichnung = _config.AppSettings.Settings["AcsPheromoneInfluence_bez"].Value;
			if (_config.AppSettings.Settings["AcsPheromoneInfluence_min"] != null)
				AcsPheromoneInfluence.Minimum = Convert.ToSingle(_config.AppSettings.Settings["AcsPheromoneInfluence_min"].Value);
			if (_config.AppSettings.Settings["AcsPheromoneInfluence_max"] != null)
				AcsPheromoneInfluence.Maximum = Convert.ToSingle(_config.AppSettings.Settings["AcsPheromoneInfluence_max"].Value);
			//SettingsManager-Kommentar: AcsAttractionInfluence
			if (_config.AppSettings.Settings["AcsAttractionInfluence_value"] != null)
				AcsAttractionInfluence.Value = Convert.ToSingle(_config.AppSettings.Settings["AcsAttractionInfluence_value"].Value);
			if (_config.AppSettings.Settings["AcsAttractionInfluence_bez"] != null)
				AcsAttractionInfluence.Bezeichnung = _config.AppSettings.Settings["AcsAttractionInfluence_bez"].Value;
			if (_config.AppSettings.Settings["AcsAttractionInfluence_min"] != null)
				AcsAttractionInfluence.Minimum = Convert.ToSingle(_config.AppSettings.Settings["AcsAttractionInfluence_min"].Value);
			if (_config.AppSettings.Settings["AcsAttractionInfluence_max"] != null)
				AcsAttractionInfluence.Maximum = Convert.ToSingle(_config.AppSettings.Settings["AcsAttractionInfluence_max"].Value);
			//SettingsManager-Kommentar: AcsQValue
			if (_config.AppSettings.Settings["AcsQValue_value"] != null)
				AcsQValue.Value = Convert.ToSingle(_config.AppSettings.Settings["AcsQValue_value"].Value);
			if (_config.AppSettings.Settings["AcsQValue_bez"] != null)
				AcsQValue.Bezeichnung = _config.AppSettings.Settings["AcsQValue_bez"].Value;
			if (_config.AppSettings.Settings["AcsQValue_min"] != null)
				AcsQValue.Minimum = Convert.ToSingle(_config.AppSettings.Settings["AcsQValue_min"].Value);
			if (_config.AppSettings.Settings["AcsQValue_max"] != null)
				AcsQValue.Maximum = Convert.ToSingle(_config.AppSettings.Settings["AcsQValue_max"].Value);
			//settingsmanager: construction end 
        }

        protected override void ExecSaveSettings()
        {
            //settingsmanager: savesettings begin

            //SettingsManager-Kommentar: no comment
			AddKey(_config, "AcsPopulationSize_value", AcsPopulationSize.Value.ToString());
			AddKey(_config, "AcsPopulationSize_bez", AcsPopulationSize.Bezeichnung);
			AddKey(_config, "AcsPopulationSize_min", AcsPopulationSize.Minimum.ToString());
			AddKey(_config, "AcsPopulationSize_max", AcsPopulationSize.Maximum.ToString());
			//SettingsManager-Kommentar: AcsIterations
			AddKey(_config, "AcsIterations_value", AcsIterations.Value.ToString());
			AddKey(_config, "AcsIterations_bez", AcsIterations.Bezeichnung);
			AddKey(_config, "AcsIterations_min", AcsIterations.Minimum.ToString());
			AddKey(_config, "AcsIterations_max", AcsIterations.Maximum.ToString());
			//SettingsManager-Kommentar: AcsEvaporationCoefficient
			AddKey(_config, "AcsEvaporationCoefficient_value", AcsEvaporationCoefficient.Value.ToString());
			AddKey(_config, "AcsEvaporationCoefficient_bez", AcsEvaporationCoefficient.Bezeichnung);
			AddKey(_config, "AcsEvaporationCoefficient_min", AcsEvaporationCoefficient.Minimum.ToString());
			AddKey(_config, "AcsEvaporationCoefficient_max", AcsEvaporationCoefficient.Maximum.ToString());
			//SettingsManager-Kommentar: AcsPheromoneInfluence
			AddKey(_config, "AcsPheromoneInfluence_value", AcsPheromoneInfluence.Value.ToString());
			AddKey(_config, "AcsPheromoneInfluence_bez", AcsPheromoneInfluence.Bezeichnung);
			AddKey(_config, "AcsPheromoneInfluence_min", AcsPheromoneInfluence.Minimum.ToString());
			AddKey(_config, "AcsPheromoneInfluence_max", AcsPheromoneInfluence.Maximum.ToString());
			//SettingsManager-Kommentar: AcsAttractionInfluence
			AddKey(_config, "AcsAttractionInfluence_value", AcsAttractionInfluence.Value.ToString());
			AddKey(_config, "AcsAttractionInfluence_bez", AcsAttractionInfluence.Bezeichnung);
			AddKey(_config, "AcsAttractionInfluence_min", AcsAttractionInfluence.Minimum.ToString());
			AddKey(_config, "AcsAttractionInfluence_max", AcsAttractionInfluence.Maximum.ToString());
			//SettingsManager-Kommentar: AcsQValue
			AddKey(_config, "AcsQValue_value", AcsQValue.Value.ToString());
			AddKey(_config, "AcsQValue_bez", AcsQValue.Bezeichnung);
			AddKey(_config, "AcsQValue_min", AcsQValue.Minimum.ToString());
			AddKey(_config, "AcsQValue_max", AcsQValue.Maximum.ToString());
			//settingsmanager: savesettings end
        }


        //settingsmanager: vardef begin 

        public static IntConfigValue AcsPopulationSize = new IntConfigValue("AcsPopulationSize", "AcsPopulationSize", 100,2,1000);
		public static IntConfigValue AcsIterations = new IntConfigValue("AcsIterations","AcsIterations",200,2,1000);
		public static FloatConfigValue AcsEvaporationCoefficient = new FloatConfigValue("AcsEvaporationCoefficient","AcsEvaporationCoefficient",0.9f,0f,1f);
		public static FloatConfigValue AcsPheromoneInfluence = new FloatConfigValue("AcsPheromoneInfluence","AcsPheromoneInfluence",0.9f,0f,1f);
		public static FloatConfigValue AcsAttractionInfluence = new FloatConfigValue("AcsAttractionInfluence","AcsAttractionInfluence",1f,0f,1f);
		public static FloatConfigValue AcsQValue = new FloatConfigValue("AcsQValue","AcsQValue",0.9f,0f,1f);
		//settingsmanager: vardef end 



        #region Singleton Instance
        public static void CreateInstance(string path_to_config_file)
        {
            lock (_instance_creation_lock)
            {
                _instance = new TourplanningConfig(path_to_config_file);
            }
        }
        public static void CreateInstance()
        {
            CreateInstance(GetPathToConfigFile());
        }





        public static TourplanningConfig Instance
        {
            get
            {
                lock (_instance_creation_lock)
                {
                    if (_instance == null)
                        throw new Exception("Instance not existing, you need to create the instance via CreateInstance()");
                    return _instance;
                }
            }
        }

        private static object _instance_creation_lock = new object();
        private static TourplanningConfig _instance;
        #endregion
    }
}

