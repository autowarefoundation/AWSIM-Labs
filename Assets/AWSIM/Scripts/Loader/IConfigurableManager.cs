using System;
using UnityEngine;
namespace AWSIM.Loader
{
    /// <summary>
    /// Interface for any configuration manager.
    /// </summary>
    public interface IConfigurableManager
    {
        /// <summary>
        /// Load and validate config.
        /// </summary>
        public bool LoadConfig(AWSIMConfiguration config);

        /// <summary>
        /// Set up the UI configuration entities.
        /// </summary>
        public void LoadUI();

        /// <summary>
        /// Log callback.
        /// </summary>
        public Action<LogLevel, string> Log { get; set; }
    }
}
