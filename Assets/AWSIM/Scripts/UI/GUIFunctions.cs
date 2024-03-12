using System;
using UnityEngine;
using AWSIM.TrafficSimulation;
using UnityEngine.Rendering;
using UnityEngine.Serialization;
using UnityEngine.UI;

namespace AWSIM
{
    /// <summary>
    /// Script for GUI functions. User will be able to control the simulation through the GUI.
    /// </summary>
    public class GUIFunctions : MonoBehaviour
    {
        [SerializeField] private Canvas guiCanvas;

        private void Update()
        {
            // If the escape key is pressed, toggle the GUI panel.
            if (Input.GetKeyDown(KeyCode.Escape))
            {
                if (guiCanvas == null)
                {
                    Debug.LogWarning(
                        "The guiCanvas reference is null. Please ensure a Canvas object is assigned to guiCanvas in the Inspector. Without it, the GUI cannot be toggled on/off.");
                }
                else
                {
                    guiCanvas.enabled = !guiCanvas.enabled;
                }
            }
        }
    }
}