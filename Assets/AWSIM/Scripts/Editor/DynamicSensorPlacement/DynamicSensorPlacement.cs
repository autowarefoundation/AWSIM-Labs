using System;
using System.IO;
using System.Xml;
using UnityEditor;
using UnityEngine;

namespace AWSIM.Scripts.Editor.DynamicSensorPlacement
{
    public class DynamicSensorPlacement : EditorWindow
    {
        private string _sensorKitPath;
        private XmlDocument _xmlDoc = new XmlDocument();

        [MenuItem("AWSIM/Dynamic Sensor Placement")]
        public static void ShowWindow()
        {
            GetWindow<DynamicSensorPlacement>("Dynamic Sensor Placement");
        }

        private void OnGUI()
        {
            GUILayout.Label("Base Settings", EditorStyles.boldLabel);

            // This can load from any path doesn't have to be from /Externals.
            // ./Assets/AWSIM/Externals/
            _sensorKitPath = EditorGUILayout.TextField("Sensor Kit Path", _sensorKitPath);

            // GUI button to load URDF file
            if (GUILayout.Button("Place sensors"))
            {
                LoadUrdf();
                ExtractTransforms();
            }

            EditorGUI.BeginChangeCheck();
        }

        private void ExtractTransforms()
        {
            // place the objects that are direct children of the 'base_link'
            XmlNodeList baseLinkNodes = _xmlDoc.SelectNodes("//joint[parent/@link='base_link']");

            foreach (XmlNode child in baseLinkNodes)
            {
                XmlNode originNode = child.SelectSingleNode("origin");
                XmlNode childNode = child.SelectSingleNode("child");

                string childLink = childNode?.Attributes?["link"].Value;
                string rpy = originNode?.Attributes?["rpy"].Value;
                string xyz = originNode?.Attributes?["xyz"].Value;

                Vector3 unityPosition = ConvertPositions(xyz);
                Vector3 unityRotation = ConvertRotations(rpy);

                var childObject = GameObject.Find(childLink);

                if (childObject != null)
                {
                    childObject.transform.localPosition = unityPosition;
                    childObject.transform.localEulerAngles = unityRotation;
                }
            }

            // place the objects that are direct children of 'sensor_kit_base_link'
            XmlNodeList sensorKitBaseLinkNodes = _xmlDoc.SelectNodes("//joint[parent/@link='sensor_kit_base_link']");

            foreach (XmlNode child in sensorKitBaseLinkNodes)
            {
                XmlNode originNode = child.SelectSingleNode("origin");
                XmlNode childNode = child.SelectSingleNode("child");

                string childLink = childNode?.Attributes?["link"].Value;
                string rpy = originNode?.Attributes?["rpy"].Value;
                string xyz = originNode?.Attributes?["xyz"].Value;

                Vector3 unityPosition = ConvertPositions(xyz);
                Vector3 unityRotation = ConvertRotations(rpy);

                GameObject childObject = GameObject.Find(childLink);

                if (childObject != null)
                {
                    childObject.transform.localPosition = unityPosition;
                    childObject.transform.localEulerAngles = unityRotation;
                }
            }
        }

        private void LoadUrdf()
        {
            // Check if the path is valid
            if (!File.Exists(_sensorKitPath))
            {
                Debug.LogError("URDF file not found at the specified path.");
                return;
            }

            // Read the URDF file
            _xmlDoc.Load(_sensorKitPath);
            Debug.Log("URDF file loaded successfully.");
        }

        // convert ROS2 positions to unity coordinate system
        private Vector3 ConvertPositions(string vectorString)
        {
            string[] values = vectorString.Split(new[] { " " }, StringSplitOptions.RemoveEmptyEntries);

            // Check if the values array has at least 3 elements before accessing them
            if (values.Length >= 3)
            {
                float x = float.Parse(values[0]);
                float y = float.Parse(values[1]);
                float z = float.Parse(values[2]);

                return new Vector3(-y, z, x);
            }

            Debug.LogError("Invalid vector format: " + vectorString);
            return Vector3.zero; // Return a default vector or handle the error as needed
        }

        // Convert ROS2 rotations to unity coordinate system
        private Vector3 ConvertRotations(string rpyString)
        {
            string[] values = rpyString.Split(new[] { " " }, StringSplitOptions.RemoveEmptyEntries);

            // Check if the values array has at least 3 elements before accessing them
            if (values.Length >= 3)
            {
                float r = float.Parse(values[0]) * Mathf.Rad2Deg;
                float p = float.Parse(values[1]) * Mathf.Rad2Deg;
                float y = float.Parse(values[2]) * Mathf.Rad2Deg;

                return new Vector3(p, -y, -r);
            }

            Debug.LogError("Invalid vector format: " + rpyString);
            return Vector3.zero; // Return a default vector or handle the error as needed
        }
    }
}
