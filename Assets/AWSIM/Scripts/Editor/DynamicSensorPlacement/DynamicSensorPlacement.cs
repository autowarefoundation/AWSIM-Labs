using UnityEditor;
using UnityEngine;
using System.Collections.Generic;
using System.IO;
using System.Xml;

namespace AWSIM.SensorPlacement
{
    public class DynamicSensorPlacement : EditorWindow
    {
        private string sensorKitPath;
        private XmlDocument xmlDoc = new XmlDocument();

        [MenuItem("AWSIM/Dynamic Sensor Placement")]
        public static void ShowWindow()
        {
            GetWindow<DynamicSensorPlacement>("Dynamic Sensor Placement");
        }

        private void OnGUI()
        {
            GUILayout.Label ("Base Settings", EditorStyles.boldLabel);

            // ./Assets/AWSIM/Externals/
            sensorKitPath = EditorGUILayout.TextField("Sensor Kit Path", sensorKitPath);

            // Button to load URDF file
            if (GUILayout.Button("Place sensors"))
            {
                LoadUrdf();
                ExtractTransforms();
            }
            
            EditorGUI.BeginChangeCheck();
        }

        public void ExtractTransforms(){
            XmlNodeList linkNodes = xmlDoc.SelectNodes("//link");
                foreach (XmlNode linkNode in linkNodes)
                {
                    string linkName = linkNode.Attributes["name"].Value;

                    // Check if the game object exists in the scene
                    GameObject gameObject = GameObject.Find(linkName);

                    // Extract and assign the transform
                    if (gameObject != null)
                    {
                        XmlNode originNode = linkNode.SelectSingleNode("origin");

                        // search for origin under link/visual
                        if(originNode == null){
                            originNode = linkNode.SelectSingleNode("visual/origin");
                        }
                        // search for origin under link/visual
                        if(originNode == null){
                            originNode = linkNode.SelectSingleNode("inertial/origin");
                        }

                        // assign transforms
                        if(originNode!=null){
                            if(originNode.Attributes["xyz"]!=null){
                                Vector3 position = convertPositions(originNode.Attributes["xyz"].Value);
                                gameObject.transform.localPosition = position;
                            }
                            if(originNode.Attributes["rpy"]!=null){
                                Vector3 rotation = convertRotations(originNode.Attributes["rpy"].Value);
                                gameObject.transform.localRotation = Quaternion.Euler(rotation);
                            }                                                
                        }                   
                        
                    }
                }
        }

        private void LoadUrdf()
        {
            // Check if the path is valid
            if (!File.Exists(sensorKitPath))
            {
                Debug.LogError("URDF file not found at the specified path.");
                return;
            }

            // Read the URDF file
            xmlDoc.Load(sensorKitPath);
            Debug.Log("URDF file loaded successfully.");
        }


        private Vector3 convertPositions(string vectorString)
        {
            string[] values = vectorString.Split(' ');

            // Check if the values array has at least 3 elements before accessing them
            if (values.Length >= 3)
            {
                float x = float.Parse(values[0]);
                float y = float.Parse(values[1]);
                float z = float.Parse(values[2]);

                // convert positions to unity's coordinate system
                return new Vector3(-y,z,x);
            }
            else
            {
                Debug.LogError("Invalid vector format: " + vectorString);
                return Vector3.zero; // Return a default vector or handle the error as needed
            }
        }

        private Vector3 convertRotations(string rpyString)
        {
            string[] values = rpyString.Split(' ');

            // Check if the values array has at least 3 elements before accessing them
            if (values.Length >= 3)
            {
                float r = float.Parse(values[0]) *Mathf.Deg2Rad;
                float p = float.Parse(values[1]) *Mathf.Deg2Rad;
                float y = float.Parse(values[2]) *Mathf.Deg2Rad;

                // convert rotations to unity's coordinate system
                return new Vector3(p,-y,-r);
            }
            else
            {
                Debug.LogError("Invalid vector format: " + rpyString);
                return Vector3.zero; // Return a default vector or handle the error as needed
            }
        }
    }
}

