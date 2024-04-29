using UnityEditor;
using UnityEngine;
using System.Collections.Generic;
using YamlDotNet.Serialization;
using System.IO;

public class DynamicSensorPlacement : EditorWindow
{
    private string sensorKitPath;
    private string yamlContent;
    private List<GameObject> sensors = new List<GameObject>();


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

        // Button to load YAML file
        if (GUILayout.Button("Load YAML"))
        {
            LoadYaml();
        }

        EditorGUI.BeginChangeCheck();
        // Sensors Field (List of GameObjects)
        GUILayout.Label("Sensors", EditorStyles.boldLabel);
        for (int i = 0; i < sensors.Count; i++)
        {
            EditorGUILayout.BeginHorizontal();
            sensors[i] = (GameObject)EditorGUILayout.ObjectField("Sensor " + (i + 1), sensors[i], typeof(GameObject), true);
            if (GUILayout.Button("Remove", GUILayout.Width(70)))
            {
                sensors.RemoveAt(i);
                break; // Exiting the loop after removal
            }
            EditorGUILayout.EndHorizontal(); // End of horizontal layout for each sensor
        }
        // Button to list files in current path
        if (GUILayout.Button("List Files in Current Path"))
        {
            ListFilesInCurrentPath();
        }

        // Button to add a new sensor
        if (GUILayout.Button("Add Sensor"))
        {
            sensors.Add(null); // Add a new null element to the list
        }

    }

    private void LoadYaml()
    {
        // Check if the path is valid
        if (!File.Exists(sensorKitPath))
        {
            Debug.LogError("YAML file not found at the specified path.");
            return;
        }

        // Read the YAML file
        yamlContent = File.ReadAllText(sensorKitPath);

        // Optionally, you can deserialize the YAML content to an object here if needed
        // For example:
        // Deserializer deserializer = new DeserializerBuilder().Build();
        // object yamlObject = deserializer.Deserialize(new StringReader(yamlContent));

        Debug.Log("YAML file loaded successfully.");
    }

    private void ListFilesInCurrentPath()
    {
        string[] files = Directory.GetFiles(sensorKitPath);
        foreach (string file in files)
        {
            Debug.Log("File found: " + file);
        }
    }
}
