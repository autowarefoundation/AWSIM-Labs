using UnityEditor;
using UnityEngine;
using System.Collections.Generic;


public class DynamicSensorPlacement : EditorWindow
{
    private string sensorKitPath;
    private List<GameObject> sensors = new List<GameObject>();


    [MenuItem("AWSIM/Dynamic Sensor Placement")]
    public static void ShowWindow()
    {
        GetWindow<DynamicSensorPlacement>("Dynamic Sensor Placement");
    }

    private void OnGUI()
    {
        GUILayout.Label ("Base Settings", EditorStyles.boldLabel);
        sensorKitPath = EditorGUILayout.TextField("Sensor Kit Path", sensorKitPath);

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
                break;
            }
            EditorGUILayout.EndHorizontal();
        }

        // Button to add a new sensor
        if (GUILayout.Button("Add Sensor"))
        {
            sensors.Add(null); // Add a new null element to the list
        }

    }
}
