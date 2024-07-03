using System.Reflection;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace AWSIM.Scripts.Utilities
{
    public class Ros2PublisherUtilities : MonoBehaviour
    {
        // Build a list of public field values from the given object
        public static List<object> ConfigListBuilder(object obj)
        {
            var type = obj.GetType();
            var valuesList = type.GetFields(BindingFlags.Public | BindingFlags.Instance)
                .Select(fieldInfo => fieldInfo.GetValue(obj)).ToList();

            //// Debug for checking collected field values
            // foreach (var value in valuesList)
            // {
            //     Debug.Log("Collection: " + value);
            //
            //     if (value.GetType() == typeof(QoSSettings))
            //     {
            //         var qosSettings = (QoSSettings)value;
            //         Debug.Log("QoS Settings: " + qosSettings.ReliabilityPolicy);
            //         Debug.Log("QoS Settings: " + qosSettings.DurabilityPolicy);
            //         Debug.Log("QoS Settings: " + qosSettings.HistoryPolicy);
            //     }
            // }

            return valuesList;
        }

        // Assign the public field values of the given object from the given list
        public static void ConfigAssigner(object sensorPublisher, List<object> configList)
        {
            var type = sensorPublisher.GetType();
            int index = 0;

            var typeFieldInfo = type.GetFields(BindingFlags.Public | BindingFlags.Instance);

            // Check if the number of public fields in the object matches the size of the config list
            if (typeFieldInfo.Length != configList.Count)
            {
                Debug.LogError(
                    "Config List size does not match the number of public fields in the object. Aborting public field assignment.");
                return;
            }

            // Assign the public field values from the config list
            foreach (var fieldInfo in typeFieldInfo)
            {
                if (index < configList.Count)
                {
                    var value = configList[index];
                    if (value != null && fieldInfo.FieldType.IsInstanceOfType(value))
                    {
                        // Debug.Log("Assigning: " + value + " to " + fieldInfo.Name + " of " + sensorPublisher);
                        fieldInfo.SetValue(sensorPublisher, value);
                    }

                    index++;
                }
                else
                {
                    break;
                }
            }
        }
    }
}
