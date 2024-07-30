using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using UnityEngine;

namespace AWSIM.Scripts.Vehicles.VPP_Integration
{
    public class VehiclePedalMapLoader : MonoBehaviour
    {
        public TextAsset _accelMapCsv;
        public TextAsset _brakeMapCsv;

        [NonSerialized] public PedalMap AccelMap;
        [NonSerialized] public PedalMap BrakeMap;

        private void Start()
        {
            // Load the pedal maps
            AccelMap = new PedalMap
            {
                Map = LoadMap(_accelMapCsv),
                MapHeaders = LoadHeaders(_accelMapCsv)
                // vertical map assigned below
            };
            AccelMap.MapVertical = VerticalDict(AccelMap.Map, AccelMap.MapHeaders);

            BrakeMap = new PedalMap
            {
                Map = LoadMap(_brakeMapCsv),
                MapHeaders = LoadHeaders(_brakeMapCsv)
                // vertical map assigned below
            };
            BrakeMap.MapVertical = VerticalDict(BrakeMap.Map, BrakeMap.MapHeaders);
        }

        private static Dictionary<float, List<float>> LoadMap(TextAsset csv)
        {
            Dictionary<float, List<float>> map = new Dictionary<float, List<float>>();
            StringReader reader = new StringReader(csv.text);
            while (reader.ReadLine() is { } line)
            {
                string[] values = line.Split(',');
                if (float.TryParse(values[0], out var key))
                {
                    List<float> data = new List<float>();
                    for (int i = 1; i < values.Length; i++)
                    {
                        data.Add(float.Parse(values[i], CultureInfo.InvariantCulture));
                    }

                    map.Add(key, data);
                }
            }

            return map;
        }

        // return adjusted headers for indexing etc.
        private static List<float> LoadHeaders(TextAsset csv)
        {
            List<float> headerList = new List<float>();
            StringReader reader = new StringReader(csv.text);

            string line = reader.ReadLine();
            string[] headers = line?.Split(',');

            if (headers == null) return headerList;
            foreach (var s in headers)
            {
                if (float.TryParse(s, out float result))
                {
                    headerList.Add(result);
                }
            }

            return headerList;
        }

        public float GetPedalPercent(PedalMap pedalMap, float targetAccel, float currentSpeed)
        {
            // Get the closest speed value from the headers
            var closestSpeed = GetClosestValueFromList(pedalMap.MapHeaders, currentSpeed);

            // Get the closest acceleration value from the map
            pedalMap.MapVertical.TryGetValue(closestSpeed, out List<float> valueList);
            var closestAccel = GetClosestValueFromList(valueList, targetAccel);
            // get index of accel from the header list
            var closestAccelIndex = pedalMap.MapHeaders.IndexOf(closestSpeed);

            foreach (var pair in pedalMap.Map)
            {
                if (Mathf.Approximately(pair.Value[closestAccelIndex], closestAccel))
                {
                    return pair.Key; // return the pedal value
                }
            }

            return 0; // return 0 if no matches found which is unlikely
        }

        // build vertical dictionary for speed & acceleration
        private static Dictionary<float, List<float>> VerticalDict(Dictionary<float, List<float>> map,
            List<float> headers)
        {
            var verticalDict = new Dictionary<float, List<float>>();
            foreach (var header in headers)
            {
                var verticalGroup = new List<float>();
                var headerIndex = headers.IndexOf(header);
                foreach (var pair in map)
                {
                    verticalGroup.Add(pair.Value[headerIndex]);
                }

                verticalDict.Add(header, verticalGroup);
            }

            return verticalDict;
        }

        // Get the closest value from a list (helper function, can be moved to a utility class)
        private static float GetClosestValueFromList(List<float> list, float value)
        {
            // null/empty check
            if (list == null || list.Count == 0)
            {
                throw new ArgumentException("List cannot be null or empty.");
            }

            // early return if the value is in the list
            if (list.Contains(value))
            {
                return value;
            }

            float closestValue = list[0];
            float smallestDifference = Math.Abs(value - closestValue);

            foreach (var listValue in list)
            {
                float currentDifference = Math.Abs(value - listValue);
                if (currentDifference < smallestDifference)
                {
                    closestValue = listValue;
                    smallestDifference = currentDifference;
                }
            }

            return closestValue;
        }
    }
}
