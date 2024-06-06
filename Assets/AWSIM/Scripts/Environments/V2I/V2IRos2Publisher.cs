using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using ROS2;

namespace AWSIM
{
    [RequireComponent(typeof(V2I))]
    public class V2IRos2Publisher : MonoBehaviour
    {
        public enum TrafficSignalID
        {
            RelationID,
            WayID
        }

        public TrafficSignalID trafficSignalID;

        [SerializeField, Tooltip("On this topic, the traffic_signals are published (as a ")]
        private string trafficSignalsTopic = "/v2x/traffic_signals";

        public QoSSettings qosSettings = new QoSSettings()
        {
            ReliabilityPolicy = ReliabilityPolicy.QOS_POLICY_RELIABILITY_RELIABLE,
            DurabilityPolicy = DurabilityPolicy.QOS_POLICY_DURABILITY_VOLATILE,
            HistoryPolicy = HistoryPolicy.QOS_POLICY_HISTORY_KEEP_LAST,
            Depth = 1,
        };

        private IPublisher<autoware_perception_msgs.msg.TrafficLightGroupArray> _trafficLightGroupPublisher;
        private autoware_perception_msgs.msg.TrafficLightGroupArray _trafficLightGroupMsg;

        private V2I _v2IComponent;

        private void Start()
        {
            _v2IComponent = GetComponent<V2I>();
            _v2IComponent.OnOutputData += UpdateMessageAndPublish;

            _trafficLightGroupMsg = new autoware_perception_msgs.msg.TrafficLightGroupArray();

            var qos = qosSettings.GetQoSProfile();
            _trafficLightGroupPublisher =
                SimulatorROS2Node.CreatePublisher<autoware_perception_msgs.msg.TrafficLightGroupArray>(
                    trafficSignalsTopic, qos);
        }

        private void UpdateMessageAndPublish(V2I.OutputData outputData)
        {
            UpdateTrafficLightGroupArrayMsg(outputData);
            _trafficLightGroupPublisher.Publish(_trafficLightGroupMsg);
        }

        private void UpdateTrafficLightGroupArrayMsg(V2I.OutputData data)
        {
            var trafficLightGroup = new List<autoware_perception_msgs.msg.TrafficLightGroup>();
            var allRelationID = new List<long>();
            foreach (var trafficLight in data.trafficLights)
            {
                var trafficLightLaneletID = trafficLight.GetComponentInParent<TrafficLightLaneletID>();
                if (trafficLightLaneletID != null)
                {
                    var ids = new List<long>();
                    if (trafficSignalID == TrafficSignalID.RelationID)
                    {
                        ids = trafficLightLaneletID.relationID;
                    }
                    else if (trafficSignalID == TrafficSignalID.WayID)
                    {
                        ids.Add(trafficLightLaneletID.wayID);
                    }

                    foreach (var relationID in ids)
                    {
                        var trafficLightGroupMsg = new autoware_perception_msgs.msg.TrafficLightGroup();
                        if (allRelationID.Contains(relationID))
                        {
                            continue;
                        }

                        trafficLightGroupMsg.Traffic_light_group_id = relationID;
                        //Get bulbData
                        var trafficLightBulbData = trafficLight.GetBulbData();
                        //Fill TrafficSignal with bulbData
                        var trafficLightElementMsgList = new List<autoware_perception_msgs.msg.TrafficLightElement>();
                        foreach (var bulbData in trafficLightBulbData)
                        {
                            if (isBulbTurnOn(bulbData.Status))
                            {
                                var trafficLightElementMsg = new autoware_perception_msgs.msg.TrafficLightElement();
                                trafficLightElementMsg.Color = V2IROS2Utility.UnityToRosBulbColor(bulbData.Color);
                                trafficLightElementMsg.Shape = V2IROS2Utility.UnityToRosBulbShape(bulbData.Type);
                                trafficLightElementMsg.Status = V2IROS2Utility.UnityToRosBulbStatus(bulbData.Status);
                                trafficLightElementMsg.Confidence = 1.0f;
                                trafficLightElementMsgList.Add(trafficLightElementMsg);
                            }
                        }

                        //Add TrafficLight signal to list
                        trafficLightGroupMsg.Elements = trafficLightElementMsgList.ToArray();
                        trafficLightGroup.Add(trafficLightGroupMsg);
                        allRelationID.Add(relationID);
                    }
                }
            }

            _trafficLightGroupMsg.Stamp = SimulatorROS2Node.GetCurrentRosTime();
            _trafficLightGroupMsg.Traffic_light_groups = trafficLightGroup.ToArray();
        }

        private bool isBulbTurnOn(TrafficLight.BulbStatus bulbStatus)
        {
            return bulbStatus == TrafficLight.BulbStatus.SOLID_ON || bulbStatus == TrafficLight.BulbStatus.FLASHING;
        }

        private void OnDestroy()
        {
            SimulatorROS2Node.RemovePublisher<autoware_perception_msgs.msg.TrafficSignalArray>(
                _trafficLightGroupPublisher);
        }
    }
}
