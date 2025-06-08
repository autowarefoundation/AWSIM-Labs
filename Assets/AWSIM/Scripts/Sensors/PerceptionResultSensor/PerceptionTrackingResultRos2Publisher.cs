using System.Collections.Generic;
using UnityEngine;
using System;
using System.Reflection;
using ROS2;

namespace AWSIM
{
    /// <summary>
    /// Convert the data output from PerceptionResultSensor to ROS2 msg and Publish.
    /// </summary>
    [RequireComponent(typeof(PerceptionResultSensor))]
    public class PerceptionTrackingResultRos2Publisher : MonoBehaviour
    {
        /// <summary>
        /// Topic name in DetectedObject msg.
        /// </summary>
        public string objectTopic = "/awsim/ground_truth/perception/object_recognition/tracking/objects";

        /// <summary>
        /// Object sensor frame id.
        /// </summary>
        public string frameId = "map";

        /// <summary>
        /// max distance that lidar can detect
        /// </summary>
        [Range(0, 200)]
        public float maxDistance = 200f;

        /// <summary>
        /// QoS settings.
        /// </summary>
        public QoSSettings qosSettings = new QoSSettings()
        {
            ReliabilityPolicy = ReliabilityPolicy.QOS_POLICY_RELIABILITY_RELIABLE,
            DurabilityPolicy = DurabilityPolicy.QOS_POLICY_DURABILITY_VOLATILE,
            HistoryPolicy = HistoryPolicy.QOS_POLICY_HISTORY_KEEP_LAST,
            Depth = 1,
        };

        public Dictionary<string, NPCs> idToNpc = new Dictionary<string, NPCs>();

        IPublisher<autoware_perception_msgs.msg.TrackedObjects> objectPublisher;
        autoware_perception_msgs.msg.TrackedObjects objectsMsg;
        PerceptionResultSensor objectSensor;

        void Start()
        {
            // Get ObjectSensor component.
            objectSensor = GetComponent<PerceptionResultSensor>();

            // Set callback.
            objectSensor.OnOutputData += Publish;

            // Create msg.
            objectsMsg = new autoware_perception_msgs.msg.TrackedObjects();

            // Create publisher.
            var qos = qosSettings.GetQoSProfile();
            objectPublisher = SimulatorROS2Node.CreatePublisher<autoware_perception_msgs.msg.TrackedObjects>(objectTopic, qos);
        }

        void Publish(PerceptionResultSensor.OutputData outputData)
        {
            if (outputData == null || outputData.objects == null || outputData.origin == null) return;
            var objectsList = new List<autoware_perception_msgs.msg.TrackedObject>();
            foreach (var detectedObject in outputData.objects)
            {
                if (detectedObject == null || detectedObject.rigidBody == null || detectedObject.dimension == null || detectedObject.bounds == null) continue;
                var NPC = detectedObject.rigidBody.gameObject.GetComponent<NPCs>();
                if (NPC != null)
                {
                    if (idToNpc.ContainsKey(BitConverter.ToString(NPC.uuid.Uuid)) == false){
                        idToNpc.Add(BitConverter.ToString(NPC.uuid.Uuid), NPC);
                    }
                }
                var rb = detectedObject.rigidBody;
                var dim = detectedObject.dimension;
                var bou = detectedObject.bounds;
                var obj = new autoware_perception_msgs.msg.TrackedObject();
                obj.Existence_probability = 1.0f;
                // add UUID 
                PropertyInfo property = obj.GetType().GetProperty("Object_id", BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Instance | BindingFlags.Static);
                property.SetValue(obj, NPC.uuid);
                //add classification
                var classification = new autoware_perception_msgs.msg.ObjectClassification();
                {
                    switch (detectedObject.classification)
                    {
                        case ObjectClassification.ObjectType.UNKNOWN:
                            classification.Label = autoware_perception_msgs.msg.ObjectClassification.UNKNOWN;
                            break;
                        case ObjectClassification.ObjectType.CAR:
                            classification.Label = autoware_perception_msgs.msg.ObjectClassification.CAR;
                            break;
                        case ObjectClassification.ObjectType.TRUCK:
                            classification.Label = autoware_perception_msgs.msg.ObjectClassification.TRUCK;
                            break;
                        case ObjectClassification.ObjectType.BUS:
                            classification.Label = autoware_perception_msgs.msg.ObjectClassification.BUS;
                            break;
                        case ObjectClassification.ObjectType.TRAILER:
                            classification.Label = autoware_perception_msgs.msg.ObjectClassification.TRAILER;
                            break;
                        case ObjectClassification.ObjectType.MOTORCYCLE:
                            classification.Label = autoware_perception_msgs.msg.ObjectClassification.MOTORCYCLE;
                            break;
                        case ObjectClassification.ObjectType.BICYCLE:
                            classification.Label = autoware_perception_msgs.msg.ObjectClassification.BICYCLE;
                            break;
                        case ObjectClassification.ObjectType.PEDESTRIAN:
                            classification.Label = autoware_perception_msgs.msg.ObjectClassification.PEDESTRIAN;
                            break;
                        default:
                            Debug.LogWarning("Unknown classification type");
                            break;
                    }
                    classification.Probability = 1.0f;
                }
                obj.Classification = new List<autoware_perception_msgs.msg.ObjectClassification> { classification }.ToArray();

                var kinematics = new autoware_perception_msgs.msg.TrackedObjectKinematics();
                // Add pose
                {
                    var p = ROS2Utility.UnityToRosPosition(rb.transform.TransformPoint(rb.centerOfMass));
                    var rosPosition = p + Environment.Instance.MgrsOffsetPosition;

                    kinematics.Pose_with_covariance.Pose.Position.X = rosPosition.x;
                    kinematics.Pose_with_covariance.Pose.Position.Y = rosPosition.y;
                    kinematics.Pose_with_covariance.Pose.Position.Z = rosPosition.z;

                    var r = ROS2Utility.UnityToRosRotation(rb.rotation);
                    kinematics.Pose_with_covariance.Pose.Orientation.X = r.x; 
                    kinematics.Pose_with_covariance.Pose.Orientation.Y = r.y; 
                    kinematics.Pose_with_covariance.Pose.Orientation.Z = r.z; 
                    kinematics.Pose_with_covariance.Pose.Orientation.W = r.w;
                }
                // Add twist
                {
                    var rosLinearVelocity = ROS2Utility.UnityToRosPosition(NPC.LinearVelocity);
                    kinematics.Twist_with_covariance.Twist.Linear.X = rosLinearVelocity.x;
                    kinematics.Twist_with_covariance.Twist.Linear.Y = rosLinearVelocity.y;
                    kinematics.Twist_with_covariance.Twist.Linear.Z = rosLinearVelocity.z;

                    var rosAngularVelocity = ROS2Utility.UnityToRosAngularVelocity(NPC.AngularVelocity);
                    kinematics.Twist_with_covariance.Twist.Angular.X = rosAngularVelocity.x;
                    kinematics.Twist_with_covariance.Twist.Angular.Y = rosAngularVelocity.y;
                    kinematics.Twist_with_covariance.Twist.Angular.Z = rosAngularVelocity.z;
                }
                // Add covariance
                {
                    kinematics.Orientation_availability = autoware_perception_msgs.msg.TrackedObjectKinematics.AVAILABLE;
                    const int size = 6;
                    for (int i = 0; i < size; i++)
                    {
                        kinematics.Pose_with_covariance.Covariance[i * size + i] = 0.001;
                        kinematics.Twist_with_covariance.Covariance[i * size + i] = 0.001;
                    }
                }
                obj.Kinematics = kinematics;

                // add shape and footprint
                {
                    var shape = new autoware_perception_msgs.msg.Shape();
                    shape.Type = autoware_perception_msgs.msg.Shape.BOUNDING_BOX;
                    shape.Dimensions.X = dim.x;
                    shape.Dimensions.Y = dim.y;
                    shape.Dimensions.Z = dim.z;
                    var footprints = new geometry_msgs.msg.Polygon();
                    // Assuming Point32 has X, Y, Z properties
                    if (bou.Length > 0)
                    {
                        var point1 = new geometry_msgs.msg.Point32() { X = bou[0].x, Y = bou[0].y, Z = 0 };
                        var point2 = new geometry_msgs.msg.Point32() { X = bou[1].x, Y = bou[1].y, Z = 0 };
                        var point3 = new geometry_msgs.msg.Point32() { X = bou[2].x, Y = bou[2].y, Z = 0 };
                        var point4 = new geometry_msgs.msg.Point32() { X = bou[3].x, Y = bou[3].y, Z = 0 };
                        footprints.Points = new[] { point1, point2, point3, point4 };
                    }
                    shape.Footprint = footprints;
                    obj.Shape = shape;
                }
                objectsList.Add(obj);
            }
            // Converts data output from ObjectSensor to ROS2 msg
            objectsMsg.Objects = objectsList.ToArray();
            // Update msg header.
            objectsMsg.Header.Stamp.Sec = outputData.seconds;
            objectsMsg.Header.Stamp.Nanosec = outputData.nanoseconds;
            objectsMsg.Header.Frame_id = frameId;

            // Publish to ROS2.
            objectPublisher.Publish(objectsMsg);
        }

        void OnDestroy()
        {
            SimulatorROS2Node.RemovePublisher<autoware_perception_msgs.msg.TrackedObjects>(objectPublisher);
        }
    }
}
