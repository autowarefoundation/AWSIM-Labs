using UnityEngine;
using ROS2;
/**************/
using System;
using System.Linq;
using System.Collections;
using System.Collections.Generic;
using System.Reflection;
/**********************/
namespace AWSIM
{
    public class ROS2NPCPedestrianPredictionController : ROS2PredictionController
    {
        public static int stopCount = 0; // debug
        List<( 
            NPCPedestrian Pedestrian, 
            double rosTime, 
            autoware_perception_msgs.msg.PredictedPath predictedPath
        )> pedestrianWithPredctedPath = new List<(NPCPedestrian, double, autoware_perception_msgs.msg.PredictedPath)>{};

        void Start() {
            Subscriber = SimulatorROS2Node.CreateSubscription<autoware_perception_msgs.msg.PredictedObjects>(
                subscribedTopic, 
                predictionCallback, 
                qoSSettings.GetQoSProfile());
            perceptionTrackingResultRos2Publisher = GetComponent<PerceptionTrackingResultRos2Publisher>();
            objectSensor = GetComponent<PerceptionResultSensor>();
            objectSensor.OnOutputData += Callback;
            ego = GameObject.FindWithTag("Ego");
        }

        void FixedUpdate() {
            int currentSec;
            uint currentNanosec;
            SimulatorROS2Node.TimeSource.GetTime(out currentSec, out currentNanosec);
            
            stopCount++;
            var PedestrianWithPredctedPath = pedestrianWithPredctedPath.Select(
                item => (item.Pedestrian, item.rosTime, item.predictedPath)).ToList();
            for(int i = 0; i < PedestrianWithPredctedPath.Count; i++)
            {
                
                var npcPedestrian = PedestrianWithPredctedPath[i].Pedestrian;
                var deltaTime =(currentSec + currentNanosec/1e9F) - PedestrianWithPredctedPath[i].rosTime;
                var predictionPointDelta = (PedestrianWithPredctedPath[i].predictedPath.Time_step.Nanosec / 1e9F);
                int target_index = (int)(deltaTime / predictionPointDelta) + 1;
                var predictedPath = PedestrianWithPredctedPath[i].predictedPath;

                // Calculate Position and Rotation.
                // Position 
                Vector3 startPosition = ROS2Utility.RosMGRSToUnityPosition(predictedPath.Path[target_index-1].Position);
                Vector3 endPosition = ROS2Utility.RosMGRSToUnityPosition(predictedPath.Path[target_index].Position);
                Vector3 Velocity = (endPosition - startPosition) / predictionPointDelta;
                Vector3 targetPosition = npcPedestrian.GetComponent<Rigidbody>().position + (Velocity * Time.fixedDeltaTime);

                // Rotation
                float step = 100F;
                Quaternion endRotation= ROS2Utility.RosToUnityRotation(predictedPath.Path[target_index].Orientation);
                Quaternion targetRotation = Quaternion.RotateTowards(npcPedestrian.GetComponent<Rigidbody>().rotation, endRotation, step);

                // avoid extternalStop (debug).  
                if((1000 <= stopCount && stopCount <= 1100) || (6000 <= stopCount && stopCount <= 7000))
                {
                    var speed = 1.667F; // [6km/h]
                    targetPosition = (npcPedestrian.transform.position + (npcPedestrian.transform.forward * speed) * Time.fixedDeltaTime);
                    targetRotation = npcPedestrian.GetComponent<Rigidbody>().rotation;
                }
                else
                {
                    Debug.Log("startPosition : [ " + stopCount + " ] : " + (startPosition));
                    Debug.Log("currentPosition : [ " + stopCount + " ] : " + (npcPedestrian.transform.position));
                    Debug.Log("endPosition : [ " + stopCount + " ] : " + endPosition);
                    Debug.Log("targetPosition : [ " + stopCount + " ] : " + (targetPosition));
                    Debug.Log("ROSspeed : [ " + stopCount + " ] : " + Vector3.Distance(endPosition, startPosition)/predictionPointDelta *3.6F + "[km/s]");
                    Debug.Log("RATEspeed : [ " + stopCount + " ] : " + Vector3.Distance(targetPosition, npcPedestrian.transform.position)/0.02F + "[km/s]");
                }

                // update
                npcPedestrian.SetPosition(targetPosition);
                npcPedestrian.SetRotation(targetRotation);
            }

            // Cache egoPosition for calculating the distance between Ego and Pedestrian.
            egoPosition = ego.GetComponent<Rigidbody>().transform.position;
        }

        void predictionCallback(autoware_perception_msgs.msg.PredictedObjects receivedMsg){
            pedestrianWithPredctedPath.Clear();
            
            var objects = receivedMsg.Objects;
            for (var i = 0; i < objects.Length; i++)
            {
                var uuid = BitConverter.ToString(objects[i].Object_id.Uuid);
                if (perceptionTrackingResultRos2Publisher.idToNpc[uuid].GetType().Name == "NPCPedestrian")
                {
                    // Find the predicted path with the highest confidence.
                    var confidence = -1f;
                    var max_index = 0;
                    for (var j = 0; j < objects[i].Kinematics.Predicted_paths.Length; j++)
                    {
                        if (objects[i].Kinematics.Predicted_paths[j].Confidence > confidence)
                        {
                            confidence = objects[i].Kinematics.Predicted_paths[j].Confidence;
                            max_index = j;
                        }
                    }

                    // Set predicted path.
                    var NPCPedestrian = (NPCPedestrian)perceptionTrackingResultRos2Publisher.idToNpc[uuid];
                    var predictionPath = objects[i].Kinematics.Predicted_paths[max_index];
                    double rosTime = (receivedMsg.Header.Stamp.Sec + receivedMsg.Header.Stamp.Nanosec/1e9F);
                    pedestrianWithPredctedPath.Add((NPCPedestrian, rosTime, predictionPath));

                    // Set prediction status.
                    var rosNpcPosition = ROS2Utility.RosMGRSToUnityPosition(objects[i].Kinematics.Initial_pose_with_covariance.Pose.Position);
                    Vector3 npcPosition = new Vector3((float)rosNpcPosition.x, (float)rosNpcPosition.y, (float)rosNpcPosition.z);
                    var distanceEgo2NPC = Vector3.Distance(egoPosition, npcPosition);
                    bool isInLidarRange =  (distanceEgo2NPC <= predictionDistance);
                    if(usePredictionControl && isInLidarRange)
                    {
                        NPCPedestrian.outerPathControl = usePathControl;
                        NPCPedestrian.outerSpeedControl = useSpeedControl;
                    } 
                    else
                    {
                        NPCPedestrian.outerPathControl = false;
                        NPCPedestrian.outerSpeedControl = false;
                    }
                }
            }
        }

        void Callback(PerceptionResultSensor.OutputData output){
            outputData = output;
        }

        void OnDestroy(){
            SimulatorROS2Node.RemoveSubscription<autoware_perception_msgs.msg.PredictedObjects>(Subscriber);
        }
    }
}