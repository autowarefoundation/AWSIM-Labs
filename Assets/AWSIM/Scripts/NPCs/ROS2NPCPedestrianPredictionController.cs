using UnityEngine;
using ROS2;
/**************/
using System;
using System.Collections;
using System.Collections.Generic;
using System.Reflection;
/**********************/
namespace AWSIM
{
    public class ROS2NPCPedestrianPredictionController : ROS2PredictionController
    {
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
            for(int i = 0; i < pedestrianWithPredctedPath.Count; i++)
            {
                // Calculate TargetPosition from predicted path. 
                var deltaTime =(currentSec + currentNanosec/1e9F) - pedestrianWithPredctedPath[i].rosTime;
                var predictionPointDelta = (pedestrianWithPredctedPath[i].predictedPath.Time_step.Nanosec / 1e9F);
                int target_index = (int)(deltaTime / predictionPointDelta) + 1;
                var predictedPath = pedestrianWithPredctedPath[i].predictedPath;
                Vector3 targetPosition = ROS2Utility.RosMGRSToUnityPosition(predictedPath.Path[target_index].Position);

                // Set the destination.  
                var npcPedestrian = pedestrianWithPredctedPath[i].Pedestrian;
                npcPedestrian.SetPosition(targetPosition);
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
                    bool isInLidarRange =  (distanceEgo2NPC <= prediction_distance);
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