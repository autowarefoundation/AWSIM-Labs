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
        float speed = 1.667F; // initial_speed [m/s] = [6km/h] 
        float step = 3F; // ration_speed [deg/s]
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
                Quaternion endRotation= ROS2Utility.RosToUnityRotation(predictedPath.Path[target_index].Orientation);
                Quaternion targetRotation = Quaternion.RotateTowards(npcPedestrian.GetComponent<Rigidbody>().rotation, endRotation, step);

                // avoid extternalStop (debug).
                var npcSpeed = Vector3.Dot(npcPedestrian.lastVelocity,  npcPedestrian.transform.forward);
                if(npcSpeed <= 0.01F)npcPedestrian.stopCount++;
                if(500 <= npcPedestrian.stopCount && npcPedestrian.stopCount <= 600 && npcPedestrian.outerPathControl){
                    npcPedestrian.stopCount++;
                    targetPosition = (npcPedestrian.transform.position + (npcPedestrian.transform.forward * speed) * Time.fixedDeltaTime);
                    targetRotation = npcPedestrian.GetComponent<Rigidbody>().rotation;
                }
                else
                {
                    if(600 <= npcPedestrian.stopCount) npcPedestrian.stopCount = 0;
                    Debug.Log("startPosition : [ " + npcPedestrian.stopCount + " ] : " + (startPosition));
                    Debug.Log("currentPosition : [ " + npcPedestrian.stopCount + " ] : " + (npcPedestrian.transform.position));
                    Debug.Log("endPosition : [ " + npcPedestrian.stopCount + " ] : " + endPosition);
                    Debug.Log("targetPosition : [ " + npcPedestrian.stopCount + " ] : " + (targetPosition));
                    Debug.Log("ROSspeed : [ " + npcPedestrian.stopCount + " ] : " + Vector3.Distance(endPosition, startPosition)/predictionPointDelta *3.6F + "[km/s]");
                    Debug.Log("RATEspeed : [ " + npcPedestrian.stopCount + " ] : " + Vector3.Distance(targetPosition, npcPedestrian.transform.position)/0.02F + "[km/s]");
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