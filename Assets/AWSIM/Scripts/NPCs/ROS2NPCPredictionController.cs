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
    public class ROS2NPCPredictionController : ROS2PredictionController
    {
        private int stopCount = 0;

        List<( 
            NPCVehicle npcVehicle, 
            double rosTime, 
            autoware_perception_msgs.msg.PredictedPath predictedPath
        )> npcVehicleWithPredctedPath = new List<(NPCVehicle, double, autoware_perception_msgs.msg.PredictedPath)>{};


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
            for(int i = 0; i < npcVehicleWithPredctedPath.Count; i++)
            {
                var deltaTime =(currentSec + currentNanosec/1e9F) - npcVehicleWithPredctedPath[i].rosTime;
                var predictionPointDelta = (npcVehicleWithPredctedPath[i].predictedPath.Time_step.Nanosec / 1e9F);
                var predictedPath = npcVehicleWithPredctedPath[i].predictedPath;
                var npcVehicle = npcVehicleWithPredctedPath[i].npcVehicle;

                // Calculate TargetPosition from predicted path. 
                var distanceOffset = npcVehicle.speed * speedWeight; 
                var finalDistance = minimumDistance + distanceOffset; 
                float distance2D;
                var pathLength = predictedPath.Path.Length;
                int targetIndex = (int)(deltaTime / predictionPointDelta) + 1;
                Vector3 targetPosition = ROS2Utility.RosMGRSToUnityPosition(predictedPath.Path[targetIndex].Position);
                for( int j = 0; targetIndex < pathLength; j++)
                {
                    
                    var currentPoint = new Vector2(npcVehicle.GetComponent<Rigidbody>().position.x, npcVehicle.GetComponent<Rigidbody>().position.z);
                    var targetPoint = new Vector2(targetPosition.x, targetPosition.z);
                    distance2D = Vector2.Distance(currentPoint, targetPoint);
                    if(distance2D >= finalDistance)
                    {
                        break;
                    }
                    else
                    {
                        targetIndex += 1;
                        targetPosition = ROS2Utility.RosMGRSToUnityPosition(predictedPath.Path[targetIndex].Position);
                    }
                }

                // Estimate Rotation.
                Quaternion estimatedRotation;
                if(useEstimateRotation)
                {
                    var nextTagetPosition =  ROS2Utility.RosMGRSToUnityPosition(predictedPath.Path[targetIndex+1].Position);
                    var estimatedDirection = nextTagetPosition - targetPosition;
                    estimatedRotation = Quaternion.LookRotation(estimatedDirection);
                    if(estimatedDirection == Vector3.zero)
                    {
                        estimatedRotation = npcVehicle.predictRotation;
                    }
                }
                else
                {
                    estimatedRotation = ROS2Utility.RosToUnityRotation(predictedPath.Path[targetIndex].Orientation);
                }

                // set predcition value.
                var direction = estimatedRotation * Vector3.forward;
                npcVehicle.outerTargetPoint = targetPosition + (direction * npcVehicle.Bounds.size.y);
                npcVehicle.outerTargetRotation = estimatedRotation;
                npcVehicle.outerTargetPointTime = targetIndex*predictionPointDelta - (float)deltaTime;

                // estimate velocity and acceleration.
                
                // velocitty
                var startPosition = ROS2Utility.RosMGRSToUnityPosition(predictedPath.Path[targetIndex - 1].Position);
                var velocity = (targetPosition - startPosition) / (float)(predictionPointDelta);
                npcVehicle.outerSpeed = Vector3.Dot(velocity, Vector3.forward);
                
                // acceleration
                if(targetIndex >= 2)
                {
                    var prevStartPosition = ROS2Utility.RosMGRSToUnityPosition(predictedPath.Path[targetIndex - 2].Position);
                    var prevVelocity = (startPosition - prevStartPosition) / (float)(predictionPointDelta);
                    var prevSpeed = Vector3.Dot(prevVelocity, Vector3.forward);
                    npcVehicle.outerAcceleration = (npcVehicle.outerSpeed - prevSpeed)/ (float)(predictionPointDelta);
                }
                else
                {
                    var nextTargetPosition = ROS2Utility.RosMGRSToUnityPosition(predictedPath.Path[targetIndex + 1].Position);
                    var nextVelocity = (nextTargetPosition - targetPosition) / (float)(predictionPointDelta);
                    var nextSpeed = Vector3.Dot(nextVelocity, Vector3.forward);
                    npcVehicle.outerAcceleration = (nextSpeed - npcVehicle.outerSpeed) / (float)(predictionPointDelta);  
                }

                // Avoid external-stop
                if(npcVehicle.outerSpeed < 1.0F)stopCount++;
                else stopCount = 0;
                var isStack = (stopCount >= 1000);
                if(isStack) {
                    npcVehicle.outerSpeed = 3.0F;
                }
            }

            // Cache egoPosition for calculating the distance between Ego and Pedestrian.
            egoPosition = ego.GetComponent<Rigidbody>().transform.position;
        }

        void predictionCallback(autoware_perception_msgs.msg.PredictedObjects receivedMsg){
            npcVehicleWithPredctedPath.Clear();

            int currentSec;
            uint currentNanosec;
            SimulatorROS2Node.TimeSource.GetTime(out currentSec, out currentNanosec);

            var objects = receivedMsg.Objects;
            for (var i = 0; i < objects.Length; i++)
            {
                var uuid = BitConverter.ToString(objects[i].Object_id.Uuid);
                if (perceptionTrackingResultRos2Publisher.idToNpc[uuid].GetType().Name == "NPCVehicle")
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
                    var npcVehicle = (NPCVehicle)perceptionTrackingResultRos2Publisher.idToNpc[uuid];
                    var predictedPath = objects[i].Kinematics.Predicted_paths[max_index];
                    double rosTime = (receivedMsg.Header.Stamp.Sec + receivedMsg.Header.Stamp.Nanosec/1e9F);
                    npcVehicleWithPredctedPath.Add((npcVehicle, rosTime, predictedPath));

                    // Set prediction status.
                    // wait until prediction output becomes stable. 
                    var timeSinceSpawn = (currentSec + currentNanosec/1e9F) - (npcVehicle.spawnSec + npcVehicle.spawnNanosec/1e9F);
                    bool isReadyToPrediction = timeSinceSpawn > 6.0F;

                    var rosNpcPosition = ROS2Utility.RosMGRSToUnityPosition(objects[i].Kinematics.Initial_pose_with_covariance.Pose.Position);
                    Vector3 npcPosition = new Vector3((float)rosNpcPosition.x, (float)rosNpcPosition.y, (float)rosNpcPosition.z);
                    var distanceEgo2NPC = Vector3.Distance(egoPosition, npcPosition);
                    bool isInLidarRange =  (distanceEgo2NPC <= predictionDistance);
                    Debug.Log("[distanceEgo2NPC] : " + distanceEgo2NPC); //for debug

                    if(usePredictionControl && isInLidarRange && isReadyToPrediction){
                        npcVehicle.outerPathControl = usePathControl;
                        npcVehicle.outerSpeedControl = useSpeedControl;
                    } else {
                        npcVehicle.outerPathControl = false;
                        npcVehicle.outerSpeedControl = false;
                        return;
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