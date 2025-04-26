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
    public class ROS2NPCPredictionController : MonoBehaviour
    {
        public PerceptionTrackingResultRos2Publisher perceptionTrackingResultRos2Publisher;
        QoSSettings qoSSettings = new QoSSettings();
        string subscribedTopic = "/awsim/perception/object_recognition/objects";
        ISubscription<autoware_perception_msgs.msg.PredictedObjects> Subscriber;

        PerceptionResultSensor objectSensor;
        PerceptionResultSensor.OutputData outputData;
        int stopCount = 0;
        float minimum_length = 1.0F;
        void Start() {
            Subscriber = SimulatorROS2Node.CreateSubscription<autoware_perception_msgs.msg.PredictedObjects>(subscribedTopic, predictionCallback, qoSSettings.GetQoSProfile());
            perceptionTrackingResultRos2Publisher = GetComponent<PerceptionTrackingResultRos2Publisher>();
            objectSensor = GetComponent<PerceptionResultSensor>();
            objectSensor.OnOutputData += Callback;
        }

        void predictionCallback(autoware_perception_msgs.msg.PredictedObjects receivedMsg){
            var objects = receivedMsg.Objects;
            int rosSec = receivedMsg.Header.Stamp.Sec;
            uint rosNanosec = receivedMsg.Header.Stamp.Nanosec;

            int currentSec;
            uint currentNanosec;
            SimulatorROS2Node.TimeSource.GetTime(out currentSec, out currentNanosec);

            for (var i = 0; i < objects.Length; i++){

                var uuid = BitConverter.ToString(objects[i].Object_id.Uuid);
                if (perceptionTrackingResultRos2Publisher.idToNpc[uuid].GetType().Name == "NPCVehicle"){
                    List<Vector3> path = new List<Vector3>();
                    List<Quaternion> rotation = new List<Quaternion>();
                    var confidence = -1f;
                    var maxindex = 0;
                    for (var j = 0; j < objects[i].Kinematics.Predicted_paths.Length; j++){
                        if (objects[i].Kinematics.Predicted_paths[j].Confidence > confidence){
                            confidence = objects[i].Kinematics.Predicted_paths[j].Confidence;
                            maxindex = j;
                        }
                    }

                    var deltaTime =(currentSec + currentNanosec/1e9F) - (rosSec + rosNanosec/1e9F);

                    var predictionPointDeltaTime = (objects[i].Kinematics.Predicted_paths[maxindex].Time_step.Nanosec / 1e9F);
                    int first_step = (int)(deltaTime / predictionPointDeltaTime);
                    int end_step = first_step + 1;

                    var endPosition = ROS2Utility.RosMGRSToUnityPosition(objects[i].Kinematics.Predicted_paths[maxindex].Path[end_step].Position);

                    // use prediction-pose Rotation
                    // var endRotation = ROS2Utility.RosToUnityRotation(objects[i].Kinematics.Predicted_paths[maxindex].Path[end_step].Orientation);

                    // estimate Rotation
                    var toPosition =  ROS2Utility.RosMGRSToUnityPosition(objects[i].Kinematics.Predicted_paths[maxindex].Path[end_step+1].Position);
                    var estimatedDirection = toPosition - endPosition;
                    // Debug.Log("toPosition" + toPosition);
                    // Debug.Log("endPosition" + endPosition);
                    var endRotation = Quaternion.LookRotation(estimatedDirection);

                    // set prediction value
                
                    var npcVehicle = (NPCVehicle)perceptionTrackingResultRos2Publisher.idToNpc[uuid];
                    if(estimatedDirection == Vector3.zero){
                        endRotation = npcVehicle.predictRotation;
                        Debug.Log("endRotation" + endRotation);
                    }

                    // minimum length
                    minimum_length = 1.0F;
                    float distance2D;
                    if(npcVehicle.speed <= 0.1F){
                        minimum_length = 0.3F;   
                    }else{
                        // minimum_length = minimum_length + npcVehicle.speed * 2.0F;  // this 
                    }
                    minimum_length = minimum_length + npcVehicle.speed * 2.0F;  // this 

                    while(true){
                        endPosition = ROS2Utility.RosMGRSToUnityPosition(objects[i].Kinematics.Predicted_paths[maxindex].Path[end_step].Position);
                        Vector2 egoPoint = new Vector2(npcVehicle.lastPosition.x, npcVehicle.lastPosition.z);
                        Vector2 targetPoint = new Vector2(endPosition.x, endPosition.z);
                        distance2D = Vector2.Distance(egoPoint, targetPoint);
                        if(distance2D >= minimum_length || end_step >= 10)break;
                        first_step = first_step + 1;
                        end_step = first_step + 1;
                    }
                    Debug.Log("distance2D: " + distance2D);
                    Debug.Log("end_step: " + end_step);

                    var direction = endRotation * Vector3.forward;
                    if(distance2D >= minimum_length){
                        npcVehicle.outerTargetPoint = endPosition + (direction * npcVehicle.Bounds.size.y);
                        npcVehicle.outerTargetRotation = endRotation;
                        npcVehicle.outerTargetPointTime = end_step*predictionPointDeltaTime - deltaTime;
                    }

                    // estimate velocity and acceleration
                    var startPosition = ROS2Utility.RosMGRSToUnityPosition(objects[i].Kinematics.Predicted_paths[maxindex].Path[first_step].Position);
                    var velocity = (endPosition - startPosition) / (float)(predictionPointDeltaTime);
                    npcVehicle.outerSpeed = Vector3.Dot(velocity, Vector3.forward);
                    if(end_step >= 2){
                        var prevPosition = ROS2Utility.RosMGRSToUnityPosition(objects[i].Kinematics.Predicted_paths[maxindex].Path[first_step-1].Position);
                        var prevVelocity = (startPosition - prevPosition) / (float)(predictionPointDeltaTime);
                        var prevSpeed = Vector3.Dot(prevVelocity, Vector3.forward);
                        npcVehicle.outerAcceleration = (npcVehicle.outerSpeed - prevSpeed)/ (float)(predictionPointDeltaTime);
                    }
                    else
                    {
                        var nextPosition = ROS2Utility.RosMGRSToUnityPosition(objects[i].Kinematics.Predicted_paths[maxindex].Path[end_step+1].Position);
                        var nextVelocity = (nextPosition - endPosition) / (float)(predictionPointDeltaTime);
                        var nextSpeed = Vector3.Dot(nextVelocity, Vector3.forward);
                        npcVehicle.outerAcceleration = (nextSpeed - npcVehicle.outerSpeed) / (float)(predictionPointDeltaTime);  
                    }

                    // Avoid external-stop
                    if(npcVehicle.outerSpeed < 1.0F)stopCount++;
                    else stopCount = 0;
                    var isStack = (stopCount >= 1000);
                    if(isStack) {
                        npcVehicle.outerSpeed = 3.0F;
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