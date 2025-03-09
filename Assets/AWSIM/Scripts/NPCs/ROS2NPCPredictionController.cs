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
        public int stopCount = 0;
        void Start() {
            Subscriber = SimulatorROS2Node.CreateSubscription<autoware_perception_msgs.msg.PredictedObjects>(subscribedTopic, myCallback, qoSSettings.GetQoSProfile());
            perceptionTrackingResultRos2Publisher = GetComponent<PerceptionTrackingResultRos2Publisher>();
            objectSensor = GetComponent<PerceptionResultSensor>();
            objectSensor.OnOutputData += Callback;
        }

        void myCallback(autoware_perception_msgs.msg.PredictedObjects receivedMsg){
            var objects = receivedMsg.Objects;
            int rosSec = receivedMsg.Header.Stamp.Sec;
            uint rosNanosec = receivedMsg.Header.Stamp.Nanosec;

            int currentSec;
            uint currentNanosec;
            SimulatorROS2Node.TimeSource.GetTime(out currentSec, out currentNanosec);

            for (var i = 0; i < objects.Length; i++){

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

                var uuid = BitConverter.ToString(objects[i].Object_id.Uuid);
                var npc = (NPCs)perceptionTrackingResultRos2Publisher.idToNpc[uuid];
                var objectPosition = objects[i].Kinematics.Predicted_paths[maxindex].Path[0].Position;
                int queueIndex = 0;
                double distance = 0.0;
                double nextDistance = 10.0;  

                var deltaTime =(currentSec + currentNanosec/1e9) - (rosSec + rosNanosec/1e9);

                uint predictionPointDeltaTime = objects[i].Kinematics.Predicted_paths[maxindex].Time_step.Nanosec;
                int first_step = (int)(deltaTime / predictionPointDeltaTime);
                int end_step = first_step + 1;

                var endPositin = ROS2Utility.RosMGRSToUnityPosition(objects[i].Kinematics.Predicted_paths[maxindex].Path[end_step].Position);
                var endRotation = ROS2Utility.RosToUnityRotation(objects[i].Kinematics.Predicted_paths[maxindex].Path[end_step].Orientation);

                if (perceptionTrackingResultRos2Publisher.idToNpc[uuid].GetType().Name == "NPCVehicle"){
                    var npcVehicle = (NPCVehicle)perceptionTrackingResultRos2Publisher.idToNpc[uuid];
                    var direction = endRotation * Vector3.forward;
                    npcVehicle.outerTargetPoint = endPositin + (direction * npcVehicle.Bounds.size.y);
                    npcVehicle.outerTargetRotation = endRotation;
                    npcVehicle.outerTargetPointTime = end_step*predictionPointDeltaTime - (float)deltaTime;

                    var startPositin = ROS2Utility.RosMGRSToUnityPosition(objects[i].Kinematics.Predicted_paths[maxindex].Path[first_step].Position);
                    var velocity = (endPositin - startPositin) / (float)(predictionPointDeltaTime);
                    npcVehicle.outerSpeed = Vector3.Dot(velocity, Vector3.forward);
                    if(end_step >= 2){
                        var prevprevPosition = ROS2Utility.RosMGRSToUnityPosition(objects[i].Kinematics.Predicted_paths[maxindex].Path[first_step-1].Position);
                        var prevVelocity = (startPositin - prevprevPosition) / (float)(predictionPointDeltaTime);
                        var prevSpeed = Vector3.Dot(prevVelocity, Vector3.forward);
                        npcVehicle.outerAcceleration = (npcVehicle.outerSpeed - prevSpeed)/ (float)(predictionPointDeltaTime);
                    }
                    else
                    {
                        var nextnextPosition = ROS2Utility.RosMGRSToUnityPosition(objects[i].Kinematics.Predicted_paths[maxindex].Path[end_step+1].Position);
                        var nextVelocity = (nextnextPosition - endPositin) / (float)(predictionPointDeltaTime);
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
                    // Debug.Log("speed : " +npcVehicle.outerSpeed);
                    // Debug.Log("accel : " +npcVehicle.outerAcceleration);
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