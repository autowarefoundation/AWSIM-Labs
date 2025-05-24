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
    public class ROS2NPCPedestrianPredictionController : MonoBehaviour
    {
        public PerceptionTrackingResultRos2Publisher perceptionTrackingResultRos2Publisher;
        QoSSettings qoSSettings = new QoSSettings();
        string subscribedTopic = "/awsim/perception/object_recognition/objects";

        ISubscription<autoware_perception_msgs.msg.PredictedObjects> Subscriber;

        PerceptionResultSensor objectSensor;
        PerceptionResultSensor.OutputData outputData;
        UnityEngine.GameObject ego;
        UnityEngine.Vector3 egoPosition;

        // set with-prediction status
        bool usePredictionControl = true;
        bool usePathControl = true;
        bool useSpeedControl = false;
        bool useEstimateRotation = true;
        float prediction_distance = 70;
        float minimum_distance = 1.0F;
        float speed_weight = 1.5F;
        Vector3 endPosittion;
        List<(NPCPedestrian ped, Vector3 posi)> Pedestrian2Position = new List<(NPCPedestrian, Vector3)>{};

        void Start() {
            Subscriber = SimulatorROS2Node.CreateSubscription<autoware_perception_msgs.msg.PredictedObjects>(subscribedTopic, predictionCallback, qoSSettings.GetQoSProfile());
            perceptionTrackingResultRos2Publisher = GetComponent<PerceptionTrackingResultRos2Publisher>();
            objectSensor = GetComponent<PerceptionResultSensor>();
            objectSensor.OnOutputData += Callback;
            ego = GameObject.FindWithTag("Ego");
        }

        void Update() {
            egoPosition = ego.GetComponent<Rigidbody>().transform.position;
            for(int i = 0; i < Pedestrian2Position.Count; i++){
                Pedestrian2Position[i].ped.SetPosition(Pedestrian2Position[i].posi);
            }
        }

        void predictionCallback(autoware_perception_msgs.msg.PredictedObjects receivedMsg){
            var objects = receivedMsg.Objects;
            int rosSec = receivedMsg.Header.Stamp.Sec;
            uint rosNanosec = receivedMsg.Header.Stamp.Nanosec;

            int currentSec;
            uint currentNanosec;
            SimulatorROS2Node.TimeSource.GetTime(out currentSec, out currentNanosec);                                        
            
            Pedestrian2Position.Clear();
            for (var i = 0; i < objects.Length; i++){
                var uuid = BitConverter.ToString(objects[i].Object_id.Uuid);
                if (perceptionTrackingResultRos2Publisher.idToNpc[uuid].GetType().Name == "NPCPedestrian"){
                    var rosNpcPosition = ROS2Utility.RosMGRSToUnityPosition(objects[i].Kinematics.Initial_pose_with_covariance.Pose.Position);
                    Vector3 npcPosition = new Vector3((float)rosNpcPosition.x, (float)rosNpcPosition.y, (float)rosNpcPosition.z);
                    var distanceEgo2NPC = Vector3.Distance(egoPosition, npcPosition);
                    bool isInLidarRange =  (distanceEgo2NPC <= prediction_distance);
                    
                    var NPCPedestrian = (NPCPedestrian)perceptionTrackingResultRos2Publisher.idToNpc[uuid];
                    if(usePredictionControl && isInLidarRange){
                        NPCPedestrian.outerPathControl = usePathControl;
                        NPCPedestrian.outerSpeedControl = useSpeedControl;
                    } else {
                        NPCPedestrian.outerPathControl = false;
                        NPCPedestrian.outerSpeedControl = false;
                    }            
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
                    
                    Vector3 endPosition = ROS2Utility.RosMGRSToUnityPosition(objects[i].Kinematics.Predicted_paths[maxindex].Path[end_step].Position);
                    Pedestrian2Position.Add((NPCPedestrian,endPosition));
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