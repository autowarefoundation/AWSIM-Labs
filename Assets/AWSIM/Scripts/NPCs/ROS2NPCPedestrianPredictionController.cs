using UnityEngine;
/**************/
using System;
using System.Linq;
using System.Collections.Generic;
/**********************/

using System.IO; // debug
namespace AWSIM
{
    public class ROS2NPCPedestrianPredictionController : ROS2PredictionController
    {
        float initialSpeed = 1.6667F; // initial_speed [m/s] = [2km/h] 
        double lastDeltaTime = -1;
        List<( 
            NPCPedestrian Pedestrian, 
            double rosTime, 
            autoware_perception_msgs.msg.PredictedPath predictedPath,
            autoware_perception_msgs.msg.PredictedObjectKinematics Kinematics
        )> pedestrianWithPredctedPath = new List<(
            NPCPedestrian, 
            double, 
            autoware_perception_msgs.msg.PredictedPath,
            autoware_perception_msgs.msg.PredictedObjectKinematics)>{};

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
            var PedestrianWithPredctedPath = pedestrianWithPredctedPath.Select(
                item => (item.Pedestrian, item.rosTime, item.predictedPath, item.Kinematics)).ToList();
            for(int i = 0; i < PedestrianWithPredctedPath.Count; i++)
            {
                var npcPedestrian        = PedestrianWithPredctedPath[i].Pedestrian;
                var deltaTime            = (float)((double)Time.fixedTime - PedestrianWithPredctedPath[i].rosTime);
                var predictionPointDelta = (PedestrianWithPredctedPath[i].predictedPath.Time_step.Nanosec / 1e9F);
                var predictedPath        = PedestrianWithPredctedPath[i].predictedPath;
                var Kinematics           = PedestrianWithPredctedPath[i].Kinematics;

                // error handling
                if(npcPedestrian.outerPathControl == false)continue;
                if(deltaTime < 0)break;
                Debug.Log(Time.time+" : predictedPath.Path: " + predictedPath.Path.Count());
                if(predictedPath.Path.Count() < 2){
                    Debug.Log("Path-size is too small.");
                    Debug.Log("Path-size is " + predictedPath.Path.Count());
                    return;
                }
                
                // Calculate Position and Rotation.
                // Position
                var interporatedPosition = ComputeInterpolatedPostion(predictedPath, deltaTime);
                Vector3 targetPosition   = ROS2Utility.RosMGRSToUnityPosition(interporatedPosition);

                // Rotation
                int start_index = (int)(deltaTime / predictionPointDelta);
                int end_index   = start_index + 1;
                float t         = (float)(deltaTime - (predictionPointDelta*start_index)) / predictionPointDelta;
                Quaternion startRotation  = ROS2Utility.RosToUnityRotation(predictedPath.Path[start_index].Orientation);
                Quaternion endRotation    = ROS2Utility.RosToUnityRotation(predictedPath.Path[end_index].Orientation);
                Quaternion targetRotation = Quaternion.Slerp(startRotation, endRotation, t);

                // Avoid externalStop (debug).
                var npcSpeed = Vector3.Dot(npcPedestrian.velocity,  npcPedestrian.transform.forward);
                if(npcSpeed <= 0.001F)npcPedestrian.stopCount++;

                // for debug@
                var str = ROS2Utility.RosMGRSToUnityPosition(predictedPath.Path[start_index].Position);
                var end = ROS2Utility.RosMGRSToUnityPosition(predictedPath.Path[end_index].Position);

                if(500 <= npcPedestrian.stopCount && npcPedestrian.stopCount <= 600 && npcPedestrian.outerPathControl){
                    npcPedestrian.stopCount++;
                    var velocity           = npcPedestrian.transform.forward * initialSpeed;
                    targetPosition         = (npcPedestrian.transform.position + velocity * Time.fixedDeltaTime);
                    targetRotation         = npcPedestrian.GetComponent<Rigidbody>().rotation;
                }
                else
                {
                    if(600 < npcPedestrian.stopCount)
                    {
                        npcPedestrian.stopCount = 0;
                        
                        // var velocity = npcPedestrian.transform.forward * initialSpeed;
                        // npcPedestrian.lastVelocity = velocity;
                        // npcPedestrian.velocity = velocity;
                        // Vector3 initialPosition = new Vector3(targetPosition.x - (velocity.x*Time.fixedDeltaTime), targetPosition.y - (velocity.y*Time.fixedDeltaTime), targetPosition.z - (velocity.z*Time.fixedDeltaTime));
                        // npcPedestrian.lastPosition = initialPosition;
                    }

                    // for test
                    if(deltaTime < lastDeltaTime){
                        // var interporatedInitialPosition = ComputeInterpolatedPostion(predictedPath, deltaTime-Time.fixedDeltaTime);
                        // Vector3 initialPosition = ROS2Utility.RosMGRSToUnityPosition(interporatedInitialPosition);
                        // npcPedestrian.lastPosition = initialPosition;
                        // npcPedestrian.lastVelocity = (targetPosition - initialPosition) / Time.fixedDeltaTime;
                        // npcPedestrian.localLinearVelocity = npcPedestrian.GetComponent<Rigidbody>().transform.InverseTransformDirection(npcPedestrian.lastVelocity);
                    }
                }

                // update
                npcPedestrian.SetPosition(targetPosition);
                npcPedestrian.SetRotation(targetRotation);

                lastDeltaTime = deltaTime;

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
                        if (objects[i].Kinematics.Predicted_paths[j].Confidence >= confidence)  // >= test default=>
                        {
                            confidence = objects[i].Kinematics.Predicted_paths[j].Confidence;
                            max_index = j;
                        }
                    }

                    // Set predicted path.
                    var NPCPedestrian = (NPCPedestrian)perceptionTrackingResultRos2Publisher.idToNpc[uuid];
                    var predictionPath = objects[i].Kinematics.Predicted_paths[max_index];
                    var Kinematics = objects[i].Kinematics;
                    double rosTime = (receivedMsg.Header.Stamp.Sec + receivedMsg.Header.Stamp.Nanosec/1e9F);

                    pedestrianWithPredctedPath.Add((
                        NPCPedestrian, 
                        rosTime, 
                        predictionPath, 
                        Kinematics));
    
                    // Set prediction status.
                    var rosNpcPosition  = ROS2Utility.RosMGRSToUnityPosition(objects[i].Kinematics.Initial_pose_with_covariance.Pose.Position);
                    Vector3 npcPosition = new Vector3((float)rosNpcPosition.x, (float)rosNpcPosition.y, (float)rosNpcPosition.z);
                    var distanceEgo2NPC = Vector3.Distance(egoPosition, npcPosition);
                    bool isInLidarRange =  (distanceEgo2NPC <= predictionDistance);
                    if(usePredictionControl && isInLidarRange)
                    {
                        NPCPedestrian.outerPathControl  = usePathControl;
                        NPCPedestrian.outerSpeedControl = useSpeedControl;
                    } 
                    else
                    {
                        NPCPedestrian.outerPathControl  = false;
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