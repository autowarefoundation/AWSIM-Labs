using UnityEngine;
/**************/
using System;
using System.Linq;
using System.Collections.Generic;
/**********************/

namespace AWSIM
{
    public class ROS2NPCPredictionController : ROS2PredictionController
    {

        float initialSpeed = 3.7778F;
        double lastDeltaTime = -1;
        bool isFixedPath = false;
        float maxSteer = 60F;

        List<( 
            NPCVehicle npcVehicle, 
            double rosTime, 
            autoware_perception_msgs.msg.PredictedPath predictedPath,
            autoware_perception_msgs.msg.PredictedObjectKinematics Kinematics
        )> npcVehicleWithPredctedPath = new List<(
            NPCVehicle, 
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
            var NPCVehicleWithPredctedPath = npcVehicleWithPredctedPath.Select(
                item => (item.npcVehicle, item.rosTime, item.predictedPath, item.Kinematics)).ToList();
            if(NPCVehicleWithPredctedPath.Count == 0)return;

            var deltaTime  = (float)((double)Time.fixedTime - NPCVehicleWithPredctedPath[0].rosTime);
            if(deltaTime < 0)return;

            var predictionPointDelta = (NPCVehicleWithPredctedPath[0].predictedPath.Time_step.Nanosec / 1e9F);
            if(predictionPointDelta < 0)return;

            for(int i = 0; i < NPCVehicleWithPredctedPath.Count; i++)
            {
                var npcVehicle           = NPCVehicleWithPredctedPath[i].npcVehicle;
                var predictedPath        = NPCVehicleWithPredctedPath[i].predictedPath;
                var Kinematics           = NPCVehicleWithPredctedPath[i].Kinematics;

                // error handling
                if(npcVehicle.outerPathControl == false)continue;
                if(predictedPath.Path.Count() < 4){
                    Debug.Log("Path-size is too short.");
                    Debug.Log("Path-size is " + predictedPath.Path.Count());
                    continue;
                }

                // Calculate Position and Rotation.
                // Position.
                var interporatedPosition = ComputeInterpolatedPostion(predictedPath, deltaTime);
                Vector3 targetPosition   = ROS2Utility.RosMGRSToUnityPosition(interporatedPosition);
                if (deltaTime <= lastDeltaTime){             
                    var interporatedInitialPosition = ComputeInterpolatedPostion(predictedPath, deltaTime-Time.fixedDeltaTime);
                    Vector3 initialPosition = ROS2Utility.RosMGRSToUnityPosition(interporatedInitialPosition);
                    npcVehicle.lastPosition = initialPosition;
                    npcVehicle.velocity = (targetPosition - initialPosition) / Time.fixedDeltaTime;
                    Debug.Log("velocity : [ " + Time.time.ToString("F4") + " ] : " + (npcVehicle.velocity.magnitude).ToString("F4"));
                }

                // Rotation.
                int start_index = (int)(deltaTime / predictionPointDelta);
                int end_index   = start_index + 1;
                float t         = (float)(deltaTime - (predictionPointDelta*start_index));
                Quaternion startRotation  = ROS2Utility.RosToUnityRotation(predictedPath.Path[start_index].Orientation);
                Quaternion endRotation    = ROS2Utility.RosToUnityRotation(predictedPath.Path[end_index].Orientation);
                Quaternion targetRotation = Quaternion.RotateTowards(startRotation, endRotation, maxSteer*t);
                
                // handling with changing with reference path. 
                if (deltaTime <= lastDeltaTime){
                    var prevTime = deltaTime - deltaTime-Time.fixedDeltaTime;

                    // Position
                    var interporatedInitialPosition = ComputeInterpolatedPostion(predictedPath, prevTime);
                    Vector3 initialPosition = ROS2Utility.RosMGRSToUnityPosition(interporatedInitialPosition);
                    npcVehicle.lastPosition = initialPosition;
                    
                    // Rotation
                    start_index = (int)(prevTime / predictionPointDelta);
                    end_index   = start_index + 1;
                    startRotation  = ROS2Utility.RosToUnityRotation(predictedPath.Path[start_index].Orientation);
                    endRotation    = ROS2Utility.RosToUnityRotation(predictedPath.Path[end_index].Orientation);
                    npcVehicle.lastRotation = new QuaternionD(Quaternion.RotateTowards(startRotation, endRotation, maxSteer*(prevTime)));
                }

                // Avoid external-stop (debug).
                var npcSpeed = Vector3.Dot(npcVehicle.velocity,  npcVehicle.transform.forward);
                if(npcSpeed <= 0.001F)npcVehicle.stopCount++;

                if(500 <= npcVehicle.stopCount && npcVehicle.stopCount <= 600){
                    npcVehicle.stopCount++;
                    npcVehicle.outerPathControl  = false;
                    npcVehicle.outerSpeedControl = false;
                    
                }
                else
                {
                    npcVehicle.outerPathControl  = usePathControl;
                    npcVehicle.outerSpeedControl = useSpeedControl;
                    if(600 < npcVehicle.stopCount)
                    {
                        npcVehicle.outerPathControl  = usePathControl;
                        npcVehicle.outerSpeedControl = useSpeedControl;
                        npcVehicle.stopCount = 0;
                        // todo: compute pose and velocity
                    } 
                }
                
                // update
                if(npcVehicle.outerPathControl){
                    npcVehicle.SetPosition(targetPosition);
                    npcVehicle.SetRotation(targetRotation);
                    Debug.Log("targetPosition : [ " + Time.time.ToString("F4") + " ] : " + (targetPosition).ToString("F4"));
                }
            }

            // Cache egoPosition for calculating the distance between Ego and Vehicle.
            egoPosition = ego.GetComponent<Rigidbody>().transform.position;
            lastDeltaTime = deltaTime;
        }

        void predictionCallback(autoware_perception_msgs.msg.PredictedObjects receivedMsg){
            if(isFixedPath)return;
            npcVehicleWithPredctedPath.Clear();

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
                    var Kinematics = objects[i].Kinematics;
                    double rosTime = (receivedMsg.Header.Stamp.Sec + receivedMsg.Header.Stamp.Nanosec/1e9F);
                    npcVehicleWithPredctedPath.Add((
                        npcVehicle, 
                        rosTime, 
                        predictedPath,
                        Kinematics));

                    // Set prediction status.
                    // wait until prediction output becomes stable. 
                    var rosNpcPosition  = ROS2Utility.RosMGRSToUnityPosition(objects[i].Kinematics.Initial_pose_with_covariance.Pose.Position);
                    Vector3 npcPosition = new Vector3((float)rosNpcPosition.x, (float)rosNpcPosition.y, (float)rosNpcPosition.z);
                    var distanceEgo2NPC = Vector3.Distance(egoPosition, npcPosition);
                    bool isInLidarRange =  (distanceEgo2NPC <= predictionDistance);
                    if(usePredictionControl && isInLidarRange){
                        npcVehicle.outerPathControl  = usePathControl;
                        npcVehicle.outerSpeedControl = useSpeedControl;
                    } else {
                        npcVehicle.outerPathControl  = false;
                        npcVehicle.outerSpeedControl = false;
                    }

                    // for debug
                    if(distanceEgo2NPC <= 40){
                        isFixedPath = true;
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