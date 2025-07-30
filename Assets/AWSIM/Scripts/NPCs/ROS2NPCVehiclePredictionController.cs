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

        double lastDeltaTime = -1;
        float maxSteer = 15F;
        float lowpassRate = 0.3F;
        
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

                if(npcVehicle.outerPathControl == false)continue;
                if(predictedPath.Path.Count() < 4)
                {
                    Debug.Log("Path-size is too short.");
                    Debug.Log("Path-size is " + predictedPath.Path.Count());
                    continue;
                }

                // Calculate Position Velocity, and Rotation.
                float offsetTime;
                if(deltaTime >= Time.fixedDeltaTime)offsetTime = deltaTime - Time.fixedDeltaTime;
                else offsetTime = deltaTime + Time.fixedDeltaTime; 

                // Position. 
                var interporatedPosition = ComputeInterpolatedPostion(predictedPath, deltaTime);
                Vector3 targetPosition   = ROS2Utility.RosMGRSToUnityPosition(interporatedPosition);

                // Velocity
                var interporatedInitialPosition = ComputeInterpolatedPostion(predictedPath, offsetTime);
                Vector3 offsetPosition = ROS2Utility.RosMGRSToUnityPosition(interporatedInitialPosition);
                if(deltaTime >= Time.fixedDeltaTime)
                {
                    Vector3 predictedVelocity = (targetPosition - offsetPosition) / Time.fixedDeltaTime;
                    npcVehicle.predictedVelocity = lowpassRate*predictedVelocity + (1-lowpassRate)*npcVehicle.predictedVelocity;
                    var accel = (predictedVelocity - npcVehicle.predictedVelocity) / Time.fixedDeltaTime;
                    npcVehicle.predictedAcceleration = accel.magnitude;
                }

                // Rotation.
                int startIndex = (int)(deltaTime / predictionPointDelta);
                int endIndex   = startIndex + 1;
                float t         = (float)(deltaTime - (predictionPointDelta*startIndex));
                Quaternion startRotation  = ROS2Utility.RosToUnityRotation(predictedPath.Path[startIndex].Orientation);
                Quaternion endRotation    = ROS2Utility.RosToUnityRotation(predictedPath.Path[endIndex].Orientation);
                Quaternion targetRotation = Quaternion.RotateTowards(startRotation, endRotation, maxSteer*t);

                // debug    
                Vector3 InitialVelocity = new Vector3(
                    (float)Kinematics.Initial_twist_with_covariance.Twist.Linear.X,
                    (float)Kinematics.Initial_twist_with_covariance.Twist.Linear.Y,
                    (float)Kinematics.Initial_twist_with_covariance.Twist.Linear.Z);

                // Debug.Log(deltaTime.ToString("F4") + "] predictionInput_Velocity :  " + (InitialVelocity.magnitude*3.6F).ToString("F4") + " [km/s]" );
                // Debug.Log(deltaTime.ToString("F4") + "] Akima_Velocity :  " + (((targetPosition - offsetPosition) / Time.fixedDeltaTime).magnitude*3.6F).ToString("F4") + " [km/s]" );
                // Debug.Log(deltaTime.ToString("F4") + "] lowpass_Velocity :  " + (npcVehicle.predictedVelocity.magnitude*3.6F).ToString("F4") + " [km/s]" );

                if(npcVehicle.outerPathControl)
                {
                    Debug.Log(deltaTime.ToString("F4") + "] setPostion :  " + (targetPosition).ToString("F4")); 
                    npcVehicle.SetPosition(targetPosition);
                    npcVehicle.SetRotation(targetRotation);
                    // npcVehicle.GetComponent<Rigidbody>().transform.position = targetPosition;
                    // npcVehicle.GetComponent<Rigidbody>().transform.rotation = targetRotation;
                }
            }

            // Cache egoPosition for calculating the distance between Ego and Vehicle.
            egoPosition = ego.GetComponent<Rigidbody>().transform.position;
            lastDeltaTime = deltaTime;
        }

        void predictionCallback(autoware_perception_msgs.msg.PredictedObjects receivedMsg){
            npcVehicleWithPredctedPath.Clear();

            var objects = receivedMsg.Objects;
            for (var i = 0; i < objects.Length; i++)
            {
                var uuid = BitConverter.ToString(objects[i].Object_id.Uuid);
                if (perceptionTrackingResultRos2Publisher.idToNpc[uuid].GetType().Name == "NPCVehicle")
                {
                    // Find the predicted path with the highest confidence.
                    var confidence = -1f;
                    var maxIndex = 0;
                    for (var j = 0; j < objects[i].Kinematics.Predicted_paths.Length; j++)
                    {
                        if (objects[i].Kinematics.Predicted_paths[j].Confidence >= confidence)
                        {
                            confidence = objects[i].Kinematics.Predicted_paths[j].Confidence;
                            maxIndex = j;
                        }
                    }

                    // Set predicted path.
                    var npcVehicle = (NPCVehicle)perceptionTrackingResultRos2Publisher.idToNpc[uuid];
                    var predictedPath = objects[i].Kinematics.Predicted_paths[maxIndex];
                    var Kinematics = objects[i].Kinematics;
                    double rosTime = (receivedMsg.Header.Stamp.Sec + receivedMsg.Header.Stamp.Nanosec/1e9F);
                    npcVehicleWithPredctedPath.Add((
                        npcVehicle, 
                        rosTime, 
                        predictedPath,
                        Kinematics));

                    // Set prediction status.
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