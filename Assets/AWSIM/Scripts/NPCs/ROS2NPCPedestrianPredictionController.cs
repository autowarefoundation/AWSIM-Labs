using UnityEngine;
/**************/
using System;
using System.Linq;
using System.Collections.Generic;
/**********************/

namespace AWSIM
{
    public class ROS2NPCPedestrianPredictionController : ROS2PredictionController
    {
        float initialSpeed = 1.6667F; // initial_speed [m/s] = [2km/h] 
        double lastDeltaTime = -1;
        int manualStartCount = 500;
        int manualEndCount   = 600;

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

        void FixedUpdate()
        {
            var PedestrianWithPredctedPath = pedestrianWithPredctedPath.Select(
                item => (item.Pedestrian, item.rosTime, item.predictedPath, item.Kinematics)).ToList();
            if(PedestrianWithPredctedPath.Count == 0)return;

            var deltaTime = (float)((double)Time.fixedTime - PedestrianWithPredctedPath[0].rosTime);
            if(deltaTime < 0)return;

            var predictionPointDelta = (PedestrianWithPredctedPath[0].predictedPath.Time_step.Nanosec / 1e9F);
            if (predictionPointDelta < 0) return;

            for (int i = 0; i < PedestrianWithPredctedPath.Count; i++)
            {
                var npcPedestrian = PedestrianWithPredctedPath[i].Pedestrian;
                var predictedPath = PedestrianWithPredctedPath[i].predictedPath;
                var Kinematics = PedestrianWithPredctedPath[i].Kinematics;

                // error handling
                if (npcPedestrian.outerPathControl == false) continue;
                if (predictedPath.Path.Count() < 2)
                {
                    Debug.Log("Path-size is too small.");
                    Debug.Log("Path-size is " + predictedPath.Path.Count());
                    return;
                }

                // Calculate Position and Rotation.
                // Position
                var interporatedPosition = ComputeInterpolatedPostion(predictedPath, deltaTime);
                Vector3 targetPosition = ROS2Utility.RosMGRSToUnityPosition(interporatedPosition);

                // Rotation
                int start_index = (int)(deltaTime / predictionPointDelta);
                int end_index = start_index + 1;
                float t = (float)(deltaTime - (predictionPointDelta * start_index)) / predictionPointDelta;
                Quaternion startRotation = ROS2Utility.RosToUnityRotation(predictedPath.Path[start_index].Orientation);
                Quaternion endRotation = ROS2Utility.RosToUnityRotation(predictedPath.Path[end_index].Orientation);
                Quaternion targetRotation = Quaternion.Slerp(startRotation, endRotation, t);

                if (deltaTime < lastDeltaTime)
                {
                    var prevTime = deltaTime - Time.fixedDeltaTime;

                    // Position
                    var interporatedInitialPosition = ComputeInterpolatedPostion(predictedPath, prevTime);
                    Vector3 initialPosition = ROS2Utility.RosMGRSToUnityPosition(interporatedInitialPosition);
                    npcPedestrian.lastPosition = initialPosition;
                    npcPedestrian.lastVelocity = (targetPosition - initialPosition) / Time.fixedDeltaTime;

                    // todo?: add lastRotation
                }

                // Avoid externalStop.
                var npcSpeed = Vector3.Dot(npcPedestrian.velocity, npcPedestrian.transform.forward);
                if (npcSpeed <= 0.001F) npcPedestrian.stopCount++;

                if (manualStartCount <= npcPedestrian.stopCount && npcPedestrian.stopCount <= manualEndCount)
                {
                    npcPedestrian.stopCount++;
                    var velocity = npcPedestrian.transform.forward * initialSpeed;
                    targetPosition = (npcPedestrian.transform.position + velocity * Time.fixedDeltaTime);
                    targetRotation = npcPedestrian.GetComponent<Rigidbody>().rotation;
                }
                else
                {
                    if (manualEndCount < npcPedestrian.stopCount)
                    {
                        npcPedestrian.stopCount = 0;
                        // todo: compute pose and velocity
                    }
                }

                // update
                if (npcPedestrian.outerPathControl)
                {
                    npcPedestrian.SetPosition(targetPosition);
                    npcPedestrian.SetRotation(targetRotation);
                }
            }

            // Cache egoPosition for calculating the distance between Ego and Pedestrian.
            egoPosition = ego.GetComponent<Rigidbody>().transform.position;
            lastDeltaTime = deltaTime;
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