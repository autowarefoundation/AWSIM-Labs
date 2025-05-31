using UnityEngine;
using ROS2;
namespace AWSIM
{
    public class ROS2PredictionController : MonoBehaviour
    {
        // set with-prediction status
        protected PerceptionTrackingResultRos2Publisher perceptionTrackingResultRos2Publisher;
        protected QoSSettings qoSSettings = new QoSSettings();
        protected ISubscription<autoware_perception_msgs.msg.PredictedObjects> Subscriber;
        protected PerceptionResultSensor objectSensor;
        protected PerceptionResultSensor.OutputData outputData;
        protected UnityEngine.GameObject ego;
        protected UnityEngine.Vector3 egoPosition;
        protected string subscribedTopic = "/perception/object_recognition/objects";
        protected bool usePredictionControl = true;
        protected bool usePathControl = true;
        protected bool useSpeedControl = false;
        protected bool useEstimateRotation = false;
        protected float predictionDistance = 85;
        protected float minimumDistance = 0.1F;
        protected float speedWeight = 0.8F;
    }
}