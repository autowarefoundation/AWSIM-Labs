using UnityEngine;
using ROS2;
using System;
using System.Collections.Generic;
using PredictedPath = autoware_perception_msgs.msg.PredictedPath;
using GeometryPoint = geometry_msgs.msg.Point;

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
        protected bool useSpeedControl = true;
        protected float predictionDistance = 80;
        
        public GeometryPoint ComputeInterpolatedPostion(PredictedPath predictedPath, double deltaTime)
        {
            List<double> x = new List<double>{};
            List<double> y = new List<double>{};
            List<double> z = new List<double>{};
            foreach (var rosPose in predictedPath.Path)
            {
                x.Add(rosPose.Position.X);
                y.Add(rosPose.Position.Y);
                z.Add(rosPose.Position.Z);
            }
            
            double predictionPointDelta = (predictedPath.Time_step.Nanosec / 1e9F);
            GeometryPoint Position3d = new GeometryPoint();
            Position3d.X = ComputeAkimaSpline(x, deltaTime, predictionPointDelta, "x");
            Position3d.Y = ComputeAkimaSpline(y, deltaTime, predictionPointDelta, "y");
            Position3d.Z = ComputeAkimaSpline(z, deltaTime, predictionPointDelta, "z");
            return Position3d;
        }


        public double ComputeAkimaSpline(List<double> Values, double deltaTime, double predictionPointDelta, string key)
        {
            List<double> m = new List<double>{};
            var n = Values.Count;
            var h = predictionPointDelta;
            for(int i = 0; i < n-1; i++){
                m.Add((Values[i+1] - Values[i]) / predictionPointDelta);
            }

            List<double> s = new List<double>{};
            s.Add(m[0]);
            s.Add((m[0] + m[1]) / 2);
            for(int i = 2; i < n-2; i++){
                if((Math.Abs(m[i+1] - m[i]) + Math.Abs(m[i-1] - m[i-2])) == 0)
                {
                    s.Add((m[i] + m[i-1]) / 2);
                }
                else
                {
                    s.Add((Math.Abs(m[i+1] - m[i]) * m[i-1] + Math.Abs(m[i-1] - m[i-2]) * m[i]) / 
                          (Math.Abs(m[i+1] - m[i]) + Math.Abs(m[i-1] - m[i-2])));
                }
            }
            s.Add((m[n-2] + m[n-3]) / 2);
            s.Add(m[n-2]);

            List<double> a_ = new List<double>{};
            List<double> b_ = new List<double>{};
            List<double> c_ = new List<double>{};
            List<double> d_ = new List<double>{};
            for(int i = 0; i < n-1; i++)
            {
                a_.Add(Values[i]);
                b_.Add(s[i]);
                c_.Add((3*m[i] - 2*s[i] - s[i+1]) / h);
                d_.Add((s[i] + s[i+1] - 2*m[i]) / (h*h));
            }

            int startIndex = (int)(deltaTime / predictionPointDelta);
            double dx = deltaTime - startIndex*predictionPointDelta;
            var value = a_[startIndex] + b_[startIndex]*dx + c_[startIndex]*dx*dx + d_[startIndex]*dx*dx*dx;

            return value;
        }
    }
}