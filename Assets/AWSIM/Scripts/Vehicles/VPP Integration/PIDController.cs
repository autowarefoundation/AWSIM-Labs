using System;
using UnityEngine;

namespace AWSIM.Scripts.Vehicles.VPP_Integration
{
    public enum VehicleState
    {
        Idle,
        Accelerating,
        Braking,
        Coasting
    }

    public class PIDController
    {
        private float Kp { get; set; }
        private float Ki { get; set; }
        private float Kd { get; set; }

        private float _integral;
        private float _previousError;
        private float _outputMin;
        private float _outputMax;
        private float _integralLimit;
        private float _derivativeFilterCoefficient;
        private float _deadband;
        private bool _useAdaptiveGains;
        private Func<float, float> _gainAdjuster;

        public PIDController(float kp, float ki, float kd, float outputMin = float.MinValue,
            float outputMax = float.MaxValue,
            float integralLimit = float.MaxValue, float derivativeFilterCoefficient = 0.1f,
            float deadband = 0.0f, bool useAdaptiveGains = false, Func<float, float> gainAdjuster = null)
        {
            Kp = kp;
            Ki = ki;
            Kd = kd;
            _outputMin = outputMin;
            _outputMax = outputMax;
            _integralLimit = integralLimit;
            _derivativeFilterCoefficient = derivativeFilterCoefficient;
            _deadband = deadband;
            _useAdaptiveGains = useAdaptiveGains;
            _gainAdjuster = gainAdjuster;
            _integral = 0;
            _previousError = 0;
        }

        public float Compute(float error, float deltaTime)
        {
            // deadband
            if (Mathf.Abs(error) < _deadband)
            {
                error = 0;
            }

            // Adaptive gain adjustment
            // if (_useAdaptiveGains && _gainAdjuster != null)
            // {
            //     float adjustmentFactor = _gainAdjuster(error);
            //     Kp *= adjustmentFactor;
            //     Ki *= adjustmentFactor;
            //     Kd *= adjustmentFactor;
            // }

            // Integral with windup prevention
            _integral += error * deltaTime;
            _integral = Mathf.Clamp(_integral, -_integralLimit, _integralLimit);

            // Derivative with filtering
            float derivative = (_previousError - error) / deltaTime;
            derivative = Mathf.Lerp(derivative, (_previousError - error) / deltaTime, _derivativeFilterCoefficient);

            float output = Kp * error + Ki * _integral + Kd * derivative;
            output = Mathf.Clamp(output, _outputMin, _outputMax);
            _previousError = error;

            // Debug.Log($"Error: {error}, Integral: {_integral}, Derivative: {derivative}, Output: {output}");

            return output;
        }

        // Reset method
        public void Reset()
        {
            _integral = 0;
            _previousError = 0;
        }
    }
}
