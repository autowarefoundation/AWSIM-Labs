using System.Collections.Generic;
using UnityEngine;

namespace AWSIM.TrafficSimulation
{
    /// <summary>
    /// Decision step implementation for a NPC vehicle simulation.
    /// Based on the results of the cognitive step, it outputs a short-term target point and a decision to decelerate or accelerate.
    /// </summary>
    public class NPCVehicleDecisionStep
    {
        private NPCVehicleConfig config;

        // MinFrontVehicleDistance is added to the threshold for the distance at which an obstacle is considered dangerous.
        // The vehicle is controlled to stop at this distance away from the obstacle(e.g. another vehicle in front of the vehicle).
        private const float MinFrontVehicleDistance = 4f;
        private const float MinStopDistance = 1.5f;

        public NPCVehicleDecisionStep(NPCVehicleConfig config)
        {
            this.config = config;
        }

        public void Execute(IReadOnlyList<NPCVehicleInternalState> states)
        {
            var _egoVehicle = GameObject.FindWithTag("Ego");
            var ego_rigidbody = _egoVehicle.GetComponent<Rigidbody>();
            foreach (var state in states)
            {
                UpdateTargetPoint(state, ego_rigidbody);
                UpdateSpeedMode(state, config);
            }
        }

        /// <summary>
        /// Set short-term target point of the vehicle to the next waypoint.
        /// </summary>
        /// <param name="state"></param>
        private static void UpdateTargetPoint(NPCVehicleInternalState state, Rigidbody ego_rigidbody)
        {
            if (state.ShouldDespawn || state.CurrentFollowingLane == null)
                return;
            
            var distanceEgo2NPC = Vector3.Distance(ego_rigidbody.transform.position, state.Vehicle.transform.position);
            bool isInLidarRange =  distanceEgo2NPC <= 200;
            bool isEmpty = 
                (state.Vehicle.outerTargetPoint.x == 0) &&
                (state.Vehicle.outerTargetPoint.y == 0) &&
                (state.Vehicle.outerTargetPoint.z == 0);
            
            bool usePrediction = (!isEmpty) && state.Vehicle.usePathControl && isInLidarRange;
            if(usePrediction){
                state.TargetPoint = state.Vehicle.outerTargetPoint;
            }
            else
            {
                state.TargetPoint = state.CurrentFollowingLane.Waypoints[state.WaypointIndex];
            }
        }

        /// <summary>
        /// Update speed mode according to the following cognition results and stoppable distance calculated by speed.<br/>
        /// Possible speed modes and conditions are the following:<br/>
        /// - SLOW when the vehicle is in a sharp curve, needs to keep distance from a front vehicle or is entering yielding lane.<br/>
        /// - STOP when the vehicle can stop safely at a stop point(e.g. a stop line or a point that an obstacle exists).<br/>
        /// - SUDDEN_STOP when the vehicle cannot stop safely at a stop point.<br/>
        /// - ABSOLUTE_STOP when the vehicle cannot stop using SUDDEN_STOP<br/>
        /// - NORMAL under other conditions.
        // /// </summary>
        private static void UpdateSpeedMode(NPCVehicleInternalState state, NPCVehicleConfig config)
        {
            if (state.ShouldDespawn)
            {
                return;
            }

            var absoluteStopDistance = CalculateStoppableDistance(state.Speed, config.AbsoluteDeceleration) + MinStopDistance;
            var suddenStopDistance = CalculateStoppableDistance(state.Speed, config.SuddenDeceleration) + 2 * MinStopDistance;
            var stopDistance = CalculateStoppableDistance(state.Speed, config.Deceleration) + 3 * MinStopDistance;
            var slowDownDistance = stopDistance + 4 * MinStopDistance;

            var distanceToStopPointByFrontVehicle = onlyGreaterThan(state.DistanceToFrontVehicle - MinFrontVehicleDistance, -MinFrontVehicleDistance);
            var distanceToStopPointByTrafficLight = CalculateTrafficLightDistance(state, suddenStopDistance);
            var distanceToStopPointByRightOfWay = CalculateYieldingDistance(state);
            var distanceToStopPoint = Mathf.Min(distanceToStopPointByFrontVehicle, distanceToStopPointByTrafficLight, distanceToStopPointByRightOfWay);

            state.IsStoppedByFrontVehicle = false;
            if (distanceToStopPointByFrontVehicle <= stopDistance)
            {
                state.IsStoppedByFrontVehicle = true;
            }

            if(state.Vehicle.useSpeedControl == true)
                state.SpeedMode = NPCVehicleSpeedMode.PREDICTION_CONTROL;
            else if (distanceToStopPoint <= absoluteStopDistance)
                state.SpeedMode = NPCVehicleSpeedMode.ABSOLUTE_STOP;
            else if (distanceToStopPoint <= suddenStopDistance)
                state.SpeedMode = NPCVehicleSpeedMode.SUDDEN_STOP;
            else if (distanceToStopPoint <= stopDistance)
                state.SpeedMode = NPCVehicleSpeedMode.STOP;
            else if (distanceToStopPoint <= slowDownDistance || state.IsTurning)
                state.SpeedMode = NPCVehicleSpeedMode.SLOW;
            else
                state.SpeedMode = NPCVehicleSpeedMode.NORMAL;
        }

        private static float CalculateTrafficLightDistance(NPCVehicleInternalState state, float suddenStopDistance)
        {
            var distanceToStopPointByTrafficLight = float.MaxValue;
            if (state.TrafficLightLane != null)
            {
                var distanceToStopLine =
                    state.SignedDistanceToPointOnLane(state.TrafficLightLane.StopLine.CenterPoint);
                switch (state.TrafficLightPassability)
                {
                    case TrafficLightPassability.GREEN:
                        break;
                    case TrafficLightPassability.YELLOW:
                        if (distanceToStopLine < suddenStopDistance) break;
                        distanceToStopPointByTrafficLight = distanceToStopLine;
                        break;
                    case TrafficLightPassability.RED:
                        distanceToStopPointByTrafficLight = distanceToStopLine;
                        break;
                }
            }
            return onlyGreaterThan(distanceToStopPointByTrafficLight, 0);
        }

        private static float CalculateYieldingDistance(NPCVehicleInternalState state)
        {
            var distanceToStopPointByRightOfWay = float.MaxValue;
            if (state.YieldPhase != NPCVehicleYieldPhase.NONE && state.YieldPhase != NPCVehicleYieldPhase.ENTERING_INTERSECTION && state.YieldPhase != NPCVehicleYieldPhase.AT_INTERSECTION)
                distanceToStopPointByRightOfWay = state.SignedDistanceToPointOnLane(state.YieldPoint);
            return onlyGreaterThan(distanceToStopPointByRightOfWay, -float.MaxValue);
        }

        private static float CalculateStoppableDistance(float speed, float deceleration)
        {
            return onlyGreaterThan(speed * speed / 2f / deceleration, 0);
        }

        private static float onlyGreaterThan(float value, float min_value = 0)
        { return value >= min_value ? value : float.MaxValue; }

        public void ShowGizmos(IReadOnlyList<NPCVehicleInternalState> states)
        {
            foreach (var state in states)
            {
                switch (state.SpeedMode)
                {
                    case NPCVehicleSpeedMode.ABSOLUTE_STOP:
                    case NPCVehicleSpeedMode.SUDDEN_STOP:
                    case NPCVehicleSpeedMode.STOP:
                        Gizmos.color = Color.red;
                        break;
                    case NPCVehicleSpeedMode.SLOW:
                        Gizmos.color = Color.yellow;
                        break;
                    default:
                        Gizmos.color = Color.green;
                        break;
                }

                var currentPosition = state.FrontCenterPosition;
                currentPosition.y += 1f;

                Gizmos.DrawLine(currentPosition, state.TargetPoint);

                var rotation = Quaternion.LookRotation(currentPosition - state.TargetPoint);
                Gizmos.matrix = Matrix4x4.TRS(state.TargetPoint, rotation, Vector3.one);
                Gizmos.DrawFrustum(Vector3.zero, 30f, 1f, 0f, 1f);
                Gizmos.matrix = Matrix4x4.identity;
            }
        }
    }
}
