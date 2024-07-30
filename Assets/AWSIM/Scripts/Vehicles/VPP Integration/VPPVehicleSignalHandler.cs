using AWSIM.Scripts.Vehicles.VPP_Integration.Enums;
using UnityEngine;
using UnityEngine.Serialization;
using VehiclePhysics;

namespace AWSIM.Scripts.Vehicles.VPP_Integration
{
    // Handle the signals for the VPP vehicle

    [RequireComponent(typeof(AutowareVPPAdapter))]
    [RequireComponent(typeof(VPVehicleController))]
    public class VPPVehicleSignalHandler : MonoBehaviour
    {
        [SerializeField] private AutowareVPPAdapter _adapter;
        [SerializeField] private VPVehicleController _vpVehicle;

        [Header("BrakeLight")]
        [SerializeField]
        private VehicleVisualEffect.EmissionMaterial[] brakeLights;

        [Header("TurnSignal")]
        [SerializeField]
        private VehicleVisualEffect.EmissionMaterial[] leftTurnSignalLights;

        [SerializeField] private VehicleVisualEffect.EmissionMaterial[] rightTurnSignalLights;
        [SerializeField] private float _turnSignalTimerIntervalSec = 0.5f;
        private float _turnSignalTimer;
        private bool _turnSignalOn;

        [Header("ReverseLight")]
        [SerializeField]
        private VehicleVisualEffect.EmissionMaterial[] _reverseLights;

        private void Reset()
        {
            if (_adapter == null)
                _adapter = GetComponent<AutowareVPPAdapter>();
            if (_vpVehicle == null)
                _vpVehicle = GetComponent<VPVehicleController>();
        }

        private void Start()
        {
            foreach (var e in brakeLights) e.Initialize();
            foreach (var e in leftTurnSignalLights) e.Initialize();
            foreach (var e in rightTurnSignalLights) e.Initialize();
            foreach (var e in _reverseLights) e.Initialize();
        }

        private void Update()
        {
            // User input for turn signal.
            if (Input.GetKey(KeyCode.Alpha1))
                _adapter.VehicleSignalInput = VPPSignal.Left;
            else if (Input.GetKey(KeyCode.Alpha2))
                _adapter.VehicleSignalInput = VPPSignal.Right;
            else if (Input.GetKey(KeyCode.Alpha3))
                _adapter.VehicleSignalInput = VPPSignal.Hazard;
            else if (Input.GetKey(KeyCode.Alpha4))
                _adapter.VehicleSignalInput = VPPSignal.None;

            // brake light.
            var isBrakeLight = IsBrakeLight();
            ApplyLights(brakeLights, isBrakeLight);

            // reverse light.
            var isReverseLight = IsReverseLight();
            ApplyLights(_reverseLights, isReverseLight);

            // User input for brake & reverse light.
            if (Input.GetKey(KeyCode.S) || Input.GetKey(KeyCode.Space))
            {
                ApplyLights(brakeLights, true);
            }

            if (_vpVehicle.data.bus[Channel.Vehicle][VehicleData.GearboxMode] == (int)Gearbox.AutomaticGear.R)
            {
                ApplyLights(brakeLights, true);
                ApplyLights(_reverseLights, true);
            }

            // turn signal light.
            if (IsTurnSignalOn() == false)
            {
                _turnSignalTimer = 0;
                _turnSignalOn = false;
                ApplyLights(leftTurnSignalLights, false);
                ApplyLights(rightTurnSignalLights, false);
                return;
            }

            _turnSignalTimer -= Time.deltaTime;
            if (_turnSignalTimer < 0f)
            {
                _turnSignalTimer = _turnSignalTimerIntervalSec;
                _turnSignalOn = !_turnSignalOn;
            }

            var isTurnLeftLight = IsTurnLeftLight();
            ApplyLights(leftTurnSignalLights, isTurnLeftLight);

            var isTurnRightLight = IsTurnRightLight();
            ApplyLights(rightTurnSignalLights, isTurnRightLight);
        }

        private bool IsBrakeLight()
        {
            return (_adapter.AutomaticShiftInput == Gearbox.AutomaticGear.D && _adapter.AccelerationInput < 0)
                   || (_adapter.AutomaticShiftInput == Gearbox.AutomaticGear.R && _adapter.AccelerationInput < 0);
        }

        private bool IsReverseLight()
        {
            return _adapter.AutomaticShiftInput == Gearbox.AutomaticGear.R;
        }

        private bool IsTurnSignalOn()
        {
            return _adapter.VehicleSignalInput is
                VPPSignal.Left or VPPSignal.Right or VPPSignal.Hazard;
        }

        private bool IsTurnLeftLight()
        {
            return _adapter.VehicleSignalInput is
                VPPSignal.Left or VPPSignal.Hazard && _turnSignalOn;
        }

        private bool IsTurnRightLight()
        {
            return _adapter.VehicleSignalInput is
                VPPSignal.Right or VPPSignal.Hazard && _turnSignalOn;
        }

        private static void ApplyLights(VehicleVisualEffect.EmissionMaterial[] emissionMaterials, bool isOn)
        {
            foreach (var e in emissionMaterials)
            {
                e.Set(isOn);
            }
        }

        private void OnDestroy()
        {
            foreach (var e in brakeLights) e.Destroy();
            foreach (var e in leftTurnSignalLights) e.Destroy();
            foreach (var e in rightTurnSignalLights) e.Destroy();
            foreach (var e in _reverseLights) e.Destroy();
        }
    }
}
