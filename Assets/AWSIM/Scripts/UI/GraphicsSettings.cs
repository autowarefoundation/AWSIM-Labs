using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.Rendering.Universal;
using UnityEngine.UI;

namespace AWSIM.Scripts.UI
{
    public class GraphicsSettings : MonoBehaviour
    {
        private Dropdown _dropdown;
        private List<GameObject> _cameraObjectsList;
        private Light _sunSource;

        private int _gpuMemorySize;
        private int _initialQualityLevel;
        private int _currentQualityLevel;
        private bool _isInitialised;

        private readonly List<string> _dropdownOptions = new()
        {
            "Preset: Low",
            "Preset: Medium",
            "Preset: High",
            "Preset: Ultra"
        };

        private void Awake()
        {
            _gpuMemorySize = SystemInfo.graphicsMemorySize;
        }

        private void Start()
        {
            // add main camera to list (BEV cam is not initialized yet will be added later)
            _cameraObjectsList = new List<GameObject>
            {
                Camera.main?.gameObject,
            };
            _sunSource = GameObject.FindGameObjectWithTag("Sun").GetComponent<Light>();

            // Subscribe to camera initialization event
            BirdEyeView.OnCameraInitialized += UpdateGraphicSettingsForAddedCamera;

            // Set initial quality level
            InitialQualityLevel();

            // Populate dropdown with quality settings
            _dropdown = GetComponent<Dropdown>();
            _dropdown.options.Clear();

            foreach (var option in _dropdownOptions)
            {
                _dropdown.options.Add(new Dropdown.OptionData(option));
            }

            _dropdown.value = _initialQualityLevel;
            _dropdown.RefreshShownValue();
        }

        public void Activate()
        {
            enabled = true;
            Start();
        }

        private void InitialQualityLevel()
        {
            if (!_isInitialised)
            {
                // Select Quality settings level (URP Asset) based on the size of the device's graphics memory
                switch (_gpuMemorySize)
                {
                    case <= 2048:
                        _initialQualityLevel = 0;
                        GraphicsLowQuality();
                        break;
                    case <= 4096:
                        _initialQualityLevel = 1;
                        GraphicsMediumQuality();
                        break;
                    case <= 6144:
                        _initialQualityLevel = 2;
                        GraphicsHighQuality();
                        break;
                    case <= 8192:
                        _initialQualityLevel = 3;
                        GraphicsUltraQuality();
                        break;
                    default:
                        _initialQualityLevel = 3;
                        GraphicsUltraQuality();
                        break;
                }

                _isInitialised = true;
            }
        }

        public void UISetQuality(int value)
        {
            switch (value)
            {
                case 0:
                    GraphicsLowQuality();
                    break;
                case 1:
                    GraphicsMediumQuality();
                    break;
                case 2:
                    GraphicsHighQuality();
                    break;
                case 3:
                    GraphicsUltraQuality();
                    break;
            }
        }

        private void GraphicsLowQuality()
        {
            _currentQualityLevel = 0;
            QualitySettings.SetQualityLevel(0);

            // update camera parameters
            foreach (var cam in _cameraObjectsList)
            {
                cam.TryGetComponent<UniversalAdditionalCameraData>(out var cameraData);
                cam.TryGetComponent<Volume>(out var cameraVolume);
                var cameraVolumeProfile = cameraVolume.profile;

                cameraData.antialiasing = AntialiasingMode.None;
                cameraData.renderPostProcessing = false;
                cameraData.renderShadows = false;
                cameraData.allowHDROutput = false;

                if (cameraVolumeProfile.TryGet(out Bloom bloom))
                {
                    bloom.active = false;
                }

                if (cameraVolumeProfile.TryGet(out Tonemapping tonemapping))
                {
                    tonemapping.active = false;
                }
            }

            _sunSource.shadows = LightShadows.Hard;
        }

        private void GraphicsMediumQuality()
        {
            _currentQualityLevel = 1;
            QualitySettings.SetQualityLevel(1);

            // update camera and volume parameters
            foreach (var cam in _cameraObjectsList)
            {
                cam.TryGetComponent<UniversalAdditionalCameraData>(out var cameraData);
                cam.TryGetComponent<Volume>(out var cameraVolume);
                var cameraVolumeProfile = cameraVolume.profile;

                cameraData.antialiasing = AntialiasingMode.FastApproximateAntialiasing;
                cameraData.antialiasingQuality = AntialiasingQuality.Medium;
                cameraData.renderPostProcessing = true;
                cameraData.renderShadows = true;
                cameraData.allowHDROutput = false;

                if (cameraVolumeProfile.TryGet(out Bloom bloom))
                {
                    bloom.active = false;
                    bloom.intensity.value = 0.25f;
                }

                if (cameraVolumeProfile.TryGet(out Tonemapping tonemapping))
                {
                    tonemapping.active = false;
                }
            }

            _sunSource.shadows = LightShadows.Hard;
        }

        private void GraphicsHighQuality()
        {
            _currentQualityLevel = 2;
            QualitySettings.SetQualityLevel(2);

            // update camera parameters
            foreach (var cam in _cameraObjectsList)
            {
                cam.TryGetComponent<UniversalAdditionalCameraData>(out var cameraData);
                cam.TryGetComponent<Volume>(out var cameraVolume);
                var cameraVolumeProfile = cameraVolume.profile;

                cameraData.antialiasing = AntialiasingMode.SubpixelMorphologicalAntiAliasing;
                cameraData.antialiasingQuality = AntialiasingQuality.High;
                cameraData.renderPostProcessing = true;
                cameraData.renderShadows = true;
                cameraData.allowHDROutput = true;

                if (cameraVolumeProfile.TryGet(out Bloom bloom))
                {
                    bloom.active = true;
                    bloom.intensity.value = 0.5f;
                    bloom.highQualityFiltering.overrideState = false;
                    bloom.highQualityFiltering.value = false;
                }

                if (cameraVolumeProfile.TryGet(out Tonemapping tonemapping))
                {
                    tonemapping.active = true;
                    tonemapping.mode.value = TonemappingMode.ACES;
                }
            }

            _sunSource.shadows = LightShadows.Soft;
        }

        private void GraphicsUltraQuality()
        {
            _currentQualityLevel = 3;
            QualitySettings.SetQualityLevel(3);

            // update camera parameters
            foreach (var cam in _cameraObjectsList)
            {
                if (cam.TryGetComponent<UniversalAdditionalCameraData>(out var cameraData))
                {
                    cameraData.antialiasing = AntialiasingMode.SubpixelMorphologicalAntiAliasing;
                    cameraData.antialiasingQuality = AntialiasingQuality.High;
                    cameraData.renderPostProcessing = true;
                    cameraData.renderShadows = true;
                    cameraData.allowHDROutput = true;
                }

                if (cam.TryGetComponent<Volume>(out var cameraVolume))
                {
                    var cameraVolumeProfile = cameraVolume.profile;
                    if (cameraVolumeProfile.TryGet(out Bloom bloom))
                    {
                        bloom.active = true;
                        bloom.intensity.value = 1.0f;
                        bloom.highQualityFiltering.overrideState = true;
                        bloom.highQualityFiltering.value = true;
                    }

                    if (cameraVolumeProfile.TryGet(out Tonemapping tonemapping))
                    {
                        tonemapping.active = true;
                        tonemapping.mode.value = TonemappingMode.ACES;
                    }
                }
            }

            _sunSource.shadows = LightShadows.Soft;
        }

        // TODO: custom user settings (mozzz)
        // private void CustomAssetProperties(){}

        // Used for adding new cameras and updating their settings
        private void UpdateGraphicSettingsForAddedCamera(GameObject cam)
        {
            _cameraObjectsList.Add(cam);
            UISetQuality(_currentQualityLevel);
        }

        private void OnDestroy()
        {
            BirdEyeView.OnCameraInitialized -= UpdateGraphicSettingsForAddedCamera;
        }
    }
}
