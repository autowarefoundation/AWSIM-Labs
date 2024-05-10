using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace AWSIM.TrafficSimulation
{
    /// <summary>
    /// Component for managing traffic simulators. Traffic manager collets all traffic simulators and manages the spawning process.
    /// - Reproducibility by Seed value
    /// </summary>
    public class TrafficManager : MonoBehaviour
    {
        [SerializeField, Tooltip("Seed value for random generator.")]
        public int seed;
        [Header("NPC Vehicle Settings")]
        [SerializeField] private NPCVehicleConfig vehicleConfig = NPCVehicleConfig.Default();

        [SerializeField, Tooltip("Vehicle layer for raytracing the collision distances.")]
        private LayerMask vehicleLayerMask;

        [SerializeField, Tooltip("Ground layer for raytracing the collision distances.")]
        private LayerMask groundLayerMask;

        [SerializeField, Tooltip("A maximum number of vehicles that can simultaneously live in the scene. Lowering this value results in less dense traffic but improves the simulator's performance.")]
        public int maxVehicleCount = 100;
        public int targetVehicleCount = 10;

        [SerializeField, Tooltip("Ego vehicle handler. If not set, the manager creates a dummy ego. This reference is also set automatically when the Ego spawns via the traffic simulator.")]
        private GameObject _egoVehicle;

        public GameObject egoVehicle
        {
            get => _egoVehicle;
            set
            {
                _egoVehicle = value;
                if (_egoVehicle != null)
                {
                    NpcVehicleSimulator.RegisterEgo(value);
                }
                else
                {
                    NpcVehicleSimulator.UnregisterEgo();
                    _egoVehicle = _dummyEgo;
                }
            }
        }

        [Header("Debug")]
        [SerializeField] protected bool showGizmos = false;
        [SerializeField] protected bool showYieldingPhase = false;
        [SerializeField] protected bool showObstacleChecking = false;
        [SerializeField] protected bool showSpawnPoints = false;
        public RandomTrafficSimulatorConfiguration[] randomTrafficSims;
        public RouteTrafficSimulatorConfiguration[] routeTrafficSims;
        public NPCVehicleSimulator NpcVehicleSimulator;
        private List<ITrafficSimulator> _trafficSimulatorNodes;
        private Dictionary<NPCVehicleSpawnPoint, Dictionary<ITrafficSimulator, GameObject>> _spawnLanes;
        private GameObject _dummyEgo;

        /// <summary>
        /// Adds traffic simulator to the manager
        /// </summary>
        public void AddTrafficSimulator(ITrafficSimulator simulator)
        {
            _trafficSimulatorNodes.Add(simulator);
        }

        /// <summary>
        /// Clears all NPC vehicles running in simulation
        /// </summary>
        private void ClearAll()
        {
            _trafficSimulatorNodes.Clear();
            NpcVehicleSimulator?.ClearAll();
            Destroy(_dummyEgo);
            _egoVehicle = null;
            Despawn();
        }

        private void Awake()
        {
            InitializeTrafficManager(seed, targetVehicleCount);
        }

        private void InitializeTrafficManager(int seedVal, int vehicleCount)
        {
            Random.InitState(seedVal);

            _spawnLanes = new Dictionary<NPCVehicleSpawnPoint, Dictionary<ITrafficSimulator, GameObject>>();
            _dummyEgo = new GameObject("DummyEgo");

            if (_egoVehicle == null)
            {
                _egoVehicle = _dummyEgo;
            }

            NpcVehicleSimulator = new NPCVehicleSimulator(vehicleConfig, vehicleLayerMask, groundLayerMask, vehicleCount, _egoVehicle);
            NpcVehicleSimulator.SetDummyEgo(_dummyEgo);

            VerifyIntegrationEnvironmentElements();
            _trafficSimulatorNodes = new List<ITrafficSimulator>();
            if (NpcVehicleSimulator == null)
            {
                Debug.LogError("Traffic manager requires NPC Vehicle Simulator script.");
                return;
            }

            foreach (var randomTrafficConf in randomTrafficSims)
            {
                RandomTrafficSimulator randomTs = new RandomTrafficSimulator(
                    this.gameObject,
                    randomTrafficConf.npcPrefabs,
                    randomTrafficConf.spawnableLanes,
                    NpcVehicleSimulator,
                    randomTrafficConf.maximumSpawns
                );
                randomTs.enabled = randomTrafficConf.enabled;
                _trafficSimulatorNodes.Add(randomTs);
            }

            foreach (var routeTrafficSimConf in routeTrafficSims)
            {
                RouteTrafficSimulator routeTs = new RouteTrafficSimulator(
                    this.gameObject,
                    routeTrafficSimConf.npcPrefabs,
                    routeTrafficSimConf.route,
                    NpcVehicleSimulator,
                    routeTrafficSimConf.maximumSpawns
                );
                routeTs.enabled = routeTrafficSimConf.enabled;
                _trafficSimulatorNodes.Add(routeTs);
            }
        }

        public void RestartTraffic()
        {
            ClearAll();
            InitializeTrafficManager(seed, targetVehicleCount);
        }

        private void VerifyIntegrationEnvironmentElements()
        {
            GameObject trafficLanesObject = GameObject.Find("TrafficLanes");
            if (trafficLanesObject == null)
            {
                Debug.LogError("VerifyIntegrationEnvironmentElements error: Object 'TrafficLanes' not found in the scene.");
            }

            Transform[] children = trafficLanesObject.GetComponentsInChildren<Transform>();
            HashSet<string> uniqueNames = new HashSet<string>();
            bool isAnyIntersectionLane = false;
            bool isAnyTrafficScript = false;
            foreach (Transform child in children)
            {
                var trafficScript = child.gameObject.GetComponent<TrafficLane>();
                if (trafficScript)
                {
                    isAnyTrafficScript = true;
                    if (trafficScript.intersectionLane)
                    {
                        isAnyIntersectionLane = true;
                    }
                    if (!uniqueNames.Add(child.name))
                    {
                        Debug.LogError("VerifyIntegrationEnvironmentElements error: Found repeated child name in the 'TrafficLanes' object: " + child.name);
                    }
                }
            }
            if (!isAnyIntersectionLane)
            {
                Debug.LogError("VerifyIntegrationEnvironmentElements error: Not found any TrafficLane with 'IntersectionLane' set to true.");
            }
            if (!isAnyTrafficScript)
            {
                Debug.LogError("VerifyIntegrationEnvironmentElements error: Not found any TrafficLane with 'TrafficScript'.");
            }

        }

        private void FixedUpdate()
        {
            // Manage NPC spawning with the traffic simulators

            // Clear null elements in current list of traffic simulator
            _trafficSimulatorNodes.RemoveAll(item => item == null);

            // Find out which lanes are used by multiple spawners
            // We can easly spawn vehicle on a lane which only one spawner hooked to that lane.
            // If there are multiple spawners for one lane, we need to manage spawning. Without
            // managing, one spawner might overcome all others because of the vehicle size check.
            foreach (var trafficSimulator in _trafficSimulatorNodes)
            {
                if (!trafficSimulator.IsEnabled()) continue;

                trafficSimulator.GetRandomSpawnInfo(out var spawnPoint, out var prefab);
                if (_spawnLanes.ContainsKey(spawnPoint))
                {
                    _spawnLanes[spawnPoint].Add(trafficSimulator, prefab);
                }
                else
                {
                    var tsims = new Dictionary<ITrafficSimulator, GameObject>();
                    tsims.Add(trafficSimulator, prefab);
                    _spawnLanes.Add(spawnPoint, tsims);
                }
            }

            // For lane with single vehicle spawner - just spawn it.
            // For lane with multiple vehicle spawners - make priorities and spawn one by one.
            foreach (var spawnLoc in _spawnLanes)
            {
                NPCVehicle spawnedVehicle;

                if (spawnLoc.Value.Count == 1)
                {
                    var tsimAndPrefab = spawnLoc.Value.First();
                    var trafficSim = tsimAndPrefab.Key;
                    var prefab = tsimAndPrefab.Value;
                    if (!NPCVehicleSpawner.IsSpawnable(prefab.GetComponent<NPCVehicle>().Bounds, spawnLoc.Key))
                        continue;
                    var spawned = trafficSim.Spawn(prefab, spawnLoc.Key, out spawnedVehicle);
                }
                else
                {
                    var priorityTrafficSimList = spawnLoc.Value.OrderByDescending(x => x.Key.GetCurrentPriority());
                    var priorityTrafficSimGo = priorityTrafficSimList.First();
                    var prefab = priorityTrafficSimGo.Value;
                    if (!NPCVehicleSpawner.IsSpawnable(prefab.GetComponent<NPCVehicle>().Bounds, spawnLoc.Key))
                    {
                        continue;
                    }
                    bool spawned = priorityTrafficSimGo.Key.Spawn(prefab, spawnLoc.Key, out spawnedVehicle);
                    if (spawned)
                    {
                        foreach (var rest in priorityTrafficSimList)
                        {
                            rest.Key.IncreasePriority(1);
                        }
                        priorityTrafficSimGo.Key.ResetPriority();
                    }
                }

                if (spawnedVehicle && spawnedVehicle.gameObject.tag == "Ego")
                {
                    NpcVehicleSimulator.EGOVehicle = spawnedVehicle.transform;
                }
            }

            _spawnLanes.Clear();
            NpcVehicleSimulator.StepOnce(Time.fixedDeltaTime);

            Despawn();
        }
        private void Despawn()
        {
            foreach (var state in NpcVehicleSimulator.VehicleStates)
            {
                if (state.ShouldDespawn)
                {
                    Object.DestroyImmediate(state.Vehicle.gameObject);
                }
            }
            NpcVehicleSimulator.RemoveInvalidVehicles();
        }

        private void OnDestroy()
        {
            NpcVehicleSimulator?.Dispose();
        }

        private void DrawSpawnPoints()
        {
            Gizmos.color = Color.cyan;
            foreach (var randomTrafficConf in randomTrafficSims)
            {
                foreach (var lane in randomTrafficConf.spawnableLanes)
                {
                    Gizmos.DrawCube(lane.Waypoints[0], new Vector3(2.5f, 0.2f, 2.5f));
                }
            }

            Gizmos.color = Color.magenta;
            foreach (var routeTrafficSimConf in routeTrafficSims)
            {
                if (routeTrafficSimConf.route.Length > 0)
                {
                    Gizmos.DrawCube(routeTrafficSimConf.route[0].Waypoints[0], new Vector3(2.5f, 0.2f, 2.5f));
                }
            }
        }

        private void OnDrawGizmos()
        {
            if (!showGizmos)
                return;

            var defaultColor = Gizmos.color;
            NpcVehicleSimulator?.ShowGizmos(showYieldingPhase, showObstacleChecking);
            if (showSpawnPoints)
                DrawSpawnPoints();

            Gizmos.color = defaultColor;
        }
    }
}
