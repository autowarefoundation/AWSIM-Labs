using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System.Linq;

/// <summary>
/// This script finds a designated "Ego" vehicle and dynamically spawns a grid of ground prefabs around it.
/// The grid is aligned to the world axes, and cells are spawned/despawned as the vehicle moves
/// to ensure only a 3x3 area around the vehicle is active at any time.
/// It also includes editor gizmos to visualize the entire world grid.
/// </summary>
public class GroundGridSpawner : MonoBehaviour
{
    [Header("Grid Configuration")]
    [Tooltip("The prefab to use for each grid cell. Should be a plane or similar object.")]
    public GameObject gridCellPrefab;

    [Tooltip("The size of each square grid cell in world units.")]
    public float cellSize = 1000f;

    [Header("Ego Vehicle")] private GameObject _egoVehicle;
    private bool _isLookingForEgoVehicle = true;

    // Stores the currently active grid cells, mapping grid coordinates to the spawned GameObject.
    private readonly Dictionary<Vector2Int, GameObject> _spawnedCells = new();

    // The vehicle's current grid position. Used to detect when to update the grid.
    private Vector2Int _currentEgoCell;

    void Start()
    {
        if (gridCellPrefab == null)
        {
            Debug.LogError("Grid Cell Prefab is not assigned! Please assign a prefab in the Inspector.", this);
            this.enabled = false;
            return;
        }

        TryInitializeEgoVehicle();
    }

    void TryInitializeEgoVehicle()
    {
        var candidates = GameObject.FindGameObjectsWithTag("Ego");
        foreach (var obj in candidates)
        {
            if (obj.layer != LayerMask.NameToLayer("Vehicle")) continue;
            _egoVehicle = obj;
            Debug.Log("Ego vehicle found: " + _egoVehicle.name);
            UpdateGrid();
            return;
        }

        Debug.LogWarning("Ego vehicle not found. Will retry every second.");
        if (_isLookingForEgoVehicle)
        {
            StartCoroutine(RetryInitialize());
        }
    }

    IEnumerator RetryInitialize()
    {
        _isLookingForEgoVehicle = false;
        while (!_egoVehicle)
        {
            yield return new WaitForSeconds(1f);
            // The retry attempt is now inside the loop
            TryInitializeEgoVehicle();
        }

        _isLookingForEgoVehicle = true;
    }

    void Update()
    {
        if (!_egoVehicle) return;
        // Calculate the vehicle's current grid cell coordinates
        Vector2Int newEgoCell = GetCellCoordinatesFromPosition(_egoVehicle.transform.position);

        // If the vehicle has moved to a new cell, update the spawned grid
        if (newEgoCell == _currentEgoCell) return;
        _currentEgoCell = newEgoCell;
        UpdateGrid();
    }

    /// <summary>
    /// Calculates the integer grid coordinates for a given world position.
    /// </summary>
    /// <param name="position">The world position (e.g., vehicle's position).</param>
    /// <returns>The (X, Z) grid coordinates as a Vector2Int.</returns>
    private Vector2Int GetCellCoordinatesFromPosition(Vector3 position)
    {
        var x = Mathf.RoundToInt(position.x / cellSize);
        var z = Mathf.RoundToInt(position.z / cellSize);
        return new Vector2Int(x, z);
    }

    /// <summary>
    /// Updates the grid by spawning new cells and despawning old ones.
    /// This maintains a 3x3 grid centered on the vehicle's current cell.
    /// </summary>
    private void UpdateGrid()
    {
        if (!_egoVehicle) return;

        // Use a HashSet for efficient lookups of which cells are required.
        var requiredCells = new HashSet<Vector2Int>();

        // Determine the 3x3 grid of cells that should be active
        for (var x = -1; x <= 1; x++)
        {
            for (var z = -1; z <= 1; z++)
            {
                requiredCells.Add(new Vector2Int(_currentEgoCell.x + x, _currentEgoCell.y + z));
            }
        }

        // 1. Despawn cells that are no longer needed
        var cellsToRemove = new List<Vector2Int>();
        foreach (var cell in _spawnedCells.Where(cell => !requiredCells.Contains(cell.Key)))
        {
            Destroy(cell.Value);
            cellsToRemove.Add(cell.Key);
        }

        foreach (var key in cellsToRemove)
        {
            _spawnedCells.Remove(key);
        }

        // 2. Spawn new cells that have entered the 3x3 area
        foreach (var cellCoord in requiredCells)
        {
            var cellPosition =
                new Vector3(cellCoord.x * cellSize, transform.position.y, cellCoord.y * cellSize);
            if (_spawnedCells.ContainsKey(cellCoord)) continue;
            // Use the spawner's own 'y' position for the cell's world position.
            var newCell = Instantiate(gridCellPrefab, cellPosition, Quaternion.identity, transform);
            newCell.name = $"GridCell_{cellCoord.x}_{cellCoord.y}";
            _spawnedCells.Add(cellCoord, newCell);
        }
    }

    /// <summary>
    /// Draws gizmos in the editor to visualize the grid system.
    /// </summary>
    void OnDrawGizmos()
    {
        // Draw the gizmo grid around the vehicle if it exists, otherwise use the spawner's position.
        var centerPosition = (_egoVehicle != null) ? _egoVehicle.transform.position : this.transform.position;
        var centerCell = GetCellCoordinatesFromPosition(centerPosition);

        var gizmoSize = new Vector3(cellSize, 0.1f, cellSize);

        // Draw a large grid to visualize the overall map structure
        Gizmos.color = Color.gray;
        for (var x = -10; x <= 10; x++)
        {
            for (var z = -10; z <= 10; z++)
            {
                var cellCoord = new Vector2Int(centerCell.x + x, centerCell.y + z);
                var cellCenter = new Vector3(cellCoord.x * cellSize, 0, cellCoord.y * cellSize);
                Gizmos.DrawWireCube(cellCenter, gizmoSize);
            }
        }

        // Highlight the 3x3 grid that should be currently active around the center position
        Gizmos.color = Color.yellow;
        for (var x = -1; x <= 1; x++)
        {
            for (var z = -1; z <= 1; z++)
            {
                var cellCoord = new Vector2Int(centerCell.x + x, centerCell.y + z);
                var cellCenter = new Vector3(cellCoord.x * cellSize, 0, cellCoord.y * cellSize);
                Gizmos.DrawWireCube(cellCenter, gizmoSize);
            }
        }
    }
}
