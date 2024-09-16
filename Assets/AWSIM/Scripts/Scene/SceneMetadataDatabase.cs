using UnityEngine;

namespace AWSIM.Scripts.Scene
{
    /// <summary>
    /// Holds scene information for scenes used in Loader.cs
    /// </summary>
    [System.Serializable]
    public class SceneInfo
    {
        public string sceneName;
        public bool isSs2Scene;
    }

    [CreateAssetMenu(fileName = "SceneMetadataDatabase", menuName = "AWSIM/Scene Meta Database")]
    public class SceneMetadataDatabase : ScriptableObject
    {
        public SceneInfo[] sceneInfos;
    }
}
