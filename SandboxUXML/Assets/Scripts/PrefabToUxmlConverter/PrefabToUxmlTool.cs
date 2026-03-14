using UnityEditor;
using UnityEngine;

namespace DefaultNamespace
{
    public class PrefabToUxmlTool
    {
        [MenuItem("Tools/UI/Convert Prefab To UXML")]
        public static void ConvertSelectedPrefab()
        {
            var prefabPath = "Assets/Prefabs/Panel.prefab";
            var model = PrefabParser.Parse(prefabPath);
            UxmlWriter.Write(model);
        }
    }
}