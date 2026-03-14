using System.Collections.Generic;
using System.IO;
using UnityEngine.UIElements;
using YamlDotNet.RepresentationModel;

public static class PrefabParser
{
    public static PrefabModel Parse(string prefabPath)
    {
        var yaml = new YamlStream();

        using var reader = new StreamReader(prefabPath);
        yaml.Load(reader);

        var model = new PrefabModel();

        foreach (var doc in yaml.Documents)
        {
            var node = (YamlMappingNode)doc.RootNode;
            foreach (var child in node.Children)
            {
                var scalarNode = (YamlScalarNode)child.Key;
                var nodeName = scalarNode.Value;
           }

            // foreach (var entry in root.Children)
            // {
            //     var node = entry.Value as YamlMappingNode;
            //     if (node == null)
            //         continue;
            //
            //     if (node.Children.ContainsKey("m_Name"))
            //     {
            //         var go = new GameObjectNode();
            //
            //         go.Name = node["m_Name"].ToString();
            //
            //         model.GameObjects.Add(go);
            //     }
            // }
        }

        return model;
    }
}