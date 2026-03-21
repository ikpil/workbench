using System.Collections.Generic;
using System.Globalization;
using System.Linq;
using YamlDotNet.RepresentationModel;

public static class YamlHelper
{
    public static string GetBlockName(YamlMappingNode root)
    {
        return ((YamlScalarNode)root.Children.First().Key).Value;
    }

    // --- scalar ---

    public static string GetString(YamlMappingNode node, string key)
    {
        if (node.Children.TryGetValue(new YamlScalarNode(key), out var val) && val is YamlScalarNode scalar)
            return scalar.Value;
        return null;
    }

    public static int GetInt(YamlMappingNode node, string key)
    {
        var s = GetString(node, key);
        return int.TryParse(s, out var v) ? v : 0;
    }

    public static float GetFloat(YamlMappingNode node, string key)
    {
        var s = GetString(node, key);
        return float.TryParse(s, NumberStyles.Float, CultureInfo.InvariantCulture, out var f) ? f : 0f;
    }

    public static long ParseLong(string s)
    {
        return long.TryParse(s, out var v) ? v : 0L;
    }

    // --- reference ---

    public static long GetFileId(YamlMappingNode node, string key)
    {
        if (node.Children.TryGetValue(new YamlScalarNode(key), out var val) && val is YamlMappingNode map)
            return ParseLong(GetString(map, "fileID"));
        return 0L;
    }

    // {fileID: ..., guid: ..., type: ...}
    public static UniAssetRef GetAssetRef(YamlMappingNode node, string key)
    {
        if (node.Children.TryGetValue(new YamlScalarNode(key), out var val) && val is YamlMappingNode map)
        {
            return new UniAssetRef
            {
                fileID = ParseLong(GetString(map, "fileID")),
                guid   = GetString(map, "guid") ?? string.Empty,
                type   = GetInt(map, "type"),
            };
        }

        return default;
    }

    // --- vector / color ---

    public static UniVector2 GetVector2(YamlMappingNode node, string key)
    {
        if (node.Children.TryGetValue(new YamlScalarNode(key), out var val) && val is YamlMappingNode map)
            return new UniVector2 { x = GetFloat(map, "x"), y = GetFloat(map, "y") };
        return default;
    }

    public static UniVector3 GetVector3(YamlMappingNode node, string key)
    {
        if (node.Children.TryGetValue(new YamlScalarNode(key), out var val) && val is YamlMappingNode map)
            return new UniVector3 { x = GetFloat(map, "x"), y = GetFloat(map, "y"), z = GetFloat(map, "z") };
        return default;
    }

    public static UniVector4 GetVector4(YamlMappingNode node, string key)
    {
        if (node.Children.TryGetValue(new YamlScalarNode(key), out var val) && val is YamlMappingNode map)
            return new UniVector4 { x = GetFloat(map, "x"), y = GetFloat(map, "y"), z = GetFloat(map, "z"), w = GetFloat(map, "w") };
        return default;
    }

    public static UniColor GetColor(YamlMappingNode node, string key)
    {
        if (node.Children.TryGetValue(new YamlScalarNode(key), out var val) && val is YamlMappingNode map)
            return new UniColor { r = GetFloat(map, "r"), g = GetFloat(map, "g"), b = GetFloat(map, "b"), a = GetFloat(map, "a") };
        return default;
    }

    // --- sequence ---

    // m_Component: [{component: {fileID: N}}, ...]
    public static List<long> GetComponentFileIds(YamlMappingNode node, string key)
    {
        var result = new List<long>();
        if (node.Children.TryGetValue(new YamlScalarNode(key), out var val) && val is YamlSequenceNode seq)
            foreach (var item in seq)
                if (item is YamlMappingNode itemMap)
                    result.Add(GetFileId(itemMap, "component"));
        return result;
    }

    // m_Children: [{fileID: N}, ...]
    public static List<long> GetFileIdList(YamlMappingNode node, string key)
    {
        var result = new List<long>();
        if (node.Children.TryGetValue(new YamlScalarNode(key), out var val) && val is YamlSequenceNode seq)
            foreach (var item in seq)
                if (item is YamlMappingNode itemMap)
                    result.Add(ParseLong(GetString(itemMap, "fileID")));
        return result;
    }
}
