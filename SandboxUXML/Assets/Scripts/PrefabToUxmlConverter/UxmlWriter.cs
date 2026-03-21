using System.Collections.Generic;
using System.IO;
using System.Text;
using UnityEngine;

public static class UxmlWriter
{
    public static void Write(UniPrefabHierarchy hierarchy, string prefabPath)
    {
        var uxmlPath = Path.ChangeExtension(prefabPath, ".uxml");
        var outputDir = Path.GetDirectoryName(uxmlPath);
        if (!string.IsNullOrEmpty(outputDir) && !Directory.Exists(outputDir))
            Directory.CreateDirectory(outputDir);

        var sb = new StringBuilder();
        sb.AppendLine("<?xml version=\"1.0\" encoding=\"utf-8\"?>");
        sb.AppendLine("<ui:UXML");
        sb.AppendLine("    xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\"");
        sb.AppendLine("    xmlns:ui=\"UnityEngine.UIElements\"");
        sb.AppendLine("    xmlns:uie=\"UnityEditor.UIElements\"");
        sb.AppendLine("    xsi:noNamespaceSchemaLocation=\"../UIElementsSchema/UIElements.xsd\">");

        foreach (var root in hierarchy.Roots)
            WriteElement(sb, UxmlConverter.Convert(root), depth: 1);

        sb.AppendLine("</ui:UXML>");

        using var fs = new FileStream(uxmlPath, FileMode.Create, FileAccess.Write);
        using var writer = new StreamWriter(fs, Encoding.UTF8);
        writer.Write(sb.ToString());

        Debug.Log($"UXML 생성: {uxmlPath}");
    }

    private static void WriteElement(StringBuilder sb, UxmlElement element, int depth)
    {
        var indent = new string(' ', depth * 4);

        sb.Append($"{indent}<{element.Tag}");

        foreach (var attr in element.Attributes)
            sb.Append($" {attr.Key}=\"{attr.Value}\"");

        var style = BuildStyle(element.Style);
        if (style.Length > 0)
            sb.Append($" style=\"{style}\"");

        if (element.Children.Count == 0)
        {
            sb.AppendLine(" />");
        }
        else
        {
            sb.AppendLine(">");
            foreach (var child in element.Children)
                WriteElement(sb, child, depth + 1);
            sb.AppendLine($"{indent}</{element.Tag}>");
        }
    }

    private static string BuildStyle(Dictionary<string, string> style)
    {
        var parts = new List<string>();
        foreach (var kv in style)
            parts.Add($"{kv.Key}: {kv.Value};");
        return string.Join(" ", parts);
    }
}
