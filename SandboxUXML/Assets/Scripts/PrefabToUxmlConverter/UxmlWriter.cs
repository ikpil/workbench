using System.IO;
using System.Text;
using UnityEngine;

public class UxmlWriter
{
//     public static void Write(PrefabModel model)
//     {
//         var sb = new StringBuilder();

//         sb.AppendLine(@"<?xml version=""1.0"" encoding=""utf-8""?>");
//         sb.AppendLine(@"
// <ui:UXML
//     xmlns:xsi=""http://www.w3.org/2001/XMLSchema-instance""
//     xmlns:ui=""UnityEngine.UIElements""
//     xmlns:uie=""UnityEditor.UIElements""
//     xsi:noNamespaceSchemaLocation=""../UIElementsSchema/UIElements.xsd"">
//         ");


//         foreach (var node in model.GameObjects)
//         {
//             Write(sb, node, 1);
//         }

//         sb.AppendLine(@"");
//         sb.AppendLine("</ui:UXML>");

//         string output = "Assets/UIConverted";
//         if (!Directory.Exists(output))
//         {
//             Directory.CreateDirectory(output);
//         }

//         var path = $"{output}/Panel.uxml";
//         using var fs = new FileStream(path, FileMode.Create, FileAccess.Write);
//         using var writer = new StreamWriter(fs, Encoding.UTF8);
//         writer.Write(sb.ToString());

//         Debug.Log($"UXML 생성 완료: {path}");
//     }


//     public static void Write(StringBuilder sb, GameObjectNode node, int depth)
//     {
//     }
}