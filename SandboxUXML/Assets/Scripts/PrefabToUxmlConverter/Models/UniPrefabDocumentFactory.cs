using System;
using System.Collections.Generic;
using System.IO;
using YamlDotNet.RepresentationModel;

public class UniPrefabDocumentFactory
{
    private readonly List<IUniBlockFactory> _factories = new();

    // 기본 세팅 — Panel.prefab 기준 블록 등록
    public static UniPrefabDocumentFactory CreateDefault()
    {
        var factory = new UniPrefabDocumentFactory();
        factory.Register<UniGameObjectBlock>(UniGameObjectBlock.CanHandle);
        factory.Register<UniRectTransformBlock>(UniRectTransformBlock.CanHandle);
        factory.Register<UniCanvasRendererBlock>(UniCanvasRendererBlock.CanHandle);
        factory.Register<UniImageBlock>(UniImageBlock.CanHandle);
        return factory;
    }

    public void Register<T>(Func<string, YamlMappingNode, bool> canHandle) where T : UniPrefabBlock, new()
    {
        _factories.Add(new UniBlockFactory<T>(canHandle));
    }

    public UniPrefabDocument LoadFromFile(string path)
    {
        var yaml = new YamlStream();
        using var reader = new StreamReader(path);
        yaml.Load(reader);

        var doc = new UniPrefabDocument();

        foreach (var yamlDoc in yaml.Documents)
        {
            var root = (YamlMappingNode)yamlDoc.RootNode;
            long fileId = YamlHelper.ParseLong(root.Anchor.Value);
            string blockName = YamlHelper.GetBlockName(root);

            if (root.Children[new YamlScalarNode(blockName)] is not YamlMappingNode content)
                continue;

            foreach (var factory in _factories)
            {
                if (factory.CanHandle(blockName, content))
                {
                    doc.AddBlock(factory.Create(fileId, blockName, content));
                    break;
                }
            }
        }

        return doc;
    }
}
