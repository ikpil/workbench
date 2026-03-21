using System;
using YamlDotNet.RepresentationModel;

public class UniBlockFactory<T> : IUniBlockFactory where T : UniPrefabBlock, new()
{
    private readonly Func<string, YamlMappingNode, bool> _canHandle;

    public UniBlockFactory(Func<string, YamlMappingNode, bool> canHandle)
    {
        _canHandle = canHandle;
    }

    public bool CanHandle(string blockName, YamlMappingNode block)
        => _canHandle(blockName, block);

    public UniPrefabBlock Create(long fileId, string blockName, YamlMappingNode block)
    {
        var b = new T();
        b.FileId    = fileId;
        b.BlockName = blockName;
        b.MergeFrom(block);
        return b;
    }
}
