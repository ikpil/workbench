using YamlDotNet.RepresentationModel;

public interface IUniBlockFactory
{
    bool CanHandle(string blockName, YamlMappingNode block);
    UniPrefabBlock Create(long fileId, string blockName, YamlMappingNode block);
}
