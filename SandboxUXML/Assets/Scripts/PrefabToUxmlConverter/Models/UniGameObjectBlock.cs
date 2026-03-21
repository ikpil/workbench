using System.Collections.Generic;
using YamlDotNet.RepresentationModel;

public class UniGameObjectBlock : UniPrefabBlock
{
    public int serializedVersion;
    public List<long> m_Component = new();
    public int m_Layer;
    public string m_Name;
    public string m_TagString;
    public long m_Icon;
    public int m_NavMeshLayer;
    public int m_StaticEditorFlags;
    public int m_IsActive;

    public override void ApplyTo(UxmlElement element)
    {
        element.Attributes["name"] = m_Name ?? "";
    }

    public static bool CanHandle(string blockName, YamlMappingNode _)
        => blockName == "GameObject";

    public override void MergeFrom(YamlMappingNode block)
    {
        base.MergeFrom(block);
        serializedVersion = YamlHelper.GetInt(block, "serializedVersion");
        m_Component = YamlHelper.GetComponentFileIds(block, "m_Component");
        m_Layer = YamlHelper.GetInt(block, "m_Layer");
        m_Name = YamlHelper.GetString(block, "m_Name");
        m_TagString = YamlHelper.GetString(block, "m_TagString");
        m_Icon = YamlHelper.GetFileId(block, "m_Icon");
        m_NavMeshLayer = YamlHelper.GetInt(block, "m_NavMeshLayer");
        m_StaticEditorFlags = YamlHelper.GetInt(block, "m_StaticEditorFlags");
        m_IsActive = YamlHelper.GetInt(block, "m_IsActive");
    }
}
