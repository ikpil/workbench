using YamlDotNet.RepresentationModel;

public class UniPrefabBlock
{
    public long FileId;
    public string BlockName;

    public int m_ObjectHideFlags;
    public long m_CorrespondingSourceObject;
    public long m_PrefabInstance;
    public long m_PrefabAsset;

    public virtual void MergeFrom(YamlMappingNode block)
    {
        m_ObjectHideFlags = YamlHelper.GetInt(block, "m_ObjectHideFlags");
        m_CorrespondingSourceObject = YamlHelper.GetFileId(block, "m_CorrespondingSourceObject");
        m_PrefabInstance = YamlHelper.GetFileId(block, "m_PrefabInstance");
        m_PrefabAsset = YamlHelper.GetFileId(block, "m_PrefabAsset");
    }
}
