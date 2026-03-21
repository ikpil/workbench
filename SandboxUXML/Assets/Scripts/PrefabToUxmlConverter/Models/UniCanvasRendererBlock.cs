using YamlDotNet.RepresentationModel;

public class UniCanvasRendererBlock : UniPrefabBlock
{
    public long m_GameObject;
    public int m_CullTransparentMesh;

    public static bool CanHandle(string blockName, YamlMappingNode _)
        => blockName == "CanvasRenderer";

    public override void MergeFrom(YamlMappingNode block)
    {
        base.MergeFrom(block);
        m_GameObject = YamlHelper.GetFileId(block, "m_GameObject");
        m_CullTransparentMesh = YamlHelper.GetInt(block, "m_CullTransparentMesh");
    }
}
