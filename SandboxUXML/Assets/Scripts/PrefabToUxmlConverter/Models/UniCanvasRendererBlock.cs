using YamlDotNet.RepresentationModel;

public class UniCanvasRendererBlock : UniComponentBlock
{
    public int m_CullTransparentMesh;

    public static bool CanHandle(string blockName, YamlMappingNode _)
        => blockName == "CanvasRenderer";

    public override void MergeFrom(YamlMappingNode block)
    {
        base.MergeFrom(block);
        m_CullTransparentMesh = YamlHelper.GetInt(block, "m_CullTransparentMesh");
    }
}
