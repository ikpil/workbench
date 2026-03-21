using YamlDotNet.RepresentationModel;

// 컴포넌트 블록 공통 기반 — m_GameObject 로 소속 GO 를 참조
public class UniComponentBlock : UniPrefabBlock
{
    public long m_GameObject;

    public override void MergeFrom(YamlMappingNode block)
    {
        base.MergeFrom(block);
        m_GameObject = YamlHelper.GetFileId(block, "m_GameObject");
    }
}
