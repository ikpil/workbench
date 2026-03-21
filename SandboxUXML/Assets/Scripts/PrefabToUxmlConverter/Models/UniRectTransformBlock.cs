using System.Collections.Generic;
using YamlDotNet.RepresentationModel;

public class UniRectTransformBlock : UniComponentBlock
{
    public UniVector4 m_LocalRotation;
    public UniVector3 m_LocalPosition;
    public UniVector3 m_LocalScale;
    public int m_ConstrainProportionsScale;
    public List<long> m_Children = new();
    public long m_Father;
    public UniVector3 m_LocalEulerAnglesHint;
    public UniVector2 m_AnchorMin;
    public UniVector2 m_AnchorMax;
    public UniVector2 m_AnchoredPosition;
    public UniVector2 m_SizeDelta;
    public UniVector2 m_Pivot;

    public override void ApplyTo(UxmlElement element)
    {
        // Unity anchor 좌표계 (Y: 아래=0, 위=1) → CSS (Y: 위=0)
        float left   = m_AnchorMin.x;
        float top    = 1f - m_AnchorMax.y;
        float width  = m_AnchorMax.x - m_AnchorMin.x;
        float height = m_AnchorMax.y  - m_AnchorMin.y;

        element.Style["position"] = "absolute";
        element.Style["left"]     = $"{left   * 100f:F2}%";
        element.Style["top"]      = $"{top    * 100f:F2}%";
        element.Style["width"]    = $"{width  * 100f:F2}%";
        element.Style["height"]   = $"{height * 100f:F2}%";
    }

    public static bool CanHandle(string blockName, YamlMappingNode _)
        => blockName == "RectTransform";

    public override void MergeFrom(YamlMappingNode block)
    {
        base.MergeFrom(block);
        m_LocalRotation = YamlHelper.GetVector4(block, "m_LocalRotation");
        m_LocalPosition = YamlHelper.GetVector3(block, "m_LocalPosition");
        m_LocalScale = YamlHelper.GetVector3(block, "m_LocalScale");
        m_ConstrainProportionsScale = YamlHelper.GetInt(block, "m_ConstrainProportionsScale");
        m_Children = YamlHelper.GetFileIdList(block, "m_Children");
        m_Father = YamlHelper.GetFileId(block, "m_Father");
        m_LocalEulerAnglesHint = YamlHelper.GetVector3(block, "m_LocalEulerAnglesHint");
        m_AnchorMin = YamlHelper.GetVector2(block, "m_AnchorMin");
        m_AnchorMax = YamlHelper.GetVector2(block, "m_AnchorMax");
        m_AnchoredPosition = YamlHelper.GetVector2(block, "m_AnchoredPosition");
        m_SizeDelta = YamlHelper.GetVector2(block, "m_SizeDelta");
        m_Pivot = YamlHelper.GetVector2(block, "m_Pivot");
    }
}
