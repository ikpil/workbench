using System;
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
        // offsetMin/offsetMax: 앵커 기준점에서 요소 모서리까지의 픽셀 오프셋
        // Unity 공식: offsetMin = anchoredPosition - sizeDelta * pivot
        //             offsetMax = anchoredPosition + sizeDelta * (1 - pivot)
        float offsetMinX = m_AnchoredPosition.x - m_SizeDelta.x * m_Pivot.x;
        float offsetMinY = m_AnchoredPosition.y - m_SizeDelta.y * m_Pivot.y;
        float offsetMaxX = m_AnchoredPosition.x + m_SizeDelta.x * (1f - m_Pivot.x);
        float offsetMaxY = m_AnchoredPosition.y + m_SizeDelta.y * (1f - m_Pivot.y);

        element.Style["position"] = "absolute";
        element.Style["overflow"] = "visible";

        // 4개 모서리를 anchor% + offset px 로 정의 (Unity RectTransform → CSS 직접 매핑)
        // left   = anchorMin.x * 100% + offsetMin.x
        element.Style["left"] = FormatEdge(m_AnchorMin.x, offsetMinX);
        // right  = (1 - anchorMax.x) * 100% - offsetMax.x
        element.Style["right"] = FormatEdge(1f - m_AnchorMax.x, -offsetMaxX);
        // top    = (1 - anchorMax.y) * 100% - offsetMax.y   (Unity Y-up → CSS Y-down)
        element.Style["top"] = FormatEdge(1f - m_AnchorMax.y, -offsetMaxY);
        // bottom = anchorMin.y * 100% + offsetMin.y
        element.Style["bottom"] = FormatEdge(m_AnchorMin.y, offsetMinY);
    }

    // anchor 비율(%)과 pixel 오프셋을 하나의 CSS 값으로 포맷
    private static string FormatEdge(float pct, float px)
    {
        bool hasPct = Math.Abs(pct) > 0.0001f;
        bool hasPx = Math.Abs(px) > 0.01f;

        if (!hasPct && !hasPx) return "0";
        if (!hasPx) return $"{pct * 100f:F2}%";
        if (!hasPct) return $"{px:F2}px";

        string sign = px >= 0 ? "+" : "-";
        return $"calc({pct * 100f:F2}% {sign} {Math.Abs(px):F2}px)";
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
