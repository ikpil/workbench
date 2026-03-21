using YamlDotNet.RepresentationModel;

public class UniImageBlock : UniComponentBlock
{
    public int m_Enabled;
    public int m_EditorHideFlags;
    public UniAssetRef m_Script;
    public string m_Name;
    public string m_EditorClassIdentifier;
    public long m_Material;
    public UniColor m_Color;
    public int m_RaycastTarget;
    public UniVector4 m_RaycastPadding;
    public int m_Maskable;
    public UniAssetRef m_Sprite;
    public int m_Type;
    public int m_PreserveAspect;
    public int m_FillCenter;
    public int m_FillMethod;
    public float m_FillAmount;
    public int m_FillClockwise;
    public int m_FillOrigin;
    public int m_UseSpriteMesh;
    public float m_PixelsPerUnitMultiplier;

    public override void ApplyTo(UxmlElement element)
    {
        int r = (int)(m_Color.r * 255);
        int g = (int)(m_Color.g * 255);
        int b = (int)(m_Color.b * 255);
        element.Style["background-color"] = $"rgba({r},{g},{b},{m_Color.a:F3})";
    }

    public static bool CanHandle(string blockName, YamlMappingNode block)
        => blockName == "MonoBehaviour"
        && YamlHelper.GetString(block, "m_EditorClassIdentifier")?.Contains("UnityEngine.UI.Image") == true;

    public override void MergeFrom(YamlMappingNode block)
    {
        base.MergeFrom(block);
        m_Enabled = YamlHelper.GetInt(block, "m_Enabled");
        m_EditorHideFlags = YamlHelper.GetInt(block, "m_EditorHideFlags");
        m_Script = YamlHelper.GetAssetRef(block, "m_Script");
        m_Name = YamlHelper.GetString(block, "m_Name");
        m_EditorClassIdentifier = YamlHelper.GetString(block, "m_EditorClassIdentifier");
        m_Material = YamlHelper.GetFileId(block, "m_Material");
        m_Color = YamlHelper.GetColor(block, "m_Color");
        m_RaycastTarget = YamlHelper.GetInt(block, "m_RaycastTarget");
        m_RaycastPadding = YamlHelper.GetVector4(block, "m_RaycastPadding");
        m_Maskable = YamlHelper.GetInt(block, "m_Maskable");
        m_Sprite = YamlHelper.GetAssetRef(block, "m_Sprite");
        m_Type = YamlHelper.GetInt(block, "m_Type");
        m_PreserveAspect = YamlHelper.GetInt(block, "m_PreserveAspect");
        m_FillCenter = YamlHelper.GetInt(block, "m_FillCenter");
        m_FillMethod = YamlHelper.GetInt(block, "m_FillMethod");
        m_FillAmount = YamlHelper.GetFloat(block, "m_FillAmount");
        m_FillClockwise = YamlHelper.GetInt(block, "m_FillClockwise");
        m_FillOrigin = YamlHelper.GetInt(block, "m_FillOrigin");
        m_UseSpriteMesh = YamlHelper.GetInt(block, "m_UseSpriteMesh");
        m_PixelsPerUnitMultiplier = YamlHelper.GetFloat(block, "m_PixelsPerUnitMultiplier");
    }
}
