using System.Collections.Generic;

public class UxmlElement
{
    public string Tag = "ui:VisualElement";
    public Dictionary<string, string> Attributes = new();
    public Dictionary<string, string> Style = new();
    public List<UxmlElement> Children = new();
}
