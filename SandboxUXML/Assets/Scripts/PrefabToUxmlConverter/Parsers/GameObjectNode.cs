using System.Collections.Generic;
using YamlDotNet.RepresentationModel;

public class GameObjectNode
{
    public YamlMappingNode Root;
    public List<GameObjectNode> Children = new();
    
    public long FileId;
    public string Name;

    public RectTransformNode RectTransform;
    public ImageNode Image;
}
