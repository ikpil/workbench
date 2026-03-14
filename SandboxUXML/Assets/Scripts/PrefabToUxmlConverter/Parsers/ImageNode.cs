public class ImageNode
{
    public float R;
    public float G;
    public float B;
    public float A;
    
    public void Convert(GameObjectNode node, UxmlElement element)
    {
        var img = node.Image;
        if (img == null)
            return;

        int r = (int)(img.R * 255);
        int g = (int)(img.G * 255);
        int b = (int)(img.B * 255);

        element.Style["background-color"] = $"rgba({r},{g},{b},{img.A})";
    }

}