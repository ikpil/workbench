class RectTransformConverter : IComponentConverter
{
    public void Convert(GameObjectNode node, UxmlElement element)
    {
        var rect = node.RectTransform;
        if (rect == null)
            return;

        float left = rect.AnchorMinX * 100f;
        float right = (1 - rect.AnchorMaxX) * 100f;
        float bottom = rect.AnchorMinY * 100f;
        float top = (1 - rect.AnchorMaxY) * 100f;

        element.Style["position"] = "absolute";
        element.Style["left"] = $"{left:F2}%";
        element.Style["right"] = $"{right:F2}%";
        element.Style["top"] = $"{top:F2}%";
        element.Style["bottom"] = $"{bottom:F2}%";
    }
}