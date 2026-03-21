public static class UxmlConverter
{
    public static UxmlElement Convert(UniGameObjectNode node)
    {
        var element = new UxmlElement();

        node.Block?.ApplyTo(element);

        foreach (var component in node.Components)
            component.ApplyTo(element);

        foreach (var child in node.Children)
            element.Children.Add(Convert(child));

        return element;
    }
}
