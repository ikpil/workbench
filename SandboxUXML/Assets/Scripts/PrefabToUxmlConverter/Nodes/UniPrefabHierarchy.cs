using System.Collections.Generic;

public class UniPrefabHierarchy
{
    public List<UniGameObjectNode> Roots = new();

    public static UniPrefabHierarchy Build(UniPrefabDocument doc)
    {
        // 1. GO 마다 Node 생성
        var nodeMap = new Dictionary<long, UniGameObjectNode>();
        foreach (var go in doc.GetBlocksOfType<UniGameObjectBlock>())
            nodeMap[go.FileId] = new UniGameObjectNode { Block = go };

        // 2. 컴포넌트 블록을 해당 Node 에 연결 (m_GameObject → nodeMap)
        foreach (var comp in doc.GetBlocksOfType<UniComponentBlock>())
            if (nodeMap.TryGetValue(comp.m_GameObject, out var node))
                node.Components.Add(comp);

        // 3. RectTransform.m_Father 로 부모-자식 연결
        foreach (var rt in doc.GetBlocksOfType<UniRectTransformBlock>())
        {
            if (rt.m_Father == 0) continue;  // 루트

            if (!nodeMap.TryGetValue(rt.m_GameObject, out var childNode)) continue;

            var parentRt = doc.GetBlock<UniRectTransformBlock>(rt.m_Father);
            if (parentRt == null) continue;

            if (!nodeMap.TryGetValue(parentRt.m_GameObject, out var parentNode)) continue;

            childNode.Parent = parentNode;
            parentNode.Children.Add(childNode);
        }

        // 4. 루트 수집 (Parent 가 없는 Node)
        var hierarchy = new UniPrefabHierarchy();
        foreach (var node in nodeMap.Values)
            if (node.Parent == null)
                hierarchy.Roots.Add(node);

        return hierarchy;
    }
}
