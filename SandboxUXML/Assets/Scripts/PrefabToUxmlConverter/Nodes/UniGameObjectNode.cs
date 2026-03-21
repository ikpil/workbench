using System.Collections.Generic;

// 하이러라키 트리의 노드 — Block 들을 연결하고 부모/자식 관계를 보유
public class UniGameObjectNode
{
    // --- linked blocks ---
    public UniGameObjectBlock Block;
    public List<UniComponentBlock> Components = new();

    // --- hierarchy ---
    public UniGameObjectNode Parent;
    public List<UniGameObjectNode> Children = new();
}
