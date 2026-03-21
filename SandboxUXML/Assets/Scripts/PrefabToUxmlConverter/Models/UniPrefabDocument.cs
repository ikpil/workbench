using System.Collections.Generic;

public class UniPrefabDocument
{
    private readonly Dictionary<long, UniPrefabBlock> _blocks = new();

    public void AddBlock(UniPrefabBlock block)
    {
        _blocks[block.FileId] = block;
    }

    public T GetBlock<T>(long fileId) where T : UniPrefabBlock
    {
        return _blocks.TryGetValue(fileId, out var block) ? block as T : null;
    }

    public IEnumerable<T> GetBlocksOfType<T>() where T : UniPrefabBlock
    {
        foreach (var block in _blocks.Values)
            if (block is T typed)
                yield return typed;
    }
}
