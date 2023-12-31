// See https://aka.ms/new-console-template for more information

using System;
using Entitas;

namespace Box;

public class TestEntity : Entity
{
}

public sealed class TestContext : Context<TestEntity>
{
    private static readonly Func<TestEntity> CreateEntityDelegate = () => new TestEntity();

    public TestContext() : base(CID.Count, CreateEntityDelegate)
    {
    }
}

public class ComponentA : IComponent
{
}

public class ComponentB : IComponent
{
}

public class ComponentC : IComponent
{
}

public class CID
{
    public const int ComponentA = 0;
    public const int ComponentB = 1;
    public const int ComponentC = 2;

    public const int Count = 3;
}

public class Program
{
    public static int Main(string[] args)
    {
        var ctx = new TestContext();
        var entity = ctx.CreateEntity();
        entity.AddComponent(CID.ComponentA, new ComponentA());
        entity.AddComponent(CID.ComponentB, new ComponentB());
        entity.AddComponent(CID.ComponentC, new ComponentC());

        var a = entity.GetComponent(CID.ComponentA);
        var b = entity.GetComponent(CID.ComponentB);
        var c = entity.GetComponent(CID.ComponentC);
        
        // var info = new ContextInfo(
        //     "test",
        //     new[] { "Health", "Position", "View" },
        //     new[] { typeof(ComponentA), typeof(ComponentB), typeof(ComponentC) }
        // );


        return 0;
    }
}