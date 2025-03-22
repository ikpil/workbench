using System;
using Shape2D;
using Silk.NET.OpenGL;
using Silk.NET.Windowing;

public class Program
{
    public static int Main(string[] args)
    {
        using var app = new S2App();
        app.Initialize(args);
        app.Run();
        
        return 0;
    }
}