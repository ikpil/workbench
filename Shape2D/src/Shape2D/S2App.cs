using System;
using System.Drawing;
using Silk.NET.Input;
using Silk.NET.Maths;
using Silk.NET.OpenGL;
using Silk.NET.OpenGL.Extensions.ImGui;
using Silk.NET.Windowing;

namespace Shape2D;

public class S2App : IDisposable
{
    private IWindow _window;
    private GL _gl;
    private IInputContext _input;
    private ImGuiController _controller;
    private bool _isClosing;
    
    public void Dispose()
    {
        // _controller?.Dispose();
        // _controller = null;
        
        _input?.Dispose();
        _input = null;
        
        _gl?.Dispose();
        _gl = null;
        
        // ..
        _window?.Dispose();
        _window = null;
    }

    public void Initialize(string[] args)
    {
        
    }

    public void Run()
    {
        Window.PrioritizeSdl();
        
        var options = WindowOptions.Default;
        options.ShouldSwapAutomatically = false;
        
        _window = Window.Create(options);
        _window.Closing += OnClosing;
        _window.Load += OnLoad;
        _window.Update += OnUpdate;
        _window.Render += OnRender;
        _window.FramebufferResize += OnFrameBufferResize;
        
        _window.Run();
    }

    private void OnClosing()
    {
        _isClosing = true;
    }

    private void OnLoad()
    {
        _gl = _window.CreateOpenGL(); // load OpenGL
        _input = _window.CreateInput(); // create an input context
        
        _controller = new ImGuiController(
            _gl,
            _window, // pass in our window
            _input
        ); 
    }

    private void OnUpdate(double dt)
    {
        if (_isClosing)
            return;
        
        // Make sure ImGui is up-to-date
        _controller.Update((float) dt);

    }

    private void OnRender(double dt)
    {
        if (_isClosing)
            return;
        
        _gl.ClearColor(Color.FromArgb(255, (int) (.45f * 255), (int) (.55f * 255), (int) (.60f * 255)));
        _gl.Clear(ClearBufferMask.ColorBufferBit | ClearBufferMask.DepthBufferBit);

        ImGuiNET.ImGui.ShowDemoWindow();

        _controller.Render(); 
        _window.SwapBuffers();
    }

    private void OnFrameBufferResize(Vector2D<int> size)
    {
        _gl.Viewport(size);        
    }
}