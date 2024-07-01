using System;
using System.Diagnostics;
using Microsoft.Diagnostics.Tracing;
using Microsoft.Diagnostics.Tracing.Analysis;
using Microsoft.Diagnostics.Tracing.Parsers;
using Microsoft.Diagnostics.Tracing.Parsers.Clr;
using Microsoft.Diagnostics.Tracing.Session;
using Test;

class Program
{
    static void Main(string[] args)
    {
        int processId = Process.GetCurrentProcess().Id;
        Console.WriteLine(processId);

        var t = new Thread(() =>
        {
            //
            ObserveGCEvents.Run(processId);
        });
        t.IsBackground = true;
        t.Start();

        while (true)
        {
            Thread.Sleep(1000);
            var s = new byte[Random.Shared.Next(1, 1024 * 1024 * 1024)];
        }
    }
}