// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

using System;
using NUnit.Framework;

namespace Box2D.NET.Engine.Test;

public class test_macros
{
    public static int RUN_SUBTEST(Func<int> T)
    {
        do
        {
            int result = T();
            if (result == 1)
            {
                Console.Write("  subtest failed: " + T.Method + "\n");
                return 1;
            }
            else
            {
                Console.Write("  subtest passed: " + T.Method + "\n");
            }
        } while (false);

        return 0;
    }

    public static int ENSURE(bool C)
    {
        do
        {
            if ((C) == false)
            {
                Console.Write("condition false: " + nameof(C) + "\n");
                Assert.That(false);
                return 1;
            }
        } while (false);

        return 0;
    }

    public static int ENSURE_SMALL(float C, float tol)
    {
        do
        {
            if ((C) < -(tol) || (tol) < (C))
            {
                Assert.That(false, $"condition false: abs({C}) < {tol}");
                return 1;
            }
        } while (false);

        return 0;
    }

    public static int ARRAY_COUNT<T>(T[] A)
    {
        return A.Length;
    }

    /// Used to prevent the compiler from warning about unused variables
    public static void MAYBE_UNUSED<T>(T x)
    {
        // ((void)(x))
    }
}