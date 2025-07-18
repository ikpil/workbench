/*
 * Copyright 2012 The Netty Project
 *
 * The Netty Project licenses this file to you under the Apache License,
 * version 2.0 (the "License"); you may not use this file except in compliance
 * with the License. You may obtain a copy of the License at:
 *
 *   https://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations
 * under the License.
 */

namespace Netty.NET.Common.Util;

/**
 * An {@link IllegalStateException} which is raised when a user attempts to access a {@link ReferenceCounted} whose
 * reference count has been decreased to 0 (and consequently freed).
 */
public class IllegalReferenceCountException : InvalidOperationException
{
    private static readonly long serialVersionUID = -2507492394288153468L;

    public IllegalReferenceCountException()
    {
    }

    public IllegalReferenceCountException(int refCnt)
        : this("refCnt: " + refCnt)
    {
    }

    public IllegalReferenceCountException(int refCnt, int increment)
        : this("refCnt: " + refCnt + ", " + (increment > 0 ? "increment: " + increment : "decrement: " + -increment))
    {
    }

    public IllegalReferenceCountException(string message) : base(message)
    {
    }

    public IllegalReferenceCountException(string message, Exception cause) : base(message, cause)
    {
    }

    public IllegalReferenceCountException(Exception cause) : base(null, cause)
    {
    }
}