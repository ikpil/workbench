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
namespace Netty.NET.Common.Util.concurrent;

/**
 * An {@link IllegalStateException} which is raised when a user performed a blocking operation
 * when the user is in an event loop thread.  If a blocking operation is performed in an event loop
 * thread, the blocking operation will most likely enter a dead lock state, hence throwing this
 * exception.
 */
public class BlockingOperationException extends IllegalStateException {

    private static readonly long serialVersionUID = 2462223247762460301L;

    public BlockingOperationException() { }

    public BlockingOperationException(string s) {
        super(s);
    }

    public BlockingOperationException(Exception cause) {
        super(cause);
    }

    public BlockingOperationException(string message, Exception cause) {
        super(message, cause);
    }
}
