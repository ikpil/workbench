/*
 * Copyright 2013 The Netty Project
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

using java.util.EventListener;

/**
 * Listens to the result of a {@link Task}.  The result of the asynchronous operation is notified once this listener
 * is added by calling {@link Task#addListener(GenericFutureListener)}.
 */
public interface GenericFutureListener<F extends Task<?>> extends EventListener {

    /**
     * Invoked when the operation associated with the {@link Task} has been completed.
     *
     * @param future  the source {@link Task} which called this callback
     */
    void operationComplete(F future) throws Exception;
}
