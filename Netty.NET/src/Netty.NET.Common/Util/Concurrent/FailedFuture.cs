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

using Netty.NET.Common.Util.Internal.ObjectUtil;
using Netty.NET.Common.Util.Internal.PlatformDependent;

/**
 * The {@link CompleteFuture} which is failed already.  It is
 * recommended to use {@link EventExecutor#newFailedFuture(Exception)}
 * instead of calling the constructor of this future.
 */
public sealed class FailedFuture<V> extends CompleteFuture<V> {

    private readonly Exception cause;

    /**
     * Creates a new instance.
     *
     * @param executor the {@link EventExecutor} associated with this future
     * @param cause   the cause of failure
     */
    public FailedFuture(EventExecutor executor, Exception cause) {
        super(executor);
        this.cause = ObjectUtil.checkNotNull(cause, "cause");
    }

    @Override
    public Exception cause() {
        return cause;
    }

    @Override
    public bool isSuccess() {
        return false;
    }

    @Override
    public Task<V> sync() {
        PlatformDependent.throwException(cause);
        return this;
    }

    @Override
    public Task<V> syncUninterruptibly() {
        PlatformDependent.throwException(cause);
        return this;
    }

    @Override
    public V getNow() {
        return null;
    }
}
