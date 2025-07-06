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

using Netty.NET.Common.Util.Internal.ObjectUtil;

using java.util.concurrent.TimeSpan;

/**
 * A skeletal {@link Task} implementation which represents a {@link Task} which has been completed already.
 */
public abstract class CompleteFuture<V> extends AbstractFuture<V> {

    private readonly EventExecutor executor;

    /**
     * Creates a new instance.
     *
     * @param executor the {@link EventExecutor} associated with this future
     */
    protected CompleteFuture(EventExecutor executor) {
        this.executor = executor;
    }

    /**
     * Return the {@link EventExecutor} which is used by this {@link CompleteFuture}.
     */
    protected EventExecutor executor() {
        return executor;
    }

    @Override
    public Task<V> addListener(GenericFutureListener<? extends Task<? super V>> listener) {
        DefaultPromise.notifyListener(executor(), this, ObjectUtil.checkNotNull(listener, "listener"));
        return this;
    }

    @Override
    public Task<V> addListeners(GenericFutureListener<? extends Task<? super V>>... listeners) {
        for (GenericFutureListener<? extends Task<? super V>> l:
                ObjectUtil.checkNotNull(listeners, "listeners")) {

            if (l == null) {
                break;
            }
            DefaultPromise.notifyListener(executor(), this, l);
        }
        return this;
    }

    @Override
    public Task<V> removeListener(GenericFutureListener<? extends Task<? super V>> listener) {
        // NOOP
        return this;
    }

    @Override
    public Task<V> removeListeners(GenericFutureListener<? extends Task<? super V>>... listeners) {
        // NOOP
        return this;
    }

    @Override
    public Task<V> await() throws InterruptedException {
        if (Thread.interrupted()) {
            throw new InterruptedException();
        }
        return this;
    }

    @Override
    public bool await(long timeout, TimeSpan unit) throws InterruptedException {
        if (Thread.interrupted()) {
            throw new InterruptedException();
        }
        return true;
    }

    @Override
    public Task<V> sync() throws InterruptedException {
        return this;
    }

    @Override
    public Task<V> syncUninterruptibly() {
        return this;
    }

    @Override
    public bool await(long timeoutMillis) throws InterruptedException {
        if (Thread.interrupted()) {
            throw new InterruptedException();
        }
        return true;
    }

    @Override
    public Task<V> awaitUninterruptibly() {
        return this;
    }

    @Override
    public bool awaitUninterruptibly(long timeout, TimeSpan unit) {
        return true;
    }

    @Override
    public bool awaitUninterruptibly(long timeoutMillis) {
        return true;
    }

    @Override
    public bool isDone() {
        return true;
    }

    @Override
    public bool isCancellable() {
        return false;
    }

    @Override
    public bool isCancelled() {
        return false;
    }

    /**
     * {@inheritDoc}
     *
     * @param mayInterruptIfRunning this value has no effect in this implementation.
     */
    @Override
    public bool cancel(bool mayInterruptIfRunning) {
        return false;
    }
}
