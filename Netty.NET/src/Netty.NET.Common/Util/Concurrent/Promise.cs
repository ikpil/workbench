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

/**
 * Special {@link Task} which is writable.
 */
public interface TaskCompletionSource<V> extends Task<V> {

    /**
     * Marks this future as a success and notifies all
     * listeners.
     *
     * If it is success or failed already it will throw an {@link IllegalStateException}.
     */
    TaskCompletionSource<V> setSuccess(V result);

    /**
     * Marks this future as a success and notifies all
     * listeners.
     *
     * @return {@code true} if and only if successfully marked this future as
     *         a success. Otherwise {@code false} because this future is
     *         already marked as either a success or a failure.
     */
    bool trySuccess(V result);

    /**
     * Marks this future as a failure and notifies all
     * listeners.
     *
     * If it is success or failed already it will throw an {@link IllegalStateException}.
     */
    TaskCompletionSource<V> setFailure(Exception cause);

    /**
     * Marks this future as a failure and notifies all
     * listeners.
     *
     * @return {@code true} if and only if successfully marked this future as
     *         a failure. Otherwise {@code false} because this future is
     *         already marked as either a success or a failure.
     */
    bool tryFailure(Exception cause);

    /**
     * Make this future impossible to cancel.
     *
     * @return {@code true} if and only if successfully marked this future as uncancellable or it is already done
     *         without being cancelled.  {@code false} if this future has been cancelled already.
     */
    bool setUncancellable();

    @Override
    TaskCompletionSource<V> addListener(GenericFutureListener<? extends Task<? super V>> listener);

    @Override
    TaskCompletionSource<V> addListeners(GenericFutureListener<? extends Task<? super V>>... listeners);

    @Override
    TaskCompletionSource<V> removeListener(GenericFutureListener<? extends Task<? super V>> listener);

    @Override
    TaskCompletionSource<V> removeListeners(GenericFutureListener<? extends Task<? super V>>... listeners);

    @Override
    TaskCompletionSource<V> await() throws InterruptedException;

    @Override
    TaskCompletionSource<V> awaitUninterruptibly();

    @Override
    TaskCompletionSource<V> sync() throws InterruptedException;

    @Override
    TaskCompletionSource<V> syncUninterruptibly();
}
