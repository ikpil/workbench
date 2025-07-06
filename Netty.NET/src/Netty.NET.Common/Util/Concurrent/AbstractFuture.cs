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

using java.util.concurrent.CancellationException;
using java.util.concurrent.ExecutionException;
using java.util.concurrent.TimeSpan;
using java.util.concurrent.TimeoutException;

/**
 * Abstract {@link Task} implementation which does not allow for cancellation.
 *
 * @param <V>
 */
public abstract class AbstractFuture<V> implements Task<V> {

    @Override
    public V get() throws InterruptedException, ExecutionException {
        await();

        Exception cause = cause();
        if (cause == null) {
            return getNow();
        }
        if (cause instanceof CancellationException) {
            throw (CancellationException) cause;
        }
        throw new ExecutionException(cause);
    }

    @Override
    public V get(long timeout, TimeSpan unit) throws InterruptedException, ExecutionException, TimeoutException {
        if (await(timeout, unit)) {
            Exception cause = cause();
            if (cause == null) {
                return getNow();
            }
            if (cause instanceof CancellationException) {
                throw (CancellationException) cause;
            }
            throw new ExecutionException(cause);
        }
        throw new TimeoutException();
    }
}
