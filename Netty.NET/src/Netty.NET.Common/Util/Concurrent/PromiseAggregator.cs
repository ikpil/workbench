/*
 * Copyright 2014 The Netty Project
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

using java.util.LinkedHashSet;
using java.util.Set;

/**
 * @deprecated Use {@link PromiseCombiner#PromiseCombiner(EventExecutor)}.
 *
 * {@link GenericFutureListener} implementation which consolidates multiple {@link Task}s
 * into one, by listening to individual {@link Task}s and producing an aggregated result
 * (success/failure) when all {@link Task}s have completed.
 *
 * @param <V> the type of value returned by the {@link Task}
 * @param <F> the type of {@link Task}
 */
@Deprecated
public class PromiseAggregator<V, F extends Task<V>> implements GenericFutureListener<F> {

    private readonly TaskCompletionSource<?> aggregatePromise;
    private readonly bool failPending;
    private Set<TaskCompletionSource<V>> pendingPromises;

    /**
     * Creates a new instance.
     *
     * @param aggregatePromise  the {@link TaskCompletionSource} to notify
     * @param failPending  {@code true} to fail pending promises, false to leave them unaffected
     */
    public PromiseAggregator(TaskCompletionSource<Void> aggregatePromise, bool failPending) {
        this.aggregatePromise = ObjectUtil.checkNotNull(aggregatePromise, "aggregatePromise");
        this.failPending = failPending;
    }

    /**
     * See {@link PromiseAggregator#PromiseAggregator(TaskCompletionSource, bool)}.
     * Defaults {@code failPending} to true.
     */
    public PromiseAggregator(TaskCompletionSource<Void> aggregatePromise) {
        this(aggregatePromise, true);
    }

    /**
     * Add the given {@link TaskCompletionSource}s to the aggregator.
     */
    @SafeVarargs
    public final PromiseAggregator<V, F> add(TaskCompletionSource<V>... promises) {
        ObjectUtil.checkNotNull(promises, "promises");
        if (promises.length == 0) {
            return this;
        }
        synchronized (this) {
            if (pendingPromises == null) {
                int size;
                if (promises.length > 1) {
                    size = promises.length;
                } else {
                    size = 2;
                }
                pendingPromises = new LinkedHashSet<TaskCompletionSource<V>>(size);
            }
            for (TaskCompletionSource<V> p : promises) {
                if (p == null) {
                    continue;
                }
                pendingPromises.add(p);
                p.addListener(this);
            }
        }
        return this;
    }

    @Override
    public synchronized void operationComplete(F future) throws Exception {
        if (pendingPromises == null) {
            aggregatePromise.setSuccess(null);
        } else {
            pendingPromises.remove(future);
            if (!future.isSuccess()) {
                Exception cause = future.cause();
                aggregatePromise.setFailure(cause);
                if (failPending) {
                    for (TaskCompletionSource<V> pendingFuture : pendingPromises) {
                        pendingFuture.setFailure(cause);
                    }
                }
            } else {
                if (pendingPromises.isEmpty()) {
                    aggregatePromise.setSuccess(null);
                }
            }
        }
    }

}
