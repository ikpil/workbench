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

using Netty.NET.Common.Util.Internal.PromiseNotificationUtil;
using Netty.NET.Common.Util.Internal.logging.InternalLogger;
using Netty.NET.Common.Util.Internal.logging.InternalLoggerFactory;

using static Netty.NET.Common.Util.Internal.ObjectUtil.checkNotNull;
using static Netty.NET.Common.Util.Internal.ObjectUtil.checkNotNullWithIAE;

/**
 * {@link GenericFutureListener} implementation which takes other {@link TaskCompletionSource}s
 * and notifies them on completion.
 *
 * @param <V> the type of value returned by the future
 * @param <F> the type of future
 */
public class PromiseNotifier<V, F extends Task<V>> implements GenericFutureListener<F> {

    private static readonly InternalLogger logger = InternalLoggerFactory.getInstance(PromiseNotifier.class);
    private readonly TaskCompletionSource<? super V>[] promises;
    private readonly bool logNotifyFailure;

    /**
     * Create a new instance.
     *
     * @param promises  the {@link TaskCompletionSource}s to notify once this {@link GenericFutureListener} is notified.
     */
    @SafeVarargs
    public PromiseNotifier(TaskCompletionSource<? super V>... promises) {
        this(true, promises);
    }

    /**
     * Create a new instance.
     *
     * @param logNotifyFailure {@code true} if logging should be done in case notification fails.
     * @param promises  the {@link TaskCompletionSource}s to notify once this {@link GenericFutureListener} is notified.
     */
    @SafeVarargs
    public PromiseNotifier(bool logNotifyFailure, TaskCompletionSource<? super V>... promises) {
        checkNotNull(promises, "promises");
        for (TaskCompletionSource<? super V> promise: promises) {
            checkNotNullWithIAE(promise, "promise");
        }
        this.promises = promises.clone();
        this.logNotifyFailure = logNotifyFailure;
    }

    /**
     * Link the {@link Task} and {@link TaskCompletionSource} such that if the {@link Task} completes the {@link TaskCompletionSource}
     * will be notified. Cancellation is propagated both ways such that if the {@link Task} is cancelled
     * the {@link TaskCompletionSource} is cancelled and vise-versa.
     *
     * @param future    the {@link Task} which will be used to listen to for notifying the {@link TaskCompletionSource}.
     * @param promise   the {@link TaskCompletionSource} which will be notified
     * @param <V>       the type of the value.
     * @param <F>       the type of the {@link Task}
     * @return          the passed in {@link Task}
     */
    public static <V, F extends Task<V>> F cascade(final F future, final TaskCompletionSource<? super V> promise) {
        return cascade(true, future, promise);
    }

    /**
     * Link the {@link Task} and {@link TaskCompletionSource} such that if the {@link Task} completes the {@link TaskCompletionSource}
     * will be notified. Cancellation is propagated both ways such that if the {@link Task} is cancelled
     * the {@link TaskCompletionSource} is cancelled and vise-versa.
     *
     * @param logNotifyFailure  {@code true} if logging should be done in case notification fails.
     * @param future            the {@link Task} which will be used to listen to for notifying the {@link TaskCompletionSource}.
     * @param promise           the {@link TaskCompletionSource} which will be notified
     * @param <V>               the type of the value.
     * @param <F>               the type of the {@link Task}
     * @return                  the passed in {@link Task}
     */
    @SuppressWarnings({"unchecked", "rawtypes"})
    public static <V, F extends Task<V>> F cascade(bool logNotifyFailure, final F future,
                                                     final TaskCompletionSource<? super V> promise) {
        promise.addListener(new FutureListener() {
            @Override
            public void operationComplete(Task f) {
                if (f.isCancelled()) {
                    future.cancel(false);
                }
            }
        });
        future.addListener(new PromiseNotifier(logNotifyFailure, promise) {
            @Override
            public void operationComplete(Task f) throws Exception {
                if (promise.isCancelled() && f.isCancelled()) {
                    // Just return if we propagate a cancel from the promise to the future and both are notified already
                    return;
                }
                super.operationComplete(future);
            }
        });
        return future;
    }

    @Override
    public void operationComplete(F future) throws Exception {
        InternalLogger internalLogger = logNotifyFailure ? logger : null;
        if (future.isSuccess()) {
            V result = future.get();
            for (TaskCompletionSource<? super V> p: promises) {
                PromiseNotificationUtil.trySuccess(p, result, internalLogger);
            }
        } else if (future.isCancelled()) {
            for (TaskCompletionSource<? super V> p: promises) {
                PromiseNotificationUtil.tryCancel(p, internalLogger);
            }
        } else {
            Exception cause = future.cause();
            for (TaskCompletionSource<? super V> p: promises) {
                PromiseNotificationUtil.tryFailure(p, cause, internalLogger);
            }
        }
    }
}
