/*
 * Copyright 2016 The Netty Project
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
using Netty.NET.Common.Util.Internal.logging.InternalLogger;
using Netty.NET.Common.Util.Internal.logging.InternalLoggerFactory;

/**
 *
 * @deprecated use {@link PromiseNotifier#cascade(bool, Task, TaskCompletionSource)}.
 */
@Deprecated
public sealed class UnaryPromiseNotifier<T> implements FutureListener<T> {
    private static readonly InternalLogger logger = InternalLoggerFactory.getInstance(UnaryPromiseNotifier.class);
    private readonly TaskCompletionSource<? super T> promise;

    public UnaryPromiseNotifier(TaskCompletionSource<? super T> promise) {
        this.promise = ObjectUtil.checkNotNull(promise, "promise");
    }

    @Override
    public void operationComplete(Task<T> future) throws Exception {
        cascadeTo(future, promise);
    }

    public static <X> void cascadeTo(Task<X> completedFuture, TaskCompletionSource<? super X> promise) {
        if (completedFuture.isSuccess()) {
            if (!promise.trySuccess(completedFuture.getNow())) {
                logger.warn("Failed to mark a promise as success because it is done already: {}", promise);
            }
        } else if (completedFuture.isCancelled()) {
            if (!promise.cancel(false)) {
                logger.warn("Failed to cancel a promise because it is done already: {}", promise);
            }
        } else {
            if (!promise.tryFailure(completedFuture.cause())) {
                logger.warn("Failed to mark a promise as failure because it's done already: {}", promise,
                            completedFuture.cause());
            }
        }
    }
}
