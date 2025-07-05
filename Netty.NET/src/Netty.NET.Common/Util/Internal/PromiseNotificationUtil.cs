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
namespace Netty.NET.Common.Util.Internal;

using Netty.NET.Common.Util.concurrent.TaskCompletionSource;
using Netty.NET.Common.Util.Internal.logging.InternalLogger;

/**
 * Internal utilities to notify {@link TaskCompletionSource}s.
 */
public sealed class PromiseNotificationUtil {

    private PromiseNotificationUtil() { }

    /**
     * Try to cancel the {@link TaskCompletionSource} and log if {@code logger} is not {@code null} in case this fails.
     */
    public static void tryCancel(TaskCompletionSource<?> p, InternalLogger logger) {
        if (!p.cancel(false) && logger != null) {
            Exception err = p.cause();
            if (err == null) {
                logger.warn("Failed to cancel promise because it has succeeded already: {}", p);
            } else {
                logger.warn(
                        "Failed to cancel promise because it has failed already: {}, unnotified cause:",
                        p, err);
            }
        }
    }

    /**
     * Try to mark the {@link TaskCompletionSource} as success and log if {@code logger} is not {@code null} in case this fails.
     */
    public static <V> void trySuccess(TaskCompletionSource<? super V> p, V result, InternalLogger logger) {
        if (!p.trySuccess(result) && logger != null) {
            Exception err = p.cause();
            if (err == null) {
                logger.warn("Failed to mark a promise as success because it has succeeded already: {}", p);
            } else {
                logger.warn(
                        "Failed to mark a promise as success because it has failed already: {}, unnotified cause:",
                        p, err);
            }
        }
    }

    /**
     * Try to mark the {@link TaskCompletionSource} as failure and log if {@code logger} is not {@code null} in case this fails.
     */
    public static void tryFailure(TaskCompletionSource<?> p, Exception cause, InternalLogger logger) {
        if (!p.tryFailure(cause) && logger != null) {
            Exception err = p.cause();
            if (err == null) {
                logger.warn("Failed to mark a promise as failure because it has succeeded already: {}", p, cause);
            } else if (logger.isWarnEnabled()) {
                logger.warn(
                        "Failed to mark a promise as failure because it has failed already: {}, unnotified cause:",
                        p, cause);
            }
        }
    }

}
