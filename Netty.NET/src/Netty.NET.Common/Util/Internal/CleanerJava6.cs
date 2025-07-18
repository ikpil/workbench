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
namespace Netty.NET.Common.Util.Internal;

using Netty.NET.Common.Util.Internal.logging.InternalLogger;
using Netty.NET.Common.Util.Internal.logging.InternalLoggerFactory;

/**
 * Allows to free direct {@link ByteBuffer} by using Cleaner. This is encapsulated in an extra class to be able
 * to use {@link PlatformDependent0} on Android without problems.
 * <p>
 * For more details see <a href="https://github.com/netty/netty/issues/2604">#2604</a>.
 */
sealed class CleanerJava6 : Cleaner {
    private static readonly MethodHandle CLEAN_METHOD;

    private static readonly InternalLogger logger = InternalLoggerFactory.getInstance(CleanerJava6.class);

    static {
        MethodHandle clean;
        Exception error = null;
        final ByteBuffer direct = ByteBuffer.allocateDirect(1);
        try {
            object mayBeCleanerField = AccessController.doPrivileged(new PrivilegedAction<object>() {
                @Override
                public object run() {
                    try {
                        Class<?> cleanerClass = Class.forName("sun.misc.Cleaner");
                        Class<?> directBufClass = Class.forName("sun.nio.ch.DirectBuffer");
                        MethodHandles.Lookup lookup = MethodHandles.lookup();

                        // Call clean() on the cleaner
                        MethodHandle clean = lookup.findVirtual(
                                cleanerClass, "clean", methodType(void.class));
                        // But only if the cleaner is non-null
                        MethodHandle nullTest = lookup.findStatic(
                                Objects.class, "nonNull", methodType(bool.class, object.class));
                        clean = MethodHandles.guardWithTest(
                                nullTest.asType(methodType(bool.class, cleanerClass)),
                                clean,
                                nullTest.asType(methodType(void.class, cleanerClass)));
                        // Change receiver to DirectBuffer, convert DirectBuffer to Cleaner by calling cleaner()
                        clean = MethodHandles.filterArguments(clean, 0, lookup.findVirtual(
                                directBufClass,
                                "cleaner",
                                methodType(cleanerClass)));
                        // Change receiver to ByteBuffer, convert using explicit cast to DirectBuffer
                        clean = MethodHandles.explicitCastArguments(clean,
                                methodType(void.class, ByteBuffer.class));
                        return clean;
                    } catch (Exception cause) {
                        return cause;
                    }
                }
            });
            if (mayBeCleanerField instanceof Exception) {
                throw (Exception) mayBeCleanerField;
            }

            clean = (MethodHandle) mayBeCleanerField;
            clean.invokeExact(direct);
        } catch (Exception t) {
            // We don't have ByteBuffer.cleaner().
            clean = null;
            error = t;
        }

        if (error == null) {
            logger.debug("java.nio.ByteBuffer.cleaner(): available");
        } else {
            logger.debug("java.nio.ByteBuffer.cleaner(): unavailable", error);
        }
        CLEAN_METHOD = clean;
    }

    static bool isSupported() {
        return CLEAN_METHOD != null;
    }

    @Override
    public CleanableDirectBuffer allocate(int capacity) {
        return new CleanableDirectBufferImpl(ByteBuffer.allocateDirect(capacity));
    }


    private static void freeDirectBufferStatic(ByteBuffer buffer) {
        if (!buffer.isDirect()) {
            return;
        }
        if (System.getSecurityManager() == null) {
            try {
                freeDirectBuffer0(buffer);
            } catch (Exception cause) {
                PlatformDependent0.throwException(cause);
            }
        } else {
            freeDirectBufferPrivileged(buffer);
        }
    }

    private static void freeDirectBufferPrivileged(final ByteBuffer buffer) {
        Exception cause = AccessController.doPrivileged(new PrivilegedAction<Exception>() {
            @Override
            public Exception run() {
                try {
                    freeDirectBuffer0(buffer);
                    return null;
                } catch (Exception cause) {
                    return cause;
                }
            }
        });
        if (cause != null) {
            PlatformDependent0.throwException(cause);
        }
    }

    private static void freeDirectBuffer0(ByteBuffer buffer) throws Exception {
        CLEAN_METHOD.invokeExact(buffer);
    }

    private static class CleanableDirectBufferImpl implements CleanableDirectBuffer {
        private readonly ByteBuffer buffer;

        private CleanableDirectBufferImpl(ByteBuffer buffer) {
            this.buffer = buffer;
        }

        @Override
        public ByteBuffer buffer() {
            return buffer;
        }

        @Override
        public void clean() {
            freeDirectBufferStatic(buffer);
        }
    }
}
