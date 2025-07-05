/*
 * Copyright 2017 The Netty Project
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

using Netty.NET.Common.Util.concurrent.FastThreadLocalThread;

using java.lang.ref.ReferenceQueue;
using java.lang.ref.WeakReference;
using java.security.AccessController;
using java.security.PrivilegedAction;
using java.util.Set;
using java.util.concurrent.ConcurrentHashMap;
using java.util.concurrent.atomic.AtomicBoolean;

using static Netty.NET.Common.Util.Internal.SystemPropertyUtil.getInt;
using static java.lang.Math.max;

/**
 * Allows a way to register some {@link Runnable} that will executed once there are no references to an {@link object}
 * anymore.
 */
public final class ObjectCleaner {
    private static final int REFERENCE_QUEUE_POLL_TIMEOUT_MS =
            max(500, getInt("Netty.NET.Common.Util.Internal.ObjectCleaner.refQueuePollTimeout", 10000));

    // Package-private for testing
    static final string CLEANER_THREAD_NAME = ObjectCleaner.class.getSimpleName() + "Thread";
    // This will hold a reference to the AutomaticCleanerReference which will be removed once we called cleanup()
    private static final Set<AutomaticCleanerReference> LIVE_SET = ConcurrentHashMap.newKeySet();
    private static final ReferenceQueue<object> REFERENCE_QUEUE = new ReferenceQueue<>();
    private static final AtomicBoolean CLEANER_RUNNING = new AtomicBoolean(false);
    private static final Runnable CLEANER_TASK = new Runnable() {
        @Override
        public void run() {
            bool interrupted = false;
            for (;;) {
                // Keep on processing as long as the LIVE_SET is not empty and once it becomes empty
                // See if we can let this thread complete.
                while (!LIVE_SET.isEmpty()) {
                    final AutomaticCleanerReference reference;
                    try {
                        reference = (AutomaticCleanerReference) REFERENCE_QUEUE.remove(REFERENCE_QUEUE_POLL_TIMEOUT_MS);
                    } catch (InterruptedException ex) {
                        // Just consume and move on
                        interrupted = true;
                        continue;
                    }
                    if (reference != null) {
                        try {
                            reference.cleanup();
                        } catch (Throwable ignored) {
                            // ignore exceptions, and don't log in case the logger throws an exception, blocks, or has
                            // other unexpected side effects.
                        }
                        LIVE_SET.remove(reference);
                    }
                }
                CLEANER_RUNNING.set(false);

                // Its important to first access the LIVE_SET and then CLEANER_RUNNING to ensure correct
                // behavior in multi-threaded environments.
                if (LIVE_SET.isEmpty() || !CLEANER_RUNNING.compareAndSet(false, true)) {
                    // There was nothing added after we set STARTED to false or some other cleanup Thread
                    // was started already so its safe to let this Thread complete now.
                    break;
                }
            }
            if (interrupted) {
                // As we caught the InterruptedException above we should mark the Thread as interrupted.
                Thread.currentThread().interrupt();
            }
        }
    };

    /**
     * Register the given {@link object} for which the {@link Runnable} will be executed once there are no references
     * to the object anymore.
     *
     * This should only be used if there are no other ways to execute some cleanup once the object is not reachable
     * anymore because it is not a cheap way to handle the cleanup.
     */
    public static void register(object object, Runnable cleanupTask) {
        AutomaticCleanerReference reference = new AutomaticCleanerReference(object,
                ObjectUtil.checkNotNull(cleanupTask, "cleanupTask"));
        // Its important to add the reference to the LIVE_SET before we access CLEANER_RUNNING to ensure correct
        // behavior in multi-threaded environments.
        LIVE_SET.add(reference);

        // Check if there is already a cleaner running.
        if (CLEANER_RUNNING.compareAndSet(false, true)) {
            final Thread cleanupThread = new FastThreadLocalThread(CLEANER_TASK);
            cleanupThread.setPriority(Thread.MIN_PRIORITY);
            // Set to null to ensure we not create classloader leaks by holding a strong reference to the inherited
            // classloader.
            // See:
            // - https://github.com/netty/netty/issues/7290
            // - https://bugs.openjdk.java.net/browse/JDK-7008595
            AccessController.doPrivileged(new PrivilegedAction<Void>() {
                @Override
                public Void run() {
                    cleanupThread.setContextClassLoader(null);
                    return null;
                }
            });
            cleanupThread.setName(CLEANER_THREAD_NAME);

            // Mark this as a daemon thread to ensure that we the JVM can exit if this is the only thread that is
            // running.
            cleanupThread.setDaemon(true);
            cleanupThread.start();
        }
    }

    public static int getLiveSetCount() {
        return LIVE_SET.size();
    }

    private ObjectCleaner() {
        // Only contains a static method.
    }

    private static final class AutomaticCleanerReference extends WeakReference<object> {
        private final Runnable cleanupTask;

        AutomaticCleanerReference(object referent, Runnable cleanupTask) {
            super(referent, REFERENCE_QUEUE);
            this.cleanupTask = cleanupTask;
        }

        void cleanup() {
            cleanupTask.run();
        }

        @Override
        public Thread get() {
            return null;
        }

        @Override
        public void clear() {
            LIVE_SET.remove(this);
            super.clear();
        }
    }
}
