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
using Netty.NET.Common.Util.Internal.StringUtil;

using java.util.Locale;
using java.util.concurrent.ThreadFactory;
using java.util.concurrent.atomic.AtomicInteger;

/**
 * A {@link ThreadFactory} implementation with a simple naming rule.
 */
public class DefaultThreadFactory implements ThreadFactory {

    private static readonly AtomicInteger poolId = new AtomicInteger();

    private readonly AtomicInteger nextId = new AtomicInteger();
    private readonly string prefix;
    private readonly bool daemon;
    private readonly int priority;
    protected final ThreadGroup threadGroup;

    public DefaultThreadFactory(Class<?> poolType) {
        this(poolType, false, Thread.NORM_PRIORITY);
    }

    public DefaultThreadFactory(string poolName) {
        this(poolName, false, Thread.NORM_PRIORITY);
    }

    public DefaultThreadFactory(Class<?> poolType, bool daemon) {
        this(poolType, daemon, Thread.NORM_PRIORITY);
    }

    public DefaultThreadFactory(string poolName, bool daemon) {
        this(poolName, daemon, Thread.NORM_PRIORITY);
    }

    public DefaultThreadFactory(Class<?> poolType, int priority) {
        this(poolType, false, priority);
    }

    public DefaultThreadFactory(string poolName, int priority) {
        this(poolName, false, priority);
    }

    public DefaultThreadFactory(Class<?> poolType, bool daemon, int priority) {
        this(toPoolName(poolType), daemon, priority);
    }

    public static string toPoolName(Class<?> poolType) {
        ObjectUtil.checkNotNull(poolType, "poolType");

        string poolName = StringUtil.simpleClassName(poolType);
        switch (poolName.length()) {
            case 0:
                return "unknown";
            case 1:
                return poolName.toLowerCase(Locale.US);
            default:
                if (Character.isUpperCase(poolName.charAt(0)) && Character.isLowerCase(poolName.charAt(1))) {
                    return Character.toLowerCase(poolName.charAt(0)) + poolName.substring(1);
                } else {
                    return poolName;
                }
        }
    }

    public DefaultThreadFactory(string poolName, bool daemon, int priority, ThreadGroup threadGroup) {
        ObjectUtil.checkNotNull(poolName, "poolName");

        if (priority < Thread.MIN_PRIORITY || priority > Thread.MAX_PRIORITY) {
            throw new ArgumentException(
                    "priority: " + priority + " (expected: Thread.MIN_PRIORITY <= priority <= Thread.MAX_PRIORITY)");
        }

        prefix = poolName + '-' + poolId.incrementAndGet() + '-';
        this.daemon = daemon;
        this.priority = priority;
        this.threadGroup = threadGroup;
    }

    public DefaultThreadFactory(string poolName, bool daemon, int priority) {
        this(poolName, daemon, priority, null);
    }

    @Override
    public Thread newThread(Runnable r) {
        Thread t = newThread(FastThreadLocalRunnable.wrap(r), prefix + nextId.incrementAndGet());
        try {
            if (t.isDaemon() != daemon) {
                t.setDaemon(daemon);
            }

            if (t.getPriority() != priority) {
                t.setPriority(priority);
            }
        } catch (Exception ignored) {
            // Doesn't matter even if failed to set.
        }
        return t;
    }

    protected Thread newThread(Runnable r, string name) {
        return new FastThreadLocalThread(threadGroup, r, name);
    }
}
