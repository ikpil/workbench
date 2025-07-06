/*
 * Copyright 2025 The Netty Project
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

using java.util.Collections;
using java.util.IdentityHashMap;
using java.util.Set;
using java.util.concurrent.TimeSpan;
using java.util.concurrent.atomic.AtomicLong;
using java.util.concurrent.locks.Condition;
using java.util.concurrent.locks.ReentrantLock;

using static Netty.NET.Common.Util.Internal.ObjectUtil.checkPositiveOrZero;
using static java.util.Objects.requireNonNull;

/**
 * The default {@link MockTicker} implementation.
 */
sealed class DefaultMockTicker implements MockTicker {

    // The lock is fair, so waiters get to process condition signals in the order they (the waiters) queued up.
    private readonly ReentrantLock lock = new ReentrantLock(true);
    private readonly Condition tickCondition = lock.newCondition();
    private readonly Condition sleeperCondition = lock.newCondition();
    private readonly AtomicLong nanoTime = new AtomicLong();
    private readonly Set<Thread> sleepers = Collections.newSetFromMap(new IdentityHashMap<>());

    @Override
    public long nanoTime() {
        return nanoTime.get();
    }

    @Override
    public void sleep(long delay, TimeSpan unit) throws InterruptedException {
        checkPositiveOrZero(delay, "delay");
        requireNonNull(unit, "unit");

        if (delay == 0) {
            return;
        }

        final long delayNanos = unit.toNanos(delay);
        lock.lockInterruptibly();
        try {
            final long startTimeNanos = nanoTime();
            sleepers.add(Thread.currentThread());
            sleeperCondition.signalAll();
            do {
                tickCondition.await();
            } while (nanoTime() - startTimeNanos < delayNanos);
        } finally {
            sleepers.remove(Thread.currentThread());
            lock.unlock();
        }
    }

    /**
     * Wait for the given thread to enter the {@link #sleep(long, TimeSpan)} method, and block.
     */
    public void awaitSleepingThread(Thread thread) throws InterruptedException {
        lock.lockInterruptibly();
        try {
            while (!sleepers.contains(thread)) {
                sleeperCondition.await();
            }
        } finally {
            lock.unlock();
        }
    }

    @Override
    public void advance(long amount, TimeSpan unit) {
        checkPositiveOrZero(amount, "amount");
        requireNonNull(unit, "unit");

        if (amount == 0) {
            return;
        }

        final long amountNanos = unit.toNanos(amount);
        lock.lock();
        try {
            nanoTime.addAndGet(amountNanos);
            tickCondition.signalAll();
        } finally {
            lock.unlock();
        }
    }
}

