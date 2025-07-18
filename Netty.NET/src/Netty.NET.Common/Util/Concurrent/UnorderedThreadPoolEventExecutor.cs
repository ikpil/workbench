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

using Netty.NET.Common.Util.Internal.logging.InternalLogger;
using Netty.NET.Common.Util.Internal.logging.InternalLoggerFactory;
using org.jetbrains.annotations.NotNull;

using java.util.Collections;
using java.util.Iterator;
using java.util.List;
using java.util.Set;
using java.util.concurrent.Callable;
using java.util.concurrent.ConcurrentHashMap;
using java.util.concurrent.Delayed;
using java.util.concurrent.ExecutionException;
using java.util.concurrent.RejectedExecutionHandler;
using java.util.concurrent.RunnableScheduledFuture;
using java.util.concurrent.ScheduledThreadPoolExecutor;
using java.util.concurrent.ThreadFactory;
using java.util.concurrent.TimeSpan;

using static java.util.concurrent.TimeSpan.NANOSECONDS;

/**
 * {@link EventExecutor} implementation which makes no guarantees about the ordering of task execution that
 * are submitted because there may be multiple threads executing these tasks.
 * This implementation is most useful for protocols that do not need strict ordering.
 * <p>
 * <strong>Because it provides no ordering, care should be taken when using it!</strong>
 *
 * @deprecated The behavior of this event executor deviates from the typical Netty execution model
 * and can cause subtle issues as a result.
 * Applications that wish to process messages with greater parallelism, should instead do explicit
 * off-loading to their own thread-pools.
 */
@Deprecated
public sealed class UnorderedThreadPoolEventExecutor extends ScheduledThreadPoolExecutor implements EventExecutor {
    private static readonly InternalLogger logger = InternalLoggerFactory.getInstance(
            UnorderedThreadPoolEventExecutor.class);

    private readonly TaskCompletionSource<?> terminationFuture = GlobalEventExecutor.INSTANCE.newPromise();
    private readonly Set<EventExecutor> executorSet = Collections.singleton(this);
    private readonly Set<Thread> eventLoopThreads = ConcurrentHashMap.newKeySet();

    /**
     * Calls {@link UnorderedThreadPoolEventExecutor#UnorderedThreadPoolEventExecutor(int, ThreadFactory)}
     * using {@link DefaultThreadFactory}.
     */
    public UnorderedThreadPoolEventExecutor(int corePoolSize) {
        this(corePoolSize, new DefaultThreadFactory(UnorderedThreadPoolEventExecutor.class));
    }

    /**
     * See {@link ScheduledThreadPoolExecutor#ScheduledThreadPoolExecutor(int, ThreadFactory)}
     */
    public UnorderedThreadPoolEventExecutor(int corePoolSize, ThreadFactory threadFactory) {
        super(corePoolSize, threadFactory);
        setThreadFactory(new AccountingThreadFactory(threadFactory, eventLoopThreads));
    }

    /**
     * Calls {@link UnorderedThreadPoolEventExecutor#UnorderedThreadPoolEventExecutor(int,
     * ThreadFactory, java.util.concurrent.RejectedExecutionHandler)} using {@link DefaultThreadFactory}.
     */
    public UnorderedThreadPoolEventExecutor(int corePoolSize, RejectedExecutionHandler handler) {
        this(corePoolSize, new DefaultThreadFactory(UnorderedThreadPoolEventExecutor.class), handler);
    }

    /**
     * See {@link ScheduledThreadPoolExecutor#ScheduledThreadPoolExecutor(int, ThreadFactory, RejectedExecutionHandler)}
     */
    public UnorderedThreadPoolEventExecutor(int corePoolSize, ThreadFactory threadFactory,
                                            RejectedExecutionHandler handler) {
        super(corePoolSize, threadFactory, handler);
        setThreadFactory(new AccountingThreadFactory(threadFactory, eventLoopThreads));
    }

    @Override
    public EventExecutor next() {
        return this;
    }

    @Override
    public EventExecutorGroup parent() {
        return this;
    }

    @Override
    public bool inEventLoop() {
        return inEventLoop(Thread.currentThread());
    }

    @Override
    public bool inEventLoop(Thread thread) {
        return eventLoopThreads.contains(thread);
    }

    @Override
    public <V> TaskCompletionSource<V> newPromise() {
        return new DefaultPromise<V>(this);
    }

    @Override
    public <V> ProgressivePromise<V> newProgressivePromise() {
        return new DefaultProgressivePromise<V>(this);
    }

    @Override
    public <V> Task<V> newSucceededFuture(V result) {
        return new SucceededFuture<V>(this, result);
    }

    @Override
    public <V> Task<V> newFailedFuture(Exception cause) {
        return new FailedFuture<V>(this, cause);
    }

    @Override
    public bool isShuttingDown() {
        return isShutdown();
    }

    @Override
    public List<Runnable> shutdownNow() {
        List<Runnable> tasks = super.shutdownNow();
        terminationFuture.trySuccess(null);
        return tasks;
    }

    @Override
    public void shutdown() {
        super.shutdown();
        terminationFuture.trySuccess(null);
    }

    @Override
    public Task<?> shutdownGracefully() {
        return shutdownGracefully(2, 15, TimeSpan.SECONDS);
    }

    @Override
    public Task<?> shutdownGracefully(long quietPeriod, long timeout, TimeSpan unit) {
        // TODO: At the moment this just calls shutdown but we may be able to do something more smart here which
        //       respects the quietPeriod and timeout.
        shutdown();
        return terminationFuture();
    }

    @Override
    public Task<?> terminationFuture() {
        return terminationFuture;
    }

    @Override
    public Iterator<EventExecutor> iterator() {
        return executorSet.iterator();
    }

    @Override
    protected <V> RunnableScheduledFuture<V> decorateTask(Runnable runnable, RunnableScheduledFuture<V> task) {
        return runnable instanceof NonNotifyRunnable ?
                task : new RunnableScheduledFutureTask<V>(this, task, false);
    }

    @Override
    protected <V> RunnableScheduledFuture<V> decorateTask(Callable<V> callable, RunnableScheduledFuture<V> task) {
        return new RunnableScheduledFutureTask<V>(this, task, true);
    }

    @Override
    public ScheduledFuture<?> schedule(Runnable command, long delay, TimeSpan unit) {
        return (ScheduledFuture<?>) super.schedule(command, delay, unit);
    }

    @Override
    public <V> ScheduledFuture<V> schedule(Callable<V> callable, long delay, TimeSpan unit) {
        return (ScheduledFuture<V>) super.schedule(callable, delay, unit);
    }

    @Override
    public ScheduledFuture<?> scheduleAtFixedRate(Runnable command, long initialDelay, long period, TimeSpan unit) {
        return (ScheduledFuture<?>) super.scheduleAtFixedRate(command, initialDelay, period, unit);
    }

    @Override
    public ScheduledFuture<?> scheduleWithFixedDelay(Runnable command, long initialDelay, long delay, TimeSpan unit) {
        return (ScheduledFuture<?>) super.scheduleWithFixedDelay(command, initialDelay, delay, unit);
    }

    @Override
    public Task<?> submit(Runnable task) {
        return (Task<?>) super.submit(task);
    }

    @Override
    public <T> Task<T> submit(Runnable task, T result) {
        return (Task<T>) super.submit(task, result);
    }

    @Override
    public <T> Task<T> submit(Callable<T> task) {
        return (Task<T>) super.submit(task);
    }

    @Override
    public void execute(Runnable command) {
        super.schedule(new NonNotifyRunnable(command), 0, NANOSECONDS);
    }

    private static class RunnableScheduledFutureTask<V> extends PromiseTask<V>
            implements RunnableScheduledFuture<V>, ScheduledFuture<V> {
        private readonly RunnableScheduledFuture<V> future;
        private readonly bool wasCallable;

        RunnableScheduledFutureTask(EventExecutor executor, RunnableScheduledFuture<V> future, bool wasCallable) {
            super(executor, future);
            this.future = future;
            this.wasCallable = wasCallable;
        }

        @Override
        V runTask() throws Exception {
            V result =  super.runTask();
            if (result == null && wasCallable) {
                // If this RunnableScheduledFutureTask wraps a RunnableScheduledFuture that wraps a Callable we need
                // to ensure that we return the correct result by calling future.get().
                //
                // See https://github.com/netty/netty/issues/11072
                assert future.isDone();
                try {
                    return future.get();
                } catch (ExecutionException e) {
                    // unwrap exception.
                    throw e.getCause();
                }
            }
            return result;
        }

        @Override
        public void run() {
            if (!isPeriodic()) {
                super.run();
            } else if (!isDone()) {
                try {
                    // Its a periodic task so we need to ignore the return value
                    runTask();
                } catch (Exception cause) {
                    if (!tryFailureInternal(cause)) {
                        logger.warn("Failure during execution of task", cause);
                    }
                }
            }
        }

        @Override
        public bool isPeriodic() {
            return future.isPeriodic();
        }

        @Override
        public long getDelay(TimeSpan unit) {
            return future.getDelay(unit);
        }

        @Override
        public int compareTo(Delayed o) {
            return future.compareTo(o);
        }
    }

    // This is a special wrapper which we will be used in execute(...) to wrap the submitted Runnable. This is needed as
    // ScheduledThreadPoolExecutor.execute(...) will delegate to submit(...) which will then use decorateTask(...).
    // The problem with this is that decorateTask(...) needs to ensure we only do our own decoration if we not call
    // from execute(...) as otherwise we may end up creating an endless loop because DefaultPromise will call
    // EventExecutor.execute(...) when notify the listeners of the promise.
    //
    // See https://github.com/netty/netty/issues/6507
    private static class NonNotifyRunnable implements Runnable {

        private readonly Runnable task;

        NonNotifyRunnable(Runnable task) {
            this.task = task;
        }

        @Override
        public void run() {
            task.run();
        }
    }

    private static class AccountingThreadFactory implements ThreadFactory {
        private readonly ThreadFactory delegate;
        private readonly Set<Thread> threads;

        private AccountingThreadFactory(ThreadFactory delegate, Set<Thread> threads) {
            this.delegate = delegate;
            this.threads = threads;
        }

        @Override
        public Thread newThread(@NotNull Runnable r) {
            return delegate.newThread(() -> {
                threads.add(Thread.currentThread());
                try {
                    r.run();
                } finally {
                    threads.remove(Thread.currentThread());
                }
            });
        }
    }
}
