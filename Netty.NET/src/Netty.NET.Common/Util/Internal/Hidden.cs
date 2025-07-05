/*
 * Copyright 2019 The Netty Project
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
using reactor.blockhound.BlockHound;
using reactor.blockhound.integration.BlockHoundIntegration;

using java.util.function.Function;
using java.util.function.Predicate;

/**
 * Contains classes that must have public visibility but are not public API.
 */
class Hidden {

    /**
     * This class integrates Netty with BlockHound.
     * <p>
     * It is public but only because of the ServiceLoader's limitations
     * and SHOULD NOT be considered a public API.
     */
    @UnstableApi
    public static final class NettyBlockHoundIntegration implements BlockHoundIntegration {

        @Override
        public void applyTo(BlockHound.Builder builder) {
            builder.allowBlockingCallsInside(
                    "io.netty.channel.nio.NioEventLoop",
                    "handleLoopException"
            );

            builder.allowBlockingCallsInside(
                    "io.netty.channel.kqueue.KQueueEventLoop",
                    "handleLoopException"
            );

            builder.allowBlockingCallsInside(
                    "io.netty.channel.epoll.EpollEventLoop",
                    "handleLoopException"
            );

            builder.allowBlockingCallsInside(
                    "Netty.NET.Common.Util.HashedWheelTimer",
                    "start"
            );

            builder.allowBlockingCallsInside(
                    "Netty.NET.Common.Util.HashedWheelTimer",
                    "stop"
            );

            builder.allowBlockingCallsInside(
                    "Netty.NET.Common.Util.HashedWheelTimer$Worker",
                    "waitForNextTick"
            );

            builder.allowBlockingCallsInside(
                    "Netty.NET.Common.Util.concurrent.SingleThreadEventExecutor",
                    "confirmShutdown"
            );

            builder.allowBlockingCallsInside(
                    "io.netty.buffer.PoolArena",
                    "lock"
            );

            builder.allowBlockingCallsInside(
                    "io.netty.buffer.PoolSubpage",
                    "lock"
            );

            builder.allowBlockingCallsInside(
                    "io.netty.buffer.PoolChunk",
                    "allocateRun"
            );

            builder.allowBlockingCallsInside(
                    "io.netty.buffer.PoolChunk",
                    "free"
            );

            builder.allowBlockingCallsInside(
                    "io.netty.buffer.AdaptivePoolingAllocator$1",
                    "initialValue"
            );

            builder.allowBlockingCallsInside(
                    "io.netty.buffer.AdaptivePoolingAllocator$1",
                    "onRemoval"
            );

            builder.allowBlockingCallsInside(
                    "io.netty.handler.ssl.SslHandler",
                    "handshake"
            );

            builder.allowBlockingCallsInside(
                    "io.netty.handler.ssl.SslHandler",
                    "runAllDelegatedTasks"
            );
            builder.allowBlockingCallsInside(
                    "io.netty.handler.ssl.SslHandler",
                    "runDelegatedTasks"
            );
            builder.allowBlockingCallsInside(
                    "Netty.NET.Common.Util.concurrent.GlobalEventExecutor",
                    "takeTask");

            builder.allowBlockingCallsInside(
                    "Netty.NET.Common.Util.concurrent.GlobalEventExecutor",
                    "addTask");

            builder.allowBlockingCallsInside(
                    "Netty.NET.Common.Util.concurrent.SingleThreadEventExecutor",
                    "pollTaskFrom");

            builder.allowBlockingCallsInside(
                    "Netty.NET.Common.Util.concurrent.SingleThreadEventExecutor",
                    "takeTask");

            builder.allowBlockingCallsInside(
                    "Netty.NET.Common.Util.concurrent.SingleThreadEventExecutor",
                    "addTask");

            builder.allowBlockingCallsInside(
                    "io.netty.handler.ssl.ReferenceCountedOpenSslClientContext$ExtendedTrustManagerVerifyCallback",
                    "verify");

            builder.allowBlockingCallsInside(
                    "io.netty.handler.ssl.JdkSslContext$Defaults",
                    "init");

            // Let's whitelist SSLEngineImpl.unwrap(...) for now as it may fail otherwise for TLS 1.3.
            // See https://mail.openjdk.java.net/pipermail/security-dev/2020-August/022271.html
            builder.allowBlockingCallsInside(
                    "sun.security.ssl.SSLEngineImpl",
                    "unwrap");

            builder.allowBlockingCallsInside(
                    "sun.security.ssl.SSLEngineImpl",
                    "wrap");

            builder.allowBlockingCallsInside(
                    "io.netty.resolver.dns.UnixResolverDnsServerAddressStreamProvider",
                    "parse");

            builder.allowBlockingCallsInside(
                    "io.netty.resolver.dns.UnixResolverDnsServerAddressStreamProvider",
                    "parseEtcResolverSearchDomains");

            builder.allowBlockingCallsInside(
                    "io.netty.resolver.dns.UnixResolverDnsServerAddressStreamProvider",
                    "parseEtcResolverOptions");

            builder.allowBlockingCallsInside(
                    "io.netty.resolver.HostsFileEntriesProvider$ParserImpl",
                    "parse");

            builder.allowBlockingCallsInside(
                    "Netty.NET.Common.Util.NetUtil$SoMaxConnAction",
                    "run");

            builder.allowBlockingCallsInside("Netty.NET.Common.Util.Internal.ReferenceCountUpdater",
                    "retryRelease0");

            builder.allowBlockingCallsInside("Netty.NET.Common.Util.Internal.PlatformDependent", "createTempFile");
            builder.nonBlockingThreadPredicate(new Function<Predicate<Thread>, Predicate<Thread>>() {
                @Override
                public Predicate<Thread> apply(final Predicate<Thread> p) {
                    return new Predicate<Thread>() {
                        @Override
                        public bool test(Thread thread) {
                            return p.test(thread) ||
                                    thread instanceof FastThreadLocalThread &&
                                            !((FastThreadLocalThread) thread).permitBlockingCalls();
                        }
                    };
                }
            });
        }

        @Override
        public int compareTo(BlockHoundIntegration o) {
            return 0;
        }
    }
}
