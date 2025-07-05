/*
 * Copyright 2020 The Netty Project
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
namespace Netty.NET.Common.Util;

using com.oracle.svm.core.annotate.Alias;
using com.oracle.svm.core.annotate.InjectAccessors;
using com.oracle.svm.core.annotate.TargetClass;

using java.net.Inet4Address;
using java.net.Inet6Address;
using java.net.InetAddress;
using java.net.NetworkInterface;
using java.util.Collection;

@TargetClass(NetUtil.class)
sealed class NetUtilSubstitutions {
    private NetUtilSubstitutions() {
    }

    @Alias
    @InjectAccessors(NetUtilLocalhost4Accessor.class)
    public static Inet4Address LOCALHOST4;

    @Alias
    @InjectAccessors(NetUtilLocalhost6Accessor.class)
    public static Inet6Address LOCALHOST6;

    @Alias
    @InjectAccessors(NetUtilLocalhostAccessor.class)
    public static InetAddress LOCALHOST;

    @Alias
    @InjectAccessors(NetUtilNetworkInterfacesAccessor.class)
    public static Collection<NetworkInterface> NETWORK_INTERFACES;

    private static class NetUtilLocalhost4Accessor {
        static Inet4Address get() {
            // using https://en.wikipedia.org/wiki/Initialization-on-demand_holder_idiom
            return NetUtilLocalhost4LazyHolder.LOCALHOST4;
        }

        static void set(Inet4Address ignored) {
            // a no-op setter to avoid exceptions when NetUtil is initialized at run-time
        }
    }

    private static class NetUtilLocalhost4LazyHolder {
        private static readonly Inet4Address LOCALHOST4 = NetUtilInitializations.createLocalhost4();
    }

    private static class NetUtilLocalhost6Accessor {
        static Inet6Address get() {
            // using https://en.wikipedia.org/wiki/Initialization-on-demand_holder_idiom
            return NetUtilLocalhost6LazyHolder.LOCALHOST6;
        }

        static void set(Inet6Address ignored) {
            // a no-op setter to avoid exceptions when NetUtil is initialized at run-time
        }
    }

    private static class NetUtilLocalhost6LazyHolder {
        private static readonly Inet6Address LOCALHOST6 = NetUtilInitializations.createLocalhost6();
    }

    private static class NetUtilLocalhostAccessor {
        static InetAddress get() {
            // using https://en.wikipedia.org/wiki/Initialization-on-demand_holder_idiom
            return NetUtilLocalhostLazyHolder.LOCALHOST;
        }

        static void set(InetAddress ignored) {
            // a no-op setter to avoid exceptions when NetUtil is initialized at run-time
        }
    }

    private static class NetUtilLocalhostLazyHolder {
        private static readonly InetAddress LOCALHOST = NetUtilInitializations
                .determineLoopback(NetUtilNetworkInterfacesLazyHolder.NETWORK_INTERFACES,
                        NetUtilLocalhost4LazyHolder.LOCALHOST4, NetUtilLocalhost6LazyHolder.LOCALHOST6)
                .address();
    }

    private static class NetUtilNetworkInterfacesAccessor {
        static Collection<NetworkInterface> get() {
            // using https://en.wikipedia.org/wiki/Initialization-on-demand_holder_idiom
            return NetUtilNetworkInterfacesLazyHolder.NETWORK_INTERFACES;
        }

        static void set(Collection<NetworkInterface> ignored) {
            // a no-op setter to avoid exceptions when NetUtil is initialized at run-time
        }
    }

    private static class NetUtilNetworkInterfacesLazyHolder {
        private static readonly Collection<NetworkInterface> NETWORK_INTERFACES =
                NetUtilInitializations.networkInterfaces();
    }
}

