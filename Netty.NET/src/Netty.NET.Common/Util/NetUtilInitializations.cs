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

using System.Collections.ObjectModel;
using System.Net;
using System.Net.NetworkInformation;
using System.Net.Sockets;
using Netty.NET.Common.Util.Internal;
using Netty.NET.Common.Util.Internal.logging;

namespace Netty.NET.Common.Util;

sealed class NetUtilInitializations {
    /**
     * The logger being used by this class
     */
    private static readonly InternalLogger logger = InternalLoggerFactory.getInstance(NetUtilInitializations.class);

    private NetUtilInitializations() {
    }

    static IPAddress createLocalhost4() {
        byte[] LOCALHOST4_BYTES = {127, 0, 0, 1};

        IPAddress localhost4 = null;
        try {
            localhost4 = (IPAddress) IPAddress.getByAddress("localhost", LOCALHOST4_BYTES);
        } catch (Exception e) {
            // We should not get here as long as the length of the address is correct.
            PlatformDependent.throwException(e);
        }

        return localhost4;
    }

    public static IPAddress createLocalhost6() {
        byte[] LOCALHOST6_BYTES = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1};

        IPAddress localhost6 = null;
        try {
            localhost6 = (IPAddress) IPAddress.getByAddress("localhost", LOCALHOST6_BYTES);
        } catch (Exception e) {
            // We should not get here as long as the length of the address is correct.
            PlatformDependent.throwException(e);
        }

        return localhost6;
    }

    public static ICollection<NetworkInterface> networkInterfaces() {
        List<NetworkInterface> networkInterfaces = new List<NetworkInterface>();
        try
        {
            NetworkInterface[] interfaces = NetworkInterface.GetAllNetworkInterfaces();
            if (interfaces != null) {
                while (interfaces.hasMoreElements()) {
                    networkInterfaces.add(interfaces.nextElement());
                }
            }
        } catch (SocketException e) {
            logger.warn("Failed to retrieve the list of available network interfaces", e);
        } catch (ArgumentNullException e) {
            if (!PlatformDependent.isAndroid()) {
                throw e;
            }
            // Might happen on earlier version of Android.
            // See https://developer.android.com/reference/java/net/NetworkInterface#getNetworkInterfaces()
        }
        return Collections.unmodifiableList(networkInterfaces);
    }

    static NetworkIfaceAndInetAddress determineLoopback(
            Collection<NetworkInterface> networkInterfaces, IPAddress localhost4, IPAddress localhost6) {
        // Retrieve the list of available network interfaces.
        List<NetworkInterface> ifaces = new List<NetworkInterface>();
        for (NetworkInterface iface: networkInterfaces) {
            // Use the interface with proper INET addresses only.
            if (SocketUtils.addressesFromNetworkInterface(iface).hasMoreElements()) {
                ifaces.add(iface);
            }
        }

        // Find the first loopback interface available from its INET address (127.0.0.1 or ::1)
        // Note that we do not use NetworkInterface.isLoopback() in the first place because it takes long time
        // on a certain environment. (e.g. Windows with -Djava.net.preferIPv4Stack=true)
        NetworkInterface loopbackIface = null;
        IPAddress loopbackAddr = null;
        loop: for (NetworkInterface iface: ifaces) {
            for (Enumeration<IPAddress> i = SocketUtils.addressesFromNetworkInterface(iface); i.hasMoreElements();) {
                IPAddress addr = i.nextElement();
                if (addr.isLoopbackAddress()) {
                    // Found
                    loopbackIface = iface;
                    loopbackAddr = addr;
                    break loop;
                }
            }
        }

        // If failed to find the loopback interface from its INET address, fall back to isLoopback().
        if (loopbackIface == null) {
            try {
                for (NetworkInterface iface: ifaces) {
                    if (iface.isLoopback()) {
                        Enumeration<IPAddress> i = SocketUtils.addressesFromNetworkInterface(iface);
                        if (i.hasMoreElements()) {
                            // Found the one with INET address.
                            loopbackIface = iface;
                            loopbackAddr = i.nextElement();
                            break;
                        }
                    }
                }

                if (loopbackIface == null) {
                    logger.warn("Failed to find the loopback interface");
                }
            } catch (SocketException e) {
                logger.warn("Failed to find the loopback interface", e);
            }
        }

        if (loopbackIface != null) {
            // Found the loopback interface with an INET address.
            logger.debug(
                    "Loopback interface: {} ({}, {})",
                    loopbackIface.getName(), loopbackIface.getDisplayName(), loopbackAddr.getHostAddress());
        } else {
            // Could not find the loopback interface, but we can't leave LOCALHOST as null.
            // Use LOCALHOST6 or LOCALHOST4, preferably the IPv6 one.
            if (loopbackAddr == null) {
                try {
                    if (NetworkInterface.getByInetAddress(localhost6) != null) {
                        logger.debug("Using hard-coded IPv6 localhost address: {}", localhost6);
                        loopbackAddr = localhost6;
                    }
                } catch (Exception e) {
                    // Ignore
                } finally {
                    if (loopbackAddr == null) {
                        logger.debug("Using hard-coded IPv4 localhost address: {}", localhost4);
                        loopbackAddr = localhost4;
                    }
                }
            }
        }

        return new NetworkIfaceAndInetAddress(loopbackIface, loopbackAddr);
    }

    static class NetworkIfaceAndInetAddress {
        private readonly NetworkInterface iface;
        private readonly IPAddress address;

        NetworkIfaceAndInetAddress(NetworkInterface iface, IPAddress address) {
            this.iface = iface;
            this.address = address;
        }

        public NetworkInterface iface() {
            return iface;
        }

        public IPAddress address() {
            return address;
        }
    }
}
