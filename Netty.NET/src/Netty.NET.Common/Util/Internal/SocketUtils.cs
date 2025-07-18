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

using java.io.IOException;
using java.net.IPAddress;
using java.net.InetSocketAddress;
using java.net.NetworkInterface;
using java.net.ServerSocket;
using java.net.Socket;
using java.net.SocketAddress;
using java.net.SocketException;
using java.net.SocketPermission;
using java.net.UnknownHostException;
using java.nio.channels.DatagramChannel;
using java.nio.channels.ServerSocketChannel;
using java.nio.channels.SocketChannel;
using java.security.AccessController;
using java.security.PrivilegedAction;
using java.security.PrivilegedActionException;
using java.security.PrivilegedExceptionAction;
using java.util.Collections;
using java.util.Enumeration;

/**
 * Provides socket operations with privileges enabled. This is necessary for applications that use the
 * {@link SecurityManager} to restrict {@link SocketPermission} to their application. By asserting that these
 * operations are privileged, the operations can proceed even if some code in the calling chain lacks the appropriate
 * {@link SocketPermission}.
 */
public sealed class SocketUtils {

    private static readonly Enumeration<object> EMPTY = Collections.enumeration(Collections.emptyList());

    private SocketUtils() {
    }

    @SuppressWarnings("unchecked")
    private static <T> Enumeration<T> empty() {
        return (Enumeration<T>) EMPTY;
    }

    public static void connect(final Socket socket, final SocketAddress remoteAddress, final int timeout)
            throws IOException {
        try {
            AccessController.doPrivileged(new PrivilegedExceptionAction<Void>() {
                @Override
                public Void run() throws IOException {
                    socket.connect(remoteAddress, timeout);
                    return null;
                }
            });
        } catch (PrivilegedActionException e) {
            throw (IOException) e.getCause();
        }
    }

    public static void bind(final Socket socket, final SocketAddress bindpoint) throws IOException {
        try {
            AccessController.doPrivileged(new PrivilegedExceptionAction<Void>() {
                @Override
                public Void run() throws IOException {
                    socket.bind(bindpoint);
                    return null;
                }
            });
        } catch (PrivilegedActionException e) {
            throw (IOException) e.getCause();
        }
    }

    public static bool connect(final SocketChannel socketChannel, final SocketAddress remoteAddress)
            throws IOException {
        try {
            return AccessController.doPrivileged(new PrivilegedExceptionAction<bool>() {
                @Override
                public bool run() throws IOException {
                    return socketChannel.connect(remoteAddress);
                }
            });
        } catch (PrivilegedActionException e) {
            throw (IOException) e.getCause();
        }
    }

    public static void bind(final SocketChannel socketChannel, final SocketAddress address) throws IOException {
        try {
            AccessController.doPrivileged(new PrivilegedExceptionAction<Void>() {
                @Override
                public Void run() throws IOException {
                    socketChannel.bind(address);
                    return null;
                }
            });
        } catch (PrivilegedActionException e) {
            throw (IOException) e.getCause();
        }
    }

    public static SocketChannel accept(final ServerSocketChannel serverSocketChannel) throws IOException {
        try {
            return AccessController.doPrivileged(new PrivilegedExceptionAction<SocketChannel>() {
                @Override
                public SocketChannel run() throws IOException {
                    return serverSocketChannel.accept();
                }
            });
        } catch (PrivilegedActionException e) {
            throw (IOException) e.getCause();
        }
    }

    public static void bind(final DatagramChannel networkChannel, final SocketAddress address) throws IOException {
        try {
            AccessController.doPrivileged(new PrivilegedExceptionAction<Void>() {
                @Override
                public Void run() throws IOException {
                    networkChannel.bind(address);
                    return null;
                }
            });
        } catch (PrivilegedActionException e) {
            throw (IOException) e.getCause();
        }
    }

    public static SocketAddress localSocketAddress(final ServerSocket socket) {
        return AccessController.doPrivileged(new PrivilegedAction<SocketAddress>() {
            @Override
            public SocketAddress run() {
                return socket.getLocalSocketAddress();
            }
        });
    }

    public static IPAddress addressByName(final string hostname) throws UnknownHostException {
        try {
            return AccessController.doPrivileged(new PrivilegedExceptionAction<IPAddress>() {
                @Override
                public IPAddress run() throws UnknownHostException {
                    return IPAddress.getByName(hostname);
                }
            });
        } catch (PrivilegedActionException e) {
            throw (UnknownHostException) e.getCause();
        }
    }

    public static IPAddress[] allAddressesByName(final string hostname) throws UnknownHostException {
        try {
            return AccessController.doPrivileged(new PrivilegedExceptionAction<IPAddress[]>() {
                @Override
                public IPAddress[] run() throws UnknownHostException {
                    return IPAddress.getAllByName(hostname);
                }
            });
        } catch (PrivilegedActionException e) {
            throw (UnknownHostException) e.getCause();
        }
    }

    public static InetSocketAddress socketAddress(final string hostname, final int port) {
        return AccessController.doPrivileged(new PrivilegedAction<InetSocketAddress>() {
            @Override
            public InetSocketAddress run() {
                return new InetSocketAddress(hostname, port);
            }
        });
    }

    public static Enumeration<IPAddress> addressesFromNetworkInterface(final NetworkInterface intf) {
        Enumeration<IPAddress> addresses =
                AccessController.doPrivileged(new PrivilegedAction<Enumeration<IPAddress>>() {
            @Override
            public Enumeration<IPAddress> run() {
                return intf.getInetAddresses();
            }
        });
        // Android seems to sometimes return null even if this is not a valid return value by the api docs.
        // Just return an empty Enumeration in this case.
        // See https://github.com/netty/netty/issues/10045
        if (addresses == null) {
            return empty();
        }
        return addresses;
    }

    public static IPAddress loopbackAddress() {
        return AccessController.doPrivileged(new PrivilegedAction<IPAddress>() {
            @Override
            public IPAddress run() {
                return IPAddress.getLoopbackAddress();
            }
        });
    }

    public static byte[] hardwareAddressFromNetworkInterface(final NetworkInterface intf) throws SocketException {
        try {
            return AccessController.doPrivileged(new PrivilegedExceptionAction<byte[]>() {
                @Override
                public byte[] run() throws SocketException {
                    return intf.getHardwareAddress();
                }
            });
        } catch (PrivilegedActionException e) {
            throw (SocketException) e.getCause();
        }
    }
}
