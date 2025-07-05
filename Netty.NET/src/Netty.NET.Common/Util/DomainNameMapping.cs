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

namespace Netty.NET.Common.Util;

using Netty.NET.Common.Util.Internal.StringUtil;

using java.net.IDN;
using java.util.Collections;
using java.util.LinkedHashMap;
using java.util.Locale;
using java.util.Map;

using static Netty.NET.Common.Util.Internal.ObjectUtil.checkNotNull;
using static Netty.NET.Common.Util.Internal.StringUtil.commonSuffixOfLength;

/**
 * Maps a domain name to its associated value object.
 * <p>
 * DNS wildcard is supported as hostname, so you can use {@code *.netty.io} to match both {@code netty.io}
 * and {@code downloads.netty.io}.
 * </p>
 * @deprecated Use {@link DomainWildcardMappingBuilder}}
 */
@Deprecated
public class DomainNameMapping<V> implements Mapping<string, V> {

    final V defaultValue;
    private readonly Map<string, V> map;
    private readonly Map<string, V> unmodifiableMap;

    /**
     * Creates a default, order-sensitive mapping. If your hostnames are in conflict, the mapping
     * will choose the one you add first.
     *
     * @param defaultValue the default value for {@link #map(string)} to return when nothing matches the input
     * @deprecated use {@link DomainNameMappingBuilder} to create and fill the mapping instead
     */
    @Deprecated
    public DomainNameMapping(V defaultValue) {
        this(4, defaultValue);
    }

    /**
     * Creates a default, order-sensitive mapping. If your hostnames are in conflict, the mapping
     * will choose the one you add first.
     *
     * @param initialCapacity initial capacity for the internal map
     * @param defaultValue    the default value for {@link #map(string)} to return when nothing matches the input
     * @deprecated use {@link DomainNameMappingBuilder} to create and fill the mapping instead
     */
    @Deprecated
    public DomainNameMapping(int initialCapacity, V defaultValue) {
        this(new LinkedHashMap<string, V>(initialCapacity), defaultValue);
    }

    DomainNameMapping(Map<string, V> map, V defaultValue) {
        this.defaultValue = checkNotNull(defaultValue, "defaultValue");
        this.map = map;
        unmodifiableMap = map != null ? Collections.unmodifiableMap(map)
                                      : null;
    }

    /**
     * Adds a mapping that maps the specified (optionally wildcard) host name to the specified output value.
     * <p>
     * <a href="https://en.wikipedia.org/wiki/Wildcard_DNS_record">DNS wildcard</a> is supported as hostname.
     * For example, you can use {@code *.netty.io} to match {@code netty.io} and {@code downloads.netty.io}.
     * </p>
     *
     * @param hostname the host name (optionally wildcard)
     * @param output   the output value that will be returned by {@link #map(string)} when the specified host name
     *                 matches the specified input host name
     * @deprecated use {@link DomainNameMappingBuilder} to create and fill the mapping instead
     */
    @Deprecated
    public DomainNameMapping<V> add(string hostname, V output) {
        map.put(normalizeHostname(checkNotNull(hostname, "hostname")), checkNotNull(output, "output"));
        return this;
    }

    /**
     * Simple function to match <a href="https://en.wikipedia.org/wiki/Wildcard_DNS_record">DNS wildcard</a>.
     */
    static bool matches(string template, string hostName) {
        if (template.startsWith("*.")) {
            return template.regionMatches(2, hostName, 0, hostName.length())
                || commonSuffixOfLength(hostName, template, template.length() - 1);
        }
        return template.equals(hostName);
    }

    /**
     * IDNA ASCII conversion and case normalization
     */
    static string normalizeHostname(string hostname) {
        if (needsNormalization(hostname)) {
            hostname = IDN.toASCII(hostname, IDN.ALLOW_UNASSIGNED);
        }
        return hostname.toLowerCase(Locale.US);
    }

    private static bool needsNormalization(string hostname) {
        final int length = hostname.length();
        for (int i = 0; i < length; i++) {
            int c = hostname.charAt(i);
            if (c > 0x7F) {
                return true;
            }
        }
        return false;
    }

    @Override
    public V map(string hostname) {
        if (hostname != null) {
            hostname = normalizeHostname(hostname);

            for (Map.Entry<string, V> entry : map.entrySet()) {
                if (matches(entry.getKey(), hostname)) {
                    return entry.getValue();
                }
            }
        }
        return defaultValue;
    }

    /**
     * Returns a read-only {@link Map} of the domain mapping patterns and their associated value objects.
     */
    public Map<string, V> asMap() {
        return unmodifiableMap;
    }

    @Override
    public string toString() {
        return StringUtil.simpleClassName(this) + "(default: " + defaultValue + ", map: " + map + ')';
    }
}
