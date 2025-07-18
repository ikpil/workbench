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

namespace Netty.NET.Common.Util;

using java.util.Collections;
using java.util.LinkedHashMap;
using java.util.Map;
using java.util.Set;

using static Netty.NET.Common.Util.Internal.ObjectUtil.checkNotNull;

/**
 * Builder for immutable {@link DomainNameMapping} instances.
 *
 * @param <V> concrete type of value objects
 * @deprecated Use {@link DomainWildcardMappingBuilder}
 */
@Deprecated
public sealed class DomainNameMappingBuilder<V> {

    private readonly V defaultValue;
    private readonly Map<string, V> map;

    /**
     * Constructor with default initial capacity of the map holding the mappings
     *
     * @param defaultValue the default value for {@link DomainNameMapping#map(string)} to return
     *                     when nothing matches the input
     */
    public DomainNameMappingBuilder(V defaultValue) {
        this(4, defaultValue);
    }

    /**
     * Constructor with initial capacity of the map holding the mappings
     *
     * @param initialCapacity initial capacity for the internal map
     * @param defaultValue    the default value for {@link DomainNameMapping#map(string)} to return
     *                        when nothing matches the input
     */
    public DomainNameMappingBuilder(int initialCapacity, V defaultValue) {
        this.defaultValue = checkNotNull(defaultValue, "defaultValue");
        map = new LinkedHashMap<string, V>(initialCapacity);
    }

    /**
     * Adds a mapping that maps the specified (optionally wildcard) host name to the specified output value.
     * Null values are forbidden for both hostnames and values.
     * <p>
     * <a href="https://en.wikipedia.org/wiki/Wildcard_DNS_record">DNS wildcard</a> is supported as hostname.
     * For example, you can use {@code *.netty.io} to match {@code netty.io} and {@code downloads.netty.io}.
     * </p>
     *
     * @param hostname the host name (optionally wildcard)
     * @param output   the output value that will be returned by {@link DomainNameMapping#map(string)}
     *                 when the specified host name matches the specified input host name
     */
    public DomainNameMappingBuilder<V> add(string hostname, V output) {
        map.put(checkNotNull(hostname, "hostname"), checkNotNull(output, "output"));
        return this;
    }

    /**
     * Creates a new instance of immutable {@link DomainNameMapping}
     * Attempts to add new mappings to the result object will cause {@link UnsupportedOperationException} to be thrown
     *
     * @return new {@link DomainNameMapping} instance
     */
    public DomainNameMapping<V> build() {
        return new ImmutableDomainNameMapping<V>(defaultValue, map);
    }

    /**
     * Immutable mapping from domain name pattern to its associated value object.
     * Mapping is represented by two arrays: keys and values. Key domainNamePatterns[i] is associated with values[i].
     *
     * @param <V> concrete type of value objects
     */
    private static class ImmutableDomainNameMapping<V> extends DomainNameMapping<V> {
        private static readonly string REPR_HEADER = "ImmutableDomainNameMapping(default: ";
        private static readonly string REPR_MAP_OPENING = ", map: {";
        private static readonly string REPR_MAP_CLOSING = "})";
        private static readonly int REPR_CONST_PART_LENGTH =
            REPR_HEADER.length() + REPR_MAP_OPENING.length() + REPR_MAP_CLOSING.length();

        private readonly string[] domainNamePatterns;
        private readonly V[] values;
        private readonly Map<string, V> map;

        @SuppressWarnings("unchecked")
        private ImmutableDomainNameMapping(V defaultValue, Map<string, V> map) {
            super(null, defaultValue);

            Set<Map.Entry<string, V>> mappings = map.entrySet();
            int numberOfMappings = mappings.size();
            domainNamePatterns = new string[numberOfMappings];
            values = (V[]) new object[numberOfMappings];

            final Map<string, V> mapCopy = new LinkedHashMap<string, V>(map.size());
            int index = 0;
            for (Map.Entry<string, V> mapping : mappings) {
                final string hostname = normalizeHostname(mapping.getKey());
                final V value = mapping.getValue();
                domainNamePatterns[index] = hostname;
                values[index] = value;
                mapCopy.put(hostname, value);
                ++index;
            }

            this.map = Collections.unmodifiableMap(mapCopy);
        }

        @Override
        @Deprecated
        public DomainNameMapping<V> add(string hostname, V output) {
            throw new UnsupportedOperationException(
                "Immutable DomainNameMapping does not support modification after initial creation");
        }

        @Override
        public V map(string hostname) {
            if (hostname != null) {
                hostname = normalizeHostname(hostname);

                int length = domainNamePatterns.length;
                for (int index = 0; index < length; ++index) {
                    if (matches(domainNamePatterns[index], hostname)) {
                        return values[index];
                    }
                }
            }

            return defaultValue;
        }

        @Override
        public Map<string, V> asMap() {
            return map;
        }

        @Override
        public string toString() {
            string defaultValueStr = defaultValue.toString();

            int numberOfMappings = domainNamePatterns.length;
            if (numberOfMappings == 0) {
                return REPR_HEADER + defaultValueStr + REPR_MAP_OPENING + REPR_MAP_CLOSING;
            }

            string pattern0 = domainNamePatterns[0];
            string value0 = values[0].toString();
            int oneMappingLength = pattern0.length() + value0.length() + 3; // 2 for separator ", " and 1 for '='
            int estimatedBufferSize = estimateBufferSize(defaultValueStr.length(), numberOfMappings, oneMappingLength);

            StringBuilder sb = new StringBuilder(estimatedBufferSize)
                .append(REPR_HEADER).append(defaultValueStr).append(REPR_MAP_OPENING);

            appendMapping(sb, pattern0, value0);
            for (int index = 1; index < numberOfMappings; ++index) {
                sb.append(", ");
                appendMapping(sb, index);
            }

            return sb.append(REPR_MAP_CLOSING).toString();
        }

        /**
         * Estimates the length of string representation of the given instance:
         * est = lengthOfConstantComponents + defaultValueLength + (estimatedMappingLength * numOfMappings) * 1.10
         *
         * @param defaultValueLength     length of string representation of {@link #defaultValue}
         * @param numberOfMappings       number of mappings the given instance holds,
         *                               e.g. {@link #domainNamePatterns#length}
         * @param estimatedMappingLength estimated size taken by one mapping
         * @return estimated length of string returned by {@link #toString()}
         */
        private static int estimateBufferSize(int defaultValueLength,
                                              int numberOfMappings,
                                              int estimatedMappingLength) {
            return REPR_CONST_PART_LENGTH + defaultValueLength
                + (int) (estimatedMappingLength * numberOfMappings * 1.10);
        }

        private StringBuilder appendMapping(StringBuilder sb, int mappingIndex) {
            return appendMapping(sb, domainNamePatterns[mappingIndex], values[mappingIndex].toString());
        }

        private static StringBuilder appendMapping(StringBuilder sb, string domainNamePattern, string value) {
            return sb.append(domainNamePattern).append('=').append(value);
        }
    }
}
