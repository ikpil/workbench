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

using Netty.NET.Common.Util.Internal;

namespace Netty.NET.Common.Util
{

    /**
     * A utility class for wrapping calls to {@link Runtime}.
     */
    public sealed class NettyRuntime
    {
        /**
         * Holder class for available processors to enable testing.
         */
        public class AvailableProcessorsHolder
        {
            private readonly object _lock = new object();
            private int availableProcessors;

            /**
             * Set the number of available processors.
             *
             * @param availableProcessors the number of available processors
             * @throws ArgumentException if the specified number of available processors is non-positive
             * @throws IllegalStateException    if the number of available processors is already configured
             */
            public void setAvailableProcessors(int availableProcessors)
            {
                lock (_lock)
                {
                    ObjectUtil.checkPositive(availableProcessors, "availableProcessors");
                    if (this.availableProcessors != 0)
                    {
                        string message = $"availableProcessors is already set to [{this.availableProcessors}], rejecting [{availableProcessors}]";
                        throw new InvalidOperationException(message);
                    }

                    this.availableProcessors = availableProcessors;
                }
            }

            /**
             * Get the configured number of available processors. The default is {@link Runtime#availableProcessors()}.
             * This can be overridden by setting the system property "io.netty.availableProcessors" or by invoking
             * {@link #setAvailableProcessors(int)} before any calls to this method.
             *
             * @return the configured number of available processors
             */
            public int AvailableProcessors()
            {
                lock (_lock)
                {
                    if (this.availableProcessors == 0)
                    {
                        int availableProcessors =
                            SystemPropertyUtil.getInt(
                                "io.netty.availableProcessors",
                                Environment.ProcessorCount);
                        setAvailableProcessors(availableProcessors);
                    }

                    return this.availableProcessors;
                }
            }
        }

        private static readonly AvailableProcessorsHolder holder = new AvailableProcessorsHolder();

        /**
         * Set the number of available processors.
         *
         * @param availableProcessors the number of available processors
         * @throws ArgumentException if the specified number of available processors is non-positive
         * @throws IllegalStateException    if the number of available processors is already configured
         */
        public static void setAvailableProcessors(int availableProcessors)
        {
            holder.setAvailableProcessors(availableProcessors);
        }

        /**
         * Get the configured number of available processors. The default is {@link Runtime#availableProcessors()}. This
         * can be overridden by setting the system property "io.netty.availableProcessors" or by invoking
         * {@link #setAvailableProcessors(int)} before any calls to this method.
         *
         * @return the configured number of available processors
         */
        public static int availableProcessors()
        {
            return holder.AvailableProcessors();
        }

        /**
         * No public constructor to prevent instances from being created.
         */
        private NettyRuntime()
        {
        }
    }
}