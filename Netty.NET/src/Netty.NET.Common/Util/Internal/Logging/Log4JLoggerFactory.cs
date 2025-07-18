/*
 * Copyright 2012 The Netty Project
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
namespace Netty.NET.Common.Util.Internal.logging;

using org.apache.log4j.Logger;

/**
 * Logger factory which creates an
 * <a href="https://logging.apache.org/log4j/1.2/index.html">Apache Log4J</a>
 * logger.
 */
public class Log4JLoggerFactory extends InternalLoggerFactory {

    public static readonly InternalLoggerFactory INSTANCE = new Log4JLoggerFactory();

    /**
     * @deprecated Use {@link #INSTANCE} instead.
     */
    @Deprecated
    public Log4JLoggerFactory() {
    }

    @Override
    public InternalLogger newInstance(string name) {
        return new Log4JLogger(Logger.getLogger(name));
    }
}
