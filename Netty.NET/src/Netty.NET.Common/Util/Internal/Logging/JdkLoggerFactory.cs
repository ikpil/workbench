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


using java.util.logging.Logger;

/**
 * Logger factory which creates a
 * <a href="https://docs.oracle.com/javase/7/docs/technotes/guides/logging/">java.util.logging</a>
 * logger.
 */
public class JdkLoggerFactory extends InternalLoggerFactory {

    public static readonly InternalLoggerFactory INSTANCE = new JdkLoggerFactory();

    /**
     * @deprecated Use {@link #INSTANCE} instead.
     */
    @Deprecated
    public JdkLoggerFactory() {
    }

    @Override
    public InternalLogger newInstance(string name) {
        return new JdkLogger(Logger.getLogger(name));
    }
}
