/*
 * Copyright 2018 The Netty Project
 *
 * The Netty Project licenses this file to you under the Apache License, version 2.0 (the
 * "License"); you may not use this file except in compliance with the License. You may obtain a
 * copy of the License at:
 *
 * https://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software distributed under the License
 * is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express
 * or implied. See the License for the specific language governing permissions and limitations under
 * the License.
 */
namespace Netty.NET.Common.Util;

sealed class ByteProcessorUtils {
    static readonly byte SPACE = (byte) ' ';
    static readonly byte HTAB = (byte) '\t';
    static readonly byte CARRIAGE_RETURN = (byte) '\r';
    static readonly byte LINE_FEED = (byte) '\n';

    private ByteProcessorUtils() {
    }
}
