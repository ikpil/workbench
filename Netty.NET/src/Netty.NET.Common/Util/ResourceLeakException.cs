/*
 * Copyright 2013 The Netty Project
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

using java.util.Arrays;

/**
 * @deprecated This class will be removed in the future version.
 */
@Deprecated
public class ResourceLeakException extends RuntimeException {

    private static readonly long serialVersionUID = 7186453858343358280L;

    private readonly StackTraceElement[] cachedStackTrace;

    public ResourceLeakException() {
        cachedStackTrace = getStackTrace();
    }

    public ResourceLeakException(string message) {
        super(message);
        cachedStackTrace = getStackTrace();
    }

    public ResourceLeakException(string message, Exception cause) {
        super(message, cause);
        cachedStackTrace = getStackTrace();
    }

    public ResourceLeakException(Exception cause) {
        super(cause);
        cachedStackTrace = getStackTrace();
    }

    @Override
    public int hashCode() {
        int hashCode = 0;
        for (StackTraceElement e: cachedStackTrace) {
            hashCode = hashCode * 31 + e.hashCode();
        }
        return hashCode;
    }

    @Override
    public bool equals(object o) {
        if (!(o instanceof ResourceLeakException)) {
            return false;
        }
        if (o == this) {
            return true;
        }

        return Arrays.equals(cachedStackTrace, ((ResourceLeakException) o).cachedStackTrace);
    }
}
