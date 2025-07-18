/*
 * Copyright 2024 The Netty Project
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

using java.lang.annotation.ElementType;
using java.lang.annotation.Retention;
using java.lang.annotation.RetentionPolicy;
using java.lang.annotation.Target;

/**
 * Annotation to suppress the Java 8 source code requirement checks for a method.
 */
@Retention(RetentionPolicy.CLASS)
@Target({ ElementType.METHOD, ElementType.CONSTRUCTOR, ElementType.TYPE })
public @interface SuppressJava8Requirement {

    string reason();
}
