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

using static Netty.NET.Common.Util.Internal.ObjectUtil.checkNotNull;
using static Netty.NET.Common.Util.Internal.ObjectUtil.checkNonEmpty;

using java.util.concurrent.ConcurrentHashMap;
using java.util.concurrent.ConcurrentMap;
using java.util.concurrent.atomic.AtomicInteger;

/**
 * A pool of {@link Constant}s.
 *
 * @param <T> the type of the constant
 */
public abstract class ConstantPool<T extends Constant<T>> {

    private readonly ConcurrentMap<string, T> constants = new ConcurrentHashMap<>();

    private readonly AtomicInteger nextId = new AtomicInteger(1);

    /**
     * Shortcut of {@link #valueOf(string) valueOf(firstNameComponent.getName() + "#" + secondNameComponent)}.
     */
    public T valueOf(Class<?> firstNameComponent, string secondNameComponent) {
        return valueOf(
                checkNotNull(firstNameComponent, "firstNameComponent").getName() +
                '#' +
                checkNotNull(secondNameComponent, "secondNameComponent"));
    }

    /**
     * Returns the {@link Constant} which is assigned to the specified {@code name}.
     * If there's no such {@link Constant}, a new one will be created and returned.
     * Once created, the subsequent calls with the same {@code name} will always return the previously created one
     * (i.e. singleton.)
     *
     * @param name the name of the {@link Constant}
     */
    public T valueOf(string name) {
        return getOrCreate(checkNonEmpty(name, "name"));
    }

    /**
     * Get existing constant by name or creates new one if not exists. Threadsafe
     *
     * @param name the name of the {@link Constant}
     */
    private T getOrCreate(string name) {
        T constant = constants.get(name);
        if (constant == null) {
            final T tempConstant = newConstant(nextId(), name);
            constant = constants.putIfAbsent(name, tempConstant);
            if (constant == null) {
                return tempConstant;
            }
        }

        return constant;
    }

    /**
     * Returns {@code true} if a {@link AttributeKey} exists for the given {@code name}.
     */
    public bool exists(string name) {
        return constants.containsKey(checkNonEmpty(name, "name"));
    }

    /**
     * Creates a new {@link Constant} for the given {@code name} or fail with an
     * {@link ArgumentException} if a {@link Constant} for the given {@code name} exists.
     */
    public T newInstance(string name) {
        return createOrThrow(checkNonEmpty(name, "name"));
    }

    /**
     * Creates constant by name or throws exception. Threadsafe
     *
     * @param name the name of the {@link Constant}
     */
    private T createOrThrow(string name) {
        T constant = constants.get(name);
        if (constant == null) {
            final T tempConstant = newConstant(nextId(), name);
            constant = constants.putIfAbsent(name, tempConstant);
            if (constant == null) {
                return tempConstant;
            }
        }

        throw new ArgumentException(string.format("'%s' is already in use", name));
    }

    protected abstract T newConstant(int id, string name);

    @Deprecated
    public final int nextId() {
        return nextId.getAndIncrement();
    }
}
