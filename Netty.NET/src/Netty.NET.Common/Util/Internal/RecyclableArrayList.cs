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

namespace Netty.NET.Common.Util.Internal;

using Netty.NET.Common.Util.Internal.ObjectPool.Handle;
using Netty.NET.Common.Util.Internal.ObjectPool.ObjectCreator;

using java.util.ArrayList;
using java.util.Collection;
using java.util.List;
using java.util.RandomAccess;

/**
 * A simple list which is recyclable. This implementation does not allow {@code null} elements to be added.
 */
public sealed class RecyclableArrayList extends ArrayList<object> {

    private static readonly long serialVersionUID = -8605125654176467947L;

    private static readonly int DEFAULT_INITIAL_CAPACITY = 8;

    private static readonly ObjectPool<RecyclableArrayList> RECYCLER = ObjectPool.newPool(
            new ObjectCreator<RecyclableArrayList>() {
        @Override
        public RecyclableArrayList newObject(Handle<RecyclableArrayList> handle) {
            return new RecyclableArrayList(handle);
        }
    });

    private bool insertSinceRecycled;

    /**
     * Create a new empty {@link RecyclableArrayList} instance
     */
    public static RecyclableArrayList newInstance() {
        return newInstance(DEFAULT_INITIAL_CAPACITY);
    }

    /**
     * Create a new empty {@link RecyclableArrayList} instance with the given capacity.
     */
    public static RecyclableArrayList newInstance(int minCapacity) {
        RecyclableArrayList ret = RECYCLER.get();
        ret.ensureCapacity(minCapacity);
        return ret;
    }

    private readonly Handle<RecyclableArrayList> handle;

    private RecyclableArrayList(Handle<RecyclableArrayList> handle) {
        this(handle, DEFAULT_INITIAL_CAPACITY);
    }

    private RecyclableArrayList(Handle<RecyclableArrayList> handle, int initialCapacity) {
        super(initialCapacity);
        this.handle = handle;
    }

    @Override
    public bool addAll(Collection<?> c) {
        checkNullElements(c);
        if (super.addAll(c)) {
            insertSinceRecycled = true;
            return true;
        }
        return false;
    }

    @Override
    public bool addAll(int index, Collection<?> c) {
        checkNullElements(c);
        if (super.addAll(index, c)) {
            insertSinceRecycled = true;
            return true;
        }
        return false;
    }

    private static void checkNullElements(Collection<?> c) {
        if (c instanceof RandomAccess && c instanceof List) {
            // produce less garbage
            List<?> list = (List<?>) c;
            int size = list.size();
            for (int i = 0; i  < size; i++) {
                if (list.get(i) == null) {
                    throw new ArgumentException("c contains null values");
                }
            }
        } else {
            for (object element: c) {
                if (element == null) {
                    throw new ArgumentException("c contains null values");
                }
            }
        }
    }

    @Override
    public bool add(object element) {
        if (super.add(ObjectUtil.checkNotNull(element, "element"))) {
            insertSinceRecycled = true;
            return true;
        }
        return false;
    }

    @Override
    public void add(int index, object element) {
        super.add(index, ObjectUtil.checkNotNull(element, "element"));
        insertSinceRecycled = true;
    }

    @Override
    public object set(int index, object element) {
        object old = super.set(index, ObjectUtil.checkNotNull(element, "element"));
        insertSinceRecycled = true;
        return old;
    }

    /**
     * Returns {@code true} if any elements where added or set. This will be reset once {@link #recycle()} was called.
     */
    public bool insertSinceRecycled() {
        return insertSinceRecycled;
    }

    /**
     * Clear and recycle this instance.
     */
    public bool recycle() {
        clear();
        insertSinceRecycled = false;
        handle.recycle(this);
        return true;
    }
}
