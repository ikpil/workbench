package org.example.atomic27.property_accessors

import atomictest.eq

class Counter {
    var value: Int = 0
        private set
    fun inc() = value++
}

fun main() {
    val counter = Counter()
    repeat(10) {
        counter.inc()
    }
    counter.value eq 10
}