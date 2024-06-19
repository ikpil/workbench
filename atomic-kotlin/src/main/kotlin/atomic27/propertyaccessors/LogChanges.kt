package org.example.atomic27.propertyaccessors

import atomictest.eq
import atomictest.trace

class LogChanges {
    var n: Int = 0
        set(value) {
            trace("$field becomes $value")
            field = value
        }
}

fun main() {
    val lc = LogChanges()
    lc.n eq 0
    lc.n = 2
    lc.n eq 2
    trace eq "0 becomes 2"
}