package org.example.atomic74.checkinstructions

import atomictest.capture
import atomictest.eq

fun notNull(n: Int?): Int {
    requireNotNull(n) {
        "notNull() argument cannot be null"
    }

    return n * 9
}

fun main() {
    val n: Int? = null
    capture {
        notNull(n)
    } eq "IllegalArgumentException: notNull() argument cannot be null"

    capture {
        requireNotNull(n)
    } eq "IllegalArgumentException: Required value was null."

    notNull(11) eq 99
}