package org.example.atomic74.checkinstructions

import atomictest.capture
import atomictest.eq

fun singleArgRequire(arg: Int): Int {
    require(arg > 5)
    return arg
}

fun main() {
    capture {
        singleArgRequire(5)
    } eq "IllegalArgumentException: Failed requirement."
    singleArgRequire(6) eq 6
}