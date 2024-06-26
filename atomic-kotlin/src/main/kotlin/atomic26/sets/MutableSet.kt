package org.example.atomic26.sets

import atomictest.eq

fun main() {
    val mutableSet = mutableSetOf<Int>()
    mutableSet += 42
    mutableSet += 42

    mutableSet eq setOf(42)
    mutableSet -= 42
    mutableSet eq setOf<Int>()
}