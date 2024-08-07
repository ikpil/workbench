package org.example.atomic51.sequences

import atomictest.eq

fun main() {
    val list = listOf(1, 2, 3, 4)
    list.filter { it % 2 == 0 }
        .map { it * it }
        .any { it < 10 } eq true


    val mid1 = list.filter { it % 2 == 0 }
    mid1 eq listOf(2, 4)

    val mid2 = mid1.map { it * it }
    mid2 eq listOf(4, 16)

    mid2.any { it < 10 } eq true
}