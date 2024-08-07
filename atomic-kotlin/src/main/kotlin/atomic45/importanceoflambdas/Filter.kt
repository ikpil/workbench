package org.example.atomic45.importanceoflambdas

import atomictest.eq

fun main() {
    val list = listOf(1, 2, 3, 4)
    val even = list.filter { it % 2 == 0 }
    val greaterThan2 = list.filter { it > 2 }
    even eq listOf(2, 4)
    greaterThan2 eq listOf(3, 4)
}