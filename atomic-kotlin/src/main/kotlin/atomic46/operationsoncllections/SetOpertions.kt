package org.example.atomic46.operationsoncllections

import atomictest.eq

fun main() {
    val set = setOf("a", "ab", "ac")
    set.maxByOrNull { it.length }?.length eq 2
    set.filter {
        it.contains('b')
    } eq listOf("ab")

    set.map { it.length } eq listOf(1, 2, 2)
}