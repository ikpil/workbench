package org.example.atomic48.higherorderfunctions

import atomictest.eq

val isPlus: (Int) -> Boolean = { it > 0 }

fun main() {
    listOf(1, 2, -3).any(isPlus) eq true
}