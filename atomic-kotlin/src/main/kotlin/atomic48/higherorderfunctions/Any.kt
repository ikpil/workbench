package org.example.atomic48.higherorderfunctions

import atomictest.eq

fun <T> List<T>.any(predicate: (T) -> Boolean): Boolean {
    for (element in this) {
        if (predicate(element))
            return true
    }
    return false
}

fun main() {
    val ints = listOf(1, 2, -3)
    ints.any { it > 0 } eq true
    val strings = listOf("abc", " ")
    strings.any { it.isBlank() } eq true
    strings.any(String::isNotBlank) eq true
}