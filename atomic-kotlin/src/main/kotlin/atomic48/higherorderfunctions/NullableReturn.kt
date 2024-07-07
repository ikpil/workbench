package org.example.atomic48.higherorderfunctions

import atomictest.eq

fun main() {
    val transform: (String) -> Int? = { s: String -> s.toIntOrNull() }
    transform("122") eq 122
    transform("abc") eq null

    val x = listOf("112", "abc")
    x.mapNotNull(transform) eq "[112]"
    x.mapNotNull { it.toIntOrNull() } eq "[112]"
}