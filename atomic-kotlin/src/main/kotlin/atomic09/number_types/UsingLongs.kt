package org.example.atomic09.number_types

fun main() {
    val i = Int.MAX_VALUE
    println(0L + i + i)
    println(i + i + 0L)
    println(1_000_000 * 1_000_000L)
}