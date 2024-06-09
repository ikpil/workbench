package org.example.NumberTypes

fun main() {
    val i = Int.MAX_VALUE
    println(0L + i + i)
    println(i + i + 0L)
    println(1_000_000 * 1_000_000L)
}