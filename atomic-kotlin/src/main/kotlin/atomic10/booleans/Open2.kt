package org.example.atomic10.booleans

fun isOpen2(hour: Int) {
    val open = 9
    val closed = 20
    println("Operating hours: $open - $closed")
    val status = hour in open..closed
    println("Open: $status")
}

fun main() = isOpen2(6)