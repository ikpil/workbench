package org.example.atomic10.booleans

fun isOpen1(hour: Int) {
    val open = 9
    val closed = 20
    println("Operating hour: $open - $closed")

    val status =
        if (hour in open..closed)
            true
        else
            false
    println("Open: $status")
}


fun main() = isOpen1(6)