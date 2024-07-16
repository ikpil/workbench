package org.example.atomic54.recursion

fun recurse(i: Int): Int = recurse(i + 1)

fun main() {
    //recurse(1)
}