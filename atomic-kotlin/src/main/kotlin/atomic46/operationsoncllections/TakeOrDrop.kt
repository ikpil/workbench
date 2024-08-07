package org.example.atomic46.operationsoncllections

import atomictest.eq

fun main() {
    val list = listOf('a', 'b', 'c', 'X', 'Z')
    list.takeLast(3) eq "[c, X, Z]"
    list.takeLastWhile { it.isUpperCase() } eq "[X, Z]"

    list.drop(1) eq "[b, c, X, Z]"
    list.dropWhile { it.isLowerCase() } eq "[X, Z]"
}