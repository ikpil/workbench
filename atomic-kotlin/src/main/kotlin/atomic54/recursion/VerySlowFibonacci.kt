package org.example.atomic54.recursion

import atomictest.eq

fun fibonacci(n: Long): Long {
    return when(n) {
        0L -> 0
        1L -> 1
        else -> fibonacci(n - 1) + fibonacci(n - 2)
    }
}

fun main() {
    fibonacci(0) eq 0
    fibonacci(22) eq 17711
    //fibonacci(50) eq 17711
}