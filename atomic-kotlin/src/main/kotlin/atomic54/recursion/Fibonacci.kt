package org.example.atomic54.recursion

import atomictest.eq

fun fibonacci2(n: Long): Long {
    tailrec fun fibonacci(n: Long, current: Long, next: Long): Long {
        if (n == 0L) return current
        return fibonacci(n - 1, next, current + next)
    }
    return fibonacci(n, 0L, 1L)
}

fun main() {
    (0L..8L).map { fibonacci2(it) } eq "[0, 1, 1, 2, 3, 5, 8, 13, 21]"
    fibonacci2(22) eq 17711
    fibonacci2(50) eq 12586269025
    fibonacci2(9999999999L) eq 2225628016866617058
}