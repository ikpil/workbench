package org.example.atomic54.recursion

import atomictest.eq

private tailrec fun sum(n: Long, accumulator: Long): Long {
    if (n == 0L) return accumulator
    return sum(n - 1, accumulator + n)
}

fun sum2(n: Long) = sum(n, 0)

fun main() {
    sum(2, 0) eq 3
    sum2(1000) eq 500500
    sum2(100000) eq 5000050000
}