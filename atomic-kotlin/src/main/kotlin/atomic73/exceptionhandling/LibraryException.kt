package org.example.atomic73.exceptionhandling

import atomictest.eq

fun testCode(code: Int) {
    if (code <= 1000) {
        throw IllegalArgumentException("'code' must be > 1000: $code")
    }
}

fun main() {
    try {
        testCode("A1".toInt(16))
    } catch (e: IllegalArgumentException) {
        e.message eq "'code' must be > 1000: 161"
    }

    try {
        testCode("0".toInt(1))
    } catch (e: IllegalArgumentException) {
        e.message eq "radix 1 was not in valid range 2..36"
    }
}