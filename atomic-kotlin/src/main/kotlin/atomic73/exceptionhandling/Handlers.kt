package org.example.atomic73.exceptionhandling

import atomictest.eq

fun toss(which: Int) = when (which) {
    1 -> throw Exception1(1)
    2 -> throw Exception2("Exception 2")
    3 -> throw Exception3("Exception 3")
    else -> "OK"
}

fun test(which: Int): Any? =
    try {
        toss(which)
    } catch (e: Exception1) {
        e.value
    } catch (e: Exception2) {
        e.message
    } catch (e: Exception3) {
        e.message
    }

fun main() {
    test(0) eq "OK"
    test(1) eq 1
    test(2) eq "Exception 2"
    test(3) eq "Exception 3"
}
