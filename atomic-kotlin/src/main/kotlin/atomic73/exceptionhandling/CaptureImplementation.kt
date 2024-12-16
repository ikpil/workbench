package org.example.atomic73.exceptionhandling

import atomictest.CapturedException

fun capture(f: () -> Unit): CapturedException =
    try {
        f()
        CapturedException(null, "<Error>: Expected an exception")
    } catch (e: Throwable) {
        CapturedException(
            e::class,
            if (e.message != null)
                ": ${e.message}"
            else ""
        )
    }

fun main() {
    capture {
        throw Exception("!!!")
    } eq "Exception: !!!"
    capture {
        1
    } eq "<Error>: Expected an exception"
}