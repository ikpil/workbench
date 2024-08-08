package org.example.atomic70.objects

import atomictest.eq

fun g() {
    Shared.i += 7
}

fun main() {
    f()
    g()
    Shared.i eq 12
}