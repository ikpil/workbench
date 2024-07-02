package org.example.atomic45.introgenerics

import atomictest.eq

class Dog {
    fun bark() = "Ruff!"
}

fun <T> identify(arg: T): T = arg

fun main() {
    identify("Yellow") eq "Yellow"
    identify(1) eq 1
    val d: Dog = identify(Dog())
    d.bark() eq "Ruff!"
}