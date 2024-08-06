package org.example.atomic66.downcasting

fun main() {
    val b1: Base = Derived1()
    if (b1 is Derived1)
        b1.g()
    val b2: Base = Derived2()
    if (b2 is Derived2)
        b2.h()
}