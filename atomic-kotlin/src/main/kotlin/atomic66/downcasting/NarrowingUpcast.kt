package org.example.atomic66.downcasting

interface Base {
    fun f()
}

class Derived1 : Base {
    override fun f() {}
    fun g() {}
}

class Derived2 : Base {
    override fun f() {}
    fun h() {}
}

fun main() {
    val b1: Base = Derived1()
    b1.f()
    val b2: Base = Derived2()
    b2.f()
}