package org.example.atomic64.inheritanceextensions

import atomictest.trace

open class MyClass {
    fun g() = trace("g()")
    fun h() = trace("h()")
}

fun useMyClass(mc: MyClass) {
    mc.g()
    mc.h()
}

class MyClassAdaptedForLib: MyClass(), LibType {
    override fun f1() = h()
    override fun f2() = g()
}

fun main() {
    val mc = MyClassAdaptedForLib()
    utility1(mc)
    utility2(mc)
    useMyClass(mc)
    trace eq """
        h()
        g()
        g()
        h()
        g()
        h()
    """.trimIndent()
}