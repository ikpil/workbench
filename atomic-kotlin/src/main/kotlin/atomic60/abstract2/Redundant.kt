package org.example.atomic60.abstract2

interface Redundant {
    abstract val x: Int
    abstract fun f(): Int
    abstract fun g(n: Double)
}

interface Removed {
    val x: Int
    fun f(): Int
    fun g(n: Double)
}