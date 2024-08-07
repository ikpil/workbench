package org.example.atomic68.typechking

import atomictest.eq

interface Shape {
    fun draw(): String
}

class Circle : Shape {
    override fun draw() = "Circle: Draw"
}

class Square : Shape {
    override fun draw() = "Square: Draw"
    fun rotate() = "Square: Rotate"
}

fun turn(s: Shape) = when (s) {
    is Square -> s.rotate()
    else -> ""
}

fun main() {
    val shapes = listOf(Circle(), Square())
    shapes.map { it.draw() } eq
            "[Circle: Draw, Square: Draw]"
    shapes.map { turn(it) } eq
            "[, Square: Rotate]"
}