package org.example.atomic68.typechking2

import atomictest.eq
import org.example.atomic68.typechking.name

sealed class Shape {
    fun draw() = "$name: Draw"
}

class Circle : Shape()

class Square : Shape() {
    fun rotate() = "Square: Rotate"
}

class Triangle : Shape() {
    fun rotate() = "Triangle: Rotate"
}

fun turn(s: Shape) = when (s) {
    is Circle -> ""
    is Square -> s.rotate()
    is Triangle -> s.rotate()
}

fun main() {
    val shapes = listOf(Circle(), Square())
    shapes.map { it.draw() } eq
            "[Circle: Draw, Square: Draw]"
    shapes.map { turn(it) } eq
            "[, Square: Rotate]"
}