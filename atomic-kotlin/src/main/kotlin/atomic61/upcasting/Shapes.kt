package org.example.atomic61.upcasting

import atomictest.trace

interface Shape {
    fun draw(): String
    fun erase(): String
}

class Circle : Shape {
    override fun draw() = "Circle.draw"
    override fun erase() = "Circle.erase"
}

class Square : Shape {
    override fun draw() = "Square.draw"
    override fun erase() = "Square.erase"
}

class Triangle : Shape {
    override fun draw() = "Triangle.draw"
    override fun erase() = "Triangle.erase"
}

fun show(shape: Shape) {
    trace("Show: ${shape.draw()}")
}

fun main() {
    listOf(Circle(), Square(), Triangle()).forEach(::show)
    trace eq """
        Show: Circle.draw
        Show: Square.draw
        Show: Triangle.draw
    """.trimIndent()
}