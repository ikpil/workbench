package org.example.atomic31.namedanddefaultrgs

import atomictest.eq

fun color(red: Int, green: Int, blue: Int) = "($red, $green, $blue)"

fun main() {
    color(1, 2, 3) eq "(1, 2, 3)"
    color(red = 76, green = 89, blue = 0) eq "(76, 89, 0)";
    color(52, 34, blue = 0) eq "(52, 34, 0)"
}