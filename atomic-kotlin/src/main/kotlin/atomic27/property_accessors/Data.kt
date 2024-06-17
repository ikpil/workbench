package org.example.atomic27.property_accessors

import atomictest.eq

class Data(var i: Int)

fun main() {
    val data = Data(10)
    data.i eq 10
    data.i = 20
}