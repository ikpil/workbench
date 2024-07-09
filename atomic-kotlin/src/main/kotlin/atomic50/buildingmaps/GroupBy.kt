package org.example.atomic50.buildingmaps

import atomictest.eq

fun main() {
    val map: Map<Int, List<Person>> = people().groupBy(Person::age)
    map[15] eq listOf(Person("Arthricia", 15))
    map[21] eq listOf(Person(name="Alice", age=21), Person(name="Charlie", age=21), Person(name="Franz", age=21))
}