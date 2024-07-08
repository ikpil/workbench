package org.example.atomic49.manipulatinglists

import atomictest.eq

data class Person(val name: String, val id: Int)

fun main() {
    val names = listOf("Bob", "Jill", "Jim")
    val ids = listOf(1731, 9274, 8378)
    names.zip(ids, ::Person) eq "[Person(name=Bob, id=1731), Person(name=Jill, id=9274), Person(name=Jim, id=8378)]"

}