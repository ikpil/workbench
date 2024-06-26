package org.example.atomic35.dataclasses

import atomictest.eq
import atomictest.neq

class Person(val name: String)

data class Contact(val name: String, val number: String)

fun main() {
    Person("Cleo") neq Person("Cleo")
    Contact("Miffy", "1-234-567890") eq Contact("Miffy", "1-234-567890")
}
