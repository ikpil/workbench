package org.example.atomic62.polymophism

import atomictest.eq

open class Pet {
    open fun speek() = "Pet"
}

class Dog : Pet() {
    override fun speek() = "Bark!"
}

class Cat : Pet() {
    override fun speek() = "Meow"
}

fun talk(pet: Pet) = pet.speek()

fun main() {
    talk(Dog()) eq "Bark!"
    talk(Cat()) eq "Meow"
}
