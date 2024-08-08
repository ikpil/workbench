package org.example.atomic69.nestedclasses

fun localClasses() {
    open class Amphibian
    class Frog : Amphibian()

    val amphibian: Amphibian = Frog()
}