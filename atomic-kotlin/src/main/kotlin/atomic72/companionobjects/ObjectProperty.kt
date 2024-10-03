package org.example.atomic72.companionobjects

import atomictest.eq

class WithObjectProperty {
    companion object {
        private var n: Int = 0
    }

    fun increment() = ++n
}

fun main() {
    val a = WithObjectProperty()
    var b = WithObjectProperty()
    a.increment() eq 1
    b.increment() eq 2
    a.increment() eq 3
}