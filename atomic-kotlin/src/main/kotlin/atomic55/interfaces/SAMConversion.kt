package org.example.atomic55.interfaces

import atomictest.trace

fun interface Action {
    fun act()
}

fun delayAction(action: Action) {
    trace("Delaying...")
    action.act()
}

fun main() {
    delayAction { trace("Hey!") }
    trace eq "Delaying... Hey!"
}