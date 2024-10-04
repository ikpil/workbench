package org.example.atomic72.companionobjects

import atomictest.trace

class CompanionInit {
    init {
        trace("CompanionInit Constructor")
    }
    companion object {
        init {
            trace("Companion Constructor")
        }
    }
}

fun main() {
    trace("Before")
    CompanionInit()
    trace("After 1")
    CompanionInit()
    trace("After 2")
    CompanionInit()
    trace("After 3")
    trace eq """
    Before
    Companion Constructor
    CompanionInit Constructor
    After 1
    CompanionInit Constructor
    After 2
    CompanionInit Constructor
    After 3
  """
}