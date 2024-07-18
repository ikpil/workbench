package org.example.atomic57.secondarycnstructors

import atomictest.trace

class WithSecondary(i: Int) {
    init {
        trace("Primary: $i")
    }
    constructor(c: Char) : this(c - 'A') {
        trace("Secondary: '$c'")
    }
    constructor(s: String) : this(s.first()) {
        trace("Secondary: \"$s\"")
    }
}

fun main() {
    fun sep() = trace("-".repeat(10))
    WithSecondary(1)
    sep()
    WithSecondary('D')
    sep()
    WithSecondary("Last Constructor")
    trace eq """
        Primary: 1
        ----------
        Primary: 3
        Secondary: 'D'
        ----------
        Primary: 11
        Secondary: 'L'
        Secondary: "Last Constructor"
    """
}