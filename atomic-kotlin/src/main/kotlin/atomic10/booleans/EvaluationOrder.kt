package org.example.atomic10.booleans

fun main() {
    val sunny = true
    val hoursSleep = 6
    val exercise = false
    val temp = 55

    // first $$
    // second ||
    val happy1 = sunny && temp > 50 || exercise && hoursSleep > 7
    println(happy1)

    val sameHappy1 = (sunny && temp > 50) || (exercise && hoursSleep > 7)
    println(sameHappy1)

    val notSame = (sunny && temp > 50 || exercise) && hoursSleep > 7
    println(notSame)
}