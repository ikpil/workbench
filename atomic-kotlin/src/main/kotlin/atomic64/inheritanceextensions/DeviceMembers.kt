package org.example.atomic64.inheritanceextensions

import atomictest.eq

interface Device {
    val model: String
    val productionYear: Int
    fun overpriced() = model.startsWith("i")
    fun outdated() = productionYear < 2050
}


class MyDevice(override val model: String, override val productionYear: Int) : Device

fun main() {
    val gadget: Device = MyDevice("my first phone", 2000)
    gadget.outdated() eq true
    gadget.overpriced() eq false
}
