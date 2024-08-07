package org.example.atomic64.inheritanceextensions

import atomictest.eq

class HVAC : Heater() {
    fun cool(temperature: Int) = "cooling to $temperature"
}

fun warmAndCool(hvac: HVAC) {
    hvac.heat(70) eq "heating to 70"
    hvac.cool(60) eq "cooling to 60"
}

fun main() {
    val heater = Heater()
    val hvac = HVAC()
    warm(heater)
    warm(hvac)
    warmAndCool(hvac)
}