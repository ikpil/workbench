package org.example.atomic69.nestedclasses

import atomictest.eq
import org.example.atomic69.nestedclasses.Airport.Plane

class Airport(private val code: String) {
    open class Plane {
        // Can access private properties:
        fun contact(airport: Airport) = "Contacting ${airport.code}"
    }

    private class PrivatePlane : Plane()

    fun privatePlane(): Plane = PrivatePlane()
}

fun main() {
    val denver = Airport("DEN")
    var plane = Plane()                   // [1]
    plane.contact(denver) eq "Contacting DEN"
    // Can't do this:
    // val privatePlane = Airport.PrivatePlane()
    val frankfurt = Airport("FRA")
    plane = frankfurt.privatePlane()
    // Can't do this:
    // val p = plane as PrivatePlane      // [2]
    plane.contact(frankfurt) eq "Contacting FRA"
}