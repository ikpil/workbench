package org.example.atomic65.classdelegation

import atomictest.eq

class ExplicitControls : Controls {
    private val controls = SpaceShipControls()
    override fun up(velocity: Int) = controls.up(velocity)
    override fun down(velocity: Int) = controls.down(velocity)
    override fun left(velocity: Int) = controls.left(velocity)
    override fun right(velocity: Int) = controls.right(velocity)
    override fun forward(velocity: Int) = controls.forward(velocity)
    override fun back(velocity: Int) = controls.back(velocity)
    override fun turboBoost() = controls.turboBoost() + "... boooooost!"
}

fun main() {
    val controls = ExplicitControls()
    controls.forward(100) eq "forward 100"
    controls.turboBoost() eq "turbo boost... boooooost!"
}