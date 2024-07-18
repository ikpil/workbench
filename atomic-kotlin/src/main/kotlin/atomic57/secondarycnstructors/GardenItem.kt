package org.example.atomic57.secondarycnstructors

import atomictest.eq

enum class Material {
    Ceramic, Metal, Plastic
}

class GardenItem(val name: String) {
    var material: Material = Material.Plastic

    constructor(name: String, material: Material) : this(name) {
        this.material = material
    }

    constructor(material: Material) : this("Strange Thing", material)

    override fun toString() = "$material $name"
}

fun main() {
    GardenItem("Elf").material eq Material.Plastic
    GardenItem("Snowman").name eq "Snowman"
    GardenItem("Gazing Ball", Material.Metal) eq "Metal Gazing Ball"
    GardenItem(material = Material.Ceramic) eq "Ceramic Strange Thing"
}
