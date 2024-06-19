package org.example.atomic30.extensionfunctions

import atomictest.eq

fun main() {
    "Single".singleQuote() eq "'Single'"
    "Double".doubleQuote() eq "\"Double\""
}
