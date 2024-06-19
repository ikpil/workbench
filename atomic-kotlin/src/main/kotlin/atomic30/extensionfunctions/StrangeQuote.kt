package org.example.atomic30.extensionfunctions

import atomictest.eq

fun String.strangeQuote() = this.singleQuote().singleQuote()
fun String.tooManyQuotes() = this.doubleQuote().doubleQuote()

fun main() {
    "Hi".strangeQuote() eq "''Hi''"
    "Hi".tooManyQuotes() eq "\"\"Hi\"\""
}