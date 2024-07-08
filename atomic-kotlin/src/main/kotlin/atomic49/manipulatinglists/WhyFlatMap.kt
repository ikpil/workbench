package org.example.atomic49.manipulatinglists

import atomictest.eq

class Book(val title: String, val authros: List<String>)

fun main() {
    val books = listOf(
        Book("1984", listOf("George Orwell")),
        Book("Ulysses", listOf("James Joyce")),
    )

    books.map { it.authros }.flatten() eq listOf("George Orwell", "James Joyce")
    books.flatMap { it.authros } eq listOf("George Orwell", "James Joyce")
}
