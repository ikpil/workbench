package org.example.atomic047.memberreferences

import atomictest.eq

fun main() {
    val messages = listOf(
        Message("kitty", "Hey!", true),
        Message("kitty", "Where are you?", false),
        Message("Boss", "Meeting today", false)
    )
    messages.sortedWith(
        compareBy(
            Message::isRead, Message::sender
        )
    ) eq listOf(
        Message("Boss", "Meeting today", false),
        Message("kitty", "Where are you?", false),
        Message("kitty", "Hey!", true),
    )
}
