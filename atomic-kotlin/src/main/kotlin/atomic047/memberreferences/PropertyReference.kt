package org.example.atomic047.memberreferences

import atomictest.eq


data class Message(val sender: String, val text: String, val isRead: Boolean)

fun main() {
    val messages = listOf(
        Message("kitty", "Hey!", true),
        Message("kitty", "Where are you?", false))
    val unread = messages.filterNot(Message::isRead)
    unread.size eq 1
    unread.single().text eq "Where are you?"
}