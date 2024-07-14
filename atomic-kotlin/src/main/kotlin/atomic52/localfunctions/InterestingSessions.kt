package org.example.atomic52.localfunctions

import atomictest.eq

fun main() {
    sessions.any(
        fun(session: Session): Boolean {
            if (session.title.contains("Kotlin") && session.speeker in favoriteSpeakers) {
                return true
            }
            return false
        }) eq true
}