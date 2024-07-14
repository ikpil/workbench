package org.example.atomic52.localfunctions

import atomictest.eq

fun main() {
    fun interesting(session: Session): Boolean {
        if (session.title.contains("Kotlin") && session.speeker in favoriteSpeakers) {
            return true
        }
        return false
    }
    sessions.any(::interesting) eq true
}