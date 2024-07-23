package org.example.atomic63.composition

interface Building
interface Kitchen

interface House: Building {
    val kitchen: Kitchen
}
