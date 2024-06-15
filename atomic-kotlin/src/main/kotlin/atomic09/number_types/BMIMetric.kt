package org.example.atomic09.number_types

fun bmiMetric(
    weight: Double,
    height: Double
): String {
    val bmi = weight / (height * height)
    return if (bmi < 18.5) {
        "Underweight"
    } else if (bmi < 25) {
        "Normal weight"
    } else {
        "Overweight"
    }
}

fun main() {
    val weight = 72.57
    val height = 1.727
    val status = bmiMetric(weight, height)
    println(status)
}