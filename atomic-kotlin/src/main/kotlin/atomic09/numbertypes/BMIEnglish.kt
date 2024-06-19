package org.example.atomic09.numbertypes

fun bmiEnglish (
    weight: Int,
    height:Int
): String {
    val bmi = weight / (height * height) * 703.07
    return if (bmi < 18.5) {
        "Underweight"
    } else if (bmi < 25) {
        "Normal weight"
    } else {
        "Overweight"
    }
}

fun main() {
    val weight = 68
    val height = 168
    val status = bmiEnglish(weight, height)
    println(status)
}