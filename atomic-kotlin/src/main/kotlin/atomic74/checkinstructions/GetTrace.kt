package org.example.atomic74.checkinstructions

import atomictest.capture
import atomictest.eq

fun getTrace(fileName: String): List<String> {
    require(fileName.startsWith("file_")) {
        "$fileName must start with 'file_'"
    }
    val file = DataFile(fileName)
    require(file.exists()) {
        "$fileName doesn't exist"
    }

    val lines = file.readLines()
    require(lines.isNotEmpty()) {
        "$fileName is empty"
    }
    return lines
}

fun main() {
    DataFile("file_empty.txt").writeText("")
    DataFile("file_wubba.txt").writeText("wubba lubba dub dub")
    capture {
        getTrace("wrong_name.txt")
    } eq "IllegalArgumentException: wrong_name.txt must start with 'file_'"

    capture {
        getTrace("file_nonexistent.txt")
    } eq "IllegalArgumentException: file_nonexistent.txt doesn't exist"

    capture {
        getTrace("file_empty.txt")
    } eq "IllegalArgumentException: file_empty.txt is empty"

    getTrace("file_wubba.txt") eq "[wubba lubba dub dub]"
}

