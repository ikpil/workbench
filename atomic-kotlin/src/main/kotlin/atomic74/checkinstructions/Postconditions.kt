package org.example.atomic74.checkinstructions

import atomictest.capture

val resultFile = DataFile("Results.txt")

fun createResultFile(create: Boolean) {
    if (create)
        resultFile.writeText("Results\n# ok")
    // ... 다른 실행 경로들

    // assert 를 사용할 수 있지만, require(), check() 쪽을 더 선호 된다고 책에 쓰여 있다.
    check(resultFile.exists()) {
        "${resultFile.name} doesn't exist!"
    }
}

fun main() {
    resultFile.erase()
    capture {
        createResultFile(false)
    } eq "IllegalStateException: Results.txt doesn't exist!"
    createResultFile(true)
}