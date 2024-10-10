package org.example.atomic73.exceptionhandling

import atomictest.eq

fun testCatchOrder(which: Int) =
    try {
        toss(which)
    } catch (e: Exception2) { // <-- 핸들러가 차례로 정의될 경우, Exception3 는 Exception2 로 구현했으므로, 여기에서 잡힌다.
        "Handler for Exception2 got ${e.message}"
    } catch (e: Exception3) {
        "Handler for Exception3 got ${e.message}"
    }

fun main() {
    testCatchOrder(2) eq "Handler for Exception2 got Exception 2"
    testCatchOrder(3) eq "Handler for Exception2 got Exception 3"
}
