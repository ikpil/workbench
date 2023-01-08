package com.ikpil.scala

import java.io.{File, PrintWriter}
import java.util.Date

// 흐름제어 추상화, 더블어 커링과 이름에 의한 호출 파라미터를 살펴 본다
object Chapter009 extends App {
  var assertionsEnable: Boolean = false

  higherOrderFunction()
  curryingTest()
  flowControl()
  callParametersByName()

  def containNeg(nums: List[Int]): Boolean = {
    for (num <- nums)
      if (num < 0)
        return true

    false
  }

  def higherOrderFunction(): Unit = {
    val a1 = containNeg(List(1, 2, 3, 4))
    println("contain neg - a1:" + a1)

    val a2 = containNeg(List(1, 2, -3, 4))
    println("contain neg - a2:" + a2)

    val a3 = List(1, 2, 3, 4).exists(_ < 0)
    println("negative exists - a3:" + a3)

    val a4 = List(1, 2, 3, 4).exists(_ % 2 == 1)
    println("odd exists - a4:" + a4)
  }

  def plainOldSum(x: Int, y: Int) = x + y

  def curriedSum(x: Int)(y: Int) = x + y

  def curryingTest(): Unit = {
    val a1 = plainOldSum(1, 1)
    println("plainOldSum : " + a1)

    /**
     * Currying은 여러 개의 인자를 가진 함수를 호출 할 경우,
     * 파라미터의 수보다 적은 수의 파라미터를 인자로 받으면 누락된 파라미터를 인자로 받는 기법을 말한다.
     * 즉 커링은 함수 하나가 n개의 인자를 받는 과정을 n개의 함수로 각각의 인자를 받도록 하는 것이다.
     */
    val a2 = curriedSum(1)(1)
    println("curriedSum: " + a2)

    val onePlus = curriedSum(1) _
    val a3 = onePlus(4)
    println("curriedSum: " + a3)
  }

  def withPrintWriter(file: File, op: PrintWriter => Unit): Unit = {
    val writer = new PrintWriter(file)
    try {
      op(writer)
    } finally {
      writer.close()
    }
  }

  def curriedWithPrintWriter(file: File)(op: PrintWriter => Unit): Unit = {
    val writer = new PrintWriter(file)
    try {
      op(writer)
    } finally {
      writer.close()
    }
  }

  def flowControl(): Unit = {
    // 일반적인 방법
    withPrintWriter(
      new File("tmp.date.txt"),
      writer => writer.println(new Date)
    )

    // 커링을 사용한 방법
    val a1 = curriedWithPrintWriter(new File("tmp.date2.txt")) _
    a1(writer => writer.println(new Date))

    // 커링을 중괄호로 묵어서 더 편하게 사용한 방법
    val f3 = new File("tmp.date3.txt")
    curriedWithPrintWriter(f3) {
      writer => writer.println(new Date)
    }
  }


  // 이름에 의한 호출
  def callParametersByName(): Unit = {
    val x = 5

    // myAssert 와 byNameAssert 의 호출 형태는 동일하나, 작동 방식에 큰 차이를 보인다
    try {
      myAssert(x / 0 == 0)
    } catch {
      case exception: Exception => {
        println(exception)
      }
    }

    byNameAssert(x / 0 == 0)
  }

  // myAssert 는 1. x / 0 == 0 을 평가한 뒤 2. myAssert 를 호출한다.
  def myAssert(predicate: Boolean): Unit = {
    if (assertionsEnable && !predicate)
      throw new AssertionError()
  }

  // byNameAssert 는 1. byNameAssert 호출 후 2. 필요할 때 predicate 서플라이 함수를 호출 한다.
  def byNameAssert(predicate: => Boolean): Unit = {
    if (assertionsEnable && !predicate)
      throw new AssertionError()
  }
}

