package com.ikpil.scala.sample

// 파리미터 필드 정의
class ArrayElement(val contents: Array[String]) extends Element {
  override def demo(): Unit = {
    println("ArrayElement's demo")
  }
}
