package com.ikpil.scala

import com.ikpil.scala.sample.{ArrayElement, Element, LineElement, UniformElement}

/** *
 * 챕터 10에서는 상속과 구성에 대해서 다룬다
 */
object Chapter010 extends App {
  classExtendTest()

  def classExtendTest(): Unit = {
    // 생성
    val a1 = new ArrayElement(Array("hello", "world"))
    println(s"array element - w:${a1.width} h:${a1.height}")

    // 서브클래스로 받는다
    val a2: Element = a1
    println(s"element - w:${a2.width} h:${a2.height}")

    // 1줄짜리 엘러먼트
    val a3 = new LineElement("hehe")
    println(s"line element - w:${a3.width} h:${a3.height}")

    val e3: Element = new UniformElement('x', 2, 3);
    println(s"uniform element - w:${e3.width} h:${e3.height}")

    demo(a1)
    demo(a3)
    demo(e3)
  }

  def demo(e: Element): Unit = {
    e.demo()
  }
}
