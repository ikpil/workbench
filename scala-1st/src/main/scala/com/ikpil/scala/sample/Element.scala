package com.ikpil.scala.sample

// 추상 클래스
abstract class Element {
  def contents: Array[String]

  def height: Int = contents.length // 파라미터 없는 메소드

  def width: Int = if (height == 0) 0 else contents(0).length

  def demo(): Unit = {
    println("Element's demo")
  }

  def above(that: Element): Element = {
    new ArrayElement(this.contents ++ that.contents)
  }

  def beside(that: Element): Element = {
    new ArrayElement(
      for (
        (line1, line2) <- this.contents zip that.contents
      ) yield line1 + line2
    )
  }

  override def toString = contents mkString "\n"
}
