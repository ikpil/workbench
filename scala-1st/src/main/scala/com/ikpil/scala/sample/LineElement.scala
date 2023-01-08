package com.ikpil.scala.sample

// 슈퍼클래스의 생성자 호출을 보여준다.
class LineElement(s: String) extends Element {
  val contents: Array[String] = Array(s);

  // width, height 에 override
  override def width: Int = s.length

  override def height: Int = 1

  // 스칼라에서는 override 유무가 필수이므로, 깨지기 쉬운 기반 클래스(fragile base class) 문제를 조금이나마 해소시켜준다
  //override def abcde: Int = 2

  override def demo(): Unit = {
    println("LineElement's demo")
  }

}
