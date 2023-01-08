package com.ikpil.scala

import com.ikpil.scala.sample.Rational

// 함수형 객체에 대해서 알아본다
object Chapter006 extends App {
  // 암시적 타입 변환
  implicit def intToRational(x: Int): Rational = new Rational(x)

  val x = new Rational(1, 2)
  println(x)

  val y = new Rational(2, 3)
  println(y)


  println("연산자 정의가 작동 되는지 확인")
  val z1 = x + y
  val z2 = x * y
  println(z1)
  println(z2)

  println("연산자 우선 순위 확인")
  val z3 = (z1 + z2) * x
  val z4 = z1 + z2 * x
  println(z3)
  println(z4)

  println("암시적 타입 변환 테스트")
  val z5 = 2 + y
  println(z5)

  println("클래스 메소드 호출 확인")
  val c = x lessThan y
  println(c)

  val three = new Rational(3);
  println(three)

  val gcd = new Rational(66, 42)
  println(gcd)
}
