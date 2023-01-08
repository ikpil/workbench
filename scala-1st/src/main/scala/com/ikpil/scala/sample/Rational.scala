package com.ikpil.scala.sample

class Rational(n: Int, d: Int) {
  // 선결 조건을 추가하여, d가 0이 아님을 보장한다.
  require(d != 0)

  private val g = gcd(n.abs, d.abs)
  val number: Int = n / g
  val denom: Int = d / g

  def this(n: Int) = this(n, 1) // 보조 생성자

  override def toString = number + "/" + denom

  // add 이름을 + 연사자 메소드로 이름을 변경했다.
  def +(that: Rational): Rational = new Rational(
    number * that.denom + that.number * denom,
    denom * that.denom
  )

  def +(i: Int): Rational = new Rational(
    number + i * denom,
    denom
  )

  def -(that: Rational): Rational = new Rational(
    number * that.denom - that.number * denom,
    denom * that.denom
  )

  def -(i: Int): Rational = new Rational(
    number - i * denom,
    denom
  )

  def *(that: Rational): Rational = new Rational(
    number * that.number, denom * that.denom
  )

  def *(i: Int): Rational = new Rational(
    number * i, denom
  )

  def /(that: Rational): Rational = new Rational(
    number * that.denom, denom * that.number
  )

  def /(i: Int): Rational = new Rational(
    number, denom * i
  )

  def lessThan(that: Rational): Boolean =
    number * that.denom < that.number * denom

  private def gcd(a: Int, b: Int): Int =
    if (b == 0) a else gcd(b, a % b)
}
