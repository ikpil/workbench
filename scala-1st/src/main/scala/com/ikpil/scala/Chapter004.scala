package com.ikpil.scala

import com.ikpil.scala.sample.ChecksumAccumulator

// 이 튜토리얼은 클래스와 객체에 대해서 알아 본다
object Chapter004 extends App {
  sampleClass()
  sampleStandalone()

  // 클래스를 생성하는 방법을 다룬다
  def sampleClass(): Unit = {
    val acc = new ChecksumAccumulator()
    acc.add(3)

    // 컴파일 불가, val 이기 때문
    // acc = new ChecksumAccumulator();

    val csa = new ChecksumAccumulator()

    println(acc.checksum())
    println(csa.checksum())
  }

  def sampleStandalone(): Unit = {
    for (s <- Array("test1", "test2", "test1"))
      println(ChecksumAccumulator.calculate(s))
  }
}
