package com.ikpil.scala

import scala.collection.immutable.HashSet
import scala.collection.mutable
import scala.io.Source

// 이 튜토리얼은 스칼라에서 배열/리스트/튜플/집합/맵등을 어떻게 사용하는지 알아 본다
object Chapter003 {
  def main(args: Array[String]): Unit = {
    immutableSet()
    mutableSet()

    immutableMap()
    mutableMap()

    printArgs(Array("1", "2", "3"))
    println(formatArgs(Array("6", "7", "8", "9")))

    printFile("./README.md")
    printFile2("./README.md")
  }

  // 변경 불가능한 컬렉션
  def immutableSet(): Unit = {
    var jetSet = HashSet("Boeing", "Airbus")
    jetSet += "Lear"
    //jetSet.+=("Lear")
    println(jetSet)
  }

  // 변경 가능한 컬랙션
  def mutableSet(): Unit = {
    val movieSet = mutable.Set("Hitch", "Poltergeist")
    movieSet += "Shrek"
    //movieSet.+=("Shrek")
    println(movieSet)
  }

  // 변경 불가능 맵
  def immutableMap(): Unit = {
    val test = Map(
      1 -> "Go to island.",
      2 -> "Find big X on ground.",
      3 -> "Dig",
    )

    //test += (4 -> "test")

    println(test(1))
  }

  // 변경 가능한 맵
  def mutableMap(): Unit = {
    val test = mutable.Map[Int, String]();
    test += (1 -> "Go to island.")
    test += (2 -> "Find big X on ground.")
    test += (3 -> "Dig")

    println(test(2))
  }

  // 반복문을 돌 릴 경우, 스칼라가 권장하는 형태
  def printArgs(args: Array[String]): Unit = {
    // 전통적인 방법
    {
      var i = 0;
      while (i < args.length) {
        println(args(i))
        i += 1;
      }
    }

    // 스칼라 같은 함수형 언어에서 방법 1
    for (arg <- args)
      println(arg)

    // 또는 방법 2
    args.foreach(println)
  }

  // printArgs 의 경우, println 의 사이드이펙트를 가지고 있다.
  // 사이드 이펙트를 더 줄이려면, 연산 결과를 리턴하여 처리해야 한다.
  def formatArgs(args: Array[String]) = args.mkString("\n")

  // 파일을 출력하는 정통적인 방법
  def printFile(args: String): Unit = {
    val lines = Source.fromFile(args).getLines().toList

    var maxWidth = 0
    for (line <- lines)
      maxWidth = maxWidth.max(widthOfLength(line))

    for (line <- lines) {
      val numSpaces = maxWidth - widthOfLength(line)
      val padding = " " * numSpaces
      println(padding + line.length + " | " + line)
    }
  }

  // 파일을 출력하는 함수형적인 방법
  def printFile2(args: String): Unit = {
    val lines = Source.fromFile(args).getLines().toList

    val longestLine = lines.reduceLeft(
      (a, b) => if (a.length > b.length) a else b
    )

    val maxWidth = widthOfLength(longestLine)

    for (line <- lines) {
      val numSpaces = maxWidth - widthOfLength(line)
      val padding = " " * numSpaces
      println(padding + line.length + " | " + line)
    }
  }

  def widthOfLength(s: String) = s.length.toString.length
}
