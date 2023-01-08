package com.ikpil.scala

import java.io.{File, IOException}
import scala.io.StdIn.readLine

// 스칼라의 내장 제어 구문에 대해서 배워본다
object Chapter007 extends App {
  //  differentIf()
  //  checkWhile(10, 30)
  //  checkDoWhile()
  //  differentAssign()
  // checkFor()
  // grep(".*gcd.*")
  // testException()
  //checkMatch(Array("hi"))
  //compareBreakOrContinue()
  println(multiTable())

  // 명령형 언어와 스칼라의 if 차이점
  def differentIf(): Unit = {
    val x = true
    val haha = "haha"
    val default = "default"

    // 명령형을 때, if 로 분기하여 다시 결정한다.
    {
      var y = default
      if (x) {
        y = haha
      }
      println(y)
    }

    // 스칼라에서는 if 가 값을 출력할 수 있다
    {
      val y = if (x) haha else default
      println(y)
    }

    // 그러므로 스칼라에서는 다음과 같은 코드가 가능하다
    // 이를 동일성 추론(equational reasoning)이라 하는데, 변수와 표현식이 동일하다는 뜻
    {
      println(if (x) haha else default)
    }
  }

  def checkWhile(x: Long, y: Long): Long = {
    var a = x
    var b = y
    while (a != 0) {
      val temp = a
      a = b % a
      b = temp
    }

    b
  }

  def checkDoWhile(): Unit = {
    var line = ""
    do {
      line = readLine()
      println("Read: " + line)
    } while (line != "")
  }

  def differentAssign(): Unit = {
    var count = 1
    var line = ""
    // 스칼라의 할당의 결과는 Unit 이기 때문에, 비교문이 성립이 안되서 무한 루프에 빠진다.
    while ((line = readLine()) != "" && count < 2) {
      println(line)
      count += 1
    }
  }

  def fileHere(): Array[File] = {
    new File("src/main/scala/com/ikpil/scala/").listFiles
  }

  def checkFor(): Unit = {
    {
      // <- 는 재너레이터라고 부르는 문법으로 fileHere 의 원소를 하나씩 꺼내 file 에 val 형태로 할당한다.
      for (file <- fileHere())
        println(file)

      // 필터링
      // for 는 표현식으로 불리는데, if 를 이용해서 원하는 값만 필터링 할 수 있드
      for (file <- fileHere() if file.getName.endsWith(".scala"))
        println("filtered - " + file)

      // 필터링 복수를 넣을 때
      for (file <- fileHere()
           if file.isFile
           if file.getName.endsWith(".scala")
           )
        println("filtered - " + file)
    }

    // Range 타입 x to y 로 사용됨
    {
      for (i <- 1 to 4)
        println("for 1 to 4 - " + i)

      for (i <- 1 until 4)
        println("for 1 until 4 - " + i)
    }

  }

  def fileLines(file: java.io.File): List[String] = {
    scala.io.Source.fromFile(file).getLines().toList
  }

  // 중첩 iterator 을 설명한다.
  def grep(pattern: String): Unit = {
    for (
      file <- fileHere() // fileHEre 을 통해 file 을 하나씩 꺼내고
      if file.getName.endsWith(".scala"); // .scala 파일을 필터링 하고(; 이 중요하다)
      line <- fileLines(file); // 파일 내용의 라인들을 하나씩 뽑아 line 에 넣고
      trimmed = line.trim // for 중에 변수를 바인딩 한다.
      if trimmed.matches(pattern) // line 에 pattern 이 있는지 필터링 하고
    ) println(file + ": ") + line.trim // 결과를 출력
  }

  // for() 을 했을 때, 순회를 했으나, for ~ yield 를 하면 새로운 컬렉션을 만들 수 있다
  def scalaFiles =
    for {
      file <- fileHere()
      if file.getName.endsWith(".scala")
    } yield file

  // for 을 이용하여 Array[File] map Array[Int] 하였다
  val forLineLengths =
    for {
      file <- fileHere()
      if file.getName.endsWith(".scala")
      line <- fileLines(file)
      trimmed = line.trim
      if trimmed.matches(".*for.*")
    } yield trimmed.length

  def testException(): Unit = {
    // 이 함수의 결과는 finally 가 있음에도 1이 될 것이다.
    val s = checkTryCatchFinally()
    println(s)
  }

  // 자바와 스칼라의 exception try-catch-finally 를 알 수 있다.
  def checkTryCatchFinally(): Int = {
    // 스칼라의 try-catch 의 결과는 값이다. 이 부분이 자바랑 다르다
    try {
      // try 의 결과는 값이다.
      throwException()
      1
    } catch {
      case ex: RuntimeException => 2
      case ex: IndexOutOfBoundsException => 3
      case ex: IOException => 4
    } finally {
      // 는 결과가 값이 아니다.
      2
    }

  }

  def throwException(): Unit = {
    // 예외 발생시키는 방법
    // throw new IllegalArgumentException
    val n = 4;
    if (n % 2 == 0)
      n / 2
    else
      throw new RuntimeException("n must be even") // 이 부분이 이상한데, 다른 언어에서는 리턴타입에 맞게 설정해야 하지만 스칼라에서는 Nothing 으로 처리 된다
  }

  // scala 에서 switch 가 match 로 대체되며
  // match 는 상수라면 문자열도 되는 것을 확인한다
  def checkMatch(args: Array[String]): Unit = {
    val firstArg = if (args.length > 0) args(0) else ""
    val friend = firstArg match {
      case "salt" => "paper"
      case "chips" => "salsa"
      case "eggs" => "bacon"
      case _ => "huh?"
    }

    println(friend)
  }

  def compareBreakOrContinue(): Unit = {
    val x = checkBreakOrContinue(Array("-ikpil.scala", "ikpil.scala"))

    val y = searchForm(Array("-ikpil.scala", "ikpil.scala"), 0)

    println("compare x:" + x + "y:" + y)
  }

  // scala 에서는 break, continue 가 없다. 그러므로 어떤 요소를 찾으면 탈출해야 할 때 다음 처럼 작성해야 한다.
  def checkBreakOrContinue(args: Array[String]): Int = {
    var i = 0
    var foundIt = -1

    while (i < args.length && 0 > foundIt) {
      if (!args(i).startsWith("-")) {
        if (args(i).endsWith(".scala"))
          foundIt = i
      }

      i = i + 1
    }

    foundIt
  }

  // checkBreakOrContinue 을 재귀로 만들면, 보다 편하게 코드 작성이 가능하다
  def searchForm(args: Array[String], i: Int): Int = {
    if (i >= args.length) -1
    else if (args(i).startsWith("-")) searchForm(args, i + 1)
    else if (args(i).endsWith(".scala")) i
    else searchForm(args, i + 1)
  }

  // 함수형 코드로 곱샘 테이블을 만들어 본다
  def makeRowSeq(row: Int) = {
    for (col <- 1 to 10) yield {
      val prod = (row * col).toString
      val padding = " " * (4 - prod.length)
      padding + prod
    }
  }

  // 하나의 행을 모두 문자열로 결합시킨다.
  def makeRow(row: Int) = makeRowSeq(row).mkString

  // 표를 한줄에 한 행의 내용을 담고 있는 문자열로 반환한다.
  def multiTable() = {
    val tableSeq =
      for (row <- 1 to 10)
        yield makeRow(row)

    tableSeq.mkString("\n");
  }

}
