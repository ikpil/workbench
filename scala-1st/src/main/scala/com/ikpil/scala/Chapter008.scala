package com.ikpil.scala

import com.ikpil.scala.sample.LongLines

// 함수와 클로저에 대해서 알아 본다
object Chapter008 extends App {

  // checkMethod()
  firstClassFunction()
  checkClosure()
  specialFunctionCall()
  tailRecursive()

  def checkMethod() = {
    val argsV2 = Array(
      "50",
      "src/main/scala/com/ikpil/scala/Tutorial007.scala",
      "src/main/scala/com/ikpil/scala/Tutorial008.scala"
    );

    val width = argsV2(0).toInt
    for (arg <- argsV2.drop(1))
      LongLines.processFile(arg, width)
  }

  def firstClassFunction() = {
    // 함수 리터럴, 함수 값은 변수에 저장할 수 있음을 보여 준다.
    var fn = (x: Int) => x + 1
    println(fn(10))

    fn = (x: Int) => x + 9999
    println(fn(1))

    fn = (x: Int) => {
      println("We")
      println("are")
      println("here!")
      x + 1
    }

    println(fn(1))

    // 모든 컬렉션에는 foreach 가 있음
    val someNumbers = List(-11, -10, -5, 0, 5, 10)
    someNumbers.foreach((x: Int) => println(x))

    // 모든 컬렉션에는 filter 가 있음
    val filtered = someNumbers.filter((x: Int) => x > 0)
    println(filtered)

    // 위치 표시자 문법, 파라미터에 대한 위치를 _ 로 표시 할 수 있다,
    // _ 는 위치 기반이고, 2개가 있다면, 두개의 파라미터를 받는 함수이다.
    val filtered2 = someNumbers.filter(_ > 0)
    println(filtered2)

    // 부분 적용 함수(partially applied function)
    // 전체 인자 목록에 대한 위치 표시자를 뜻하며, "함수명 _" 로 사용 된다.
    // 함수명 다음에 공백이 한칸 있어야 함을 주의 해야 한다.
    someNumbers.foreach(println _)


    // 다음은 sum 함수를 만든다.
    def sum(a: Int, b: Int, c: Int) = a + b + c

    // 다음처럼 사용하면, 함수를 변수로 만들수 있다
    // a는 함수 값 객체를 나타내며 캄퍼일러가 자동으로 클래스 인스턴스로 만든다.
    val a = sum _

    // 그러므로 함수 호출() -> apply() 호출 -> sum() 호출로 이어진다.
    println(a(1, 2, 3))

    // 이 클래스에는 apply 메소드가 있다. 이 apply 메소드와 sum 이 결합된다
    println(a.apply(1, 2, 3))

    // 다음 표현 으로 생성한 s1, s2 는 같은 결과를 나타낸다.
    // 함수 아규먼트 자리에 _ 가 있으면, 전부 부분 적용 함수로 컴파일러가 이해하기 때문이다.
    val s1 = (a1: Int) => sum(1, a1, 2)
    val s2 = sum(1, _: Int, 2)

    println(s1(4))
    println(s2(4))
  }

  // 클로져는 함수 리터럴의 본문에 있는 모든 자유 변수에 대한 바인딩을 포획해서
  // 자유 변수가 없게 닫는 행위에서 따온 말이다. (프로그래밍 스칼라 책에서)
  def checkClosure() = {
    println("클로져 체크")

    var more = 1 // more 는 자유 변수이다

    // 자유 변수가 포함된 함수 리터럴을 열린 코드 조각(open term)이라 부른다. 없으면 닫힌 코드 조각
    val addMore = (x: Int) => x + more
    println(addMore(10))

    // more 가 캡쳐되어 addMore 함수 객체가 되었으므로, more 를 바꾸면 값이 변경된다.
    more = 2;
    println(addMore(10))
  }

  // 특별한 형태의 함수 호출
  def specialFunctionCall(): Unit = {
    println("repeat parameter")
    println("----------------------------------------------")

    // 반복 파라미터는 C++ 의 가변인자랑 비슷한데, 하나의 타입만 된다.
    repeatParameterFunction("one")
    repeatParameterFunction("one", "two")
    repeatParameterFunction("hello", "world!")

    // 배열을 반복 파라미터로 남길 경우, 다음 처림 :_* 를 추가하면 된다
    val arr = Array("What's", "up", "doc?");
    repeatParameterFunction(arr: _*) // 이렇게 하면 배열의 원소를 반복파라미터화 해서 전달한다.


    // 이름 붙인 인자로, 대개의 경우, 함수 파라미터 목록의 순서로 값을 전달 한다. 하지만 이름 붙이면, 순서와 무관하게
    // 값을 전달할 수 있다. 대개의 경우, 디폴트 파라미터가 있을 경우, 이름 붙인 인자를 쓰는 경우가 많다.
    println("named argument")
    println("----------------------------------------------")
    println(speed(100, 10))
    println(speed(time = 10, distance = 100))

    // 디폴트 인자값
    println("default parameter")
    println("----------------------------------------------")
    printTime()
    printTime(divisor = 1000)
    printTime(divisor = 1000 * 60, out = Console.err)
  }

  // 반복 파라미터 함수
  def repeatParameterFunction(args: String*) = for (arg <- args) println(arg)

  // 이름 붙인 진자
  def speed(distance: Float, time: Float): Float = distance / time

  def printTime(out: java.io.PrintStream = Console.out, divisor: Int = 1) = out.println("time = " + System.currentTimeMillis() / divisor)


  def tailRecursive(): Unit = {
    println("------------------------------------------------------ tail recursive")

    // 스칼라의 꼬리 호출이 다른 언어와 비슷하다는 것을 배운다.
    // 꼬리 호출 구조로 재귀 함수를 작성할 경우, 스택 오버플로우 위험에서 벗어날 수 있다
    // 규칙은 함수의 제일 마지막이 함수 호출만으로 끝나야 한다 예를 들어 return 1 + call() 등은 안된다
    approximate(10)
  }

  def approximate(guess: Double): Double = {
    if (isGoodEnough(guess)) guess
    else approximate(improve(guess))
  }

  def isGoodEnough(guess: Double): Boolean = {
    val condition = guess < 1
    println("step guess:" + guess + " condition:" + condition)
    condition
  }
  def improve(guess: Double): Double = guess * 0.25f
}

