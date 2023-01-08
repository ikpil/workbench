package com.ikpil.scala.sample

import scala.io.Source

object LongLines {
  def processFile(filename: String, width: Int) = {
    val source = Source.fromFile(filename)
    for (line <- source.getLines())
      processLine(line)

    // 스칼라에서는 지역함수라고 하여, 함수 안에 함수를 만들수 있다.
    def processLine(line: String) = {
      // 지역함수를 감싸는 블럭의 변수, 파라미터등에 접근 할 수 있다
      if (line.length > width)
        println(filename + ": " + line.trim)
    }
  }
}
