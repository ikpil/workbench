package com.ikpil.scala.sample

import scala.collection.mutable

class ChecksumAccumulator {
  // 접근 수식자가 없을 때, public 이다
  private var sum = 0

  def add(b: Byte): Unit = sum += b

  // 매개변수는 val 이기 때문에, 아래 코드는 컴파일 되지 않는다
  // b = 1;
  //sum += b
  //}

  def checksum(): Int = ~(sum & 0xFF) + 1

  // 리턴문이 없어도 맨 나중값을 계산하여 반환한다.
  //    ~(sum & 0xFF) + 1
  //}
}


// 동반 클래스 companion class, C++ 의 friend 와 비슷한 느낌이다.
// 동반 클래스 관계일 때, 서로 private 영역에 접근 할 수 있다.
// new 가 불가능하다, 자바기준으로는 static 생성자가 있는 느낌이다.
object ChecksumAccumulator {
  private val cache = mutable.Map.empty[String, Int]

  def calculate(s: String): Int =
    if (cache.contains(s))
      cache(s)
    else {
      val acc = new ChecksumAccumulator
      for (c <- s)
        acc.add(c.toByte)

      val cs = acc.checksum()
      cache += (s -> cs)
      cs
    }
}