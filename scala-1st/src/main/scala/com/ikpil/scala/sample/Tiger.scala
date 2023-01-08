package com.ikpil.scala.sample

// 파라미터 필드 정ㅇ, 중요한것은 인자 이름이 스코프에 있는 다른 이름과 충돌되지 않아야 한다
class Tiger(param1: Boolean, param2: Int) extends Cat {
  override val dangerous: Boolean = param1
  private var age = param2
}
