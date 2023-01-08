class C {
  foo: string;
  constructor(foo: string) {
    this.foo = foo;
  }

  function abc(): int {
    return 1;
  }
}

const c = new C('instance of C');
const d: C = { foo: 'object literal' };  // OK!

console.log(d instanceof C);  // d 는 C 타입이 아니다
console.log(d.foo); // 그런데 foo 를 사용해되서 볼 수 있다
