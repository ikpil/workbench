// 유니온 타입 정의

type AB = 'A' | 'B';
type AB12 = 'A' | 'B' | 12;


const a: AB = 'A';
const b: AB = 'B';
//const c: AB = 'c'; 컴파일 안됨, c 는 AB 타입이 아니기 때문
// let a: AB = 'A'; 컴파일 안됨, 상수만 수용 가능하다