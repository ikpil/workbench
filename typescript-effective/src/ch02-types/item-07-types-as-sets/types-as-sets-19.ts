// C++ 템플릿 느낌이 든다

// Exclude<T, U> 는 T 가 U로 부터 확장 되었다면 ? never를 리턴하고 아니라면 T를 리턴한다
// Exclude<(타입A|타입B), (타입C|타입D)> 일 경우, 
// a. 타입A - (타입C|타입D)
// b. 타입B - (타입C|타입D)
// 로 연산 되어, (a|b) 타입이 되고, never 는 소거되어 남은 것이 실제 타입이 된다

// Exclude 된 T의 타입 연산 과정을 보면 다음과 같다
// type a = string - (string|number) = never
// type b = Date - (string|number) = Date

// 그러므로 type T = never|Date 가 되므로 Date가 된다
type T = Exclude<string|Date, string|number>;  // Type is Date



type NonZeroNums = Exclude<number, 0>;  // Type is still just number
type NonZeroNums2 = Exclude<number, number>;  // Type is still just number

// const a: T = new Date('1234/12/12'); // OK
// const b: T = 'asdf';
const a: NonZeroNums = 0;
//const b: NonZeroNums2 = 0; // never 이기 때문에 컴파일 에러
