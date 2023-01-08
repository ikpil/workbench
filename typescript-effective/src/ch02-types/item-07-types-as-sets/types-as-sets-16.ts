interface Point {
  x: number;
  y: number;
}
type PointKeys = keyof Point;  // Type is "x" | "y"

// keyof 를 이렇게 쓸수 있다니, 신선하다
// sortBy 
// - vals 은 모든 타입의 배열
// - key 는 T 의 이름(문자) 들 중 하나
// C# 에서는 함수로 정렬 기준값을 뽑았다면, typescript 에서는 키 타입을 keyof 로 정의하여, 키를 입력 받게 더 깔끔하게 만들 수 있다.
// 일단 C# 에서의 람다 표현식을 안쓰고, 키 타입을 정의해서, 키를 입력받아 처리하는게 신기하다.

// 타입스크립트는 타입도 스크립팅 할 수 있다는걸 배운다

function sortBy<K extends keyof T, T>(vals: T[], key: K): T[] {
  // COMPRESS
  vals.sort((a, b) => a[key] === b[key] ? 0 : a[key] < b[key] ? -1 : +1);
  return vals;
  // END
}
const pts: Point[] = [{x: 1, y: 1}, {x: 2, y: 0}];
sortBy(pts, 'x');  // OK, 'x' extends 'x'|'y' (aka keyof T)
sortBy(pts, 'y');  // OK, 'y' extends 'x'|'y'
sortBy(pts, Math.random() < 0.5 ? 'x' : 'y');  // OK, 'x'|'y' extends 'x'|'y'
sortBy(pts, 'z');
         // ~~~ Type '"z"' is not assignable to parameter of type '"x" | "y"

