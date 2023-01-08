interface Vector2D {
  x: number;
  y: number;
}
function calculateLength(v: Vector2D) {
  return Math.sqrt(v.x * v.x + v.y * v.y);
}
interface NamedVector {
  name: string;
  x: number;
  y: number;
}
const v: NamedVector = { x: 3, y: 4, name: 'Zee' };

// 1. 컴파일 되는 것은 NamedVector 의 인터페이스가 Vector2D의 x,y 인터페이스 조건을 만족하기 때문입니다.
calculateLength(v);  // OK, result is 5
