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
interface Vector3D {
  x: number;
  y: number;
  z: number;
}

// 컴파일 됨
calculateLength({x:1, y:2});

// 컴파일 됨
const b: NamedVector = {
  name: "asdfsdf",
  x: 1,
  y: 2,
};
calculateLength(b);

// 컴파일 됨
const c: Vector3D = {
  x: 1,
  y: 2,
  z: 3, 
};
calculateLength(c);

