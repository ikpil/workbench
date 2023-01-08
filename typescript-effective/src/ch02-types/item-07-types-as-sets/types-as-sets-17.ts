// 튜플과 배열의 차이에 대한 이야기

// 이건 넘버 타입의 배열 list 를 정의 한것
const list = [1, 2];  // Type is number[]

// 이건 튜플 넘버, 넘버인 타입의 정의
// 그러므로, 할당이 안된다
const tuple: [number, number] = list;
   // ~~~~~ Type 'number[]' is missing the following
   //       properties from type '[number, number]': 0, 1
