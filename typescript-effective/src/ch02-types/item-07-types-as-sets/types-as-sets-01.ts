// 네버 타입은 아무값도 할당하지 못하기 떄문에 12를 할당할 경우, 컴파일 에러가 납니다
const x: never = 12;
   // ~ Type '12' is not assignable to type 'never'

const y: never = undefined;