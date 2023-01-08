// 타입은 특정 값들의 집합으로 정의 된다.

interface Identified {
  id: string;
}
interface Person {
  name: string;
}
interface Lifespan {
  birth: Date;
  death?: Date;
}

// 타입 PersonSpan 은 Person & Lifespan 은 합집합 타입으로 정의 된다.
type PersonSpan = Person & Lifespan;

const ps: PersonSpan = {
  name: 'Samsung Electro-Mechanics',
  birth: new Date('1234/12/12'),
  //death: new Date('1234/12/12'), -- death 가 없어도 공집합을 허용하므로, 컴파일이 된다
}