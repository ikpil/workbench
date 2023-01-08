type A = 'A'; // A는 문자 'A' 타입이고
type B = 'B'; // B는 문자 'B' 타입이다.
type Twelve = 12; // 12는 Twelve 타입니다.
//const Twelve = 12;

// 테스트
function typesAsSets02(a: A, b: B, twelve: Twelve) {
    return a + b;
}

// var t1 = typesAsSets02('B', 'A', 13); -- 컴파일 에러, 'B' 는 타입 A가 아니다
 //var t2 = typesAsSets02('A', 'A', 13); -- 컴파일 에러, 'A' 는 타입 B가 아니다
 //var t3 = typesAsSets02('A', 'B', 13); -- 컴파일 에러, 13은 타입 Twelve가 아니다;
 var t4 = typesAsSets02('A', 'B', 12); // 값 자체를 타입으로 만들수 있다. 특정 함수 호출시 컴파일 타입에 뭔가 처리 할 수 있나 보다
 
console.log(t4);
