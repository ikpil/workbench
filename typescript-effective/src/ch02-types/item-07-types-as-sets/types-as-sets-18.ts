// 튜플(넘버, 너버, 넘버)를 튜블(넘버, 넘버)에 할당할 수 없다
// 왜냐하면 플의 길이가 다르기 때문에
const triple: [number, number, number] = [1, 2, 3];
const double: [number, number] = triple;
   // ~~~~~~ '[number, number, number]' is not assignable to '[number, number]'
   //          Types of property 'length' are incompatible
   //          Type '3' is not assignable to type '2'

