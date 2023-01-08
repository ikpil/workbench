// tsConfig: {"noImplicitAny":false}

// function add(a: number, b: number): number;
// function add(a: string, b: string): string;

function add(a: boolean, b: boolean): boolean {
  if (a || b)
    return false
  else
    return true;
}

//const three = add(1, 2);  // Type is number
//const twelve = add('1', '2');  // Type is string
const haha = add(true, true);
