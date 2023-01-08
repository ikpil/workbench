function getKey<K extends string>(val: any, key: K) {
  // ...
}

const s: string = '1';

getKey(1, s);