function asNumber(val: number | string): number {
  return val as number;
}

console.log(asNumber("1"));