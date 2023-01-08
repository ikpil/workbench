function asNumber(val: number | string): number {
  return typeof(val) === 'string' ? Number(val) : val;
}

console.log(asNumber(0.5));