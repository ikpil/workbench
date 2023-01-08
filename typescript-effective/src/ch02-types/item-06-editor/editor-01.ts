function getElement(elOrId: string|HTMLElement|null): HTMLElement {
  // tyoeof 변수 일때, elOrId 가 null 이면, 무조건 object 타입이기 때문에, 컴파일 에러가 난다.
  if (typeof elOrId === 'object') {
    return elOrId;
 // ~~~~~~~~~~~~~~ 'HTMLElement | null' is not assignable to 'HTMLElement'
  } else if (elOrId === null) {
    return document.body;
  } else {
    // 리턴되는 el이 널일 수 있는데, HTMLElement 로 반환되기 떄문에, 컴파일 에러가 난다
    const el = document.getElementById(elOrId);
    return el;
 // ~~~~~~~~~~ 'HTMLElement | null' is not assignable to 'HTMLElement'
  }
}
