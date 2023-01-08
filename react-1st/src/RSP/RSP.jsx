import React, { Component, useState, useRef, useEffect } from 'react';

// 클래스의 경우
// 최초 생성 될 때 -> constructor -> render -> ref -> componentDidMount -> 
// setState/props 바뀔 때 -> shouldComponentUpdate -> redner -> componentDidUpdate 
// 부모가 자식 컴포넌트를 지울 때 -> componentWillUnmount -> 소멸

const RSP_VALUES = [
  '가위',
  '바위',
  '보',
];

const WIN_MAP = [
  2,
  0,
  1,
];

const RSP = () => {
  const [result, setResult] = useState('');
  const [score, setScore] = useState(0);
  const [imgCoord, setImgCoord] = useState(0);

  const interval = useRef();

  //console.log('RSP 최초 실행');
  useEffect(() => { // componentDidMount, componentDidUpdate
    //console.log('유즈 이펙트에서 subscribe');
    // 컴포넌트가 첫 렌더링 된 후
    interval.current = startCoroutine();

    return () => { // componentWillUnmount
    //console.log('유즈 이펙트에서 unsubscribe');
      clearInterval(interval.current);
    }
  }, [])
  //console.log('유즈 이펙트 호출 완료');

  const startCoroutine = () => {
    return setInterval(() => {
      setImgCoord((prev) => (prev + 1) % RSP_VALUES.length)
    }, 1000);
  }

  const onClickBtn = (idx) => {
    return () => {
      // 인터벌이 없다면 로직을 돌리지 않는다
      if (null === interval.current) {
        return;
      }

      clearInterval(interval.current);
      interval.current = null;

      // 플레이어 기준
      if (idx == imgCoord) {
        // 비겼을 때
        setResult('비겼습니다.');

      } else if (WIN_MAP[idx] == imgCoord) {
        // 이겼을 때
        setResult('이겼습니다.');
        setScore((prevScore) => prevScore + 1);

      } else {
        // 졌을 때
        setResult('졌습니다.');
        setScore((prevScore) => prevScore - 1);
      };

      setTimeout(() => {
        interval.current = startCoroutine();
      }, 2000);
    }
  }

  return (
    <>
      <div>{RSP_VALUES[imgCoord]}</div>
      <div>
        <button id="scissor" className="btn" onClick={onClickBtn(0)}>{RSP_VALUES[0]}</button>
        <button id="rock" className="btn" onClick={onClickBtn(1)}>{RSP_VALUES[1]}</button>
        <button id="paper" className="btn" onClick={onClickBtn(2)}>{RSP_VALUES[2]}</button>
      </div>
      <div>{result}</div>
      <div>현재 {score}점</div>
    </>
  );
};

module.exports = RSP;
