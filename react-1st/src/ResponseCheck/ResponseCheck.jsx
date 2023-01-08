import React, { useState, useRef } from 'react';

const ResponseCheck = () => {
  console.log('시작!!');
  const [state, setState] = useState('waiting');
  const [message, setMessage] = useState('클릭해서 시작하세요.');
  const [result, setResult] = useState([]);
  const timeout = useRef(null);
  const startTime = useRef(0);
  const endTime = useRef(0);

  const onClickScreen = () => {
    if (state === 'waiting') {
      setState('ready');
      setMessage('초록색이 되면 클릭하세요');

      timeout.current = setTimeout(() => {
        setState('now');
        setMessage('지금 클릭');
        startTime.current = new Date();
      }, Math.floor(Math.random() * 1000) + 2000);
    } else if (state === 'ready') {
      clearTimeout(timeout.current);

      setState('waiting');
      setMessage('너무 성급하시군요! 초록색이 된 후에 클립하세요.');
    } else if (state === 'now') {
      endTime.current = new Date();
      setState('waiting');
      setMessage('클릭해서 시작하세요.');
      setResult((prevResult) => [...prevResult, endTime.current - startTime.current]);
    }
  };

  const onReset = () => {
    setResult([]);
  };

  const renderAverage = () => (
    result.length === 0
      ? null
      : (
        <>
          <div>
            평균 시간:
            {result.reduce((a, c) => a + c) / result.length}
            ms
          </div>
          <button type="button" onClick={onReset}>리셋</button>
        </>
      )
  );

  return (
    <>
      <div id="screen" className={state} onCLick={onClickScreen} onKeyDown={onClickScreen} role="button" tabIndex="0">
        {message}
      </div>
      {renderAverage()}
    </>
  );
};

module.exports = ResponseCheck;
