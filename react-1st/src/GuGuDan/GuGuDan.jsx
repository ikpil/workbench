import React from 'react';

const { useState, useRef } = React;

const GuGuDan = () => {
  const [first, setFirst] = useState(Math.ceil(Math.random() * 9));
  const [second, setSecond] = useState(Math.ceil(Math.random() * 9));
  const [value, setValue] = useState('');
  const [result, setResult] = useState('');
  const inputRef = useRef(null);

  const onChangeInput = (e) => {
    setValue(e.target.value);
  }

  const onSubmitForm = (e) => {
    e.preventDefault();
    const resolvedVale = first * second;
    if (parseInt(value, 10) === resolvedVale) {
      setResult(`정답 : ${value}`);
      setFirst(Math.ceil(Math.random() * 9));
      setSecond(Math.ceil(Math.random() * 9));
      setValue('');
    } else {
      setResult('땡');
      setFirst(Math.ceil(Math.random() * 9));
      setSecond(Math.ceil(Math.random() * 9));
      setValue('');
    }

    inputRef.current.focus();
  }

  return (
    <>
      <div>{`${first} 곱하기 ${second}`}</div>
      <form onSubmit={onSubmitForm}>
        <input ref={inputRef} onChange={onChangeInput} value={value} />
        <button type="button">입력!</button>
      </form>
      <div>{result}</div>
    </>
  )
}

module.exports = GuGuDan;
