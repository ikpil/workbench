import React from "react";
import { Link } from 'react-router-dom'

function Main(props) {
  return (
    <>
      <h3>메인입니다</h3>
      <ul>
        <Link to="/product/1"><li>상품 1번</li></Link>
        <Link to="/product/1"><li>상품 2번</li></Link>
      </ul>
    </>
  );
}

export default Main;
