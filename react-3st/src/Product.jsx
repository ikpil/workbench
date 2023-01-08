import React from "react";
import { useLocation, useNavigate, useParams } from "react-router-dom";

function Product() {
  const { productId } = useParams();
  const location = useLocation();
  const nav = useNavigate();

  return (
    <>
      <h1>{productId} 상품페이지</h1>
      <ul>
        <li>hash : {location.hash}</li>
        <li>pathname : {location.pathname}</li>
        <li>search : {location.search}</li>
        <li>state : {location.state}</li>
        <li>key : {location.key}</li>
      </ul>
      <ul>
        <li><button onClick={() => nav(-2)}>2단계 뒤로 이동</button></li>
        <li><button onClick={() => nav(-1)}>1단계 이동</button></li>
      </ul>
    </>
  );
}

export default Product;
