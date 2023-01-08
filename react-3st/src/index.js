import React from 'react';
import { BrowserRouter, Routes, Route } from 'react-router-dom';
import ReactDOM from 'react-dom/client';
import './index.css';
import reportWebVitals from './reportWebVitals';
import Header from './Header';
import Main from './Main';
import Product from './Product';
import NotFound from './NotFound';
import LoginForm from './LoginForm'
import CPUUsage from './CPUUsage';
import Foot from './Foot';

const root = ReactDOM.createRoot(document.getElementById('root'));
root.render(
  <React.StrictMode>
    <BrowserRouter>
      <Header />
      <Routes>
        <Route path="/" element={<Main />} />
        <Route path="/pay" element={<LoginForm />} />
        <Route path="/order" element={<Product />} />
        <Route path="/shop" element={<CPUUsage />} />
        <Route path="/other" element={<CPUUsage />} />
        <Route path="*" element={<NotFound />} />
      </Routes>
      <Foot />
    </BrowserRouter>
  </React.StrictMode>
);

// If you want to start measuring performance in your app, pass a function
// to log results (for example: reportWebVitals(console.log))
// or send to an analytics endpoint. Learn more: https://bit.ly/CRA-vitals
reportWebVitals();
