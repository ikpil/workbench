import React from 'react';
import ReactDom from 'react-dom';
import { hot } from 'react-hot-loader/root';
import MineSearch from './MineSearch/MineSearch'

const HotLoader = hot(MineSearch);

ReactDom.render(<HotLoader />, document.querySelector('#root'));