import {
  BrowserRouter as Router,
  Routes,
  Route,
} from "react-router-dom";

import Home from "./routes/Home";
import Details from "./routes/Details";

function App() {
  return <Router>
      <Routes>
        <Route path="/" element={<Home />}/>
        <Route path="/movie/:id" element={<Details />}/>
        <Route path="/hello" element={"hello"}/>
      </Routes>
  </Router>
}

export default App;
