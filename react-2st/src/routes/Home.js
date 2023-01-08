import { useState, useEffect } from "react"
import Movie from "../components/Movie"

const Home = () => {
    //const url = "https://yts.mx/api/v2/list_movies.json?minimum_rating=9=8.8&sort_by=year";
    const url = "http://localhost:3000/data.json";
    const detailUrl = "";
    const [loading, setLoading] = useState(true);
    const [movies, setMovies] = useState([]);
    const getMovies = async () => {
        const json = await (
            await fetch(url, {
                method: 'GET',
            })
        ).json();

        //const json = await response.json();
        setMovies(json.data.movies);
        setLoading(false);

        //console.log(movies)
    }

    useEffect(() => {
        getMovies()
    }, []);

    console.log(movies);
    return (
        <div>
            {loading
                ? <h1>Loading...</h1>
                : <div>{movies.map(movie => <Movie
                    key={movie.id}
                    id={movie.id}
                    coverImg={movie.medium_cover_image}
                    title={movie.title}
                    summary={movie.summary}
                    genres={movie.genres}
                />)}</div>}
        </div>
    );
}

export default Home;