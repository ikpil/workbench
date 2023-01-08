import React, { memo, useContext, useRef, useEffect, useMemo } from 'react';
import Td from './Td';

const Tr = memo(({ dispatch, rowIndex, rowData }) => {
    console.log('tr rendered');

    const ref = useRef([]);
    useEffect(() => {
        console.log("tr", dispatch === ref.current[0], rowIndex === ref.current[1], rowData === ref.current[2]);

        ref.current = [dispatch, rowIndex, rowData] ;
    }, [dispatch, rowIndex, rowData] );

    return (
        <tr>
            {Array(rowData.length).fill().map((td, i) => (
                useMemo(() => <Td Key={i} dispatch={dispatch} rowIndex={rowIndex} cellIndex={i} cellData={rowData[i]} />, [rowData[i]])
            ))}
        </tr>
    );
});

export default Tr;