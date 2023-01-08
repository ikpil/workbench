import React, { useCallback, useEffect, useRef, memo } from 'react';
import { CLICK_CELL, CHANGE_TURN } from './TicTacToe'

const Td = memo(({ dispatch, rowIndex, cellIndex, cellData }) => {
    console.log('td rendered');

    const ref = useRef([]);

    useEffect(() => {
        console.log("td", rowIndex === ref.current[0], cellIndex === ref.current[1], dispatch === ref.current[2], cellData === ref.current[3])
        ref.current = [rowIndex, cellIndex, dispatch, cellData];
    }, [rowIndex, cellIndex, dispatch, cellData]);

    const onClickTd = useCallback(() => {
        console.log(rowIndex, cellIndex, cellData);

        dispatch({ type: CLICK_CELL, row: rowIndex, cell: cellIndex });
    }, [cellData]);

    return (
        <td onClick={onClickTd}>{cellData}</td>
    );
});

export default Td;
