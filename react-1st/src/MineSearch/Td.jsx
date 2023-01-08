import React, { useContext, useCallback } from 'react';
import { CODE, TableContext, OPEN_CELL, CLICK_MINE, FLAG_CELL, QUESTION_CELL, NORMALIZE_CELL } from './MineSearch'

const getTdStyle = (code) => {
    switch (code) {
        case CODE.NORMAL:
        case CODE.MINE:
            return {
                background: '#444',
            }
        case CODE.OPENED:
            return {
                background: 'white',
            }
        case CODE.QUESTION_MINE:
        case CODE.QUESTION:
            return {
                background: 'yellow',
            }
        case CODE.FLAG_MINE:
        case CODE.FLAG:
            return {
                background: 'red',
            }
        default:
            return {
                background: 'white'
            }
    }
};

const getTdText = (code) => {
    switch (code) {
        case CODE.NORMAL:
            return '';
        case CODE.MINE:
            return 'X';
        case CODE.CLICKED_MINE:
            return '펑';
        case CODE.FLAG_MINE:
        case CODE.FLAG:
            return '!';
        case CODE.QUESTION_MINE:
        case CODE.QUESTION:
            return '?';
        case 0:
            return '';
        default:
            return code;
    }
};

const Td = ({ rowIdx, cellIdx }) => {
    const { tableData, dispatch, halted } = useContext(TableContext);

    const onClickTd = useCallback((e) => {
        console.log(`cell clicked - row(${rowIdx}) cell((${cellIdx})) data(${tableData[rowIdx][cellIdx]})`);

        e.preventDefault();
        if (halted) {
            return;
        }

        switch (tableData[rowIdx][cellIdx]) {
            case CODE.NORMAL:
                dispatch({ type: OPEN_CELL, row: rowIdx, cell: cellIdx });
                return
            case CODE.MINE:
                dispatch({ type: CLICK_MINE, row: rowIdx, cell: cellIdx });
                return;
            default:
                return;
        }
    }, [tableData[rowIdx][cellIdx], halted]);

    const onRightClickTd = useCallback((e) => {
        console.log(`cell right clicked - row(${rowIdx}) cell((${cellIdx})) data(${tableData[rowIdx][cellIdx]})`);

        e.preventDefault();
        if (halted) {
            return;
        }

        switch (tableData[rowIdx][cellIdx]) {
            case CODE.NORMAL:
            case CODE.MINE:
                dispatch({ type: FLAG_CELL, row: rowIdx, cell: cellIdx });
                return;

            case CODE.FLAG_MINE:
            case CODE.FLAG:
                dispatch({ type: QUESTION_CELL, row: rowIdx, cell: cellIdx });
                return;

            case CODE.QUESTION_MINE:
            case CODE.QUESTION:
                dispatch({ type: NORMALIZE_CELL, row: rowIdx, cell: cellIdx });
                return;
            default:
                return;
        }
    }, [tableData[rowIdx][cellIdx], halted]);

    return (
        <td
            style={getTdStyle(tableData[rowIdx][cellIdx])}
            onClick={onClickTd}
            onContextMenu={onRightClickTd}
        >{getTdText(tableData[rowIdx][cellIdx])}
        </td>
    )
};

export default Td;