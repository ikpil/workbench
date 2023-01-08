import React, { useReducer, createContext, useMemo, useEffect } from 'react';
import Table from './Table'
import Form from './Form'

export const CODE = {
    OPENED: 0,
    NORMAL: -1,
    QUESTION: -2,
    FLAG: -3,
    QUESTION_MINE: -4,
    FLAG_MINE: -5,
    CLICKED_MINE: -6,
    MINE: -7,
};


export const TableContext = createContext({
    tableData: [
        [-1, -1, -1, -1, -1, -1, -1],
        [-1, -1, -1, -1, -1, -1, -1],
        [-1, -1, -1, -1, -1, -1, -1],
        [-1, -1, -1, -1, -1, -1, -1],
        [-1, -1, -1, -1, -1, -1, -1],
        [-1, -1, -1, -1, -1, -1, -1],
        [-1, -1, -1, -1, -1, -1, -1],
    ],
    dispatch: () => { },
    halted: false,
});

const initialState = {
    tableData: [],
    timer: 0,
    result: '',
    halted: true,
    openedCount: 0,
    data: {
        row: 0,
        cell: 0,
        mine: 0,
    },
}

export const START_GAME = 'START_GAME';
export const OPEN_CELL = 'OPEN_CELL';
export const CLICK_MINE = 'CLICK_MINE';
export const FLAG_CELL = 'FLAG_CELL';
export const QUESTION_CELL = 'QUESTION_CELL';
export const NORMALIZE_CELL = 'NORMALIZE_CELL';
export const INCREMENT_TIMER = 'INCREMENT_TIMER';

const plantMine = (row, cell, mine) => {
    console.log(row, cell, mine);
    const candidate = Array(row * cell).fill().map((arr, i) => {
        return i;
    });

    const shuffle = [];
    while (candidate.length > row * cell - mine) {
        const chosen = candidate.splice(Math.floor(Math.random() * candidate.length), 1)[0];
        shuffle.push(chosen);
    }

    const data = [];
    for (let i = 0; i < row; i++) {
        const rowData = [];
        data.push(rowData);
        for (let j = 0; j < cell; j++) {
            rowData.push(CODE.NORMAL);
        }
    }

    for (let k = 0; k < shuffle.length; k++) {
        const ver = Math.floor(shuffle[k] / cell);
        const hor = shuffle[k] % cell;

        data[ver][hor] = CODE.MINE;
    }

    console.log(data);
    return data;
};

const reducer = (state, action) => {
    console.log(`reducer - ${action.type}`);

    switch (action.type) {
        case START_GAME:
            return {
                ...state,
                data: {
                    row: action.row,
                    cell: action.cell,
                    mine: action.mine,
                },
                openedCount: 0,
                timer: 0,
                halted: false,
                tableData: plantMine(action.row, action.cell, action.mine),
            }
        case OPEN_CELL: {
            const tableData = [...state.tableData];
            tableData[action.row] = [...tableData[action.row]];
            tableData[action.row][action.cell] = CODE.OPENED;

            tableData.forEach((row, i) => {
                tableData[i] = [...state.tableData[i]]
            });

            const checked = [];
            const checkAround = (row, cell) => {
                //console.log('row/cel : ', row + ',' + cell);
                // 이미 열린 칸이라면 패스
                const isIncluded = [CODE.OPENED, CODE.FLAG_MINE, CODE.FLAG, CODE.QUESTION_MINE, CODE.QUESTION].includes(tableData[row][cell]);
                if (isIncluded) {
                    return;
                }

                // 이미 검사한 칸을 경우 패스
                if (checked.includes(row + ',' + cell)) {
                    return;
                }

                checked.push(row + ',' + cell);

                let dirs = [
                    [-1, -1], [0, -1], [1, -1],
                    [-1, 0], [1, 0],
                    [-1, 1], [0, 1], [1, 1],
                ]
    
                //console.log(`loop - dirs(${dirs})`);
    
                let around = [];
                for (let i = 0; i < dirs.length; ++i) {
                    let dirX = dirs[i][0];
                    let dirY = dirs[i][1];
    
                    //console.log(`loop - dirX(${dirX}) dirY(${dirY})`)
    
                    let rangeX = cell + dirX;
                    let rangeY = row + dirY;
                    
                    //console.log('!tableData[rangeY]', !tableData[rangeY]);
                    if (!tableData[rangeY]) {
                        continue;
                    }
    
                    if (0 > rangeX || tableData[0].length <= rangeX) {
                        continue;
                    }
    
                    if (tableData[row + dirY][cell + dirX]) {
                        //console.log(`table data - ${tableData[row + dirY][cell + dirX]}`)
                        around = around.concat(
                            tableData[row + dirY][cell + dirX]
                        )
                    }
                }
    
                const count = around.filter(v => [CODE.MINE, CODE.FLAG_MINE, CODE.QUESTION_MINE].includes(v)).length;
                tableData[row][cell] = count;

                if (0 === count) {
                    const near = [];
                    for (let i = 0; i < dirs.length; ++i) {
                        let dirX = dirs[i][0];
                        let dirY = dirs[i][1];

                        let rangeX = cell + dirX;
                        let rangeY = row + dirY;
        
                        if (!tableData[rangeY]) {
                            continue;
                        }

                        if (0 > rangeX || tableData[0].length <= rangeX) {
                            continue;
                        }

                        near.push([rangeY, rangeX]);
                    }

                    //console.log('near : ', near);
                    near.forEach(n => checkAround(n[0], n[1]));
                }
    
                //console.log(`around - ${around}`);
            }

            checkAround(action.row, action.cell);
            let halted = false;
            let result = '';
            if (state.data.row * state.data.cell - state.data.mine === state.openedCount + checked.length) {
                halted = true;
                result = '승리';
            }
 
            return {
                ...state,
                tableData,
                openedCount: state.openedCount + checked.length,
                halted: halted,
                result: result,
            }
        }
        case CLICK_MINE: {
            const tableData = [...state.tableData];
            tableData[action.row] = [...state.tableData[action.row]];
            tableData[action.row][action.cell] = CODE.CLICKED_MINE;
            return {
                ...state,
                tableData,
                halted: true,
            };
        }
        case FLAG_CELL: {
            const tableData = [...state.tableData];
            tableData[action.row] = [...state.tableData[action.row]];
            if (CODE.MINE === tableData[action.row][action.cell]) {
                tableData[action.row][action.cell] = CODE.FLAG_MINE;
            } else {
                tableData[action.row][action.cell] = CODE.FLAG;
            }

            return {
                ...state,
                tableData,
            };
        }
        case QUESTION_CELL: {
            const tableData = [...state.tableData];
            tableData[action.row] = [...state.tableData[action.row]];
            if (CODE.FLAG_MINE === tableData[action.row][action.cell]) {
                tableData[action.row][action.cell] = CODE.QUESTION_MINE;
            } else {
                tableData[action.row][action.cell] = CODE.QUESTION;
            }

            return {
                ...state,
                tableData,
            };
        }
        case NORMALIZE_CELL: {
            const tableData = [...state.tableData];
            tableData[action.row] = [...state.tableData[action.row]];
            if (CODE.QUESTION_MINE === tableData[action.row][action.cell]) {
                tableData[action.row][action.cell] = CODE.MINE;
            } else {
                tableData[action.row][action.cell] = CODE.NORMAL;
            }

            return {
                ...state,
                tableData,
            };
        }
        case INCREMENT_TIMER: {
            return {
                ...state,
                timer: state.timer + 1,
            }
        }
        default:
            return state;
    }
}

const MineSearch = () => {
    const [state, dispatch] = useReducer(reducer, initialState);
    const { tableData, halted, timer, result } = state;

    const value = useMemo(() => {
        return {
            tableData: tableData,
            halted: halted,
            dispatch,
        }
    }, [tableData, halted]);

    useEffect(() => {
        console.log(`halted 의 상태 변화 - ${halted}`)
        let timer;
        if (halted === false) {
            timer = setInterval(() => {
                dispatch({ type: INCREMENT_TIMER })
            }, 1000);
        }
        return () => {
            clearInterval(timer);
        };
    }, [halted]);

    return (
        <TableContext.Provider value={value}>
            <Form />
            <div>{timer}</div>
            <Table />
            <div>{result}</div>
        </TableContext.Provider>
    )
};

export default MineSearch;
