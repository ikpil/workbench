import React, { useState, useReducer, useCallback, useEffect, memo } from 'react';
import Table from './Table';

const initialState = {
    winner: '',
    turn: 'O',
    tableData: [
        ['', '', ''],
        ['', '', ''],
        ['', '', '']
    ],
    recentCell: {
        row: -1, 
        cell: -1
    },
};


export const SET_WINNER = 'SET_WINNER';
export const CLICK_CELL = 'CLICK_CELL';
export const CHANGE_TURN = 'CHANGE_TURN';
export const RESET_GAME = 'RESET_GAME';

const reducer = (state, action) => {
    console.log(`dispatch - type(${action.type})`);
    switch (action.type) {
        case SET_WINNER:
            return {
                ...state,
                winner: action.winner
            }
        case CLICK_CELL:
            const tableData = [...state.tableData];
            tableData[action.row] = [...tableData[action.row]];
            tableData[action.row][action.cell] = state.turn;
            console.log(tableData)

            return {
                ...state,
                tableData,
                recentCell: {
                    row: action.row, 
                    cell: action.cell,
                }
            }

        case CHANGE_TURN:
            return {
                ...state,
                turn: state.turn === 'O' ? 'X' : 'O',
            }
        case RESET_GAME:
            return {
                winner: state.winner,
                turn: 'O',
                tableData: [
                    ['', '', ''],
                    ['', '', ''],
                    ['', '', '']
                ],
                recentCell: {
                    row: -1,
                    cell: -1
                },
            };
        default:
            return state;
    }
};

const TicTacToe = memo(() => {
    const [state, dispatch] = useReducer(reducer, initialState);
    const { tableData, turn, recentCell } = state;

    useEffect(() => {

        const { row, cell } = recentCell;
        let win = false;

        //console.log(state);
        //console.log(`useEffect - ${tableData}`);
        //console.log(`useEffect - ${row}`);

        if (row < 0) {
            return;
        }

        if (tableData[row][0] === turn && tableData[row][1] === turn && tableData[row][2] === turn) {
            win = true
        }

        if (tableData[0][cell] === turn && tableData[1][cell] === turn && tableData[2][cell] === turn) {
            win = true
        }

        if (tableData[0][0] === turn && tableData[1][1] === turn && tableData[2][2] === turn) {
            win = true
        }

        if (tableData[0][2] === turn && tableData[1][1] === turn && tableData[2][0] === turn) {
            win = true
        }

        console.log(win, row, cell, tableData, turn)

        if (win) {
            dispatch({ type: SET_WINNER, winner: turn });
            dispatch({ type: RESET_GAME });
        } else {
            let all = true;
            tableData.forEach((row) => {
                row.forEach((cell) => {
                    if (!cell) {
                        all = false;
                    }
                })
            })

            if (all) {
                dispatch({ type: RESET_GAME });
            } else {
                dispatch({ type: CHANGE_TURN });
            }

        }
    }, [state.recentCell])

    return (
        <>
            <Table dispatch={dispatch} tableData={state.tableData} />
            {state.winner && <div>{state.winner}님의 승리</div>}
        </>
    );
});

export default TicTacToe;