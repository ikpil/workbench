import React, { memo } from 'react';
import Tr from './Tr';

const Table = memo(({ dispatch, onClick, tableData }) => {
    console.log('table rednered');

    return (
        <table onClick={onClick}>
            {Array(tableData.length).fill().map((tr, i) =>
                (<Tr Key={i} dispatch={dispatch} rowIndex={i} rowData={tableData[i]} />)
            )}
        </table>
    )
});

export default Table;