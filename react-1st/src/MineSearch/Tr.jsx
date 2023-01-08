import React, { useContext } from 'react';
import { TableContext } from './MineSearch'
import Td from './Td'

const Tr = ({ rowIdx }) => {
    const { tableData } = useContext(TableContext);

    return (
        <tr>
            {tableData[0] && Array(tableData[0].length).fill().map((td, i) =>
                <Td key={i} rowIdx={rowIdx} cellIdx={i} />
            )}
        </tr>
    );
};

export default Tr;