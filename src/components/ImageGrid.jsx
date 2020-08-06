// @flow
import React, { useState } from 'react';
import ButtonToolbar from 'react-bootstrap/ButtonToolbar';
import Table from 'react-bootstrap/Table';
import ImageRow from './ImageRow';
import SmallWorld from './SmallWorld';
import ParameterGroup from './ParameterGroup';

type Props = { n_values: Array<number>; k_values: Array<number>; };

function ImageGrid(props: Props) {
    const p_values = [0.0, 0.2, 0.5, 1.0];
    const sigmas = [2, 5, 10, 25, 50];
    const { n_values, k_values } = props;
    const [current_n, setN] = useState(n_values[0]);
    const [current_k, setK] = useState(k_values[0]);

    return (
        <div>
            <ButtonToolbar className="mb-2">
                <ParameterGroup values={n_values} setValue={setN} current={current_n} />
                <ParameterGroup values={k_values} setValue={setK} current={current_k} />
            </ButtonToolbar>
            <Table size="sm">
                <thead>
                    <tr>
                        <th>#</th>
                        {sigmas.map(s => <th key={s.toFixed(1)}>{s}</th>)}
                    </tr>
                </thead>
                <tbody>
                    {p_values.map(p =>
                        <ImageRow key={p} n={current_n} k={current_k} p={p} sigmas={sigmas} />
                    )}
                </tbody>
            </Table>
            <SmallWorld n={current_n} k={current_k} p={0.0} s={50} />
        </div >
    );
}
export default ImageGrid;
