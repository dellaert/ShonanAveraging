// @flow
import React, { useState } from 'react';
import ButtonToolbar from 'react-bootstrap/ButtonToolbar';
import Table from 'react-bootstrap/Table';
import PriorsRow from './PriorsRow';
import ParameterGroup from './ParameterGroup';

type Props = { gamma_values: Array<number>; };

function PriorsGrid(props: Props) {
    const { gamma_values } = props;
    const alpha_values = [0, 1, 100, 10000];
    const beta_values = [0, 1, 100, 10000];
    const [current_gamma, setN] = useState(gamma_values[0]);

    return (
        <div>
            <ButtonToolbar className="mb-2">
                <ParameterGroup values={gamma_values} setValue={setN} current={current_gamma} />
            </ButtonToolbar>
            <Table size="sm">
                <thead>
                    <tr>
                        <th key="left">#</th>
                        {beta_values.map(beta => <th key={beta.toFixed(1)}>beta={beta}</th>)}
                    </tr>
                </thead>
                <tbody>
                    {alpha_values.map(alpha =>
                        <PriorsRow alpha={alpha} beta_values={beta_values} gamma={current_gamma} />
                    )}
                </tbody>
            </Table>
        </div >
    );
}
export default PriorsGrid;
