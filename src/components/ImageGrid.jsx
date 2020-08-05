// @flow
import React, { useState } from 'react';
import Button from 'react-bootstrap/Button';
import ButtonGroup from 'react-bootstrap/ButtonGroup';
import ButtonToolbar from 'react-bootstrap/ButtonToolbar';
import Table from 'react-bootstrap/Table';
import ImageRow from './ImageRow';
import SmallWorld from './SmallWorld';

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
                <ButtonGroup className="mr-2" aria-label="n_values button group">
                    {n_values.map(n =>
                        <Button onClick={() => setN(n)}
                            key={n} active={n === current_n}
                            variant="outline-primary">{n}
                        </Button>
                    )}
                </ButtonGroup>
                <ButtonGroup aria-label="k_values button group">
                    {k_values.map(k =>
                        <Button onClick={() => setK(k)}
                            key={k} active={k === current_k}
                            variant="outline-primary">{k}
                        </Button>
                    )}
                </ButtonGroup>
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
            />
        </div>
    );
}
export default ImageGrid;
