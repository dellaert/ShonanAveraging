// @flow
import React from 'react';
import Image from 'react-bootstrap/Image';

function Sankey(props: { alpha: number; beta: number; gamma: number; }) {
    const { alpha, beta, gamma } = props;
    var dir = `assets/frank18/${alpha.toFixed(1)}-${beta.toFixed(1)}-${gamma.toFixed(1)}`;
    return (
        <Image fluid src={dir + "/sankey.png"} />
    );
}

type Props = { alpha: number; beta_values: Array<number>; gamma: number; };

// One row of Sankey diagrams, with varying beta (Karcher) parameter
function PriorsRow(props: Props) {
    const { alpha, beta_values, gamma } = props;
    const key = alpha.toFixed(1);

    return (
        <tr key={key}>
            <td key="left" align="center">alpha={key}</td>
            {beta_values.map(beta =>
                <td key={beta.toFixed(1)}>
                    <Sankey alpha={alpha} beta={beta} gamma={gamma} />
                </td>)}
        </tr>
    );
}

export default PriorsRow;
