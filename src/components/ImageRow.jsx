// @flow
import React from 'react';
import SmallWorld from './SmallWorld';

type Props = { n: number; k: number; p: number; sigmas: Array<number>; };

// One row of images, with varying sigma parameter
function ImageRow(props: Props) {
    const { n, k, p, sigmas } = props;
    const key = p.toFixed(1);
    return (
        <tr key={key}>
            <td key="0" align="center">p={key}</td>
            {sigmas.map(s =>
                <td key={s.toFixed(1)}>
                    <SmallWorld n={n} k={k} p={p} s={s} />
                </td>)}
        </tr>
    );
}

export default ImageRow;
