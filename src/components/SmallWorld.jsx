// @flow
import React from 'react';
import Image from 'react-bootstrap/Image';

type Props = { n: number; k: number; p: number; s: number; };

function SmallWorld(props: Props) {
    const { n, k, p, s } = props;
    var dir = `assets/frank16/(n=${n}, k=${k}, p=${p.toFixed(1)}, s=${s})`;
    return (
        <Image fluid src={dir + "/sankey0.png"} />
    );
}

export default SmallWorld;
