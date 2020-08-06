// @flow
import React from 'react';
import ReactDOM from 'react-dom';
import PaperHeader from './components/PaperHeader'
import Media from './components/Media'
import Video from './components/Video'
import ImageGrid from './components/ImageGrid'
import PriorsGrid from './components/PriorsGrid'
import PivotTable from './components/PivotTable'

ReactDOM.render(<PaperHeader
    title="Shonan Rotation Averaging"
    subtitle="Global Optimality by Surfing SO(p)^n"
    venue="ECCV 2020 (Spotlight)"
    authors={[{ name: "Frank Dellaert", affiliation: 1, link: "https://dellaert.github.io/" },
    { name: "David M. Rosen", affiliation: 2, link: "https://marinerobotics.mit.edu/people/david-rosen" },
    { name: "Jing Wu", affiliation: 1 },
    { name: "Robert Mahony", affiliation: 3 },
    { name: "Luca Carlone", affiliation: 2, link: "https://lucacarlone.mit.edu/" },
    ]}
    affiliations={["Georgia Institute of Technology", "Massachusetts Inst. of Technology", "Australian National University"]}
/>,
    document.getElementById('header'));

ReactDOM.render(<Media paper="assets/Dellaert20arxiv_Shonan_eccv20.pdf"
    video="https://youtu.be/5ppaqMyHtE0"
    repo="https://github.com/dellaert/ShonanAveraging" />,
    document.getElementById('media'));

ReactDOM.render(<Video videoId="5ppaqMyHtE0"/>, document.getElementById('video'));

ReactDOM.render(<PriorsGrid gamma_values={[0, 1, 100, 10000]} />, document.getElementById('priors'));

ReactDOM.render(<ImageGrid n_values={[20, 40]} k_values={[4, 8, 16]} />, document.getElementById('matrix'));

    // ReactDOM.render(<PivotTable />, document.getElementById('pivot'));
