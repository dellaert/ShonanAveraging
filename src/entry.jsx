// @flow
import React from 'react';
import ReactDOM from 'react-dom';
import ImageGrid from './components/ImageGrid'
import PivotTable from './components/PivotTable'

ReactDOM.render(<ImageGrid n_values={[20, 40]} k_values={[4, 8, 16]} />, document.getElementById('matrix'));
ReactDOM.render(<PivotTable />, document.getElementById('pivot'));
