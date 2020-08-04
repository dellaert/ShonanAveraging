// @flow
import React from 'react';
import tips from '../tips';
import { sortAs } from 'react-pivottable/Utilities';
import TableRenderers from 'react-pivottable/TableRenderers';
import createPlotlyComponent from 'react-plotly.js/factory';
import createPlotlyRenderers from 'react-pivottable/PlotlyRenderers';
import PivotTableUI from 'react-pivottable/PivotTableUI';
import 'react-pivottable/pivottable.css';

const Plot = createPlotlyComponent(window.Plotly);

type Props = {};

type State = {
    pivotState: any,
};

class PivotTableUISmartWrapper extends React.PureComponent<Props, State> {
    constructor(props) {
        super(props);
        this.state = { pivotState: props };
    }

    componentWillReceiveProps(nextProps) {
        this.setState({ pivotState: nextProps });
    }

    render() {
        return (
            <PivotTableUI
                renderers={Object.assign(
                    {},
                    TableRenderers,
                    createPlotlyRenderers(Plot)
                )}
                {...this.state.pivotState}
                onChange={s => this.setState({ pivotState: s })}
                unusedOrientationCutoff={Infinity}
            />
        );
    }
}

type Props2 = {};

type State2 = {
    mode: string,
    filename: string,
    pivotState: any,
    textarea: string
};

type DroppedFile = { name: string };

export default class PivotTable extends React.Component<Props2, State2> {
    componentWillMount() {
        this.setState({
            mode: 'demo',
            filename: 'Sample Dataset: Tips',
            pivotState: {
                data: tips,
                rows: ['Payer Gender'],
                cols: ['Party Size'],
                aggregatorName: 'Sum over Sum',
                vals: ['Tip', 'Total Bill'],
                rendererName: 'Grouped Column Chart',
                sorters: {
                    Meal: sortAs(['Lunch', 'Dinner']),
                    'Day of Week': sortAs([
                        'Thursday',
                        'Friday',
                        'Saturday',
                        'Sunday',
                    ]),
                },
                plotlyOptions: { width: 640, height: 480 },
                plotlyConfig: {},
                tableOptions: {
                    clickCallback: function (e, value, filters, pivotData) {
                        var names = [];
                        pivotData.forEachMatchingRecord(filters, function (
                            record
                        ) {
                            names.push(record.Meal);
                        });
                        alert(names.join('\n'));
                    },
                },
            },
        });
    }

    render() {
        return (
            <div>
                <div className="row">
                    <h2 className="text-center">{this.state.filename}</h2>
                    <br />

                    <PivotTableUISmartWrapper {...this.state.pivotState} />
                </div>
            </div>
        );
    }
}
