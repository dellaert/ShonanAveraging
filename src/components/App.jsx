// // @flow
// import React from 'react';
// import tips from '../tips';
// import PlotlyRenderers from 'react-plotly';
// import TableRenderers from 'react-plotly';

// import PivotTableUI from 'react-pivottable/PivotTableUI';
// import sortAs from 'react-pivottable/Utilities';
// import 'react-pivottable/pivottable.css';
// import Dropzone from 'react-dropzone';
// import Papa from 'papaparse';

// const Plot = PlotlyRenderers.createPlotlyComponent(window.Plotly);

// type Props = {};

// type State = {
//     pivotState: any,
// };

// class PivotTableUISmartWrapper extends React.PureComponent<Props, State> {
//     constructor(props) {
//         super(props);
//         this.state = { pivotState: props };
//     }

//     componentWillReceiveProps(nextProps) {
//         this.setState({ pivotState: nextProps });
//     }

//     render() {
//         return (
//             <PivotTableUI
//                 renderers={Object.assign(
//                     {},
//                     TableRenderers,
//                     PlotlyRenderers.createPlotlyRenderers(Plot)
//                 )}
//                 {...this.state.pivotState}
//                 onChange={s => this.setState({ pivotState: s })}
//                 unusedOrientationCutoff={Infinity}
//             />
//         );
//     }
// }

// type Props2 = {};

// type State2 = {
//     mode: string,
//     filename: string,
//     pivotState: any,
//     textarea: string
// };

// type DroppedFile = { name: string };

// export default class App extends React.Component<Props2, State2> {
//     componentWillMount() {
//         this.setState({
//             mode: 'demo',
//             filename: 'Sample Dataset: Tips',
//             pivotState: {
//                 data: tips,
//                 rows: ['Payer Gender'],
//                 cols: ['Party Size'],
//                 aggregatorName: 'Sum over Sum',
//                 vals: ['Tip', 'Total Bill'],
//                 rendererName: 'Grouped Column Chart',
//                 sorters: {
//                     Meal: sortAs(['Lunch', 'Dinner']),
//                     'Day of Week': sortAs([
//                         'Thursday',
//                         'Friday',
//                         'Saturday',
//                         'Sunday',
//                     ]),
//                 },
//                 plotlyOptions: { width: 900, height: 500 },
//                 plotlyConfig: {},
//                 tableOptions: {
//                     clickCallback: function (e, value, filters, pivotData) {
//                         var names = [];
//                         pivotData.forEachMatchingRecord(filters, function (
//                             record
//                         ) {
//                             names.push(record.Meal);
//                         });
//                         alert(names.join('\n'));
//                     },
//                 },
//             },
//         });
//     }

//     onDrop(files: Array<DroppedFile>) {
//         this.setState(
//             {
//                 mode: 'thinking',
//                 filename: '(Parsing CSV...)',
//                 textarea: '',
//                 pivotState: { data: [] },
//             },
//             () =>
//                 Papa.parse(files[0], {
//                     skipEmptyLines: true,
//                     error: e => alert(e),
//                     complete: parsed =>
//                         this.setState({
//                             mode: 'file',
//                             filename: files[0].name,
//                             pivotState: { data: parsed.data },
//                         }),
//                 })
//         );
//     }

//     onType(event: SyntheticInputEvent<HTMLInputElement>) {
//         Papa.parse(event.target.value, {
//             skipEmptyLines: true,
//             error: e => alert(e),
//             complete: parsed =>
//                 this.setState({
//                     mode: 'text',
//                     filename: 'Data from <textarea>',
//                     textarea: event.target.value,
//                     pivotState: { data: parsed.data },
//                 }),
//         });
//     }

//     render() {
//         return (
//             <div>
//                 <div className="row text-center">
//                     <div className="col-md-3 col-md-offset-3">
//                         <p>Try it right now on a file...</p>
//                         <Dropzone
//                             onDrop={this.onDrop.bind(this)}
//                             accept="text/csv"
//                             className="dropzone"
//                             activeClassName="dropzoneActive"
//                             rejectClassName="dropzoneReject"
//                         >
//                             <p>
//                                 Drop a CSV file here, or click to choose a file
//                                 from your computer.
//                             </p>
//                         </Dropzone>
//                     </div>
//                     <div className="col-md-3 text-center">
//                         <p>...or paste some data:</p>
//                         <textarea
//                             value={this.state.textarea}
//                             onChange={this.onType.bind(this)}
//                             placeholder="Paste from a spreadsheet or CSV-like file"
//                         />
//                     </div>
//                 </div>
//                 <div className="row text-center">
//                     <p>
//                         <em>Note: the data never leaves your browser!</em>
//                     </p>
//                     <br />
//                 </div>
//                 <div className="row">
//                     <h2 className="text-center">{this.state.filename}</h2>
//                     <br />

//                     <PivotTableUISmartWrapper {...this.state.pivotState} />
//                 </div>
//             </div>
//         );
//     }
// }
