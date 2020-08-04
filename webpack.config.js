const webpack = require('webpack');
const path = require('path');
// import path from 'path';

module.exports = {
  // webpack folder's entry js - excluded from jekll's build process.
  mode: 'development',
  entry: path.resolve(__dirname, 'src', 'entry.jsx'),
  output: {
    // we're going to put the generated file in the assets folder so jekyll will grab it.
    path: path.resolve(__dirname, 'assets/js'),
    filename: "bundle.js"
  },
  resolve: {
    extensions: [".ts", ".tsx", ".js", ".jsx"]
  },
  module: {
    rules: [{
        test: /\.(js)$/,
        exclude: /node_modules/,
        use: ['babel-loader']
      },
      {
        test: /\.css$/,
        use: ['style-loader', 'css-loader']
      },
      {
        test: /\.jsx/,
        exclude: /node_modules/,
        use: {
          loader: "babel-loader",
          options: {
            presets: ['@babel/preset-env',
              '@babel/preset-react', {
                'plugins': ['@babel/plugin-proposal-class-properties']
              }
            ]
          }
        }
      },
    ],
  },
  plugins: [
    new webpack.NamedModulesPlugin(),
    new webpack.HotModuleReplacementPlugin()
  ],
  devServer: {
    contentBase: './examples',
    hot: true
  },
};