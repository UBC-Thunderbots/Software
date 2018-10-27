const path = require('path');

const HtmlWebPackPlugin = require("html-webpack-plugin");

module.exports = {
    entry: path.resolve(__dirname, '../src/index.tsx'),
    output: {
        path: path.resolve(__dirname, '../build'),
        filename: 'index.js'
    },
    target: 'electron-renderer',
    mode: "development",
    module: {
      rules: [
        {
          test: /\.tsx?$/,
          exclude: /node_modules/,
          use: {
            loader: "ts-loader"
          }
        },
        {
            test: /\.html$/,
            use: [
                {
                loader: "html-loader",
                options: { minimize: false }
                }
            ]
        },
        {
            test:/\.css$/,
            use:[
                'style-loader',
                'css-loader'
            ]
        }
      ]
    },
    resolve: {
        extensions: ['.js', '.ts', '.tsx', '.jsx', '.json'],
        alias: {
            "~": path.resolve(__dirname, '../src')
        }
    },
    devServer: {
        watchContentBase: true
    },
    plugins: [
        new HtmlWebPackPlugin({
            template: "./public/index.html",
            filename: "./index.html"
        })
    ]
};