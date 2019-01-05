/**
 * @fileoverview Specifies the webpack configuration for development
 * and production. Provides support for Typescript and CSS files.
 */

const path = require('path');
const HtmlWebPackPlugin = require('html-webpack-plugin');
const nodeExternal = require('webpack-node-externals');

/**
 * @description Webpack build.
 */
const generalWebpackBuild = {
    // We support js, typescript, html, css, and image files.
    // To add additional file support, add the required loader here.
    module: {
        rules: [
            {
                test: /\.tsx?$/,
                exclude: /node_modules/,
                use: {
                    loader: 'ts-loader',
                },
            },
            {
                test: /\.html$/,
                use: [
                    {
                        loader: 'html-loader',
                        options: { minimize: false },
                    },
                ],
            },
            {
                test: /\.css$/,
                use: ['style-loader', 'css-loader'],
            },
            {
                test: /\.(png|jpg|gif|svg)$/,
                use: [
                    {
                        loader: 'file-loader',
                        options: {},
                    },
                ],
            },
        ],
    },
    resolve: {
        // Make sure if you add a new file support to add its extension here.
        extensions: ['.mjs', '.js', '.ts', '.tsx', '.json'],

        // We set an alias to our src directory to reduce the need for relative paths.
        alias: {
            SRC: path.resolve(__dirname, '../src/client'),
        },
    },
};

module.exports = [
    // Client build configuration
    {
        ...generalWebpackBuild,
        // Our project entry point.
        entry: path.resolve(__dirname, '../src/client/index.ts'),

        // We generate a bundle in the build folder
        output: {
            path: path.resolve(__dirname, '../build'),
            filename: 'client.js',
        },

        // This target allows us to access DOM libraries simultaneously
        target: 'web',
        // Our plugins go here.
        plugins: [
            // This plugins autogenerates our index.html files and links the javascript bundle.
            new HtmlWebPackPlugin({
                template: './src/client/index.html',
                filename: './index.html',
            }),
        ],
    },
    {
        ...generalWebpackBuild,
        // Our project entry point.
        entry: path.resolve(__dirname, '../src/main/index.ts'),

        // We generate a bundle in the build folder
        output: {
            path: path.resolve(__dirname, '../build'),
            filename: 'main.js',
        },

        // This target allows us to access DOM libraries simultaneously
        target: 'electron-main',
    },
];
