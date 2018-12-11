/**
 * @fileoverview Specifies the webpack configuration for development
 * and production. Provides support for Typescript and CSS files.
 */

const path = require('path');
const HtmlWebPackPlugin = require('html-webpack-plugin');

/**
 * @description Webpack build shared by renderer and main build.
 */
const generalWebpackBuild = {
    // We support js, typescript, html and css files.
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
        ],
    },

    resolve: {
        // Make sure if you add a new file support to add its extension here.
        extensions: ['.js', '.ts', '.tsx', '.json'],

        // We set an alias to our src directory to reduce the need for relative paths.
        alias: {
            RENDERER: path.resolve(__dirname, '../src/renderer'),
            MAIN: path.resolve(__dirname, '../src/main'),
            SHARED: path.resolve(__dirname, '../src/shared'),
        },
    },
};

module.exports = [
    // Renderer build configuration
    {
        ...generalWebpackBuild,
        // Our project entry point.
        entry: path.resolve(__dirname, '../src/renderer/index.ts'),

        // We generate a bundle in the build folder
        output: {
            path: path.resolve(__dirname, '../build'),
            filename: 'renderer.js',
        },

        // This target allows us to access DOM and Node libraries simultaneously
        target: 'electron-renderer',
        // Our plugins go here.
        plugins: [
            // This plugins autogenerates our index.html files and links the javascript bundle.
            new HtmlWebPackPlugin({
                template: './src/renderer/index.html',
                filename: './index.html',
            }),
        ],
    },
    // Main build configuration
    {
        ...generalWebpackBuild,
        // Our project entry point.
        entry: path.resolve(__dirname, '../src/main/index.ts'),

        // We generate a bundle in the build folder
        output: {
            path: path.resolve(__dirname, '../build'),
            filename: 'main.js',
        },

        // This target allows us to access Node libraries
        target: 'electron-main',
    },
];
