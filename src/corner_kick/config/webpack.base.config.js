/**
 * Specifies the webpack configuration for development
 * and production. Provides support for Typescript and CSS files.
 */

const path = require('path');
const webpack = require('webpack');
const HtmlWebPackPlugin = require('html-webpack-plugin');
const FriendlyErrorPlugin = require('friendly-errors-webpack-plugin');

/**
 * @description Webpack build.
 */
const generalWebpackBuild = {
    stats: false,
    // We support js, typescript, html, css, and image files.
    // To add additional file support, add the required loader here.
    module: {
        rules: [
            {
                test: /\.tsx?$/,
                exclude: [/node_modules/, /__.*__/],
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
                test: /\.(jpe?g|gif|bmp|mp3|mp4|ogg|wav|eot|ttf|woff|woff2|png|svg)$/,
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
        // These are the extensions that Webpack will try to resolve (aka transpile into
        // a .js bundle)
        extensions: ['.js', '.ts', '.tsx', '.json'],

        // We set an alias to our src directory to reduce the need for relative paths.
        alias: {
            SRC: path.resolve(__dirname, '../src'),
        },
    },
};

// Build configurations
module.exports = {
    ...generalWebpackBuild,
    // Our project entry point.
    entry: path.resolve(__dirname, '../src/index.ts'),

    // We generate a bundle in the build folder
    output: {
        path: path.resolve(__dirname, '../build'),
        filename: '[name].[contenthash].js',
    },

    // We use the web target to give us access to DOM-related JS functions
    // A good analogy is selecting the STD library used in a C/C++ project
    target: 'web',

    // Our plugins go here.
    plugins: [
        // This plugins autogenerates our index.html files and links the javascript bundle.
        new HtmlWebPackPlugin({
            template: './src/index.html',
            filename: './index.html',
        }),
        // This plugin simplifies the webpack output and provides easy to read suggestions
        // when code does not compile.
        new FriendlyErrorPlugin(),
        new webpack.HashedModuleIdsPlugin(),
    ],
};
