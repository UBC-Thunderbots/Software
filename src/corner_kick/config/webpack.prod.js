/**
 * Specifies the webpack configuration for production.
 * Extends the general webpack configuration.
 *
 * @see {@link ./webpack.config.js}
 */

const path = require('path');
const CleanWebpackPlugin = require('clean-webpack-plugin');

const webpackConfig = require('./webpack.base.config.js');

module.exports = {
    ...webpackConfig[0],
    mode: 'production',
    plugins: [
        ...webpackConfig[0].plugins,
        // Removes the build folder
        new CleanWebpackPlugin(path.resolve(__dirname, '../build'), {
            allowExternal: true,
            verbose: false,
        }),
    ],
};
