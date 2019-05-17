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
    ...webpackConfig,
    mode: 'production',

    // We use source-maps to debug from our typescript files, rather than
    // from the bundle directly
    devtool: 'inline-source-map',

    // Webpack complains that our bundle is too large.
    // We run the visualizer locally so we can keep our bundles bigger
    // than the default max size.
    performance: {
        hints: false,
        maxEntrypointSize: 512000,
        maxAssetSize: 512000,
    },

    plugins: [
        ...webpackConfig.plugins,
        // Removes the build folder before bundling
        new CleanWebpackPlugin(path.resolve(__dirname, '../build'), {
            allowExternal: true,
            verbose: false,
        }),
    ],
};
