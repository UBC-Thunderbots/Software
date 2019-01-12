/**
 * Specifies the webpack configuration for the application
 * in development. Extends the general webpack configuration.
 *
 * @see {@link ./webpack.config.js}
 */

const webpackConfig = require('./webpack.base.config.js');

module.exports = {
    ...webpackConfig,
    mode: 'development',

    // We use source-maps to debug from our typescript files, rather than
    // from the bundle directly
    devtool: 'inline-source-map',

    // Because the client uses the history API, we redirect all
    // browser request to the root path
    devServer: {
        publicPath: '/',
        historyApiFallback: true,
        quiet: true,
    },
};
