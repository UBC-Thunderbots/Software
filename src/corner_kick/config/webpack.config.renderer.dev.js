/**
 * @fileoverview Specifies the webpack configuration for the renderer
 * in development. Extends the general webpack configuration.
 *
 * @see {@link ./webpack.config.js}
 */

const webpackConfig = require('./webpack.config.js');

module.exports = {
    ...webpackConfig[0],
    mode: 'development',

    // We use source-maps to debug from our typescript files, rather than
    // from the bundle directly
    devtool: 'inline-source-map',
};
