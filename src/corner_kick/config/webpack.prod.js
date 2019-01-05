/**
 * @fileoverview Specifies the webpack configuration for production.
 * Extends the general webpack configuration.
 *
 * @see {@link ./webpack.config.js}
 */

const webpackConfig = require('./webpack.config.js');

module.exports = [
    // Client build configuration
    {
        ...webpackConfig[0],
        mode: 'production',
    },
    {
        ...webpackConfig[1],
        mode: 'production',
    },
];
