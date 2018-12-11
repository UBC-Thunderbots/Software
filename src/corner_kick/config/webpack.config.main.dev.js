/**
 * @fileoverview Specifies the webpack configuration for the main in
 * development. Extends the general webpack configuration.
 *
 * @see {@link ./webpack.config.js}
 */

const webpackConfig = require('./webpack.config.js');

module.exports = {
    ...webpackConfig[1],
    mode: 'development',
};
