/*
 * Webpack configuration file for Storybook
 */

const webpackConfig = require('../config/webpack.base.config.js');

module.exports = (baseConfig, env, config) => {
    // Add Typescript and absolute paths support from the application
    // general webpack build
    return {
        ...config,
        ...webpackConfig.general,
    };
};
