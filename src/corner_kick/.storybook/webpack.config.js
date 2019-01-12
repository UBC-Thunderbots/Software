/**
 * This file specifies the webpack config for Storybook
 */

const webpackConfig = require('../config/webpack.base.config.js');

// Export a webpack config for Storybook. Based from the default
// webpack config for this project.
module.exports = (storybookBaseConfig) => {
    return {
        ...storybookBaseConfig,
        ...webpackConfig[1],
    };
};
