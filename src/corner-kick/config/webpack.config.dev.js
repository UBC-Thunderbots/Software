const webpackConfig = require('./webpack.config.js');

// The development webpack config. Uses settings defined in ./webpack.config.js.
module.exports = {
  ...webpackConfig,
  mode: 'development',
};
