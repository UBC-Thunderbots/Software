const webpackConfig = require('./webpack.config.js');

// The production webpack config. Uses settings defined in ./webpack.config.js.
module.exports = {
  ...webpackConfig,
  mode: 'production',
};
