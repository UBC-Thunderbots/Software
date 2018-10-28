const webpackConfig = require('./webpack.config.js');

// The development webpack config. Uses settings defined in ./webpack.config.js.
module.exports = {
  ...webpackConfig,
  mode: 'development',

  // We use source-maps to debug from our typescript files, rather than
  // from the bundle directly
  devtool: 'inline-source-map',
};
