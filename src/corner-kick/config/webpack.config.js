const path = require('path');

const HtmlWebPackPlugin = require('html-webpack-plugin');

// Webpack configuration for development and production build.
module.exports = {
  // Our project entry point.
  entry: path.resolve(__dirname, '../src/index.tsx'),

  // We generate a bundle in the build folder
  output: {
    path: path.resolve(__dirname, '../build'),
    filename: 'index.js',
  },

  // This target allows us to access DOM and Node libraries simultaneously
  target: 'electron-renderer',

  // We use source-maps to debug from our typescript files, rather than
  // from the bundle directly
  devtool: 'inline-source-map',

  // We support js, typescript, html and css files.
  // To add additional file support, add the required loader here.
  module: {
    rules: [
      {
        test: /\.tsx?$/,
        exclude: /node_modules/,
        use: {
          loader: 'ts-loader',
        },
      },
      {
        test: /\.html$/,
        use: [
          {
            loader: 'html-loader',
            options: { minimize: false },
          },
        ],
      },
      {
        test: /\.css$/,
        use: ['style-loader', 'css-loader'],
      },
    ],
  },

  resolve: {
    // Make sure if you add a new file support to add its extension here.
    extensions: ['.js', '.ts', '.tsx', '.jsx', '.json'],

    // We set an alias to our src directory to reduce the need for relative paths.
    alias: {
      '~': path.resolve(__dirname, '../src'),
    },
  },

  // Our plugins go here.
  plugins: [
    // This plugins autogenerates our index.html files and links the javascript bundle.
    new HtmlWebPackPlugin({
      template: './public/index.html',
      filename: './index.html',
    }),
  ],
};
