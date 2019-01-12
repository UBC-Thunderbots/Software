/**
 * Specifies the webpack configuration for production.
 * Extends the general webpack configuration.
 *
 * @see {@link ./webpack.config.js}
 */

const path = require('path');
const CleanWebpackPlugin = require('clean-webpack-plugin');

const webpackConfig = require('./webpack.base.config.js');

module.exports = {
    ...webpackConfig,
    mode: 'production',
    devtool: false,
    // Move node-modules to a separate individual bundles
    optimization: {
        runtimeChunk: 'single',
        splitChunks: {
            chunks: 'all',
            maxInitialRequests: Infinity,
            minSize: 0,
            cacheGroups: {
                vendor: {
                    test: /[\\/]node_modules[\\/]/,
                    name(module) {
                        // get the name. E.g. node_modules/packageName/not/this/part.js
                        // or node_modules/packageName
                        const packageName = module.context.match(
                            /[\\/]node_modules[\\/](.*?)([\\/]|$)/,
                        )[1];

                        // npm package names are URL-safe, but some servers don't like @ symbols
                        return `npm.${packageName.replace('@', '')}`;
                    },
                },
            },
        },
    },
    // We run the visualizer locally so we can keep our bundles a bit bigger
    // than the default max size
    performance: {
        hints: false,
        maxEntrypointSize: 512000,
        maxAssetSize: 512000,
    },
    plugins: [
        ...webpackConfig.plugins,
        // Removes the build folder
        new CleanWebpackPlugin(path.resolve(__dirname, '../build'), {
            allowExternal: true,
            verbose: false,
        }),
    ],
};
