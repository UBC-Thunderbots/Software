/*
 * Webpack configuration file for Storybook
 */

const path = require('path');

module.exports = (baseConfig, env, config) => {
    // We support Typescript
    config.module.rules.push({
        test: /\.(ts|tsx)$/,
        exclude: /node_modules/,
        use: [
            {
                loader: require.resolve('ts-loader'),
            },
        ],
    });
    config.resolve.extensions.push('.js', '.ts', '.tsx', '.json');
    // And map SRC to the src directory
    config.resolve.alias.SRC = path.resolve(__dirname, '../src');
    return config;
};
