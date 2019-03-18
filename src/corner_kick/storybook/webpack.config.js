/**
 * This file specifies the configuration for Storybook
 */

const path = require('path');
module.exports = ({ config, mode }) => {
    // Add support for Typescript
    config.module.rules.push({
        test: /\.(ts|tsx)$/,
        use: [
            {
                loader: require.resolve('ts-loader'),
            },
        ],
    });
    config.resolve.extensions.push('.ts', '.tsx');

    // Add alias to src folder
    config.resolve.alias['SRC'] = path.resolve(__dirname, '../src');

    return config;
};
