const path = require('path');

module.exports = (baseConfig, env, config) => {
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
    config.resolve.alias.SRC = path.resolve(__dirname, '../src');
    return config;
};
