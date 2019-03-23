/**
 * The configuration file for Jest
 *
 * Jest is the library used for unit testing components
 * such as actions and reducers
 */
module.exports = {
    rootDir: '..',
    roots: ['<rootDir>/tests'],
    // Assigning all tsx files to be interpreted by ts-jest
    transform: {
        '^.+\\.tsx?$': 'ts-jest',
    },
    // Any files with extension (spec|test).ts(x) is a test file
    testRegex: '(\\.|/)(test|spec)\\.tsx?$',
    moduleFileExtensions: ['ts', 'tsx', 'js', 'jsx', 'json', 'node'],
    // SRC points to the src folder
    moduleNameMapper: {
        '^SRC(.*)$': '<rootDir>/src$1',
    },
    coverageDirectory: './coverage/',
    collectCoverageFrom: ['**/*.{ts,tsx}', '!**/node_modules/**', '!**/build/**'],
};
