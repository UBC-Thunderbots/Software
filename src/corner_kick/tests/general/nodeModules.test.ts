import * as fs from 'fs';
import * as path from 'path';

const packageJSON = require(path.resolve(__dirname, '../../package.json'));

describe('check for fixed version', () => {
    describe('for dependency', () => {
        // Static dependencies ensure reliability in the codebase accross
        // computers. However, yarn and npm does not peg dependency version
        // automatically. Hence this test makes sure that any new dependencies
        // has its version fixed.

        Object.keys(packageJSON.dependencies).forEach((key: string) => {
            test(`${key}`, () => {
                expect(packageJSON.dependencies[key]).not.toMatch(/^\^.*$/);
                expect(packageJSON.dependencies[key]).not.toMatch(/^\~.*$/);
            });
        });
    });

    describe('for devDependencies', () => {
        // Static dependencies ensure reliability in the codebase accross
        // computers. However, yarn and npm does not peg dependency version
        // automatically. Hence this test makes sure that any new dependencies
        // has its version fixed.

        Object.keys(packageJSON.devDependencies).forEach((key: string) => {
            test(`${key}`, () => {
                expect(packageJSON.devDependencies[key]).not.toMatch(/^\^.*$/);
                expect(packageJSON.devDependencies[key]).not.toMatch(/^\~.*$/);
            });
        });
    });
});

test('that the yarn version in package.json matches the yarn file committed', () => {
    const yarnVersion = packageJSON.engines.yarn;

    expect(
        fs.existsSync(path.resolve(__dirname, `../../scripts/yarn-${yarnVersion}.js`)),
    ).toBe(true);
});
