/**
 * This file specifies test for Pool
 */

import { Pool } from '../pool';

describe('pool allocation/deallocation', () => {
    test('pool allocation returns the correct number of elements', () => {
        const creatorFunction = jest.fn(() => 'test');
        const pool = new Pool(creatorFunction, 100);

        creatorFunction.mockReset();
        const objects = pool.allocate(100);

        expect(objects.length).toBe(100);
        expect(creatorFunction).not.toBeCalled();
    });

    test('pool allocation calls the creator function', () => {
        const creatorFunction = jest.fn(() => 'test');
        const pool = new Pool(creatorFunction, 100);

        const objects = pool.allocate(200);

        expect(objects.length).toBe(200);
        expect(creatorFunction).toBeCalled();
    });

    test('test pool deallocation correctly adds the elements back to the pool', () => {
        const creatorFunction = jest.fn(() => 'test');
        const pool = new Pool(creatorFunction, 100);

        const oldObjects = new Array(100).fill('test');
        pool.unallocate(oldObjects);

        creatorFunction.mockReset();
        const objects = pool.allocate(200);

        expect(objects.length).toBe(200);
        expect(creatorFunction).not.toBeCalled();
    });
});
