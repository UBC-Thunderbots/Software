/**
 * This file is for testing ROS actions
 * Each test case has a description of what it tests
 */

import * as rosActions from '../ros';

describe('rosActions', () => {
    describe('when we connect to ROS', () => {
        it('dispatches the correct start action and payload', () => {
            expect(rosActions.start()).toMatchSnapshot();
        });
    });

    describe('when we disconnect to ROS', () => {
        it('dispatches the correct stop action and payload', () => {
            expect(rosActions.stop()).toMatchSnapshot();
        });
    });

    describe('when we are connected to ROS', () => {
        it('dispatches the correct connect action and payload', () => {
            expect(rosActions.connected()).toMatchSnapshot();
        });
    });

    describe('when we are disconnected to ROS', () => {
        it('dispatches the correct disconnect action and payload', () => {
            expect(rosActions.disconnected()).toMatchSnapshot();
        });
    });

    describe('when there is an error connecting to ROS', () => {
        it('dispatches the correct error action and payload', () => {
            expect(rosActions.error('Error getting pan-tilt limit')).toMatchSnapshot();
        });
    });
});
