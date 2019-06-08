/*
 * This file groups all application saga's and specifies a way to start them all
 *
 * Sagas provide asynchronous business logic to a Redux state-driven application.
 * @see https://github.com/redux-saga/redux-saga
 */

import { spawn } from 'redux-saga/effects';

import initROS from './ros';
import initThunderbots from './thunderbots';

/**
 * Starts all application sagas
 */
export function* init() {
    yield spawn(initROS);
    yield spawn(initThunderbots);
}
