import { spawn } from 'redux-saga/effects';

import initLogger from './logger';
import initROS from './ros';
import initSettings from './settings';

export function* init() {
    yield spawn(initSettings);
    yield spawn(initROS);
    yield spawn(initLogger);
}
