import { spawn } from 'redux-saga/effects';

import initLogger from './logger';
import initROS from './ros';
import initSettings from './settings';
import initVisualizer from './visualizer';

export function* init() {
    yield spawn(initSettings);
    yield spawn(initROS);
    yield spawn(initLogger);
    yield spawn(initVisualizer);
}
