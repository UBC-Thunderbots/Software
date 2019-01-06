import { ILoggerState } from './logger';
import { IROSState } from './ros';
import { ISettingsState } from './settings';
import { IVisualizerState } from './visualizer';

export interface IRootState {
    logger: ILoggerState;
    ros: IROSState;
    visualizer: IVisualizerState;
    settings: ISettingsState;
}
