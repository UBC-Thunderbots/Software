import { IROSState } from './ros';
import { ISettingsState } from './settings';
import { IVisualizerState } from './visualizer';

export interface IRootState {
    ros: IROSState;
    visualizer: IVisualizerState;
    settings: ISettingsState;
}
