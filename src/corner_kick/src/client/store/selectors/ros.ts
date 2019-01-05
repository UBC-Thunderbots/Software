import { IRootState } from '../state';

export const nodesSelector = (state: IRootState) => state.ros.nodes;
export const topicsSelector = (state: IRootState) => state.ros.topics;
export const paramsSelector = (state: IRootState) => state.ros.params;
export const servicesSelector = (state: IRootState) => state.ros.services;
