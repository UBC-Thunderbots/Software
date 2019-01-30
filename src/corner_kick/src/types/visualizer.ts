/*
 * This file describes a layer datatype
 */

export interface ILayerMessage {
    name: string;
    shapes: IShape[];
}

export interface ILayer extends ILayerMessage {
    visible: boolean;
}

export interface IShape {
    type: string;
    data: number[];
    fill?: string;
    stroke?: string;
    stroke_weight?: number;
    transform_rotation?: number;
    transform_scale?: number;
}
