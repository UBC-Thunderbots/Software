/**
 * This file defines the visualizer graphics in the application
 */

import * as React from 'react';
import { connect } from 'react-redux';

import { IBall, IField, IRootState } from 'SRC/types';
import styled from 'SRC/utils/styled-components';

import { Ball } from './components/Ball';
import { Field } from './components/Field';

const Wrapper = styled.div`
    width: 100%;
    height: 100%;

    display: flex;
    justify-content: center;
    align-items: center;

    & svg {
        fill: transparent;
        stroke-width: 0.05px;
        stroke: ${(props) => props.theme.colors.subdued};
    }
`;

const mapStateToProps = (state: IRootState) => ({
    ball: state.visualizer.ball,
    field: state.visualizer.field,
});

interface IVisualizerMainProps {
    ball: IBall;
    field: IField;
}

class VisualizerMainInternal extends React.Component<IVisualizerMainProps> {
    public render() {
        const { ball, field } = this.props;
        return (
            <Wrapper>
                <svg
                    width="100%"
                    height="100%"
                    preserveAspectRatio="meet"
                    viewBox={[
                        0,
                        0,
                        field.boundaryWidth * 2 + field.fieldLength,
                        field.boundaryWidth * 2 + field.fieldWidth,
                    ].join(' ')}
                    xmlns="http://www.w3.org/2000/svg"
                >
                    <Field
                        boundaryWidth={field.boundaryWidth}
                        fieldLength={field.fieldLength}
                        fieldWidth={field.fieldWidth}
                        defenseLength={field.defenseLength}
                        defenseWidth={field.defenseWidth}
                        centerCircleRadius={field.centerCircleRadius}
                        goalWidth={0}
                    >
                        <Ball
                            x={ball.x}
                            y={ball.y}
                            dX={ball.dX}
                            dY={ball.dY}
                            fieldLength={field.fieldLength}
                            fieldWidth={field.fieldWidth}
                        />
                    </Field>
                </svg>
            </Wrapper>
        );
    }
}

export const VisualizerMain = connect(mapStateToProps)(VisualizerMainInternal);
