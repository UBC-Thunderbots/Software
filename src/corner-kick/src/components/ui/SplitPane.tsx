import * as React from 'react';
import PanelGroup from 'react-panelgroup';

/**
 * A React component to display a split pane, a la VS Code.
 */
export const SplitPane = (props: {
    left: JSX.Element,
    top: JSX.Element,
    bottom: JSX.Element
}) => (
    <PanelGroup borderColor="grey" panelWidths={[
        {size: 300, resize: "dynamic"},
        {resize: "stretch"},
    ]}>
        {props.left}
        <PanelGroup direction="column" borderColor="grey" panelWidths={[
            {resize: "stretch"},
            {size: 300, resize: "dynamic"},
        ]}>
            {props.top}
            {props.bottom}
        </PanelGroup>
    </PanelGroup>
);
