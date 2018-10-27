import * as React from 'react';
import PanelGroup from 'react-panelgroup';
import { Container, Subscribe } from 'unstated';

import { Persist } from '~/components/containers/Persist';

interface ISplitpaneProps {
    top: JSX.Element;
    bottom: JSX.Element;
}

/**
 * A React component to display a split pane, a la VS Code.
 */
export const SplitPane = (props: ISplitpaneProps) => (
    <Subscribe to={[Persist('panels')]}>
        {(storage: Container<any>) => mapSplitPanes(props, storage)}
    </Subscribe>
);

const mapSplitPanes = (props: ISplitpaneProps, storage: any) => {
    
    const horizontalWidths = storage.state.horizontal 
    ||  [
            {resize: "stretch"},
            {size: 300, resize: "dynamic"},
        ];

        
    return (
        <PanelGroup direction="column" borderColor="grey" panelWidths={horizontalWidths} onUpdate={(data: any) => {
            storage.set({horizontal: data});
        }}>
            {props.top}
            {props.bottom}
        </PanelGroup>
    );
};
