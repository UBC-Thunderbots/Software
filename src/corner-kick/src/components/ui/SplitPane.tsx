import * as React from 'react';
import PanelGroup from 'react-panelgroup';
import { Container, Subscribe } from 'unstated';

import { Persist } from '~/components/containers/Persist';

interface ISplitpaneProps {
    left: JSX.Element;
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

const mapSplitPanes = (props: ISplitpaneProps, storage: Container<any>) => {

    const verticalWidths = storage.state.vertical 
        ||  [
                {size: 300, resize: "dynamic"},
                {resize: "stretch"},
            ];
    
    const horizontalWidths = storage.state.horizontal 
    ||  [
            {resize: "stretch"},
            {size: 300, resize: "dynamic"},
        ];

        
    return (
        <PanelGroup borderColor="grey" panelWidths={verticalWidths} onUpdate={(data: any) => {
            console.log(data);
        }}>
                {props.left}
                <PanelGroup direction="column" borderColor="grey" panelWidths={horizontalWidths}>
                    {props.top}
                    {props.bottom}
                </PanelGroup>
        </PanelGroup>
    );
};
