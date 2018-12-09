/**
 * @fileoverview Defines a React Component that helps
 * organize our UI into various resizable pane.
 */

import * as React from 'react';
import PanelGroup from 'react-panelgroup';
import { Container, Subscribe } from 'unstated';

import { Persist } from 'RENDERER/components/containers/Persist';
import { DEFAULT_PANE_HEIGHT } from 'SHARED/constants';

/**
 * @description Props object for our PaneLayout.
 */
interface IPaneLayoutProps {
    /**
     * @description Top pane element
     */
    top: JSX.Element;

    /**
     * @description Bottom pane element
     */
    bottom: JSX.Element;
}

/**
 * @description A React component to display a split pane.
 * Pane size will be persisted between application runs.
 */
export const PaneLayout = (props: IPaneLayoutProps) => (
    <Subscribe to={[Persist('panels')]}>
        {(paneDimen: Container<any>) => mapPaneLayout(props, paneDimen)}
    </Subscribe>
);

/**
 * @description Function that takes persisted pane position data
 * and renders a pane layout
 * @param props the props object given by our parent
 * @param paneDimen a dimension object persisted between application run
 */
const mapPaneLayout = (props: IPaneLayoutProps, paneDimen: any) => {
    const horizontalWidths = paneDimen.state.horizontal || [
        { resize: 'stretch' },
        { size: DEFAULT_PANE_HEIGHT, resize: 'dynamic' },
    ];

    return (
        <PanelGroup
            direction="column"
            borderColor="grey"
            panelWidths={horizontalWidths}
            onUpdate={(data: any) => paneDimen.set({ horizontal: data })}
        >
            {props.top}
            {props.bottom}
        </PanelGroup>
    );
};
