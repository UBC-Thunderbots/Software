import * as React from 'react';
import PanelGroup from 'react-panelgroup';
import { Container, Subscribe } from 'unstated';

import { Persist } from '~/components/containers/Persist';

interface ISplitpaneProps {
  top: JSX.Element;
  bottom: JSX.Element;
}

/**
 * A React component to display a split pane.
 * Pane size will be persisted between application runs.
 */
export const SplitPane = (props: ISplitpaneProps) => (
  <Subscribe to={[Persist('panels')]}>
    {(paneDimen: Container<any>) => mapSplitPanes(props, paneDimen)}
  </Subscribe>
);

/**
 *
 * @param props the props object given by our parent
 * @param paneDimen a dimension object persisted between application run
 */
const mapSplitPanes = (props: ISplitpaneProps, paneDimen: any) => {
  const horizontalWidths = paneDimen.state.horizontal || [
    { resize: 'stretch' },
    { size: 300, resize: 'dynamic' },
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
