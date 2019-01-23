# Sidebar - Design notes

This document outlines the design and implementation of the sidebar. The sidebar consists of a number
of developer specified panels containing various controls complementing the current page. Those pane
can be opened or closed and resized, allowing the entire sidebar UI to adapt to the user's current
needs.

## How to use

The sidebar is composed of panels, which hold any custom UI specified by the developer. In this implementation,
panels are represented by a React component, which provides the sidebar with information on the title of the panel,
as well as its content.

The content of the sidebar (panels + resizeable logic) will automagically be added to the corresponding sidebar area in
the application layout.

### Example

The following gist creates a sidebar, adds it to the sidebar area in the application's main layout, and creates three panels
with the content specified inside the `<Panel>` tag

```JSX
<Sidebar>
    <Panel title="Layers">
        This is the content of the layers panel
    </Panel>
    <Panel title="AI Controls">
        This is the content of the AI control panel
    </Panel>
    <Panel title="Game Status">
        This is the content of the Game Status panel
    </Panel>
</Sidebar>
```

## Implementation overview

### The resize operation

## Performance
