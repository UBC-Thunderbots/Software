# Portals - Design notes

This document outlines and specifies the implementation of portals, and why they are
used in this application. I strongly recommend reading more about
[React Portal in the React Doc](https://reactjs.org/docs/portals.html).

**Table of content**

- [Portals - Design notes](#portals---design-notes)
    - [How to use](#how-to-use)
        - [Example](#example)
    - [Implementation overview](#implementation-overview)

## How to use

Portals allow us to access specific areas in the application layout, from
anywhere in the React component tree. Any children to a portal will be
added to the area specified by the `portalLocation` parameter.

Here are the areas available via Portals:

-   **Main:** Contains the primary focus of the page
-   **Sidebar title:** Contains the page title and additional options specific to the page
-   **Sidebar:** Contains additional controls or information relevant to the current page
-   **Sidebar control:** Contains navigation controls to switch between pages
-   **Console:** Contains views visible across the application
-   **Footer:** Contains simple breadcrumbs of information relevant to the application as a whole or to the current page

### Example

The following gist adds content to the main and sidebar area.

```JSX
<>
    <Portal portalLocation={PortalLocation.MAIN}>
        This is main
    </Portal>
    <Portal portalLocation={PortalLocation.SIDEBAR}>
        This is sidebar
    </Portal>
</>
```

## Implementation overview

Portals are used to ensure the application layout is enforced. Any content rendered outside
of a portal is hidden from the user by design. Because the application uses a consistent layout
(sidebar + main + console), we want to make sure that at all time we are respecting that layout.

React portals ensure this happens by offering us a way to insert content into our application
layout, without worrying about how it is defined.
