import { dark } from 'ayu';
import { createGlobalStyle } from 'styled-components';

export const Style = createGlobalStyle`
    @import url('https://fonts.googleapis.com/css?family=Open+Sans');

    * {
        box-sizing: border-box;
    }

    body {
        margin: 0;

        overflow: hidden;
        user-select: none;

        background: ${dark.common.bg};

        font-family: 'Open Sans', sans-serif;
        font-size: 14px;
        color: ${dark.common.fg}
    }

    a {
        text-decoration: none;
    }

    canvas {
        /*
        Canvas should take 100% of its parent.
        Necessary for automatic canvas resizing.
        */
        width: 100% !important;
    }

    #react {
        display: hidden;
    }

    #root {
        width: 100vw;
        height: 100vh;

        display: grid;
        grid-template-columns: 56px 300px 1fr;
        grid-template-rows: 40px 1fr 20px;
    }

    #sidebarTitle {
        grid-row: 1;
        grid-column: 2;
        overflow: hidden;

        border-right: 1px solid ${dark.ui.line};
    }

    #sidebarControl {
        grid-row: 1 / 3;
        grid-column: 1;
        border-top: 1px solid ${dark.ui.line};
        border-right: 1px solid ${dark.ui.line};

        display: flex;
        flex-flow: column nowrap;

        overflow: hidden;
    }

    #sidebar {
        grid-row: 2;
        grid-column: 2;
        border-top: 1px solid ${dark.ui.line};
        border-right: 1px solid ${dark.ui.line};
        overflow: hidden;
    }

    #mainTitle {
        grid-row: 1;
        grid-column: 3;

        display: flex;

        overflow: hidden;
    }

    #main {
        grid-row: 2;
        grid-column: 3;
        background: ${dark.ui.panel.bg.hex()};
        overflow: hidden;
        border-top: 1px solid ${dark.ui.line.hex()};
    }

    #footerLeft {
        grid-row: 3;
        grid-column: 1 / 3;
        border-top: 1px solid ${dark.ui.line};

        display: flex;
        justify-content: flex-start;
        align-items: center;
        padding: 0px 4px;
        overflow: hidden;
    }

    #footerRight {
        grid-row: 3;
        grid-column: 3;
        border-top: 1px solid ${dark.ui.line};

        display: flex;
        justify-content: flex-end;
        align-items: center;
        padding: 0px 4px;
        overflow: hidden;
    }
`;
