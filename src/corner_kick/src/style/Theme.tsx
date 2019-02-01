/*
 * This file defines the main theme of the application and connects the
 * theme to all styled-components instances. The application layout is
 * specified in style.css
 */

import * as React from 'react';

import { theme } from 'SRC/constants';
import { createGlobalStyle, ThemeProvider } from 'SRC/utils/styled-components';

const GlobalStylesheet = createGlobalStyle`
    @import url('https://fonts.googleapis.com/css?family=Open+Sans');

    * {
        box-sizing: border-box;
    }


    body {
        margin: 0;

        overflow: hidden;
        user-select: none;

        background: ${(props) => props.theme.colors.bg};
        color: ${(props) => props.theme.colors.fg};

        font-family: 'Open Sans', sans-serif;
        font-size: 14px;
    }

    a {
        text-decoration: none;
    }

    #root > div {
        border-color: ${(props) => props.theme.colors.border} !important;
    }

    #main {
        background: ${(props) => props.theme.colors.panel};
    }
`;

/**
 * Adds application theme to the React tree
 */
export const Theme = (props: { children: React.ReactNode | React.ReactNodeArray }) => {
    return (
        <ThemeProvider theme={theme}>
            <>
                <GlobalStylesheet />
                {props.children}
            </>
        </ThemeProvider>
    );
};
