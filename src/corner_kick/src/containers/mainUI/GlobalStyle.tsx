/**
 * This file defines the main styling of the application and connects the
 * color theme to styled-components
 */

import * as React from 'react';
import { connect } from 'react-redux';

import { IRootState, ISettingsState, IThemeProvider } from 'SRC/types';
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

        font-family: 'Open Sans', sans-serif;
        font-size: 14px;
        color: ${(props) => props.theme.colors.fg}
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
        grid-template-rows: 40px 1fr 300px 20px;
    }

    #sidebarTitle {
        grid-row: 1;
        grid-column: 2;
        overflow: hidden;

        border-right: 1px solid ${(props) => props.theme.colors.border};
    }

    #sidebarControl {
        grid-row: 1 / 4;
        grid-column: 1;
        border-top: 1px solid ${(props) => props.theme.colors.border};
        border-right: 1px solid ${(props) => props.theme.colors.border};

        display: flex;
        flex-flow: column nowrap;

        overflow: hidden;
    }

    #sidebar {
        grid-row: 2 / 4;
        grid-column: 2;
        border-top: 1px solid ${(props) => props.theme.colors.border};
        border-right: 1px solid ${(props) => props.theme.colors.border};
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
        background: ${(props) => props.theme.colors.panel};
        overflow: hidden;
        border-top: 1px solid ${(props) => props.theme.colors.border};
    }

    #console {
        grid-row: 3;
        grid-column: 3;
        overflow: hidden;
        border-top: 1px solid ${(props) => props.theme.colors.border};
    }

    #footerLeft {
        grid-row: 4;
        grid-column: 1 / 3;
        border-top: 1px solid ${(props) => props.theme.colors.border};

        display: flex;
        justify-content: flex-start;
        align-items: center;
        padding: 0px 4px;
        overflow: hidden;
    }

    #footerRight {
        grid-row: 4;
        grid-column: 3;
        border-top: 1px solid ${(props) => props.theme.colors.border};

        display: flex;
        justify-content: flex-end;
        align-items: center;
        padding: 0px 4px;
        overflow: hidden;
    }
`;

const mapStateToProps = (state: IRootState) => ({
    settings: state.settings,
});

interface ISettingsMainProps {
    settings: ISettingsState;
}

class GlobalStyleInternal extends React.Component<ISettingsMainProps> {
    public render() {
        const { settings } = this.props;

        // tslint:disable:object-literal-sort-keys
        const theme: IThemeProvider = {
            colors: {
                fg: settings.colors_fg,
                bg: settings.colors_bg,

                accent: settings.colors_accent,
                subdued: settings.colors_subdued,
                selected: settings.colors_selection,

                panel: settings.colors_panel,
                border: settings.colors_border,

                success: settings.colors_success,
                error: settings.colors_error,
                warn: settings.colors_warn,
                info: settings.colors_info,
                debug: settings.colors_debug,

                red: settings.colors_red,
                orange: settings.colors_orange,
                yellow: settings.colors_yellow,
                green: settings.colors_green,
                cyan: settings.colors_cyan,
                blue: settings.colors_blue,
                purple: settings.colors_purple,
            },
        };

        return (
            <ThemeProvider theme={theme}>
                <>
                    <GlobalStylesheet />
                    {this.props.children}
                </>
            </ThemeProvider>
        );
    }
}

export const GlobalStyle = connect(mapStateToProps)(GlobalStyleInternal);
