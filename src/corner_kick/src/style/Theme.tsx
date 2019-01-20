/**
 * This file defines the main theme of the application and connects the
 * theme to all styled-components instances. The application layout is
 * specified in style.css
 */

import * as React from 'react';
import { connect } from 'react-redux';

import { IRootState, ISettingsState, IThemeProvider } from 'SRC/types';
import { createGlobalStyle, ThemeProvider } from 'SRC/utils/styled-components';

const GlobalStylesheet = createGlobalStyle`
    body {
        background: ${(props) => props.theme.colors.bg};
        color: ${(props) => props.theme.colors.fg};
    }

    #root > div {
        border-color: ${(props) => props.theme.colors.border} !important;
    }

    #main {
        background: ${(props) => props.theme.colors.panel};
    }
`;

const mapStateToProps = (state: IRootState) => ({
    settings: state.settings,
});

interface ISettingsMainProps {
    settings: ISettingsState;
}

/**
 * Adds application theme to the React tree
 */
class ThemeInternal extends React.Component<ISettingsMainProps> {
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

export const Theme = connect(mapStateToProps)(ThemeInternal);
