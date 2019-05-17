import { Color } from './primitives';

/**
 * This file defines the type of the application theme object
 * @see https://www.styled-components.com/docs/advanced#theming
 */

/**
 * The application theme
 */
export interface IThemeProvider {
    colors: IColorProvider;
}

/**
 * The colors available in the application
 */
interface IColorProvider {
    fg: Color;
    bg: Color;

    accent: Color;
    subdued: Color;
    selected: Color;

    panel: Color;
    border: Color;

    success: Color;
    error: Color;
    warn: Color;
    info: Color;
    debug: Color;
}
