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
 * The color available in the application
 */
interface IColorProvider {
    fg: string;
    bg: string;

    accent: string;
    subdued: string;
    selected: string;

    panel: string;
    border: string;

    success: string;
    error: string;
    warn: string;
    info: string;
    debug: string;

    red: string;
    orange: string;
    yellow: string;
    green: string;
    cyan: string;
    blue: string;
    purple: string;
}
