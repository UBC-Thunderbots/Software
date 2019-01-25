/*
 * This file specifies the theme typing for the style-component library
 * @see https://www.styled-components.com/docs/api#typescript
 */
import * as styledComponents from 'styled-components';
import { ThemedStyledComponentsModule } from 'styled-components';

import { IThemeProvider } from 'SRC/types';

// We recreate styled-components with theme typings attached
const {
    default: styled,
    css,
    createGlobalStyle,
    keyframes,
    ThemeProvider,
} = styledComponents as ThemedStyledComponentsModule<IThemeProvider>;

export { css, createGlobalStyle, keyframes, ThemeProvider };
export default styled;
