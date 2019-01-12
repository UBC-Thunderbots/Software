import * as styledComponents from 'styled-components';
import { ThemedStyledComponentsModule } from 'styled-components';

import { IThemeProvider } from 'SRC/types';

const {
    default: styled,
    css,
    createGlobalStyle,
    keyframes,
    ThemeProvider,
} = styledComponents as ThemedStyledComponentsModule<IThemeProvider>;

export { css, createGlobalStyle, keyframes, ThemeProvider };
export default styled;
