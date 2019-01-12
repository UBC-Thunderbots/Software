/**
 * File used to configure Storybook, a UI development tool.
 */

import { addDecorator, configure } from '@storybook/react';
import * as React from 'react';
import { Provider } from 'react-redux';

import { GlobalStyle } from '../src/containers/mainUI/GlobalStyle';
import { createStore } from '../src/store';
import styled from '../src/utils/styled-components';

// We add all stories (UI mockups) from files with *.story.* in their
// file name
const req = require.context('../src', true, /\.story\.tsx?$/);

// We create the redux store to ensure connected components work correctly
const store = createStore();

// We add some styling to position the UI element in the center
// of the screen
const Wrapper = styled.div`
    width: 100vw;
    height: 100vh;

    display: flex;
    justify-content: center;
    align-items: center;
`;

// A decorator is a React components that wraps all Storybook stories.
// This decorator adds some styling and attaches the Redux store to the React tree
addDecorator((story) => (
    <Provider store={store}>
        <GlobalStyle>
            <Wrapper>{story()}</Wrapper>
        </GlobalStyle>
    </Provider>
));

function loadStories() {
    req.keys().forEach((filename) => req(filename));
}

configure(loadStories, module);
