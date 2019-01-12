/**
 * Story for the EditText form UI
 */

import { storiesOf } from '@storybook/react';
import * as React from 'react';

import styled from 'SRC/utils/styled-components';

import { EditText } from './EditText';

const Item = styled.div`
    width: 400px;
`;

storiesOf('EditText', module)
    .addDecorator((story) => <Item>{story()}</Item>)
    .add('with value specified', () => <EditText />);
