import { create } from '@storybook/theming';

import logo from '../src/assets/logo.svg';

export default create({
    base: 'light',

    brandTitle: 'Corner Kick',
    brandUrl: 'http://localhost:6006',
    brandImage: logo,
});
