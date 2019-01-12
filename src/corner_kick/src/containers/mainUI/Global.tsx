/**
 * This file defines a React component responsible for adding the main styling
 * and logo to the application
 */

import * as React from 'react';
import { connect } from 'react-redux';

import { Logo } from 'SRC/components/portals/Logo';
import { IRootState } from 'SRC/types';

import { GlobalStyle } from './GlobalStyle';

const mapStateToProps = (state: IRootState) => ({
    status: state.ros.status,
});

interface IGlobalProps {
    status: string;
}

class GlobalInternal extends React.Component<IGlobalProps> {
    public render() {
        return (
            <GlobalStyle>
                <Logo />
                {this.props.children}
            </GlobalStyle>
        );
    }
}

export const Global = connect(mapStateToProps)(GlobalInternal);
