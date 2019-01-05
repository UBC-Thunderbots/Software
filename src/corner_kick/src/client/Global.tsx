import * as React from 'react';
import { connect } from 'react-redux';
import { Redirect } from 'react-router';

import { Style } from './style';

import { FooterIndicator } from './components/portals/FooterIndicator';
import { Logo } from './components/portals/Logo';

import { IRootState } from './store/state';

const mapStateToProps = (state: IRootState) => ({
    status: state.ros.status,
});

interface IGlobalProps {
    status: string;
}

class GlobalInternal extends React.Component<IGlobalProps> {
    public render() {
        return (
            <>
                <Style />
                <Logo />
                <Redirect exact={true} from="/" to="/visualizer" />
                <FooterIndicator
                    text={this.props.status}
                    icon="group_work"
                    direction="left"
                    link="/ros"
                />
            </>
        );
    }
}

export const Global = connect(mapStateToProps)(GlobalInternal);
