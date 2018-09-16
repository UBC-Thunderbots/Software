import * as React from 'react';
import { Subscribe } from 'unstated';

import { ConnectedIndicator } from '../../components/ConnectedIndicator';
import { ROSContainer } from '../../services/ros';

import { StyledWrapper } from './styled';

export class Header extends React.Component {
    public render() {
        return (
            <Subscribe to={[ROSContainer]}>
                { (ros: ROSContainer) => (
                    <StyledWrapper>
                        <ConnectedIndicator state={ros.state.state}/>
                    </StyledWrapper>
                )}
            </Subscribe>
        );
    }
}