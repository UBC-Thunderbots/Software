import * as React from 'react';
import { Subscribe } from 'unstated';

import {
    Alignment,
    Button,
    Classes,
    Navbar,
} from '@blueprintjs/core';

import { ConnectedIndicator } from '../components/ConnectedIndicator';
import { ROSContainer } from '../services/ros';

export class Header extends React.Component {
    public render() {
        return (
            <Subscribe to={[ROSContainer]}>
                { (ros: ROSContainer) => (
                    <Navbar>
                        <Navbar.Group align={Alignment.LEFT}>
                            <Button className={Classes.MINIMAL} icon={(
                                <ConnectedIndicator state={ros.state.state}/>
                            )} />
                            <Navbar.Heading>Corner Kick</Navbar.Heading>
                            <Navbar.Divider />
                        </Navbar.Group>
                    </Navbar>
                )}
            </Subscribe>
        );
    }
}