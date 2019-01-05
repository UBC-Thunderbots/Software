import * as React from 'react';
import { connect } from 'react-redux';
import { bindActionCreators, Dispatch } from 'redux';

import { RosIndicator } from './components/RosIndicator';

import { Sidebar } from 'SRC/components/portals/Sidebar';
import { SidebarTitle } from 'SRC/components/portals/SidebarTitle';
import { CategoryItem } from 'SRC/components/ui/CategoryItem';

import { IRootState, RootAction, selectors } from 'SRC/store';

const mapStateToProps = (state: IRootState) => ({
    errorMessage: state.ros.errorMessage,
    nodes: selectors.ros.nodesSelector(state),
    params: selectors.ros.paramsSelector(state),
    services: selectors.ros.servicesSelector(state),
    status: state.ros.status,
    topics: selectors.ros.topicsSelector(state),
});

const mapDispatchToProps = (dispatch: Dispatch<RootAction>) =>
    bindActionCreators({}, dispatch);

interface IROSPageProps {
    status: string;
    errorMessage: string;
    nodes: string[];
    topics: string[];
    services: string[];
    params: string[];
}

class ROSPageInternal extends React.Component<IROSPageProps> {
    public render() {
        return (
            <>
                <SidebarTitle text="ROS" />
                <Sidebar>
                    <RosIndicator {...this.props} />
                    <CategoryItem text="Nodes" />
                    <CategoryItem text="Topics" />
                    <CategoryItem text="Services" />
                    <CategoryItem text="Params" />
                </Sidebar>
            </>
        );
    }
}

export const ROSPage = connect(
    mapStateToProps,
    mapDispatchToProps,
)(ROSPageInternal);
