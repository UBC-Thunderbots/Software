import * as React from 'react';
import { connect } from 'react-redux';
import { bindActionCreators, Dispatch } from 'redux';

import { Main } from 'SRC/components/portals/Main';
import { Sidebar } from 'SRC/components/portals/Sidebar';
import { SidebarTitle } from 'SRC/components/portals/SidebarTitle';
import { CategoryItem } from 'SRC/components/ui/CategoryItem';
import { EmptyMain } from 'SRC/components/ui/EmptyMain';
import { RootAction } from 'SRC/store';
import { IRootState } from 'SRC/types';

import { RosIndicator } from './components/RosIndicator';

const mapStateToProps = (state: IRootState) => ({
    errorMessage: state.ros.errorMessage,
    status: state.ros.status,
});

const mapDispatchToProps = (dispatch: Dispatch<RootAction>) =>
    bindActionCreators({}, dispatch);

interface IROSPageProps {
    status: string;
    errorMessage: string;
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
                <Main>
                    <EmptyMain />
                </Main>
            </>
        );
    }
}

export const ROSPage = connect(
    mapStateToProps,
    mapDispatchToProps,
)(ROSPageInternal);
