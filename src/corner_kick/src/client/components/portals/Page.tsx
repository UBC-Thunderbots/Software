import * as React from 'react';
import { Link, Route } from 'react-router-dom';

import { PageIcon } from './PageIcon';

interface IPageProps {
    icon: string;
    path: string;
    text: string;
    component: React.ComponentClass;
}

export const Page = (props: IPageProps) => {
    return (
        <>
            <Link to={props.path}>
                <PageIcon
                    active={new RegExp(`^${props.path}.*`).test(window.location.pathname)}
                    icon={props.icon}
                    title={props.text}
                />
            </Link>
            <Route path={props.path} component={props.component} />
        </>
    );
};
