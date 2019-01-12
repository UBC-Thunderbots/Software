/**
 * This file defines a page in the application, and adds various
 * corresponding UI elements in the sidebar and main area of the application.
 */

import * as React from 'react';
import { Link, Route } from 'react-router-dom';

import { PageIcon } from './PageIcon';

interface IPageProps {
    icon: string;
    path: string;
    text: string;
    component: React.ComponentClass;
}

/**
 * Defines a page, and adds it to the application navigation
 */
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
