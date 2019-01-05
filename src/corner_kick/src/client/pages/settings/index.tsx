import * as React from 'react';
import { Redirect, Route } from 'react-router-dom';

import { Main } from 'SRC/components/portals/Main';
import { Sidebar } from 'SRC/components/portals/Sidebar';
import { SidebarTitle } from 'SRC/components/portals/SidebarTitle';
import { CategoryItem } from 'SRC/components/ui/CategoryItem';

import { settings } from './settings';
import { SettingsMain } from './SettingsMain';

interface ISettingsPageProps {
    match: any;
}

export class SettingsPage extends React.Component<ISettingsPageProps> {
    public render() {
        return (
            <>
                <SidebarTitle text="Settings" />
                <Redirect
                    from={this.props.match.url}
                    to={`${this.props.match.url}/${settings[0].path}`}
                />
                <Sidebar>
                    {settings.map((category) => (
                        <CategoryItem
                            icon={category.icon}
                            text={category.title}
                            link={`${this.props.match.url}/${category.path}`}
                        />
                    ))}
                </Sidebar>
                <Main>
                    {settings.map((category) => (
                        <Route
                            path={`${this.props.match.path}/${category.path}`}
                            component={() => <SettingsMain category={category} />}
                        />
                    ))}
                </Main>
            </>
        );
    }
}
