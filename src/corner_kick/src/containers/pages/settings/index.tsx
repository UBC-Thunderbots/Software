/**
 * This file describes the settings page of the application. The UI for this
 * application is generated from a JSON object for easy customization.
 */

import * as React from 'react';
import { Redirect, Route } from 'react-router-dom';

import { Main } from 'SRC/components/portals/Main';
import { Sidebar } from 'SRC/components/portals/Sidebar';
import { SidebarTitle } from 'SRC/components/portals/SidebarTitle';
import { CategoryItem } from 'SRC/components/ui/CategoryItem';
import { settingsUI } from 'SRC/constants';

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
                    to={`${this.props.match.url}/${settingsUI[0].path}`}
                />
                <Sidebar>
                    {settingsUI.map((category) => (
                        <CategoryItem
                            icon={category.icon}
                            text={category.title}
                            link={`${this.props.match.url}/${category.path}`}
                        />
                    ))}
                </Sidebar>
                <Main>
                    {settingsUI.map((category) => (
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
