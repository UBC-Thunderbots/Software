import * as React from 'react';
import { connect } from 'react-redux';
import { bindActionCreators, Dispatch } from 'redux';

import { ISettingsCategory } from './settings';

import { EditText } from 'SRC/components/forms/EditText';

import { actions, IRootState, RootAction } from 'SRC/store';
import { ISettingsState } from 'SRC/store/state/settings';

import { SettingsCategory } from './components/SettingsCategory';
import { SettingsEntry } from './components/SettingsEntry';
import { SettingsHeading } from './components/SettingsHeading';

const mapStateToProps = (state: IRootState) => ({
    settings: state.settings,
});

const mapDispatchToProps = (dispatch: Dispatch<RootAction>) =>
    bindActionCreators(
        {
            set: actions.settings.set,
        },
        dispatch,
    );

interface ISettingsMainProps {
    category: ISettingsCategory;
    settings: ISettingsState;
    set: typeof actions.settings.set;
}

class SettingsMainInternal extends React.Component<ISettingsMainProps> {
    public render() {
        const { category } = this.props;
        return (
            <SettingsCategory>
                <SettingsHeading text={category.title} />
                {category.settings.map((entry) => (
                    <SettingsEntry title={entry.title} description={entry.description}>
                        <EditText
                            value={this.props.settings[entry.id]}
                            onChange={(event) => {
                                this.props.set(entry.id, event.target.value);
                            }}
                        />
                    </SettingsEntry>
                ))}
            </SettingsCategory>
        );
    }
}

export const SettingsMain = connect(
    mapStateToProps,
    mapDispatchToProps,
)(SettingsMainInternal);
