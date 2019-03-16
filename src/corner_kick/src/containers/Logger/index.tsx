/***
 * This file maps the state (which contains /rosout messages) to the Logger view
 */

import { connect } from 'react-redux';

import { IRootState } from 'SRC/types';

import { Logger as LoggerInternal } from './Logger';

const mapStateToProps = (state: IRootState) => ({
    messages: state.console.rosout,
});

export const Logger = connect(mapStateToProps)(LoggerInternal);
