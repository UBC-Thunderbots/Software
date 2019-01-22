/*
 * This file groups all Redux related exports and creates the Redux store
 */

import { applyMiddleware, compose, createStore as createReduxStore } from 'redux';
import createSagaMiddleware from 'redux-saga';

import reducers from './reducers';
import { init } from './sagas';

/**
 * Create a Redux store and attaches Chrome debugging tools
 */
export const createStore = () => {
    // We use Redux Saga for our application logic. It acts as a middleware in order to
    // interact with our Redux application
    const reduxSaga = createSagaMiddleware();

    // We use the Redux debug Chrome/Firefox extension to see what
    // happens inside our Redux store
    const composeEnhancers =
        (window as any).__REDUX_DEVTOOLS_EXTENSION_COMPOSE__ || compose;

    const store = createReduxStore(
        reducers,
        composeEnhancers(applyMiddleware(reduxSaga)),
    );

    // Start the application's sagas
    reduxSaga.run(init);

    return store;
};

export { actions, RootAction } from './actions';
