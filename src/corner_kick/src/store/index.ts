import { applyMiddleware, compose, createStore as createReduxStore } from 'redux';
import createSagaMiddleware from 'redux-saga';

import reducers from './reducers';
import { init } from './sagas';

export const createStore = () => {
    const reduxSaga = createSagaMiddleware();
    const composeEnhancers =
        (window as any).__REDUX_DEVTOOLS_EXTENSION_COMPOSE__ || compose;
    const store = createReduxStore(
        reducers,
        composeEnhancers(applyMiddleware(reduxSaga)),
    );

    reduxSaga.run(init);

    return store;
};

export { actions } from './actions';
export { RootAction } from './reducers';
