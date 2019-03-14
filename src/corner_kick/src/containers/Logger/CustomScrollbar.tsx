/***
 * This file allows for custom scrollbars (used accross the application)
 * to be added to the third-party list view
 */

import * as React from 'react';
import Scrollbars from 'react-custom-scrollbars';

/**
 * Sets the logic to notify the list view that it has been scrolled and
 * injects the list items within the view.
 */
const CustomScrollbarsInternal = ({ onScroll, forwardedRef, style, children }: any) => {
    const refSetter = React.useCallback((scrollbarsRef) => {
        if (scrollbarsRef) {
            forwardedRef(scrollbarsRef.view);
        } else {
            forwardedRef(null);
        }
    }, []);

    return (
        <Scrollbars
            ref={refSetter}
            style={{ ...style, overflow: 'hidden' }}
            onScroll={onScroll}
        >
            {children}
        </Scrollbars>
    );
};

/**
 * Adds custom scrollbar on a third-party list view.
 * Pass this element to the `outerElementType` of a `react-window` list view.
 */
export const CustomScrollbars = React.forwardRef((props, ref) => (
    <CustomScrollbarsInternal {...props} forwardedRef={ref} />
));
