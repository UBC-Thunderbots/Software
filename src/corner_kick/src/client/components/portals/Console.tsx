import * as React from 'react';
import * as ReactDOM from 'react-dom';

export const Console = (props: { children: React.ReactNode }) => {
    return ReactDOM.createPortal(props.children, document.getElementById('console')!);
};
