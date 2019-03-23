import * as React from 'react';

export class InjectPortals extends React.Component {
    public root: HTMLDivElement;

    constructor(props: any) {
        super(props);
        this.root = document.createElement('div');
        this.root.id = 'root';
        (this.root as any).style = `
            width: 100vw;
            height: 100vh;

            display: grid;
            grid-template-columns: 56px 300px 1fr;
            grid-template-rows: 40px 1fr 300px 20px;`;

        this.root.innerHTML = `
            <div
                id="sidebarTitle"
                style="
                    grid-row: 1;
                    grid-column: 2;
                    overflow: hidden;

                    border-right: 1px solid;
                "
            ></div>
            <div
                id="sidebarControl"
                style="
                    grid-row: 1 / 4;
                    grid-column: 1;

                    border-top: 1px solid;
                    border-right: 1px solid;

                    display: flex;
                    flex-flow: column nowrap;

                    overflow: hidden;
                "
            ></div>
            <div
                id="sidebar"
                style="
                    grid-row: 2 / 4;
                    grid-column: 2;

                    border-top: 1px solid;
                    border-right: 1px solid;

                    overflow: hidden;
                "
            ></div>
            <div
                id="mainTitle"
                style="
                    grid-row: 1;
                    grid-column: 3;

                    overflow: hidden;
                "
            ></div>
            <div
                id="main"
                style="
                    grid-row: 2;
                    grid-column: 3;

                    border-top: 1px solid;

                    overflow: hidden;
                "
            ></div>
            <div
                id="console"
                style="
                    grid-row: 3;
                    grid-column: 3;

                    border-top: 1px solid;

                    overflow: hidden;
                "
            ></div>
            <div
                id="footerLeft"
                style="
                    grid-row: 4;
                    grid-column: 1 / 3;

                    padding: 0px 4px;

                    border-top: 1px solid;

                    display: flex;
                    justify-content: flex-start;
                    align-items: center;

                    overflow: hidden;
                "
            ></div>
            <div
                id="footerRight"
                style="
                    grid-row: 4;
                    grid-column: 3;
                    border-top: 1px solid;

                    padding: 0px 4px;

                    display: flex;
                    justify-content: flex-end;
                    align-items: center;

                    overflow: hidden;
                "
            ></div>`;

        document.body.appendChild(this.root);
    }

    public render() {
        return <>{this.props.children}</>;
    }

    public componentWillUnmount() {
        this.root.remove();
    }
}
