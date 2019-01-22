export interface IPanel {
    size: number;
    active: boolean;
}

export const getInitialSize = (count: number, parent: number): IPanel[] => {
    return Array(count)
        .fill(0)
        .map(() => {
            return {
                active: true,
                size: parent / count,
            };
        });
};

export const resize = (
    index: number,
    panels: IPanel[],
    mouseY: number,
    minSize: number,
): number => {
    const resizeFactor = mouseY;
    const minSizeFactor = minSize;

    const indexA = findPanelUpward(index, panels, resizeFactor, minSizeFactor);
    const indexB = findPanelDownward(index + 1, panels, -resizeFactor, minSizeFactor);

    if (indexA !== -1 && indexB !== -1) {
        const indexAValue = panels[indexA].size + resizeFactor;
        const indexBValue = panels[indexB].size - resizeFactor;

        let unusedResizeFactorA = 0;
        if (indexAValue < minSizeFactor) {
            unusedResizeFactorA = resizeFactor + (panels[indexA].size - minSizeFactor);
        }

        let unusedResizeFactorB = 0;
        if (indexBValue < minSizeFactor) {
            unusedResizeFactorB = resizeFactor - (panels[indexB].size - minSizeFactor);
        }

        let unusedResizeFactor = 0;
        if (Math.abs(unusedResizeFactorA) > Math.abs(unusedResizeFactorB)) {
            unusedResizeFactor = unusedResizeFactorA;
        } else {
            unusedResizeFactor = unusedResizeFactorB;
        }

        panels[indexA].size += resizeFactor - unusedResizeFactor;
        panels[indexB].size -= resizeFactor - unusedResizeFactor;

        return unusedResizeFactor;
    } else {
        return mouseY;
    }
};

const findPanelUpward = (
    index: number,
    panels: IPanel[],
    resizeFactor: number,
    minSizeFactor: number,
) => {
    for (let i = index; i >= 0; i--) {
        if (
            panels[i].active &&
            ((resizeFactor > 0 && panels[i].size >= minSizeFactor) ||
                (resizeFactor < 0 && panels[i].size > minSizeFactor))
        ) {
            return i;
        }
    }

    return -1;
};

const findPanelDownward = (
    index: number,
    panels: IPanel[],
    resizeFactor: number,
    minSizeFactor: number,
) => {
    for (let i = index; i < panels.length; i++) {
        if (
            panels[i].active &&
            ((resizeFactor > 0 && panels[i].size >= minSizeFactor) ||
                (resizeFactor < 0 && panels[i].size > minSizeFactor))
        ) {
            return i;
        }
    }
    return -1;
};

export const makePanelInactive = (index: number, panels: IPanel[], finalSize: number) => {
    panels[index].active = false;

    const indexA = findPanelDownward(index, panels, panels[index].size - finalSize, 100);
    const indexB = findPanelUpward(index, panels, panels[index].size - finalSize, 100);
    if (indexA !== -1) {
        panels[indexA].size += panels[index].size - finalSize;
        panels[index].size = finalSize;
    } else if (indexB !== -1) {
        panels[indexB].size += panels[index].size - finalSize;
        panels[index].size = finalSize;
    } else {
        panels[index].size = finalSize;
    }
};

export const makePanelActive = (
    index: number,
    panels: IPanel[],
    parentHeight: number,
    finalSize: number,
) => {
    panels[index].active = true;

    const activePanelCount = panels.reduce(
        (count, panel) => count + (panel.active ? 1 : 0),
        0,
    );

    if (activePanelCount === 1) {
        panels[index].size = parentHeight - 32 * (panels.length - 1);
    } else {
        let size = finalSize - panels[index].size;

        for (let i = index + 1; size > 0 && i < panels.length; i++) {
            if (panels[i].active) {
                const availableSize = panels[i].size - finalSize;
                if (size < availableSize) {
                    panels[i].size -= size;
                    size = 0;
                } else {
                    panels[i].size -= availableSize;
                    size -= availableSize;
                }
            }
        }

        for (let i = index - 1; size > 0 && i >= 0; i--) {
            if (panels[i].active) {
                const availableSize = panels[i].size - finalSize;
                if (size < availableSize) {
                    panels[i].size -= size;
                    size = 0;
                } else {
                    panels[i].size -= availableSize;
                    size -= availableSize;
                }
            }
        }

        panels[index].size = finalSize;
    }
};

export const resizeParent = (panels: IPanel[], resizeFactor: number, minSize: number) => {
    if (resizeFactor < 0) {
        let size = -resizeFactor;
        for (let i = panels.length - 1; size > 0 && i >= 0; i--) {
            if (panels[i].active) {
                const availableSize = panels[i].size - minSize;
                if (size < availableSize) {
                    panels[i].size -= size;
                    size = 0;
                } else {
                    panels[i].size -= availableSize;
                    size -= availableSize;
                }
            }
        }
    } else {
        const index = findPanelUpward(panels.length - 1, panels, +1, minSize);
        if (index !== -1) {
            panels[index].size += resizeFactor;
        }
    }
};
