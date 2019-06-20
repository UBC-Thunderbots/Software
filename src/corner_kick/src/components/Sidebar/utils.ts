/*
 * This file specifies helper function for managing panel
 * resize
 */

/**
 * Defines information related to panels
 */
export interface IPanel {
    /**
     * Height of a panel, can be changed by the user
     */
    height: number;

    /**
     * Whether or not a panel is opened or closed
     */
    active: boolean;
}

/**
 * Specifies the initial size of all panels, based on the height of the
 * parent container
 * @param count Number of panel in the sidebar
 * @param parentHeight The height of the parent container
 */
export const getInitialSize = (count: number, parentHeight: number): IPanel[] => {
    return Array(count)
        .fill(0)
        .map(() => {
            return {
                active: true,
                height: parentHeight / count,
            };
        });
};

/**
 * This function performs the resize operation
 * @param index The index where the resize operation is being performed
 * @param panels An array of panels
 * @param mouseDelta The delta between current resizer position and mouse position
 * @param minHeight The minimum height a panel can have
 */
export const resize = (
    index: number,
    panels: IPanel[],
    mouseDelta: number,
    minHeight: number,
): number => {
    // Find an adequate upper and lower panel to perform this operation
    // Namely, we are looking for panels that are active and larger than `minHeight`
    const indexUpper = findPanelUpward(index, panels, mouseDelta >= 0, minHeight);
    const indexLower = findPanelDownward(index + 1, panels, mouseDelta <= 0, minHeight);

    // Check if we found two panels
    if (indexUpper !== -1 && indexLower !== -1) {
        // Test what the result height would be if the mouseDelta was fully applied
        const indexUpperValue = panels[indexUpper].height + mouseDelta;
        const indexLowerValue = panels[indexLower].height - mouseDelta;

        // Have we exceed the minHeight constraint on upper panel?
        // If so, figure out how much of `mouseDelta` we cannot use
        let unusedDeltaUpper = 0;
        if (indexUpperValue < minHeight) {
            unusedDeltaUpper = mouseDelta + (panels[indexUpper].height - minHeight);
        }

        // Have we exceed the minHeight constraint on lower panel?
        // If so, figure out how much of `mouseDelta` we cannot use
        let unusedDeltaLower = 0;
        if (indexLowerValue < minHeight) {
            unusedDeltaLower = mouseDelta - (panels[indexLower].height - minHeight);
        }

        // Take the larger of the unused delta
        // We have to respect the most stringent constraint
        let unusedDelta = 0;
        if (Math.abs(unusedDeltaUpper) > Math.abs(unusedDeltaLower)) {
            unusedDelta = unusedDeltaUpper;
        } else {
            unusedDelta = unusedDeltaLower;
        }

        panels[indexUpper].height += mouseDelta - unusedDelta;
        panels[indexLower].height -= mouseDelta - unusedDelta;

        return unusedDelta;
    } else {
        return mouseDelta;
    }
};

/**
 * Search for a potential panel candidate for a resize operation, looking
 * for panels at indices lower than the provided index.
 * Return -1 if none are found
 * @param index start index for our search
 * @param panels an array of panels
 * @param increaseHeight are we increasing the height of the panel
 * @param minHeight minimum height of panels
 */
const findPanelUpward = (
    index: number,
    panels: IPanel[],
    increaseHeight: boolean,
    minHeight: number,
) => {
    for (let i = index; i >= 0; i--) {
        // Select panel if active and at minHeight or larger if increasing size
        // of panel, or larger than minHeight if decreasing size of panel
        if (
            panels[i].active &&
            ((increaseHeight && panels[i].height >= minHeight) ||
                (!increaseHeight && panels[i].height > minHeight))
        ) {
            return i;
        }
    }

    // Return -1 if none found
    return -1;
};

/**
 * Search for a potential panel candidate for a resize operation, looking for
 * panels at indices higher than the provided index.
 * Return -1 if none are found
 * @param index start index for our search
 * @param panels an array of panels
 * @param increaseHeight are we increasing the height of the panel
 * @param minHeight minimum height of panels
 */
const findPanelDownward = (
    index: number,
    panels: IPanel[],
    increaseHeight: boolean,
    minHeight: number,
) => {
    for (let i = index; i < panels.length; i++) {
        // Select panel if active and at minHeight or larger if increasing size
        // of panel, or larger than minHeight if decreasing size of panel
        if (
            panels[i].active &&
            ((increaseHeight && panels[i].height >= minHeight) ||
                (!increaseHeight && panels[i].height > minHeight))
        ) {
            return i;
        }
    }

    // Return -1 if none found
    return -1;
};

/**
 * This function makes a panel inactive, and resize panels accordingly
 * @param index the panel to inactivate/close
 * @param panels an array of panels
 * @param inactivePanelHeight the size of an inactive panel
 * @param minHeight the minimum allowed height of a panel
 */
export const makePanelInactive = (
    index: number,
    panels: IPanel[],
    inactivePanelHeight: number,
    minHeight: number,
) => {
    // Make panel inactive
    panels[index].active = false;

    // Find an adequate upper panel
    const indexUpper = findPanelDownward(
        index,
        panels,
        panels[index].height - inactivePanelHeight >= 0,
        minHeight,
    );
    // Find an adequate lower panel
    const indexLower = findPanelUpward(
        index,
        panels,
        panels[index].height - inactivePanelHeight >= 0,
        minHeight,
    );
    if (indexUpper !== -1) {
        // If we found an upper panel, assign height of inactive panel to found panel
        panels[indexUpper].height += panels[index].height - inactivePanelHeight;
        panels[index].height = inactivePanelHeight;
    } else if (indexLower !== -1) {
        // If we found a lower panel, assign height of inactive panel to found panel
        panels[indexLower].height += panels[index].height - inactivePanelHeight;
        panels[index].height = inactivePanelHeight;
    } else {
        // If we have not found any panels, give up
        panels[index].height = inactivePanelHeight;
    }
};

/**
 * This function makes a panel active, and resize panels accordingly
 * @param index the panel to activate
 * @param panels an array of panels
 * @param parentHeight the height of the parent container
 * @param inactivePanelHeight the size of an inactive panel
 * @param minHeight the minimum allowed height of a container
 */
export const makePanelActive = (
    index: number,
    panels: IPanel[],
    parentHeight: number,
    inactivePanelHeight: number,
    minHeight: number,
) => {
    // Make a panel active
    panels[index].active = true;

    // Count the number of active panels
    const activePanelCount = panels.reduce(
        (count, panel) => count + (panel.active ? 1 : 0),
        0,
    );

    if (activePanelCount === 1) {
        // If the panel we just made active is the only active panel, then
        // set its height to the height of the parent, minus the height of all
        // inactive panels
        panels[index].height = parentHeight - inactivePanelHeight * (panels.length - 1);
    } else {
        // Figure out the height we need to collect from other panels
        let height = minHeight - panels[index].height;

        // First, look below the current panel, until we collect all the height
        // we need
        for (let i = index + 1; height > 0 && i < panels.length; i++) {
            // If the current panel is active
            if (panels[i].active) {
                // Figure out how much height we can take from a panel
                // so that it doesn't get smaller than minHeight
                const availableHeight = panels[i].height - minHeight;

                if (height < availableHeight) {
                    // Yay! We found a panel
                    panels[i].height -= height;
                    // and we collected all the height we need
                    height = 0;
                } else {
                    // Almost! We didn't collect all the height we need
                    panels[i].height -= availableHeight;
                    // So we report the height we managed to collect
                    height -= availableHeight;
                }
            }
        }

        // Darn, we didn't collect all the height we needed.
        // We now look for panels above the current one
        for (let i = index - 1; height > 0 && i >= 0; i--) {
            // If the current panel is active
            if (panels[i].active) {
                // Figure out how much height we can take from a panel
                // so that it doesn't get smaller than minHeight
                const availableHeight = panels[i].height - minHeight;

                if (height < availableHeight) {
                    // Yay! We found a panel
                    panels[i].height -= height;
                    // and we collected all the height we need
                    height = 0;
                } else {
                    // Almost! We didn't collect all the height we need
                    panels[i].height -= availableHeight;
                    // So we report the height we managed to collect
                    height -= availableHeight;
                }
            }
        }

        // We know we can collect enough height to make this panel active
        // because our parent cannot be smaller than the sum of the minHeight
        // for all panels
        panels[index].height = minHeight;
    }
};

/**
 * Function to modify panel height when the parent is resized
 * @param panels an array of panels
 * @param parentResize the amount the parent is getting resized by
 * @param minHeight the minimum height of a panel
 */
export const resizeParent = (
    panels: IPanel[],
    parentResize: number,
    minHeight: number,
) => {
    if (parentResize < 0) {
        // If parent is getting smaller, we need to collect extra height
        // the panels. We know that the parent cannot be smaller than the
        // sum of the minHeight for all panels, so we will be able to collect
        // the height
        let height = -parentResize;

        // Starting from the bottom, we collect height from each panel
        // if possible
        for (let i = panels.length - 1; height > 0 && i >= 0; i--) {
            // If the panel is active
            if (panels[i].active) {
                // calculate the height we can collect
                const availableHeight = panels[i].height - minHeight;

                // if our height is smaller than that
                if (height < availableHeight) {
                    // Yay! We found a panel
                    panels[i].height -= height;
                    // and we collected all the height we need
                    height = 0;
                } else {
                    // Almost! We didn't collect all the height we need
                    panels[i].height -= availableHeight;
                    // So we report the height we managed to collect
                    height -= availableHeight;
                }
            }
        }
    } else {
        // If parent is getting larger, we find the first active panel from the bottom
        // and give it the new height available
        const index = findPanelUpward(panels.length - 1, panels, true, minHeight);

        // Make sure there is an available panel
        // In the off chance that all panels are inactive
        if (index !== -1) {
            panels[index].height += parentResize;
        }
    }
};
