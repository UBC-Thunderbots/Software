/**
 * Thrown when we are trying to portal to an unknown location
 * in the DOM.
 *
 * This is most likely due to a change in index.html that was
 * not taken into account in the React Portal component
 */
export class UnknownPortalException extends Error {
    constructor(message: string) {
        super(message);
        this.name = 'UnknownPortalException';
    }
}
