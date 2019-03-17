/**
 * Thrown if the data received from the shape WebSocket does not match
 * the expected number of bytes.
 */
export class MalformedShapesException extends Error {
    constructor(message: string) {
        super(message);
        this.name = 'MalformedShapeException';
    }
}
