/**
 * Thrown if an error occurs when parsing layers
 */
export class LayerParsingException extends Error {
    constructor(message: string) {
        super(message);
        this.name = 'LayerParsingException';
    }
}
