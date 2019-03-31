import * as PIXI from 'pixi.js';

/**
 * Manages a pool of sprites. Creates new sprites if needed and
 * stores unallocated sprites for future use.
 */
export class SpritePool {
    private availableSprites: PIXI.Sprite[] = [];

    /**
     * Creates a new sprite pool and initializes an initial amount of sprites.
     * @param startCount Initial count of sprites to create for this pool
     */
    constructor(startCount: number = 1000) {
        this.createSprites(startCount);
    }

    /**
     * Allocates sprites for use by a layer. Creates new sprites
     * if needed, which might affect performance.
     */
    public allocateSprites = (count: number) => {
        // Check if we have enough available sprites to allocate. If not, we create more.
        if (count > this.availableSprites.length) {
            // We increase the number of sprites by the requested amount * 2
            this.createSprites(count * 2);
        }

        // We remove the allocated sprites from the pool and give them to the user
        return this.availableSprites.splice(this.availableSprites.length - count);
    };

    /**
     * Unallocates sprites and adds them back in the pool for future use.
     */
    public unallocateSprites = (sprites: PIXI.Sprite[]) => {
        this.availableSprites.push(...sprites);
    };

    private createSprites = (count: number) => {
        for (let i = 0; i < count; i++) {
            this.availableSprites.push(new PIXI.Sprite());
        }
    };
}
