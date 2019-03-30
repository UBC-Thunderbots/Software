import * as PIXI from 'pixi.js';

export class SpritePool {
    private lastIncrease: number;
    private spritePool: PIXI.Sprite[] = [];

    constructor(startCount: number = 1000) {
        this.createSprites(startCount);
    }

    public allocateSprites = (count: number) => {
        if (count > this.spritePool.length) {
            this.createSprites(Math.round(this.lastIncrease * 1.1));
        }

        return this.spritePool.splice(this.spritePool.length - count);
    };

    public unallocateSprites = (sprites: PIXI.Sprite[]) => {
        this.spritePool.push(...sprites);
    };

    private createSprites = (count: number) => {
        for (let i = 0; i < count; i++) {
            this.spritePool.push(new PIXI.Sprite());
        }
        this.lastIncrease = count;
    };
}
