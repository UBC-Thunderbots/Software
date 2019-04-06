/**
 * Manages a pool of objects. Creates new objects if needed and
 * stores unallocated objects for future use.
 */
export class Pool<T> {
    private availableObjects: T[] = [];

    /**
     * Creates a new pool and initializes an initial amount of objects.
     * @param startCount Initial count of objects to create for this pool
     */
    constructor(private createObject: () => T, startCount: number = 1000) {
        this.create(startCount);
    }

    /**
     * Allocates objects for use. Creates new objects
     * if needed, which might affect performance.
     */
    public allocate = (count: number): T[] => {
        // Check if we have enough available objects to allocate. If not, we create more.
        if (count > this.availableObjects.length) {
            // We increase the number of objects by the requested amount * 2
            this.create(count * 2);
        }

        // We remove the allocated objects from the pool and give them to the user
        return this.availableObjects.splice(this.availableObjects.length - count);
    };

    /**
     * Unallocates objects and adds them back in the pool for future use.
     */
    public unallocate = (objects: T[]) => {
        this.availableObjects.push(...objects);
    };

    private create = (count: number) => {
        for (let i = 0; i < count; i++) {
            this.availableObjects.push(this.createObject());
        }
    };
}
