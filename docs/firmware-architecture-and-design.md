# Architecture and Design Rationales

# Table of Contents
* [General Principles](#general-principles)
* [App/IO Split](#appio-split) 
* [Design Patterns](#design-patterns)
  * [Pseudo-Class](#pseudo-class)

# General Principles
* Avoid memory allocation wherever possible. This means that we usually try to limit our use of pseudo-classes to fairly high level abstractions.
* **REALLY** avoid allocating memory anywhere you don't have a guaranteed upper-bound on the number of times that the allocation will be executed. This is so we can have more guarantees on the upper bound of memory usage, and violating this can potentially cause *very* serious bugs.
* Never use `static` in anything in the `app` layer. As the `app` layer is intended to be portable to our simulator, this means that any `static` variables will be re-used between multiple robots in the simulator, almost guaranteeing undesired behavior.

# Design Patterns

## Pseudo-Class
In order to hide implementations away and enable things like the [App/IO Split](#appio-split), we require something _similar_ to a class in C++. How we do this is most easily explained with an example:

Let's say I have a dog that has a few functions. I would declare the header like so:

_dog.h_
```
typedef struct Dog Dog_t;

enum DogColor {BROWN, BLACK};

/**
 * Create a dog
 * @param color The color of the dog
 * @param attempt_to_run_at_speed A function that can be called to attempt to 
 *                                set the speed of this dog, returning the max 
 *                                speed achieved, in m/s.
 * @return A pointer to a dog with the given parameters
 */
Dog_t* app_dog_create(DogColor color, float (*attempt_to_run_at_speed)(float)); 

/**
 * Destroy the given Dog, freeing any memory allocated for it
 * @param dog The Dog to destroy
 */
void app_dog_destroy(Dog_t* dog);

/**
 * Get the color of a dog
 * @param dog The dog to get the color of
 * @return The color of the given dog
 */
DogColor app_dog_getColor(Dog_t* dog);

/**
 * Attempt to set the speed of a Dog
 * @param desired_speed The speed to attempt, in m/s
 * @return The maximum speed actually achieved, in m/s
 */
float app_dog_attemptToRunAtSpeed(Dog_t* dog, float desired_speed);
```

There are a few things to note here: 
* We have declared a `struct Dog` and typedef'd it to just `Dog_t`, but we *haven't actually said what's in the struct*. This is a crucial point, because it means that anything that includes `dog.h` can only interact with a `struct Dog` through the functions we define here. 
* We passed a function into `dog_create`. This is a really useful technique that is part of what enables us to do stuff like providing different "implementations" of a robot to the `app` layer depending on if we're in simulation or running on the real robots.

We would implement this "class" as follows:

_dog.c_
``` C
#include "dog.h"

struct Dog {
    DogColor color;
    float (*attempt_to_run_at_speed)(float);
};

Dog_t* app_dog_create(DogColor color, float (*attempt_to_run_at_speed)(float)){
    // NOTE: We can't do this outside this `.c` file, because the only place
    //       that we know the size of the `Dog` struct is here.
    Dog_t* dog = (Dog_t*)(malloc(sizeof(Dog_t)));

    dog->color = color;
    dog->attempt_to_run_at_speed = attempt_to_run_at_speed;

    return dog;
}

void app_dog_destroy(Dog_t* dog){
    // NOTE: We can't do this outside this `.c` file, because the only place
    //       that we know the size of the `Dog` struct is here.
    free(dog);
}

DogColor app_dog_getColor(Dog_t* dog){
    return dog->color;
}

float app_dog_attemptToRunAtSpeed(Dog_t* dog, float desired_speed){
    return dog->attempt_to_run_at_speed(desired_speed);
}
```

and we would use this class as follows:
``` C
#include "dog.h"

// Assume we get our "attempt_to_run_at_speed" method from this header
#include "dog_simulator.h"

Dog_t* my_dog = app_dog_create(BROWN, &simulated_dog_attempt_speed);

assert(app_dog_getColor(my_dog) == BROWN);

float achieved_speed = app_dog_attemptToRunAtSpeed(my_dog, 9001);

app_dog_destroy(my_dog);
```

# App/IO Split
The firmware is divided into `app` and `io` layers. 
* The `app` layer contains anything that is portable between all our robots and our simulator. Ex. the implementation of primitives in firmware, or our controllers for motion execution
* The `io` layer contains everything below the `app` layer, down to and including calls to libraries like `HAL` (`HAL` is a library that provides thin wrappers around hardware). Ex. the function that actually applies a wheel force or reads the encoders
* The interface between these layers is the `FirmwareWorld` class, which represents an abstraction of the world from the perspective of the robot. **This is the only interface that the `app` layer should ever be able to use to access the outside world**.
