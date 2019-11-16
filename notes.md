## What are primitives using right now?
`Physbot`, `chicker_*`, `dribbler_*`

## Cutoff for simulator/hardware
- `apply_wheel_force`
- `dr_*` (to be split up and renamed to `robot_state_estimation` and `ball_state_estimation`)
- `chicker_*`
- `dribbler_*`

## General Notes On How To Do The Cutoff
- create a `Robot` struct that's passed by pointer around, containing pointers to functions for all the "cutoff" functions
- pass the "current primitive" either in the `Robot` struct or alongside it to `app_main`
- this is similar to how primitives work in firmware now
- have a `app_main(World world, Primitive current_primitive)` function that does all the app-level work currently done directly in the 200hz task
- maybe have a `World` struct instead that contains `Robot`, `Ball`?
