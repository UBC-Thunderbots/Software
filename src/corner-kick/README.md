# Corner Kick

Corner Kick is an Electron app that allows for the visualization of UBC Thunderbots' simulator.

## How to use

After running `catkin_make`, you can start the visualizer by running:

```
roslaunch corner-kick corner-kick.launch
```

_MVP Specific_
This demo makes use of the turtle sim demo. After starting Corner Kick, run the following in a terminal:

```
rosrun turtlesim turtlesim_node
```

## Contribute

### Installing the project

Clone the project and install the dependencies.

```
rosdep install corner-kick
```

### Running the dev server

You can test your code by running:

```
yarn start
```

A window should open with the application in development mode.

## Project Structure

The project is defined as follow:

- public - contains the HTML base file and the Electron start code
- src
    - components
        - containers - contains React stateful components
        - modules - contains high-level UI components (logger, visualizer, etc.)
        - ui - contains repeatable UI components
        - visualizer - contains visualizer specific components
    - pages - contains the various application pages
    - service - contains various communication layers (including the service communicating with ROS)
    - types - contains standard ROS topics schema
    - App.tsx - Top-level React component
    - index.css - Base styling
    - index.tsx - Contains the code that initializes React