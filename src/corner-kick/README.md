# Corner Kick

Corner Kick is an Electron app that allows for the visualization of UBC Thunderbots' simulator.

## Contribute

### Node

You will need Node to contribute to this project. To install Node, run the following:

```
sudo apt-get install nodejs npm
```

### Yarn

Yarn is considered a more powerful package manager than npm. You should install it by running:

```
npm install -g yarn
```

### Installing the project

With Node installed, clone the project and install the dependencies.

```
yarn install
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