# Design Rationale

### TODO: Document These Things
* [ ] Namespaces
* [ ] Overall architecture
* [ ] Class Hierarchies
* [ ] Factory design pattern for Play, Primitive, etc.
* [ ] Singleton design pattern for Logger, Drawer, etc.
* [ ] Constants vs. Dynamic parameters (why we have both, where each should be used, etc.)
* [ ] Visualizer -> Ai interfaces (ROS messages, websockets, etc.)
* [ ] Difference between HL components and Navigator components. Be clear about where the separation is, and why. Actiona and Intents are not combined because Actions are part of HL, while Intents are part of Navigator. Combining them would break the abstraction and couple STP to the navigator, removing our flexibility to implement different HL systems in the future


# System Architecture
* [Glossary](#glossary)
* [Overview](#overview)
* [Backend](#backend)
* [AI](#ai)
* [Visualizer](#visualizer)

## Glossary
A few commonly-used terms to be familiar with:
1. SSL-Vision
    This is the shared vision system used by the Small Size League. It is what connects to the cameras above the field, does the vision processing, and transmits the positional data of everything on the field to our AI computers. The GitHub repository can be found [here](https://github.com/RoboCup-SSL/ssl-vision)
2. SSL-Gamecontroller
    Sometimes referred to as the "Refbox", this is another shared piece of Small Size League software that is used to send gamecontroller and referee commands to the teams. A human controls this application during the games to send the appropriate commands to the robots. For examples, some of these commands
   

## Overview
At a high-level our system is made of 3 main components: The Backend, The AI, and the Visualizer. These 3 components each run in their own thread, and communicate with each other using the [Observer design pattern](TODO use actual link).

## Backend
The Backend is responsible for all communication with the "outside world". Its responsabilities include:
1. Receiving camera data from [SSL-Vision](https://github.com/RoboCup-SSL/ssl-vision)
2. Receiving referee commands from the 

## AI

## Visualizer

