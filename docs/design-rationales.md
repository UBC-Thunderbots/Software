# Design Rationale

### TODO: Document These Things
* [ ] Namespaces
* [ ] Constants vs. Dynamic parameters (why we have both, where each should be used, etc.)
* [ ] Difference between HL components and Navigator components. Be clear about where the separation is, and why. Actions and Intents are not combined because Actions are part of HL, while Intents are part of Navigator. Combining them would break the abstraction and couple STP to the navigator, removing our flexibility to implement different HL systems in the future
* [ ] contribution / editing guide.  Diagrams in GitHub: https://github.com/jgraph/drawio-github
* [ ] coroutinesG

# Table of Contents
* [Tools](#tools)
  * [SSL-Vision](#ssl-vision)
  * [SSL-Gamecontroller](#ssl-gamecontroller)
  * [grSim](#grsim)
* [Important Classes](#important-classes)
  * [World](#world)
    * [Team](#team)
    * [Robot](#robot)
    * [Ball](#ball)
    * [Field](#field)
    * [Refbox and Gamestate](#refbox-/-gamestate)
  * [Primitives](#primitives)
  * [Intents](#intents)
  * [Dynamic Parameters](#dynamic-parameters)
  * [Robot Status](#robot-status)
* [Design Patterns](#design-patterns)
  * [Abstract Classes and Inheritance](#abstract-classes-and-inheritance)
  * [Singleton Design Pattern](#singleton-design-pattern)
  * [Factory Design Pattern](#factory-design-pattern)
  * [Visitor Design Pattern](#visitor-design-pattern)
  * [Observer Design Pattern](#observer-design-pattern)
  * [C++ Templating](#c++-templating)
* [Conventions](#conventions)
  * [Coordinates](#coordinates)
* [Architecture Overview](#architecture-overview)
  * [Diagram](#architecture-overview-diagram)
* [Backend](#backend)
  * [Input](#input-responsabilities)
  * [Output](#output-responsabilities)
  * [Diagram](#backend-diagram)
* [AI](#ai)
  * [Strategy](#strategy)
    * [Skills / Actions](#skills-/-actions)
    * [Tactics](#tactics)
    * [Plays](#plays)
  * [Navigation](#navigation)
  * [Diagram](#ai-diagram)
* [Visualizer](#visualizer)


# Tools
A few commonly-used terms and tools to be familiar with:
#### SSL-Vision
  * This is the shared vision system used by the Small Size League. It is what connects to the cameras above the field, does the vision processing, and transmits the positional data of everything on the field to our AI computers.
  * The GitHub repository can be found [here](https://github.com/RoboCup-SSL/ssl-vision)
#### SSL-Gamecontroller
  * Sometimes referred to as the "Refbox", this is another shared piece of Small Size League software that is used to send gamecontroller and referee commands to the teams. A human controls this application during the games to send the appropriate commands to the robots. For examples, some of these commands are what stage the gameplay is in, such as `HALT`, `STOP`, `READY`, or `PLAY`.
  * The GitHub repository can be found [here](https://github.com/RoboCup-SSL/ssl-game-controller)
#### grSim
  * The general robot simulator used by the Small-Size-League. We use this to manually test strategy since it is easy to place the robots and ball in desired locations, run a strategy, and see what the robots do. It is not perfectly accurate, but is useful for testing high-level logic.
  * The GitHub repository can be found [here](https://github.com/RoboCup-SSL/grSim)


# Important Classes
These are classes that are either heavily used in our code, or are very important for understanding how the AI works.

## World
The `World` class is what we use to represent the state of the world at any given time. In this context, the world includes the positions and orientations of all robots on the field, the position and velocity of the ball, the dimensions of the field being played on, and the current refbox commands. Alltogether, it's all the information we have at any given time that we can use to make decisions.

### Team
A team is a collection of robots

### Robot
A Robot class represents the state of a single robot on the field. This includes its position, orientation, velocity, angular velocity, and any other information about its current state.

### Ball
The Ball class represents the state of the ball. This includes its position and velocity, and any other information about its current state.

### Field
The Field class represents the state of the physical field being played on, which is primarily its physical dimensions. The Field class provides many functions that make it easy to get points of interest on the field, such as the enemy net, friendly corner, or center circle.

### Refbox / GameState
These represent the current state of the game as dictated by the Gamecontroller. These provide functions like `isPlaying()`, `isHalted()` that allow the rest of the system to tell what state of the game we are in, and make decisions accordingly. We need to obey the rules!


## Primitives
Primitives are very simple actions of things a robot can do. It does not represent or include _how_ these things are done. Some examples are:
* Moving in a straight line to a position
* Pivoting around a point
* Kicking the ball at a certain direction

Primitives act as the abstraction between our AI, and our robot firmware. It's much easier for our AI to send a `Primitive` to a robot telling it what it wants it to do, and have the robot responsible for making sure it does what it's told. For every `Primitive` in our `AI` software, there is an equivalent `Primitive` implementation in our robot firmware. When robots receive a `Primitive` command, they perform their own logic and control in order to perform the task specified by the `Primitive`.


## Intents
An `Intent` represents a simple thing the `AI` wants (or intends for) a robot to do. It does not represent or include _how_ these things are achieved. Some examples are:
* Moving to a position (without colliding with anything on its way)
* Pivoting around a point
* Kicking the ball at a certain direction or at a target

Intents are very similar to Primitives, but include slightly more logic. `Intents` can include extra parameters or data that `Primitives` do not, such as how much to avoid the ball by while moving. In this way, `Intents` are more "context-aware" than `Primitives`, and represent slightly higher-level commands.


## Dynamic Parameters
`Dynamic Parameter` are the system we use to change values in our code at runtime. The reason we want to change values at runtime is primarily because we may want to tweak our strategy or aspects of our gameplay very quickly, and want to avoid having to stop the `AI`, make a change, recompile the code, and re-launch the `AI`. During games we are only allowed to touch our computers and make changes during halftime or a timeout, so every second counts! It's also very useful for testing things manually, and we also use it to communicate between the `Visualizer` and the rest of the system, so that the `Visualizer` can control the system.


## Robot Status
The `Robot Status` class contains information about the status of a single robot. Examples of the information they include are:
* Robot battery voltage
* Whether or not the robot senses the ball in the breakbeam
* The capacitor charge on the robot
* The temperature of the dribbler motor

Information received from the robots is stored in `Robot Status` objects so that the rest of the system can easily access and make sense of the information if necessary. For example, we monitor incoming `Robot Status` and display warnings in the [Visualizer](#visualizer) if anything looks wrong so we can be alerted. For example, during a game we may get a "Low battery warning" for a certain robot, and then we know to substitute it and replace the battery before it dies on the field.


# Design Patterns
Below are the main design patterns we make use of in our code, and what they are used for.

## Abstract Classes and Inheritance
Abstract classes let us define interfaces for various components of our code. Then we can implement different objects that obey the interface, and use them interchangeably, with the guarantee that as long as they follow the same interface we can use them in the same way.

Read [https://www.geeksforgeeks.org/inheritance-in-c/] for more information.

Examples of this can be found in many places, including:
* [Plays](#plays)
* [Tactics](#tactics)
* [Actions](#skills-/-actions)
* [Intents](#intents)
* [Primitives](#primitives)
* Different implementations of the [Backend](#backend)


## Singleton Design Pattern
The Singleton pattern is useful for having a single, global instance of an object that can be accessed from anywhere. Though it's generally considered an anti-pattern (aka _bad_), it is useful in specific scenarios.

Read [https://www.tutorialspoint.com/Explain-Cplusplus-Singleton-design-pattern] for more information.

We use the Singleton pattern for our logger. This allows us to create a single logger for the entire system, and code can make calls to the logger from anywhere, rather than us having to pass a `logger` object literally everywhere.


## Factory Design Pattern
The Factory Design Pattern is useful for hiding or abstracting how certain objects are created.

Read [https://www.geeksforgeeks.org/design-patterns-set-2-factory-method/] for more information.

Because the Factory needs to know about what objects are available to be created, it can be taken one step further to auto-register these object types. Rather than a developer having to remember to add code to the Factory every time they create a new class, this can be done "automatically" with some clever code. This helps reduce mistakes and saves developers work.

Read [http://derydoca.com/2019/03/c-tutorial-auto-registering-factory/] for more information.

The auto-registering factory is particularily useful for our `PlayFactory`, which is responsible for creating [Plays](#plays). Every time we run our [AI](#ai) we want to know what [Plays](#plays) are available to choose from. The Factory pattern makes this really easy, and saves us having to remember to update some list of "available Plays" each time we add or remove one.

The Factory pattern is also used to create different [Backends](#backend)


## Visitor Design Pattern
The `Visitor Design Pattern` is arguably the most "advanced" design pattern we use. It is used when we need to perform different operations on a group of "similar" objects, for example a bunch of objects that inherit from the same parent class (eg. [Primitives](#primitives) or [Intents](#intents)). We might only know all these objects are an [Intent](#intent), but we don't know specifically which type each one is (eg. `MoveIntent` vs `KickIntent`). The Visitor Pattern helps us "recover" that type information so we can perform different operations on the different types of objects. It is generally preferred to a big `if-block` with a case for each type, because the compiler can help warn you when you've forgotten to handle a certain type, and therefore helps prevent mistakes.

Read [https://www.geeksforgeeks.org/visitor-design-pattern/] for more information.

Examples of the Visitor Pattern can be found with the following classes:
* [Intents](#intents)
* [Primitives](#primitives)
* [Tactics](#tactics)


## Observer Design Pattern
The Observer Design Pattern is useful for letting components of a system "notify" each other when something happens. This saves us having to waste CPU time polling components for updates.

Read [https://www.geeksforgeeks.org/observer-pattern-set-1-introduction/] for more information.

We use the Observer Design Pattern to connect all our top-level modules in the system as shown in the [Architecture Overview](#architecture-overview). The components can notify each other and provide data whenever it is available. For example, the [AI](#ai) will "observer" the [Backend](#backend) so that when the backend gets new information, it can notify the [AI](#ai) and the [AI](#ai) can choose to take action with the new data.


## C++ Templating
While debatably not a design pattern depending on who you ask, templating in C++ is a powerful tool that is very useful to understand. [https://www.geeksforgeeks.org/templates-cpp/] gives a great explanantion and example.

We use templtaing in a few places around the codebase, with the most notable examples being our [Factory Design Patterns](#factory-design-pattern), and our `Gradient Descent` optimizer.


# Conventions
Various conventions we use and follow that you need to know.


## Coordinates
We use a slightly custom coordinate convention to make it easier to write our code in a consistent and understandable way. This is particularily important for any code handling gameplay logic and positions on the field.

The coordinate system is a simple 2D x-y plane. The x-dimension runs between the friendly and enemy goals, along the longer dimension of the field. The y-dimension runs perpendicular to the x-dimension, along the short dimension of the field.

Because we have to be able to play on either side of a field during a game, this means the "friendly half of the field" will not always be in the positive or negative x part of the coordinate plane. This inconsistency is a problem when we want to specify points like "the friendly net", or "the enemy corner". We can't simple say the friendly net is `(-4.5, 0)` all the time, because this would not be the case if we were defending the other side of the field where the friendly net would be `(4.5, 0)`.

In order to overcome this, our conention is that:
* The friendly half of the field is **always negative x**, and the enemy half of the field is **always positive x**
* `y` is positive to the "left" of someone looking at the enemy goal from the friendly goal
* The center of the field (inside the center-circle) is the origin / `(0, 0)`

This is easiest to understand in the diagram below:
![Coordinate Convention Diagram](images/coordinate_convention_diagram.svg)

Based on what side we are defending, the [Backend](#backend) will transform all the coordinates of incoming data so that it will match our convention. This means that from the perspective of the rest of the system, the friendly half of the field is always negative x and the enemy half is always positive x. Now when we want to tell a robot to move to the friendly goal, we can simply tell it so move to `(-4.5, 0)` and we know this will _always_ be the friendly side. All of our code is written with the assumption in mind.


# Architecture Overview
At a high-level our system is made of 3 main components: The [Backend](#backend), the [AI](#ai), and the [Visualizer](#visualizer). These 3 components each run in their own thread, and communicate with each other using the [Observer design pattern](TODO use actual link). Together, they are what make up our AI.

The Backend is responsible for communicating with the outside world (network and radio), the AI is what makes the actual gameplay decisions, and the Visualizer shows us what's happening and lets us control the AI.

Each component is described in more detail in their own sections.


#### Architecture Overview Diagram
![High Level Architecture Diagram](images/high_level_architecture_diagram.svg)


# Backend
The `Backend` is responsible for all communication with the "outside world". The responsabilities of the `Backend` can be broken down into Input and Output.

### Input Responsabilities
1. Receiving robot status messages
2. Receiving vision data about where the robots and ball are (typically provided by [SSL-Vision](#ssl-vision) or [grSim](#grsim))
2. Receiving referee commands (typically from the [SSL-Gamecontroller](#ssl-gamecontroller)
3. Filtering the received data
    * **Why we need to do this:** Programs that provide data like [SSL-Vision](#ssl-vision) only provide raw data. This means that if there are several orange blobs on the field, [SSL-Vision](#ssl-vision) will tell us the ball is in several different locations. It is up to us to filter this data to determine the "correct" position of the ball. The same idea applies to robot positions and other data we receive.
4. Storing the filtered data into the [World](#world) datastructures understood by our system
5. Sending the filtered data to the rest of the system

### Output Responsabilities
1. Sending robot primitives to the robots

In practice, the `Backend` is just a simple interface that specifies [World](#world) and [Robot Status](#robot-status) objects must be produced, and [Primitves](#primitives) may be consumed. The interface is very generic so that different implementations may be swapped out in order to communicate with different hardware / protocols / programs. For example, we have multiple implementations of the "output" part of the backend: one that lets us send data to our real robots using the radio, and one that sends commands to simulated robots in [grSim](#grsim).


#### Backend Diagram
![Backend Diagram](images/backend_diagram.svg)


# AI
The `AI` is where all of our gameplay logic takes place, and is the main "brain" of our system. It uses the information received from the [Backend](#backend) to make decisions, and sends [Primitives](#primitives) back to the [Backend](#backend) for the robots to execute. All together this feedback loop is what allows us to react to what's happening on the field and play soccer in real-time.

The 2 main components of the AI are stratgy and navigation.


## Strategy
We use a framework called `STP (Skills, Tactics, Plays)` to implement our stratgy. The `STP` framework was originally proposed by Carnegie Mellon University back in 2004. The original paper can be found [here](https://kilthub.cmu.edu/articles/STP_Skills_Tactics_and_Plays_for_Multi-Robot_Control_in_Adversarial_Environments/6561002/1).

`STP` is a way of breaking down roles and responsabilities into a simple hierarchy, making it easier to build up more complex strategies from simpler pieces. This is the core of where our strategy is implemented.

When the [AI](#ai) is given new information and asked to make a decision, our `STP` strategy is what is executed first. It takes in a [World](#world) and returns [Intents](#intents).


#### Skills / Actions
The `S` in `STP` stands for `Skills`. In our system, we call these `Actions`. Actions represent simple tasks an individual robot can do. Examples include:
1. Moving to a position (without colliding with anything)
2. Shooting the ball at a target
3. Intercepting a moving ball

Actions use [ntents](#intents) to implement their behavior. Actions are responsible for obeying any preconditions `Intents` have.


#### Tactics
The `T` in `STP` stands for `Tactics`. A `Tactic` represents a "single-robot role" on a team. Examples include:
1. Being a goalie
2. Being a passer or pass receiver
3. Being a defender that shadows enemy robots
4. Being a defender that tries to steal the ball from enemies

Tactics use [Actions](#skills-/-actions) to implement their behavior. Using the [Action](#skills-/-actions) abstraction makes it much easier for Tactics to express what they want to do, and make it easier to design and implement behavior. Tactics can focus more on what things to do, and when to do them, in order to implement more intelligent and adaptable behavior.

#### Plays
The `P` in `STP` stands for `Plays`. A `Play` represents a "team-wide goal" for the robots. They can be thought of much like Plays in real-life soccer. Examples include:
1. A Play for taking friendly corner kicks
2. A Play for defending enemy kickoffs
3. A general defense play
4. A passing-based offense play
5. A dribbling-based offense play

Plays are made up of `Tactics`. Plays can have "stages" and change what `Tactics` are being used as the state of the game changes, which allows us to implement more complex behavior. TODO coroutines

Furthermore, every play specifies an `Applicable` and `Invariant` condition. These are used to determine what plays should be run at what time, and when a Play should terminate.

`Applicable` indicates when a `Play` can be started. For example, we would not want to start a `Defense Play` if our team is in possession of the ball. The `Invariant` condition is a condition that must always be met for the `Play` to continue running. If this condition ever becomes false, the current `Play` will stop running and a new once will be chosen. For example, once we start running a friendly `Corner Kick` play, we want the `Play` to continue running as long as the enemy team does not have possession of the ball.


## Navigation
The `Navigator` is responsible for path planning and navigation. Once our strategy has decided what it wants to do, it passes the resulting [Intents](#intents) to the `Navigator`. The `Navigator` is then responsible for breaking down the [Intents](#intents) and turning them into [Primitives](#primitives).

Most [Intents](#intents) are easy to break down into primitives, and can typically just be converted directly without having to do any extra work. However, some [Intents](#intents) like the `MoveIntent` rely on the navigator to implement more complex behavior like obstacle avoidance. This is where the "Navigation" part of the `Navigator` comes in.

In order for a robot to move to the desired destination of a `MoveIntent`, the Navigator will use various path-planning algorithms to find a path across the field that does not collide with any robots or violate any restrictions set on the `MoveIntent`. The Navigator then translates this path into a series of `MovePrimitives`, which are sent to the robot sequentially so that it follows the planned path across the field.

## AI Diagram
![Backend Diagram](images/ai_diagram.svg)


# Visualizer
The `Visualizer` is exactly what it sounds like: A visualizion of our [AI](#ai). It provides a GUI that shows us the state of the `World` as the [Backend](#backend) sees it, and is also able to display extra information that the [AI](#ai) would like to show. For example, it can show the planned paths of each friendly robot on the field, or highlight which enemy robots it thinks are a threat. Furthermore, it displays any warnings and status messages from the robots, such as if a robot is low on battery.

The `Visualizer` also lets us control the [AI](#ai) by setting `Parameters` (TODO: link here). Through the `Visualizer`, we can manually choose what strategy the [AI](#ai) should use, what team we think we are playing as, and tune more granular behavior such as how close an enemy must be to the ball before we consider them a threat.
