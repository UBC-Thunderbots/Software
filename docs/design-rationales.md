# Design Rationale

### TODO: Document These Things
* [ ] Namespaces
* [ ] Overall architecture
* [ ] Class Hierarchies
* [ ] Factory design pattern for Play, Primitive, etc.
* [ ] Singleton design pattern for Logger, Drawer, etc.
* [ ] Constants vs. Dynamic parameters (why we have both, where each should be used, etc.)
* [ ] Visualizer -> Ai interfaces (ROS messages, websockets, etc.)
* [ ] Difference between HL components and Navigator components. Be clear about where the separation is, and why. Actions and Intents are not combined because Actions are part of HL, while Intents are part of Navigator. Combining them would break the abstraction and couple STP to the navigator, removing our flexibility to implement different HL systems in the future
#Misc
Diagrams in GitHub: https://github.com/jgraph/drawio-github

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

## The World
The `World` class/object is what we use to represent the state of the world at any given time. In this context, the world includes the positions and orientations of all robots on the field, the position and velocity of the ball, the dimensions of the field being played on, and the current refbox commands. Alltogether, it's all the information we have at any given time that we can use to make decisions.

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

## Geom
The `geom` folder is pretty self-explanaory and contains all of our geometry classes and geometry-related utility functions. These are all heavily used, since we represent robots and the ball as circles, navigation obstacles as polygons, navigation paths as Curves or Line Segments, and in general we use a lot of geometry in our strategy and evaluation functions when making decisions.

## AI
 
### High-Level / STP (Skills, Tactics, Plays)
`STP` is a framework for gameplay strategy originally proposed by Carnegie Mellon University back in 2004. The original paper can be found [here](https://kilthub.cmu.edu/articles/STP_Skills_Tactics_and_Plays_for_Multi-Robot_Control_in_Adversarial_Environments/6561002/1).

`STP` is a way of breaking down roles and responsabilities into a simple hierarchy, making it easier to build up more complex strategies from simpler pieces.

#### Skills / Actions
The `S` in `STP` stands for `Skills`. In our system, we call these `Actions`. Actions represent simple tasks an individual robot can do. Examples include:
1. Moving to a position (without colliding with anything)
2. Shooting at a target
3. Intercepting the ball

Actions use evaluation functions in combination with `Intents` to implement their behavior. Actions are responsible for obeying any preconditions `Intents` have.

#### Tactics
A `Tactic` represents a "single robot role" on a team. Examples include:
1. Being a goalie
2. Being a passer or pasee
3. Being a defender that shadows enemy robots
4. Being a defender that tries to steal the ball from enemies

Tactics use evaluation functions and `Actions` to implement their behavior. Using the `Action` abstraction makes it much easier for Tactics to express what they want to do, and make it easier to design and implement behavior.

#### Plays
A `Play` represents a "team-wide goal" for the robots, usually focused on a certain goal. They can be thought of much like Plays in real-life soccer. Examples include:
1. A Play for taking friendly corner kicks
2. A Play for defending enemy kickoffs
3. A general defense play
4. A passing-based offense play
5. A dribbling-based offense play

Plays are made up of `Tactics` and sometimes some evaluation functions. Plays can have "stages" and change what `Tactics` are being used as the state of the game changes, which allows us to implement more complex behavior.

Furthermore, every play specifies an `Applicable` and `Invariant` condition. These are used to determine what plays should be run at what time, and when a Play should terminate.

`Applicable` indicates when a `Play` can be started. For example, we would not want to choose the `Defense Play` if our team is in possession of the ball. The `Invariant` condition is a condition that must always be met for the `Play` to continue running. If this condition ever becomes false, the current `Play` will stop running and a new once will be chosen. For example, once we start running a friendly `Corner Kick` play, we want the `Play` to continue running as long as the enemy team does not have possession of the ball, so that would be our `Invariant` condition.


### Navigator
TODO

## Dynamic Parameters
`Dynamic Parameter` are the system we use to change values in our code at runtime. The reason we want to change values at runtime is primarily because we may want to tweak our strategy or aspects of our gameplay very quickly, and want to avoid having to stop the `AI`, make a change, recompile the code, and re-launch the `AI`. During games we are only allowed to touch our computers and make changes during halftime or a timeout, so every second counts! It's also very useful for testing things manually, and we also use it to communicate between the `Visualizer` and the rest of the system, so that the `Visualizer` can control the system.



# System Architecture
* [Glossary](#glossary)
* [Overview](#overview)
* [Backend](#backend)
* [AI](#ai)
* [Visualizer](#visualizer)
* [Diagram](#diagram)

## Glossary
A few commonly-used terms to be familiar with:
1. `SSL-Vision`
    * This is the shared vision system used by the Small Size League. It is what connects to the cameras above the field, does the vision processing, and transmits the positional data of everything on the field to our AI computers.
    * The GitHub repository can be found [here](https://github.com/RoboCup-SSL/ssl-vision)
2. `SSL-Gamecontroller`
    * Sometimes referred to as the "Refbox", this is another shared piece of Small Size League software that is used to send gamecontroller and referee commands to the teams. A human controls this application during the games to send the appropriate commands to the robots. For examples, some of these commands are what stage the gameplay is in, such as `HALT`, `STOP`, `READY`, or `PLAY`.
    * The GitHub repository can be found [here](https://github.com/RoboCup-SSL/ssl-game-controller)
3.  `grSim`
    * The general robot simulator used by the Small-Size-League. We use this to manually test strategy since it is easy to place the robots and ball in desired locations, run a strategy, and see what the robots do. It is not perfectly accurate, but is useful for testing high-level logic.
    * The GitHub repository can be found [here](https://github.com/RoboCup-SSL/grSim)
   

## Overview
At a high-level our system is made of 3 main components: The [Backend](#backend), the [AI](#ai), and the [Visualizer](#visualizer). These 3 components each run in their own thread, and communicate with each other using the [Observer design pattern](TODO use actual link).


## Backend
The `Backend` is responsible for all communication with the "outside world". It receives data over the network and also handles communication with the robots using our radio. The responsabilities of the `Backend` can be broken down into Input and Output.

### Input Responsabilities
1. Receiving robot status messages over the radio, and publishing this data to the rest of the system
2. Receiving camera data from SSL-Vision
2. Receiving referee commands from the Gamecontroller
3. Filtering the received vision and gamecontroller data
    * SSL-Vision only does vision processing, not filtering. This means that if there are several orange blobs on the field, SSL-Vision will send multiple locations for the ball. It is up to us to filter this data to determine the "correct" state of everything.
4. Storing the filtered data into the `World` datastructures understood by our system
5. Sending the filtered data to the rest of the system

### Output Responsabilities
1. Sending robot primitives to the robots

In practice, the `Backend` is just a simple interface that specifies `World` objects must be produced, and `Primitves` may be consumed. We have several implementations of the `Backend` to suite our needs. One imeplementation is used when we are controlling the physical robots, and need to use the radio. We have another implementation used to control the robots in grSim that uses the network rather than the radio to control the robots. The specifics of how this communication happens is hidden from the rest of the system making it very easy to switch backends depending on our needs.

## AI
The `AI` is where all of our gameplay logic takes place, and is the main "brain" of our system. It uses the information received by the [Backend](#backend) to make decisions, and sends `Primitives` back to the [Backend](#backend) for the robots to execute. Alltogether this feedback loop is what allows us to react to what's happening on the field and play soccer.

The `AI` is what implements things like strategy and navigation.


## Visualizer
The `Visualizer` is exactly what it sounds like: A visualizion of our [AI](#ai). It provides a GUI that shows us the state of the `World` as the [Backend](#backend) sees it, and is also able to display extra information that the [AI](#ai) would like to show. For example, it can show the planned paths of each friendly robot on the field, or highlight which enemy robots it thinks are a threat. Furthermore, it displays any warnings and status messages from the robots, such as if a robot is low on battery.

The `Visualizer` also lets us control the [AI](#ai) by setting `Parameters` (TODO: link here). Through the `Visualizer`, we can manually choose what strategy the [AI](#ai) should use, what team we think we are playing as, and tune more granular behavior such as how close an enemy must be to the ball before we consider them a threat.


## Diagram
![High Level Architecture Diagram](images/high_level_architecture_diagram.svg)
