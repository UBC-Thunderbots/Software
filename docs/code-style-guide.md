# Code Style Guide

### Table of Contents

<!--TOC-->

- [Table of Contents](#table-of-contents)
- [Names and Variables](#names-and-variables)
- [Comments](#comments)
- [Headers](#headers)
- [Includes](#includes)
- [Namespaces](#namespaces)
- [Exceptions](#exceptions)
- [Tests](#tests)
- [Getter And Setter Functions](#getter-and-setter-functions)
- [Static Creators](#static-creators)
- [Spelling](#spelling)
- [Miscellaneous](#miscellaneous)
- [Protobuf](#protobuf)

<!--TOC-->

Our C++ coding style is based off of [Google's C++ Style Guide](https://google.github.io/styleguide/cppguide.html). We use [clang-format](https://clang.llvm.org/docs/ClangFormat.html) to enforce most of the nit-picky parts of the style, such as brackets and alignment, so this document highlights the important rules to follow that clang-format cannot enforce.

If you want to know more about our coding style you can take a look at our [clang-format configuration file](https://github.com/UBC-Thunderbots/Software/blob/master/.clang-format).


### Names and Variables

* Classes, structures, namespaces, unions, enumerates, "typename"-type template parameters, and typedefs names uses `CamelCase` with a capital letter.

  ```cpp
  // Incorrect
  class eventHandler;

  // Incorrect
  class event_handler;

  // Correct
  class EventHandler;
  ```

* Constant variables uses `ALL_CAPS_WITH_UNDERSCORES`. Constant variables include _static const_, _const class members_, _const_ global variables, and _const_ enumerations.
* All variable names are `lowercase_with_underscores`
  ```cpp
  // Incorrect
  float calculatedDistnace;

  // Correct
  float calculated_distance;
  ```
* Function and method names are `camelCase` with leader lowercase letter.
  ```cpp
  // Incorrect
  float get_distance();

  // Incorrect
  float GetDistance();

  // Correct
  float getDistance();
  ```
* File names are lower case and can contain underscores. The name should match or similar to the class/classes/functions it contains. Avoid using ambiguous names or names already exist in common libraries.
  ```cpp
  // Incorrect
  ShootGoal.h

  // Incorrect
  sg.h

  // Acceptable
  shootgoal.h

  // Correct
  shoot_goal.h
  ```
* Avoid "obvious" or "magic" numbers unless it's part of a mathematical or physics formula (ex `A=0.5(b*h)`).
  ```cpp
  // Incorrect
  float distance = catch_distance * 2.15;

  // Correct
  const float CATCH_DISTANCE_SCALE_FACTOR = 2.15;
  float distance = catch_distance * CATCH_DISTANCE_SCALE_FACTOR;
  ```
* Avoid initializing multiple variables on the same line.
  ```cpp
  // Incorrect
  int x, y, z = 0;

  // Correct and equivalent to the above
  int x;
  int y;
  int z = 0;

  // However, the author may have intended the following
  // or a code reader may have assumed the following
  int x = 0;
  int y = 0;
  int z = 0;
  ```


### Comments

Make sure to comment both the interface for a function or class, as well as the logic in the implementation. In general, try to make as many in-code documentations whenever possible.

As much code documentation as possible should live with the code itself \(in the form of comments\). This makes it easier for people working on the code to find the information, and because the code and comments are version-controlled together if we ever go back to an older version of the code, we will have the corresponding older documentation as well.

*Code comments are very important. Be sure to comment your code well enough so that another member of the team would be able to quickly get an understanding of what your code is doing, **and why**. Try not to make your comments unnecessarily verbose, but include as much detail you feel is necessary adequately explain the code. We realize that sounds contradictory, but use your best judgement as to what you think is clear and readable.*

If you think some ASCII art will help explain something better, go for it! [asciiflow](http://asciiflow.com/) is a good online tool for creating this.

* Comments regarding the interface of a class and its methods must be in the header file.
* In-code documentation comments and function comments follow the [javadoc style](https://www.tutorialspoint.com/java/java_documentation.htm).
  ```cpp
  // Incorrect
  // This function returns the power
  float power(float a, float b);

  // Correct
  /**
   * This function returns the power
   * @param a the base
   * @param b the exponent
   * @return a^b
   */
  float power(float a, float b);
  ```
    * Functions that perform similar tasks should be commented as a group if it improves clarity. 
    
    Example 1: functions with different ordering on arguments
    ```cpp
    /**
     * Returns true if the polygon intersects the circle, false otherwise.
     *
     * @param first
     * @param second
     * @return true if the polygon intersects the circle, false otherwise
     */
    bool intersects(const Polygon &first, const Circle &second);
    bool intersects(const Circle &first, const Polygon &second);
    ```
    Example 2: visitor functions
    ```cpp
    // The javadoc comment for all methods here can be read as:
    /**
     * Visits an instance of X to perform an operation
     *
     * @param tactic The tactic to visit
     */

    virtual void visit(MoveTactic &tactic)               = 0;
    virtual void visit(ShadowFreekickerTactic &tactic)   = 0;
    virtual void visit(GoalieTactic &tactic)             = 0;
    etc...
    ```



### Headers

* Use _header guards_ to prevent issues of duplicate or circular includes which cause the source code to be compiled multiple times and cause build errors. 
    * If you're writing C++ (ie. not C), use `#pragma once` at the very top of the file. 
      ```cpp
      #pragma once
      ```
    * If you're writing C, header guards should be used. They should be fully capitalized and include `_H_` at the end.
      ```c
      #ifndef AI_WORLD_FIELD_H_
      #define AI_WORLD_FIELD_H_
      ```
      At the end of the file, make sure to close the conditional preprocessor. An inline comment must be at the end with the same name as the _\#define_
      ```cpp
      #endif /* AI_WORLD_FIELD_H_ */
      ```
*In general the reason a header guard's name is so complicated is to make sure that it is unique. There cannot be any other header guards or constants defined with #define anywhere else in the code, or weird build issues may occur.*


### Includes

* Use `#include` sparingly and only include the necessary sources to build the file. Do not include headers whos class or implementation is not used.
* Often `.cpp` files include its corresponding header `.h` file, it means the `.cpp` file include everything included in the header. Do not have duplicate `#include`'s in both `.h` and `.cpp` files.
* `#include`s are generally preferred written on the `.cpp` side; use minimum `#include` in the header file. Use _forward declarations_ in headers if necessary.
* Specify full path of the include file on the file system, relative to the top-level project directory or _WORKSPACE_ file. Do not use relative paths.
  ```cpp
  // Incorrect
  #include "../../tactic/tactic.h"
 
  // Incorrect
  #include "tactic.h"

  // Correct
  #include "software/ai/hl/stp/tactic/tactic.h"
  ```

### Namespaces
* We generally don't use namespaces because they add extra complexity, and we do not really need the protection they provide
    * We have found it's generally more work than it's worth to have everyone remember to put things in the right namespaces when writing code, and remember to manage the namespaces when code is being used
    * The main purpose of namespaces is to compartmentalize code and help avoid conflicts. This way if 2 libraries define functions called `add()`, as long as they are in different namespaces they can be specified and used independently without issue. Because we aren't publishing our code as a library, and most libraries we use already do their own namespacing, we don't _really_ need the protection namespaces provide in this case.
* Do not use `using namespace ...` in header files.
    * This is because any file that includes this header will also implicitly be using the namespace, which can cause subtle issues and naming conflicts.


### Exceptions

* Throwing an exception indicates that the AI has entered an unrecoverable state.
* In almost all cases, it is preferable to return a `std::optional` type, so the caller has to handle the case of the called function "failing", perhaps alongside some logging that the error occurred.


### Tests
Some general guidelines when writing tests are:

* **Tests should test a single, distinct behaviour, isolated to a single class (or group of functions in C).** For example, if you have a test called `assign_and_clear_goalie`, it should probably be broken up into `assign_goalie` and `clear_goalie`. While `clear_goalie` might depend on `assign_goalie`, structuring things like this allows us to quickly narrow down where the problem might be by just looking at what tests failed, without having to go tear apart large unit tests.
* **Don't be afraid to use long test names.**: When naming things, as programmers we keep names short so that they can be used elsewhere without taking up an entire line. But no one is going to be using your test name elsewhere, so feel free to be verbose. Instead of `equality_operator_3`, call your test `equality_operator_two_teams_different_number_of_robots`

### Getter And Setter Functions
* in general, getter and setter methods on classes should be written like `getName()`, `setName(string name)`, with the following exceptions
  * getters with the return type `bool` may be prefixed with `is` instead of `get`, ie. `bool isActive()`
  * getters that are used _incredibly_ frequently and are _incredibly_ obvious may not require the `get` prefix. For example `Point::x()` and `Point::y()` 
  * getters that return specific units should be written as `toUnit()`. For example `Angle::toDegrees()`


### Static Creators
* static creators that take an argument of a specific unit should be written as `fromUnit(datatype unit)`. For example `Angle::fromDegrees(double deg)`


### Spelling

* Code, comment, and documentations should not contain spelling errors.
* Locale-specific English words follow Canadian/British spelling ("colour", "neighbour", "honour", "metre").
    * Exceptions:
      * Use "defense" in lieu of "defence" as it is more similar to the word "defensive".
      * Use "offense" in lieu of "offence".


### Miscellaneous

* All source files should have an empty line at the end of the file to avoid issues with GitHub.
* Avoid using C-style casts like `(int) x`. Use C++ casts such as static cast:
  ```cpp
  static_cast<int>(x);
  ```
* Simple data types \(`char`, `short`, `int`, `long`, `float`, `double`, `pointers`\) are generally pass by value.
  ```cpp
  void foo(double x);
  ```
* Non-simple data types are generally passed by _const_ references whenever possible. Try avoid setting values by reference, since this makes it harder to follow the flow of control and data in the program.
  ```cpp
  // Not ideal
  // Pass by reference to set data
  void getVisionPacket(Packet& packet);

  // Preferred
  // Pass by const reference
  Point predictBallPosition(const Ball& ball);
  ```

* Use private instead of impl when naming files that are used for
private functions for a class/file that should not be exposed to users. impl can be confused with pimpl, so using private in the naming is more clear.


* All constructors should be marked with the `explicit` keyword. In the case of a one-argument constructor, this prevents it from being used as an implicit type conversion; in the case of other constructors, it acts as a safeguard in case arguments are later added or removed.
  ```cpp
  explicit AI(const World& world);
  ```
* Use C++ smart pointers and avoid raw pointers. (See also: [what are smart pointers and why they are good](https://stackoverflow.com/questions/106508/what-is-a-smart-pointer-and-when-should-i-use-one))
* Use C++11 keyword `using` to make _type alias_ instead of `typedef` as they're equivalent except the former is compatible with templates.
  ```cpp
  // Incorrect
  typedef std::vector<std::pair<int, int>> PointsArray;

  // Correct
  using PointsArray = std::vector<std::pair<int, int>>;
  ```
* Avoid ternary operators. Clarity is more important than line count.
  ```cpp
  // Incorrect
  c = ((a == 0) || ((a + b) < 10)) ? a : a + b;

  // Correct
  if ((a == 0) || ((a + b) < 10))
  {
    c = a;
  }
  else
  {
    c = a + b;
  }
  ```
* Always use curly braces around code blocks, even if the braces surround a single statement.
  ```cpp
  // Incorrect
  while (i < 10)
    i++;
    c[i] = i + 1;

  // Correct
  while (i < 10)
  {
    i++;
  }
  c[i] = i + 1;
  ```

### Protobuf

Protobufs that we define should follow [Google's Protobuf Style Guide](https://developers.google.com/protocol-buffers/docs/style).

* When initializing a protobuf from a unique pointer, avoid passing the `release()`'d pointer to `set_allocated` and `AddAllocated`. Releasing unique pointers makes it too easy to cause memory leaks. This convention does result in an extra copy operation for the field being added to the proto, but we have decided that this is acceptable to reduce the risk of memory leaks.

  ```cpp
  message TbotsRobotMsg
  {
  ...
      repeated ErrorCode error_code             = 2;
  ...
      TimestampMsg time_sent                    = 11;
  }

  TbotsRobotMsg tbots_robot_msg;
  std::unique<TimestampMsg> timestamp_msg = // creator function
  std::unique<ErrorCode> error_code = // creator function
  google::protobuf::RepeatedPtrField<SSL_FieldLineSegment> segments;
  std::unique<SSL_FieldLineSegment> field_segment = // creator function

  // Incorrect
  tbots_robot_msg.set_allocated_time_sent(timestamp_msg.release());
  segments.AddAllocated(field_segment.release());
  *(tbots_robot_msg.add_error_code()) = *error_code;

  // Correct
  *(tbots_robot_msg.mutable_time_sent()) = *timestamp_msg;
  *(segments.Add()) = *field_segment;
  *(tbots_robot_msg.add_error_code()) = *error_code;
  ```
