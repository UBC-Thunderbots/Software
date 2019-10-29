# Code Style Guide

## Table of Contents
* [Table of Contents](#table-of-contents)
* [Coding Style and Conventions](#coding-style-and-conventions)
    * [Names and Variables](#names-and-variables)
    * [Comments](#comments)
    * [Headers](#headers)
    * [Includes](#includes)
    * [Spelling](#spelling)
    * [Exceptions](#exceptions)
    * [Miscellaneous](#miscellaneous)

## Coding Style and Conventions

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
  const float SCALE_FACTOR = 2.15;
  float distance = catch_distance * SCALE_FACTOR;
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

### Headers

* Use _header guards_ to prevent issues of duplicate or circular includes which cause the source code to be compiled multiple times and cause build errors. 
    * If you're writing C++ (ie. not C), use `#pragma once` at the very top of the file. 
    * If header guards are used, they should be fully capitalized and include `_H_` at the end.
      ```cpp
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
* Often `.cpp` files include its corresponding header `.h` file, it means the `.cpp` file include everything included in the header. Do not have duplicate `#include` in both `.h` and `.cpp` files.
* `#include`s are generally preferred written on the `.cpp` side; use minimum `#include` in the header file. Use _forward declarations_ in headers if necessary.
* Specify full path of the include file on the file system, relative to the top-level project directory or _CMakeList.txt_ file. Do not use relative paths.
  ```cpp
  // Incorrect
  #include "../../tactic/tactic.h"

  // Correct
  #include "ai/hl/stp/tactic/tactic.h"
  ```

### Spelling

* Code, comment, and documentations should not contain spelling errors.
* Locale-specific English words follow Canadian/British spelling ("colour", "neighbour", "honour").
    * Exceptions:
      * Use "defense" in lieu of "defence" as it is more similar to the word "defensive".
      * Use "offense" in lieu of "offence".

### Exceptions

* Throwing an exception indicates that the AI has entered an unrecoverable state.
* In almost all cases, it is preferable to return a `std::optional` type, so the caller has to handle the case of the called function "failing", perhaps alongside some logging that the error occured.

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
* Non-simple data types are generally passed by _const_ references. Use non-const references if the parameter is used to return data.
  ```cpp
  // Pass by const reference
  Point predictBallPosition(const Ball& ball);

  // Pass by reference to return data
  void getVisionPacket(Packet& packet);
  ```
* All constructors should be marked with the `explicit` keyword. In the case of a one-argument constructor, this prevents it from being used as an implicit type conversion; in the case of other constructors, it acts as a safeguard in case arguments are later added or removed.
  ```cpp
  explicit AI(const World& world);
  ```
* Use C++ smart pointers. Do not use raw pointers unless _absolutely_ necessary (and even then, probably not) (See also: [what are smart pointers and why they are good](https://stackoverflow.com/questions/106508/what-is-a-smart-pointer-and-when-should-i-use-one))
* Explicitly use the namespace when using invoking classes or functions with namespaces. Do not use `using namespace` in the source. If the namespace hierarchy is long, shorten it.
  ```cpp
  // Incorrect
  using namespace AI::HL::STP::Action;
  int id = getId();

  // Correct
  namespace Action = AI::HL::STP::Action;
  int id = Action::getId();
  ```
* Do not use `using namespace ...` in header files.
* Use C++11 keyword `using` to make _type alias_ instead of `typedef` as they're equivalent except the former is compatible with templates.
  ```cpp
  // Incorrect
  typedef std::vector<std::pair<int, int>> PointsArray;

  // Correct
  using PointsArray = std::vector<std::pair<int, int>>;
  ```
* Avoid initializing multiple variables on the same line.
  ```cpp
  // Incorrect
  int x, y, z = 0;

  // Correct
  int x;
  int y;
  int z = 0;
  ```
* Reference \(`&`\) and pointer \(`*`\) symbols should be attached to the type and not the variable name.
  ```cpp
  // Incorrect
  explicit AI(const World &world);
  int *num;

  // Correct
  explicit AI(const World& world);
  int* num;
  ```
* Opening and closing braces \(`{}`\) must be on the same indentation level.
  ```cpp
  // Incorrect
  int foo() {
      return 0;
  }

  // Correct
  int foo()
  {
      return 0;
  }
  ```
