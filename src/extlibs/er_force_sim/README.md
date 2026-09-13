# The Simulator from Roboterfußballmannschaft der Friedrich-Alexander-Universität Erlangen-Nürnberg Robotics Erlangen's SSL-Team ER-Force's Framework

The simulator is adapted from [robotics-erlangen/framework](https://github.com/robotics-erlangen/framework). It contains several bug fixes and general code quality improvements, and it is modified so that time steps can be controlled by a `stepSimulation` function.


## Upstream sync

This is a fork, not a vendored copy. It was forked at upstream `73e139db` (2021-12-16)
and individual upstream changes have been ported onto it by hand since, because the fork
has diverged too far for cherry-picks to apply: Qt was removed (upstream is on Qt6), the
API was made synchronous for our deterministic test runner, and the ball model was
replaced with our own.

Last sync: upstream `38563d11` (2026-07-27), covering every change to
`src/amun/simulator` up to that commit.

To find what is new upstream since then:

```
git clone https://github.com/robotics-erlangen/framework
git -C framework log 38563d11..HEAD -- src/amun/simulator src/protobuf/protobuf/ssl_sim
```

Deliberately not ported:

- the Qt6 migration, the protobuf folder reorganisation and compiler warning fixes,
  which conflict with our own versions of the same code
- `fastsimulator` and `erroraggregator`, which serve upstream's own tooling
- `Specs.simulation_limits` in the upstream unit of wheel rotations; ours uses linear
  units, matching our robot constants
- everything outside of the simulator (Ra, strategy, tracking, logging)


## Copyright

```
/***************************************************************************
 *   Copyright 2020 Michael Eischer, Philipp Nordhus, Andreas Wendler      *
 *   Robotics Erlangen e.V.                                                *
 *   http://www.robotics-erlangen.de/                                      *
 *   info@robotics-erlangen.de                                             *
 *                                                                         *
 *   This program is free software: you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation, either version 3 of the License, or     *
 *   any later version.                                                    *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>. *
 ***************************************************************************/
```
