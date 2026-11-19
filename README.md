# Robot simulator: Socket.IO API, simulation engine and traffic manager

## Install and run
1. Install Node.js 18 or newer (20 LTS is fine): https://nodejs.org
2. In this folder:
   ```
   npm install
   npm start
   ```
3. Open http://localhost:3000 (not the .html file directly: the page must come from the server).
4. Tests: `npm test` (no server needed).
5. Server terminal log: `LOG=all` (PowerShell: `$env:LOG='all'`) for every movement update, `LOG=off` for none.

## What to try
1. The page loads the nav graph in `data/` and sends it to the server (top bar shows "Server: connected").
2. Robots > Add robot on map. The server creates it and gives it its ID.
3. Select the robot, choose a destination (and a lift, if it is on another level), press **Assign task**.
   The page sends `task:assign` (creates the job) and then `robot:navigate` (the robot takes it and drives).
4. Add a second robot and send both across the same crossing or corridor: one waits (Waiting) and carries on by itself.
5. **Stop** pauses (Paused) until **Resume**; **Cancel task** ends the job; **Emergency stop** puts the robot in Error.

## Replies (draft 7.5)
`{ success: true, data }` or `{ success: false, failureCode, failureMessage }`

| Code | Meaning |
|---|---|
| 1001 | Missing or wrong field |
| 1002 / 1003 | Robot / task not found |
| 1004 | Robot not in the right state (for example it already has a job) |
| 1006 | Event is in the draft but not built yet |
| 2002 | Destination is not a point of interest or node on the nav graph |
| 2003 | No route to the destination |
| 2004 | No nav graph imported yet |
| 2005 | Obstacle not found |
| 3005 | Emergency stop |
| 3006 | Task cancelled |

These codes are placeholders (`src/protocol.js`).

## Jobs
- `task:assign { destination, level?, lift?, robotID?, taskID? }` creates a job (Queued). Nothing moves.
  With `robotID` the job is reserved for that robot.
- `robot:navigate { robotID, taskID }` gives a Queued job to the robot: it plans the route (BFS on the nav graph, by
  lift if needed) and starts driving. One active job per robot. No route: 2003, and the job stays Queued.

## Robot states
Idle, Moving, Waiting (working out a way past an obstacle or another robot, or in a lift), Stuck (still no way past
after a while; it keeps trying), Paused (`robot:stop`), Completed, Error.

## Traffic (several robots)
`src/traffic.js` is a central Traffic Manager. Every tick it looks at every robot's route, position, speed and
priority, and decides how far along its route each robot may drive. The movement engine then moves the robot as usual,
no further than that.
- **Reservations.** A robot on a job reserves the stretch of route it will drive in the next 3 s. Reservations and
  robots always stay 1 m apart (centre to centre: 0.3 m robot radius each side plus a 0.4 m gap). A robot that drives
  only inside its own reservation cannot hit another robot.
- **Priority.** The job started first (by `robot:navigate`) goes first.
- **Crossings.** A lower-priority robot may not stop in the path a higher-priority robot will drive in the next 6 s.
  So at a crossing it waits short of it (Waiting) and carries on once the other has passed. It may still cross in one
  go when that robot is held up itself. Robots going the same way just follow each other.
- **Narrow corridors.** These are worked out from the floor plan: links with no room for two robots to pass. A robot
  in one holds it in its direction. A robot coming the other way waits 1 m before the way in.
- **Giving way.** Sometimes robots on jobs block each other. Two robots can be head-on, or several can wait in a
  ring, each for the next. If none of them has moved for 5 s, the one whose job started last gives way:
  1. It goes round the others, with the same detour or nav graph reroute used for obstacles.
  2. If it cannot, it makes way: it drives to a nearby node off their path, waits there until they have passed,
     then carries on to its destination.
  If it can do neither, the next robot in the ring tries 2 s later, and so on. All of this happens before they are
  Stuck (10 s). A Stuck robot never gives way or turns round: it stays put until you drag one of them away.
- **Waiting, not turning.** Nobody gives way to a robot that is not waiting for it. That covers an idle robot
  standing in the way, a robot that is just passing, and a paused one. The robot waits where it is, on its route,
  and carries on along that same route once the way is clear. If the robots it waits for have not moved for 10 s it
  shows Stuck, but it is still waiting there. You can drag a robot out of the way to help.
- **Paused robots.** A paused robot (`robot:stop`) keeps its right of way: others keep out of its path, so it can carry
  on when it is resumed. A robot waiting for a paused robot stays Waiting (never Stuck).
- `collision:event` with `type: 'robot'` reports `wait`, `resume`, `replan` and `stuck`. `robotIDs` lists the robot,
  then the robots in its way. `blockedBy` on the robot lists them too.
- `robot:move` stops 1 m short of another robot. A robot coming out of a lift waits in it while the way out is taken.

## Moving a robot by hand
Drag a robot on the map, which sends `robot:localize`.
- A robot with no job just moves there, and that becomes its start point for `sim:reset`.
- A robot on a job can be moved while it is Waiting, Stuck or Paused, like pushing it out of the way in real life.
  - It plans its route again from where you drop it, and carries on (a paused robot stays paused until Resume).
  - A robot that was waiting for it carries on by itself as soon as its way is clear.
  - Its `sim:reset` start point does not change.
- Refused while the robot is driving (pause it first), in a lift, or when there is no route from the drop point.

## Files
- `src/engine.js`: robots, jobs and rules, movement each tick, obstacle and traffic responses. No network code, so it
  can be tested alone.
- `src/simulation.js`: driving along a route (turning, lifts), obstacle checks, and the clock.
- `src/traffic.js`: the Traffic Manager (reservations, priority, corridors).
- `src/planner.js`: BFS route planning and detours round obstacles.
- `src/navgraph.js`, `src/occupancy.js`: the nav graph reader and its floor plans.
- `src/protocol.js`: reply format and failure codes. `src/terminal-log.js`: the server terminal log.
- `server.js`: Express and Socket.IO adapter.
- `public/index.html`: the viewer. `public/api.js`: connects it to the server.
- `data/`: nav graph files. The page's Save writes `*_edited.json` here.

## Known limits
- Loading a nav graph (including a page reload) resets the server's robots and tasks.
- A robot that has finished its job stays where it stopped. If that is on another robot's route, the other robot
  waits there (Stuck) until you drag one of them out of the way (see Moving a robot by hand).
- Giving way needs room to go round, or a node within 20 m that is off the other robots' path. Without either (say,
  head-on in a single dead-end corridor), both stay Stuck until you drag one away.
- Not built yet (they reply 1006): `robot:pause`, `robot:return-to-dock`, `test:list` / `test:run` / `test:get`.
- Local development only: the server listens on 127.0.0.1 and has no login.
