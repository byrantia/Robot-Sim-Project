const test = require('node:test'), assert = require('node:assert/strict');
const { Engine } = require('../src/engine');
const { parseNavGraph } = require('../src/navgraph');
const { go } = require('./helpers');   // task:assign + robot:navigate

// A(0,0) - B(10,0) - C(20,0) "End". Other routes on the nav graph: near via D(10,3), or far via F(10,20).
// floor: the floor plan, as free(x, y, radius); none = no floor plan (open floor everywhere).
const WALLS = { free: () => false };                                              // nothing off the nav graph is free
const CORRIDOR = { free: (x, y, rad) => Math.abs(y) + rad <= 1 };                 // a 2 m wide corridor along y = 0
const map = (around, floor) => {
  const nav = parseNavGraph({ graphs: { L1: {
    A: { pose: { x: 0, y: 0 }, branches: ['B', ...around] }, B: { pose: { x: 10, y: 0 }, branches: ['C'] }, C: { pose: { x: 20, y: 0 }, locationName: 'End', branches: [] },
    D: { pose: { x: 10, y: 3 }, branches: ['C'] }, F: { pose: { x: 10, y: 20 }, branches: ['C'] } } } });
  if (floor) nav.occupancy.L1 = floor;
  return nav;
};
const setup = (around, robot = {}, floor) => {
  const e = new Engine(); e.setNav(map(around, floor));
  const id = e.createRobot({ speed: 1, sensorRange: 2, maxDetour: 5, startPosition: { level: 'L1', x: 0, y: 0, orientation: 0 }, ...robot }).data.robotID;
  const events = []; e.on('collision:event', (ev) => events.push(ev.response));
  return { e, id, events };
};
// runs the clock; returns how close the robot's centre came to (x, y)
const run = (e, seconds, id, x, y) => {
  let closest = Infinity;
  for (let i = 0; i < Math.round(seconds * 10); i++) {
    e.tick(0.1);
    if (id) { const p = e.getRobot({ robotID: id }).data.position; closest = Math.min(closest, Math.hypot(p.x - x, p.y - y)); }
  }
  return closest;
};
const robot = (e, id) => e.getRobot({ robotID: id }).data;
const task = (e) => e.getTask({ taskID: 'task_1' }).data;

test('an obstacle in the way inside sensor range: stop, then drive round it and rejoin the route', () => {
  const { e, id, events } = setup([]);
  e.addObstacle({ level: 'L1', x: 10, y: 0, radius: 0.5 });
  go(e, { robotID: id, destination: 'End' });
  run(e, 7.6);                                   // senses it at x = 7.5 (2 m from its edge)
  assert.deepEqual([robot(e, id).state, task(e).status, robot(e, id).blockedBy], ['Waiting', 'Waiting', ['obstacle_1']]);
  assert.ok(Math.abs(robot(e, id).position.x - 7.5) < 0.11);
  run(e, 1);                                     // decides after 1 s: a detour off the nav graph, then back onto B-C
  const route = robot(e, id).route;
  assert.ok(route[0].detour && route.at(-1).nodeID === 'C', JSON.stringify(route));
  assert.ok(route.some((p) => Math.abs(p.y) >= 0.85 - 1e-9), 'it steps aside at least obstacle radius + robot radius');
  assert.match(task(e).note, /Going round obstacle_1 \(up to 0\.\d m off the planned path\)/);
  const closest = run(e, 30, id, 10, 0);
  assert.ok(closest >= 0.8 - 1e-6, `never touches the obstacle: centre came within ${closest.toFixed(3)} m`);
  assert.deepEqual([task(e).status, events], ['Completed', ['stop', 'replan']]);
});

test('a narrow corridor: goes round a small obstacle; at one that leaves no room it waits, then is Stuck after 10 s', () => {
  const small = setup([], {}, CORRIDOR);
  small.e.addObstacle({ level: 'L1', x: 10, y: 0.2, radius: 0.2 });   // leaves room on the -y side
  go(small.e, { robotID: small.id, destination: 'End' });
  run(small.e, 30);
  assert.deepEqual([small.e.getTask({ taskID: 'task_1' }).data.status, small.events], ['Completed', ['stop', 'replan']]);

  const big = setup([], {}, CORRIDOR);
  big.e.addObstacle({ level: 'L1', x: 10, y: 0, radius: 0.5 });        // needs 0.85 m aside + 0.3 m off the wall: no room
  go(big.e, { robotID: big.id, destination: 'End' });
  run(big.e, 18);                                                      // senses it at 7.5 s, first reroute fails at 8.5 s
  assert.deepEqual([robot(big.e, big.id).state, task(big.e).status, big.events], ['Waiting', 'Waiting', ['stop', 'wait']]);
  run(big.e, 1);                                                       // 10 s after that first failed try: Stuck
  assert.deepEqual([robot(big.e, big.id).state, task(big.e).status, big.events], ['Stuck', 'Waiting', ['stop', 'wait', 'stuck']]);
  assert.match(task(big.e).note, /Stuck at obstacle_1: still no way past after 10 s \(no space to go round it on the floor plan, and no other route on the nav graph\)/);
  big.e.removeObstacle({ obstacleID: 'obstacle_1' });                 // still carries on by itself once the way is clear
  run(big.e, 15);
  assert.deepEqual([task(big.e).status, big.events], ['Completed', ['stop', 'wait', 'stuck', 'resume']]);
});

test('no room on the floor plan: another route on the nav graph close to the planned path', () => {
  const { e, id, events } = setup(['D'], {}, WALLS);
  e.addObstacle({ level: 'L1', x: 10, y: 0, radius: 0.5 });
  go(e, { robotID: id, destination: 'End' });
  run(e, 8.6);
  assert.deepEqual(robot(e, id).route.map((p) => p.nodeID), ['A', 'D', 'C']);   // back along its link to A, then round via D
  assert.match(task(e).note, /Rerouted round obstacle_1 on the nav graph/);
  run(e, 40);
  assert.deepEqual([task(e).status, events], ['Completed', ['stop', 'replan']]);
});

test('every way round too far from the planned path: stop near the obstacle and wait; carry on once it is removed', () => {
  const { e, id, events } = setup(['F'], {}, WALLS);
  e.addObstacle({ level: 'L1', x: 10, y: 0, radius: 0.5 });
  go(e, { robotID: id, destination: 'End' });
  run(e, 15);
  const r = robot(e, id);
  assert.deepEqual([r.state, task(e).status, r.blockedBy], ['Waiting', 'Waiting', ['obstacle_1']]);
  assert.ok(Math.abs(r.position.x - 9) < 1e-6, `stops 0.2 m short of touching: x ${r.position.x}`);   // 10 - 0.5 - 0.3 - 0.2
  assert.match(task(e).note, /Waiting at obstacle_1: no space to go round it .*the other route on the nav graph goes 20\.00 m from the planned path \(limit 5 m\)/);
  assert.deepEqual(events, ['stop', 'wait']);
  e.removeObstacle({ obstacleID: 'obstacle_1' });
  run(e, 15);
  assert.deepEqual([task(e).status, events], ['Completed', ['stop', 'wait', 'resume']]);
});

test('a bigger maxDetour lets it take the far way round', () => {
  const { e, id, events } = setup(['F'], { maxDetour: 25 }, WALLS);
  e.addObstacle({ level: 'L1', x: 10, y: 0, radius: 0.5 });
  go(e, { robotID: id, destination: 'End' });
  run(e, 9);
  assert.deepEqual(robot(e, id).route.map((p) => p.nodeID), ['A', 'F', 'C']);
  assert.deepEqual(events, ['stop', 'replan']);
});

test('a detour must also stay within maxDetour', () => {
  const { e, id, events } = setup([], { maxDetour: 0.5 });              // going round needs 0.85 m aside
  e.addObstacle({ level: 'L1', x: 10, y: 0, radius: 0.5 });
  go(e, { robotID: id, destination: 'End' });
  run(e, 15);
  assert.deepEqual([robot(e, id).state, events], ['Waiting', ['stop', 'wait']]);   // a way round exists, just too far
  assert.match(task(e).note, /Waiting at obstacle_1: the way round on the floor plan goes 0\.\d\d m from the planned path \(limit 0\.5 m\)/);
  run(e, 5);                                                           // too far is no way past either: Stuck after 10 s
  assert.deepEqual([robot(e, id).state, events], ['Stuck', ['stop', 'wait', 'stuck']]);
});

test('an obstacle in sensor range but not in the way only slows the robot down', () => {
  const { e, id, events } = setup([]);
  e.addObstacle({ level: 'L1', x: 5, y: 1.5, radius: 0.3 });   // 1.5 m beside the path: the footprint passes clear
  go(e, { robotID: id, destination: 'End' });
  run(e, 5);                                     // full speed to x ≈ 3.26 (enters sensor range), then half speed
  const x = robot(e, id).position.x;
  assert.ok(x > 3.9 && x < 4.4, `x ${x}`);
  run(e, 30);
  assert.deepEqual([task(e).status, events], ['Completed', []]);
});

test('an obstacle beyond sensor range is ignored until the robot gets close', () => {
  const { e, id } = setup([], { sensorRange: 0.5 });
  e.addObstacle({ level: 'L1', x: 10, y: 0, radius: 0.5 });
  go(e, { robotID: id, destination: 'End' });
  run(e, 8.5);
  assert.equal(robot(e, id).state, 'Moving');
  run(e, 0.6);
  assert.equal(robot(e, id).state, 'Waiting');
  assert.ok(robot(e, id).position.x <= 9 + 1e-6);   // never closer than STOP_GAP short of touching
});

test('an obstacle on the destination: no way round can help, so it waits and then is Stuck', () => {
  const { e, id } = setup(['D']);
  e.addObstacle({ level: 'L1', x: 20, y: 0, radius: 0.5 });
  go(e, { robotID: id, destination: 'End' });
  run(e, 30);
  assert.deepEqual([robot(e, id).state, task(e).status], ['Stuck', 'Waiting']);
  assert.match(task(e).note, /Stuck at obstacle_1: still no way past after 10 s \(it is on the destination/);
});

test('sensorRange and maxDetour are set on create and configure, and checked', () => {
  const e = new Engine();
  const r = e.createRobot({}).data;
  assert.deepEqual([r.sensorRange, r.maxDetour, r.blockedBy], [2, 5, []]);
  assert.equal(e.createRobot({ sensorRange: 0 }).failureCode, 1001);
  assert.equal(e.configureRobot({ robotID: r.robotID, fields: { sensorRange: 3.5, maxDetour: 0 } }).data.sensorRange, 3.5);
  assert.equal(e.configureRobot({ robotID: r.robotID, fields: { maxDetour: -1 } }).failureCode, 1001);
});
