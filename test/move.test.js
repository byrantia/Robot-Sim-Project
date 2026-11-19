const test = require('node:test'), assert = require('node:assert/strict');
const { Engine } = require('../src/engine');
const { parseNavGraph } = require('../src/navgraph');
const { go } = require('./helpers');   // task:assign + robot:navigate

// L1: A(0,0) - B(10,0) "End"
const setup = (floor) => {
  const e = new Engine(), nav = parseNavGraph({ graphs: { L1: { A: { pose: { x: 0, y: 0 }, branches: ['B'] }, B: { pose: { x: 10, y: 0 }, locationName: 'End', branches: [] } } } });
  if (floor) nav.occupancy.L1 = floor;
  e.setNav(nav);
  const id = e.createRobot({ speed: 1, startPosition: { level: 'L1', x: 0, y: 0, orientation: 0 } }).data.robotID;
  const events = []; e.on('collision:event', (ev) => events.push(ev));
  return { e, id, events };
};
const run = (e, seconds) => { for (let i = 0; i < Math.round(seconds * 10); i++) e.tick(0.1); };
const robot = (e, id) => e.getRobot({ robotID: id }).data;
const near = (a, b, msg) => assert.ok(Math.abs(a - b) < 1e-6, `${msg || ''} ${a} is not ${b}`);

test('forward drives the distance along its heading at its speed, then goes Idle', () => {
  const { e, id } = setup();
  const r = e.moveRobot({ robotID: id, command: 'forward', distance: 2 }).data;
  assert.deepEqual([r.state, r.activeMove], ['Moving', { command: 'forward', distance: 2, done: 0 }]);
  run(e, 1);
  near(robot(e, id).position.x, 1); assert.equal(robot(e, id).activeMove.done.toFixed(2), '1.00');
  run(e, 1.1);
  const done = robot(e, id);
  near(done.position.x, 2); near(done.position.y, 0);
  assert.deepEqual([done.state, done.activeMove], ['Idle', null]);
});

test('backward reverses without turning; rotate turns on the spot (+ left, - right)', () => {
  const { e, id } = setup();
  e.moveRobot({ robotID: id, command: 'backward', distance: 1 });
  run(e, 1.1);
  near(robot(e, id).position.x, -1); near(robot(e, id).orientation, 0);
  e.moveRobot({ robotID: id, command: 'rotate', angle: 90 });
  run(e, 0.3);
  assert.equal(robot(e, id).state, 'Moving'); near(robot(e, id).orientation, 54);   // 180°/s
  run(e, 0.3);
  near(robot(e, id).orientation, 90); assert.equal(robot(e, id).state, 'Idle');
  e.moveRobot({ robotID: id, command: 'forward', distance: 1 });                      // now drives along +y
  run(e, 1.1);
  near(robot(e, id).position.x, -1); near(robot(e, id).position.y, 1);
  e.moveRobot({ robotID: id, command: 'rotate', angle: -180 });
  run(e, 1.1);
  near(robot(e, id).orientation, -90);
});

test('stop ends a move where the robot is; a new move replaces the one running', () => {
  const { e, id } = setup();
  e.moveRobot({ robotID: id, command: 'forward', distance: 5 });
  run(e, 1.5);
  assert.deepEqual([e.moveRobot({ robotID: id, command: 'stop' }).data.state, robot(e, id).activeMove], ['Idle', null]);
  run(e, 1);
  near(robot(e, id).position.x, 1.5);
  e.moveRobot({ robotID: id, command: 'forward', distance: 5 });
  run(e, 0.5);
  e.moveRobot({ robotID: id, command: 'backward', distance: 1 });
  run(e, 1.1);
  near(robot(e, id).position.x, 1);
});

test('a drive stops short of an obstacle in its way and reports it; it can back away again', () => {
  const { e, id, events } = setup();
  e.addObstacle({ level: 'L1', x: 3, y: 0, radius: 0.5 });
  e.moveRobot({ robotID: id, command: 'forward', distance: 5 });
  run(e, 5);
  const r = robot(e, id);
  near(r.position.x, 2, 'stops 0.2 m short of touching:');                         // 3 - 0.5 - 0.3 - 0.2
  assert.deepEqual([r.state, events.map((ev) => [ev.type, ev.response, ev.obstacleIDs[0]])], ['Idle', [['obstacle', 'stop', 'obstacle_1']]]);
  assert.match(events[0].message, /forward 5 m stopped after 2\.00 m: obstacle_1 is in the way/);
  e.moveRobot({ robotID: id, command: 'backward', distance: 1 });
  run(e, 1.1);
  near(robot(e, id).position.x, 1);
});

test('a drive stops short of a wall on the floor plan', () => {
  const { e, id, events } = setup({ free: (x, y, rad) => x + rad <= 4 });            // a wall at x = 4
  e.moveRobot({ robotID: id, command: 'forward', distance: 10 });
  run(e, 10);
  const x = robot(e, id).position.x;
  assert.ok(x <= 3.7 + 1e-9 && x > 3.6, `keeps its 0.3 m radius off the wall: x ${x}`);
  assert.deepEqual(events.map((ev) => [ev.type, ev.response]), [['wall', 'stop']]);
});

test('robot:move is refused on a task, in Error, and with bad values; a moving robot takes no task', () => {
  const { e, id } = setup();
  assert.equal(e.moveRobot({ robotID: id, command: 'jump' }).failureCode, 1001);
  assert.equal(e.moveRobot({ robotID: id, command: 'forward' }).failureCode, 1001);
  assert.equal(e.moveRobot({ robotID: id, command: 'forward', distance: -1 }).failureCode, 1001);
  assert.equal(e.moveRobot({ robotID: id, command: 'rotate', angle: 0 }).failureCode, 1001);
  assert.equal(e.moveRobot({ robotID: 'robot_9', command: 'stop' }).failureCode, 1002);
  e.moveRobot({ robotID: id, command: 'forward', distance: 1 });
  assert.equal(go(e, { robotID: id, destination: 'End' }).failureCode, 1004);
  assert.equal(e.localize({ robotID: id, position: { x: 0, y: 0 } }).failureCode, 1004);
  run(e, 1.1);
  go(e, { robotID: id, destination: 'End' });
  assert.equal(e.moveRobot({ robotID: id, command: 'forward', distance: 1 }).failureCode, 1004);
  assert.equal(e.moveRobot({ robotID: id, command: 'stop' }).failureCode, 1004);
  e.emergencyStop({ robotID: id });
  assert.equal(e.moveRobot({ robotID: id, command: 'rotate', angle: 10 }).failureCode, 1004);
  e.startRobot({ robotID: id });
  assert.equal(e.moveRobot({ robotID: id, command: 'rotate', angle: 10 }).success, true);
});
