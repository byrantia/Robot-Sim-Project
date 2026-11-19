const test = require('node:test'), assert = require('node:assert/strict');
const { Engine } = require('../src/engine');
const { parseNavGraph } = require('../src/navgraph');
const { go } = require('./helpers');   // task:assign + robot:navigate

// L1: A(0,0) - B(10,0) "End"; a lift X from B to L2's E(0,0) - F(0,5) "Ward"
const setup = () => {
  const e = new Engine();
  e.setNav(parseNavGraph({ graphs: {
    L1: { A: { pose: { x: 0, y: 0 }, branches: ['B'] }, B: { pose: { x: 10, y: 0 }, locationName: 'End', branches: [] } },
    L2: { E: { pose: { x: 0, y: 0 }, branches: ['F'] }, F: { pose: { x: 0, y: 5 }, locationName: 'Ward', branches: [] } } },
    liftConfig: { lifts: [{ id: 'X', details: [{ mapName: 'L1', liftPoint: 'B' }, { mapName: 'L2', liftPoint: 'E' }] }] } }));
  const id = e.createRobot({ speed: 1, startPosition: { level: 'L1', x: 0, y: 0, orientation: 0 } }).data.robotID;
  return { e, id };
};
const run = (e, seconds) => { for (let i = 0; i < Math.round(seconds * 10); i++) e.tick(0.1); };
const robot = (e, id) => e.getRobot({ robotID: id }).data;
const task = (e) => e.getTask({ taskID: 'task_1' }).data;

test('robot:stop pauses a task where the robot is; robot:start carries on from there', () => {
  const { e, id } = setup();
  go(e, { robotID: id, destination: 'End' });
  run(e, 3);
  const r = e.stopRobot({ robotID: id }).data;
  assert.deepEqual([r.state, r.stopped, r.activeTask, task(e).status], ['Paused', true, 'task_1', 'Waiting']);
  assert.match(task(e).note, /Paused \(robot:stop\)/);
  run(e, 5);
  assert.ok(Math.abs(robot(e, id).position.x - 3) < 1e-6, 'holds where it is');
  assert.equal(e.stopRobot({ robotID: id }).success, true, 'stopping again changes nothing');
  const s = e.startRobot({ robotID: id }).data;
  assert.deepEqual([s.state, s.stopped, task(e).status, task(e).note], ['Moving', false, 'Running', 'Resumed (robot:start).']);
  run(e, 7.5);
  assert.equal(task(e).status, 'Completed');
});

test('task:cancel cancels a stopped (or running) task; the robot goes Idle where it is', () => {
  const { e, id } = setup();
  go(e, { robotID: id, destination: 'End' });
  run(e, 2);
  e.stopRobot({ robotID: id });
  const t = e.cancelTask({ taskID: 'task_1' }).data;
  assert.deepEqual([t.status, t.outcome, t.error.failureCode], ['Failed', 'Cancelled before reaching the destination.', 3006]);
  const r = robot(e, id);
  assert.deepEqual([r.state, r.stopped, r.activeTask, r.route.length], ['Idle', false, null, 0]);
  assert.ok(Math.abs(r.position.x - 2) < 1e-6);
  go(e, { robotID: id, destination: 'End' });                     // free for a new task
  run(e, 1);
  assert.equal(e.cancelTask({ taskID: 'task_2' }).data.status, 'Failed');   // running, not stopped
  assert.equal(e.cancelTask({ taskID: 'task_2' }).failureCode, 1001);       // already finished
  assert.equal(e.cancelTask({ taskID: 'task_9' }).failureCode, 1003);
});

test('stopping in a lift pauses the ride; resuming goes back to waiting for it', () => {
  const { e, id } = setup();
  go(e, { robotID: id, destination: 'Ward' });
  run(e, 11);                                         // at the lift after 10 s, 3 s ride
  assert.equal(robot(e, id).state, 'Waiting');
  assert.equal(e.stopRobot({ robotID: id }).data.state, 'Paused');
  run(e, 10);
  assert.deepEqual([robot(e, id).state, robot(e, id).level], ['Paused', 'L1'], 'the ride is paused too');
  assert.deepEqual([e.startRobot({ robotID: id }).data.state, task(e).status], ['Waiting', 'Waiting']);
  run(e, 2.2);
  assert.equal(robot(e, id).level, 'L2');
});

test('robot:stop pauses a robot:move; robot:start finishes it; robot:move stop ends it', () => {
  const { e, id } = setup();
  e.moveRobot({ robotID: id, command: 'forward', distance: 4 });
  run(e, 1);
  e.stopRobot({ robotID: id });
  run(e, 3);
  assert.ok(Math.abs(robot(e, id).position.x - 1) < 1e-6);
  assert.equal(robot(e, id).state, 'Paused');
  e.startRobot({ robotID: id });
  run(e, 3.1);
  assert.deepEqual([robot(e, id).state, Math.round(robot(e, id).position.x)], ['Idle', 4]);
  e.moveRobot({ robotID: id, command: 'backward', distance: 4 });
  run(e, 1);
  e.stopRobot({ robotID: id });
  const r = e.moveRobot({ robotID: id, command: 'stop' }).data;
  assert.deepEqual([r.state, r.stopped, r.activeMove], ['Idle', false, null]);
});

test('robot:stop needs something to stop; robot:start does not restart a robot that is not stopped', () => {
  const { e, id } = setup();
  assert.equal(e.stopRobot({ robotID: id }).failureCode, 1004);
  assert.equal(e.stopRobot({ robotID: 'robot_9' }).failureCode, 1002);
  go(e, { robotID: id, destination: 'End' });
  assert.equal(e.startRobot({ robotID: id }).failureCode, 1004);           // Moving, not stopped
  e.emergencyStop({ robotID: id });
  assert.equal(e.stopRobot({ robotID: id }).failureCode, 1004);            // emergency stop already ended the task
});
