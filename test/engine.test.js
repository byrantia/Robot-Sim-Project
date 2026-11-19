const test = require('node:test'), assert = require('node:assert/strict');
const { Engine } = require('../src/engine');
const { parseNavGraph } = require('../src/navgraph');
const { go } = require('./helpers');   // task:assign + robot:navigate

const nav = parseNavGraph({ graphs: { L1: { A: { pose: { x: 0, y: 0, z: 0 }, locationName: 'Pharmacy', branches: ['B'] }, B: { pose: { x: 1, y: 1, z: 0 }, locationName: '', branches: [] } } } });
const fresh = () => { const e = new Engine({ now: () => 'T0' }); e.setNav(nav); return e; };

test('task:assign creates a Queued job and nothing moves', () => {
  const e = fresh(), r = e.createRobot({ name: 'R' }).data, seen = [];
  e.on('task:update', (t) => seen.push(t.status));
  const res = e.assignTask({ destination: 'Pharmacy' });
  assert.equal(res.success, true);
  assert.deepEqual([res.data.taskID, res.data.status, res.data.robotID, res.data.createdAt, res.data.steps], ['task_1', 'Queued', null, 'T0', []]);
  assert.deepEqual(seen, ['Queued']);
  for (let i = 0; i < 20; i++) e.tick(0.1);
  const after = e.getRobot({ robotID: r.robotID }).data;
  assert.deepEqual([after.state, after.activeTask, after.route.length], ['Idle', null, 0]);
});
test('robot:navigate gives the job to the robot and starts it: task Running, robot Moving', () => {
  const e = fresh(), r = e.createRobot({ name: 'R' }).data;
  e.assignTask({ destination: 'B', level: 'L1' });
  const res = e.navigateRobot({ robotID: r.robotID, taskID: 'task_1' });
  assert.equal(res.success, true);
  assert.deepEqual([res.data.status, res.data.robotID, res.data.steps[0].path], ['Running', r.robotID, ['A', 'B']]);
  const after = e.getRobot({ robotID: r.robotID }).data;
  assert.deepEqual([after.state, after.activeTask, after.route.map((p) => p.nodeID)], ['Moving', 'task_1', ['A', 'B']]);
});
test('one active job per robot: navigating a second one is refused with 1004', () => {
  const e = fresh(), id = e.createRobot({}).data.robotID;
  go(e, { destination: 'Pharmacy', robotID: id });
  e.assignTask({ destination: 'Pharmacy' });
  assert.equal(e.navigateRobot({ robotID: id, taskID: 'task_2' }).failureCode, 1004);
  assert.equal(e.getTask({ taskID: 'task_2' }).data.status, 'Queued');      // still free for another robot
  const other = e.createRobot({}).data.robotID;
  assert.equal(e.navigateRobot({ robotID: other, taskID: 'task_2' }).data.status, 'Running');
});
test('a job runs once: it cannot be navigated again while running or after it ends', () => {
  const e = fresh(), a = e.createRobot({}).data.robotID, b = e.createRobot({}).data.robotID;
  go(e, { destination: 'B', level: 'L1', robotID: a });
  assert.equal(e.navigateRobot({ robotID: b, taskID: 'task_1' }).failureCode, 1001);
  e.cancelTask({ taskID: 'task_1' });
  assert.equal(e.navigateRobot({ robotID: b, taskID: 'task_1' }).failureCode, 1001);
});
test('task:assign with a robotID reserves the job for that robot', () => {
  const e = fresh(), a = e.createRobot({}).data.robotID, b = e.createRobot({}).data.robotID;
  assert.equal(e.assignTask({ destination: 'Pharmacy', robotID: a }).data.robotID, a);
  assert.equal(e.getRobot({ robotID: a }).data.state, 'Idle');
  assert.equal(e.navigateRobot({ robotID: b, taskID: 'task_1' }).failureCode, 1001);
  assert.equal(e.navigateRobot({ robotID: a, taskID: 'task_1' }).success, true);
});
test('a Queued job can be cancelled before any robot takes it', () => {
  const e = fresh();
  e.assignTask({ destination: 'Pharmacy' });
  assert.equal(e.cancelTask({ taskID: 'task_1' }).data.status, 'Failed');
});
test('bad requests get the right failure codes', () => {
  const e = fresh(), id = e.createRobot({}).data.robotID;
  assert.equal(e.assignTask({ robotID: 'nope', destination: 'Pharmacy' }).failureCode, 1002);
  assert.equal(e.assignTask({ destination: 'Nowhere' }).failureCode, 2002);
  assert.equal(e.assignTask({}).failureCode, 1001);
  assert.equal(e.getTask({ taskID: 'task_9' }).failureCode, 1003);
  assert.equal(e.navigateRobot({ robotID: id }).failureCode, 1001);
  assert.equal(e.navigateRobot({ robotID: id, taskID: 'task_9' }).failureCode, 1003);
  assert.equal(e.navigateRobot({ robotID: 'nope', taskID: 'task_9' }).failureCode, 1002);
  assert.equal(new Engine().assignTask({ robotID: 'x', destination: 'y' }).failureCode, 1002);
});
test('no nav graph: 2004', () => {
  const e = new Engine();
  assert.equal(e.assignTask({ destination: 'Pharmacy' }).failureCode, 2004);
});
test('emergency stop fails the task and sets Error; start clears it', () => {
  const e = fresh(), id = e.createRobot({}).data.robotID;
  go(e, { robotID: id, destination: 'Pharmacy' });
  e.emergencyStop({ robotID: id });
  const t = e.getTask({ taskID: 'task_1' }).data;
  assert.deepEqual([t.status, t.error.failureCode], ['Failed', 3005]);
  assert.equal(e.robotStatus({ robotID: id }).data.state, 'Error');
  assert.equal(e.navigateRobot({ robotID: id, taskID: e.assignTask({ destination: 'Pharmacy' }).data.taskID }).failureCode, 1004);
  e.startRobot({ robotID: id });
  assert.equal(e.robotStatus({ robotID: id }).data.state, 'Idle');
});
test('reset gives the same result again (repeatable)', () => {
  const e = fresh(), id = e.createRobot({}).data.robotID;
  go(e, { robotID: id, destination: 'Pharmacy' }); e.reset();
  assert.equal(go(e, { robotID: id, destination: 'Pharmacy' }).data.taskID, 'task_1');
});
test('a destination can also be a node id (picked on the map)', () => {
  const e = fresh();
  const t = e.assignTask({ destination: 'B', level: 'L1' });
  assert.deepEqual([t.success, t.data.node, t.data.level], [true, 'B', 'L1']);
});
test('z is the level index, not the node pose.z', () => {
  const e = new Engine();
  e.setNav(parseNavGraph({ graphs: { L1: { A: { pose: { x: 0, y: 0, z: -1.64 }, branches: [] } }, L2: { B: { pose: { x: 1, y: 1, z: 2.5 }, locationName: 'Lab', branches: [] } } } }));
  const r = e.createRobot({ startPosition: { level: 'L2', x: 1, y: 1 } }).data;
  assert.equal(r.position.z, 1);
  assert.equal(e.createRobot({}).data.position.z, 0);
  assert.equal(e.createRobot({ startPosition: { z: 1.1 } }).failureCode, 1001);
  assert.equal(e.assignTask({ destination: 'Lab' }).data.z, 1);
});
test('reordering levels updates z on robots, start points and tasks', () => {
  const e = new Engine();
  e.setNav(parseNavGraph({ graphs: { L1: { A: { pose: { x: 0, y: 0 }, branches: [] } }, L2: { B: { pose: { x: 1, y: 1 }, locationName: 'Lab', branches: [] } } } }));
  const r = e.createRobot({ startPosition: { level: 'L2' } }).data;
  go(e, { robotID: r.robotID, destination: 'Lab' });
  assert.equal(e.setLevelOrder({ levels: ['L2', 'L1'] }).success, true);
  assert.equal(e.getRobot({ robotID: r.robotID }).data.position.z, 0);
  assert.equal(e.getTask({ taskID: 'task_1' }).data.z, 0);
  e.emergencyStop({ robotID: r.robotID }); e.reset();
  assert.equal(e.getRobot({ robotID: r.robotID }).data.position.z, 0);
  assert.equal(e.setLevelOrder({ levels: ['L2', 'L2'] }).failureCode, 1001);
});
test('deleting a robot removes its tasks', () => {
  const e = fresh(), id = e.createRobot({}).data.robotID, removed = [];
  e.on('task:removed', (d) => removed.push(d.taskID));
  go(e, { robotID: id, destination: 'Pharmacy' });
  assert.equal(e.deleteRobot({ robotID: id }).failureCode, 1004);           // not while it is on the task
  e.emergencyStop({ robotID: id });
  assert.equal(e.deleteRobot({ robotID: id }).success, true);
  assert.deepEqual(removed, ['task_1']);
  assert.equal(e.getTask({ taskID: 'task_1' }).failureCode, 1003);
  assert.equal(e.taskList().length, 0);
});
test('task can name the lift to use between levels', () => {
  const e = new Engine();
  e.setNav(parseNavGraph({ graphs: {
    L1: { A: { pose: { x: 0, y: 0 }, branches: ['LA1', 'LB1'] }, LA1: { pose: { x: 1, y: 0 }, branches: ['A'] }, LB1: { pose: { x: -1, y: 0 }, branches: ['A'] } },
    L2: { LA2: { pose: { x: 1, y: 0 }, branches: ['Lab'] }, LB2: { pose: { x: -1, y: 0 }, branches: ['Lab'] }, Lab: { pose: { x: 0, y: 5 }, locationName: 'Lab', branches: ['LA2', 'LB2'] } } },
    liftConfig: { lifts: [{ id: 'A', details: [{ mapName: 'L1', liftPoint: 'LA1' }, { mapName: 'L2', liftPoint: 'LA2' }] },
      { id: 'B', details: [{ mapName: 'L1', liftPoint: 'LB1' }, { mapName: 'L2', liftPoint: 'LB2' }] }, { id: 'C', details: [{ mapName: 'L2', liftPoint: 'LB2' }] }] } }));
  assert.deepEqual(e.getMap().data.lifts.map((l) => l.id), ['A', 'B', 'C']);
  const id = e.createRobot({ startPosition: { level: 'L1' } }).data.robotID;
  assert.equal(e.assignTask({ destination: 'Lab', lift: 'X' }).failureCode, 1001);   // no such lift
  const c = e.assignTask({ destination: 'Lab', lift: 'C' }).data.taskID;           // goes to L2, but not from L1
  assert.equal(e.navigateRobot({ robotID: id, taskID: c }).failureCode, 1001);
  const b = e.assignTask({ destination: 'Lab', lift: 'B' }).data;
  assert.equal(b.lift, 'B');
  assert.equal(e.navigateRobot({ robotID: id, taskID: b.taskID }).data.steps[1].lift, 'B');
});
