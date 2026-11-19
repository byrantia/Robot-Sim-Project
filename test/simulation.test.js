const test = require('node:test'), assert = require('node:assert/strict');
const { Engine } = require('../src/engine');
const { parseNavGraph } = require('../src/navgraph');
const { go } = require('./helpers');   // task:assign + robot:navigate
const { LIFT_SECONDS } = require('../src/simulation');

// L1: A(0,0) - B(4,0) - C(4,3) "Lab", D(9,9) on its own. Lifts X (at B) and Y (at A) both go to L2, where E(0,0) - F(0,2) "Ward".
const nav = () => parseNavGraph({
  graphs: {
    L1: { A: { pose: { x: 0, y: 0 }, branches: ['B'] }, B: { pose: { x: 4, y: 0 }, branches: ['C'] }, C: { pose: { x: 4, y: 3 }, locationName: 'Lab', branches: [] },
      D: { pose: { x: 9, y: 9 }, locationName: 'Island', branches: [] } },
    L2: { E: { pose: { x: 0, y: 0 }, branches: ['F'] }, F: { pose: { x: 0, y: 2 }, locationName: 'Ward', branches: [] } },
  },
  liftConfig: { lifts: [{ id: 'X', details: [{ mapName: 'L1', liftPoint: 'B' }, { mapName: 'L2', liftPoint: 'E' }] },
    { id: 'Y', details: [{ mapName: 'L1', liftPoint: 'A' }, { mapName: 'L2', liftPoint: 'E' }] }] },
});
const setup = () => {
  const e = new Engine({ now: () => 'T0' }); e.setNav(nav());
  const r = e.createRobot({ speed: 1, startPosition: { level: 'L1', x: 0, y: 0, orientation: 0 } }).data;
  return { e, id: r.robotID };
};
const run = (e, seconds) => { for (let i = 0; i < Math.round(seconds * 10); i++) e.tick(0.1); };
const robot = (e, id) => e.getRobot({ robotID: id }).data;
const near = (a, b) => assert.ok(Math.abs(a - b) < 1e-6, `${a} is not ${b}`);

test('robot drives its route at its speed, turns at corners, then completes', () => {
  const { e, id } = setup();
  const t = go(e, { robotID: id, destination: 'Lab' }).data;
  assert.deepEqual(robot(e, id).route.map((p) => p.nodeID), ['A', 'B', 'C']);
  assert.deepEqual([t.status, robot(e, id).state], ['Running', 'Moving']);
  run(e, 1);
  near(robot(e, id).position.x, 1); near(robot(e, id).position.y, 0);
  run(e, 3.2);                                    // reaches B at 4 s, then turns 90° at 180°/s
  near(robot(e, id).orientation, 36);
  run(e, 3.4);                                    // 4 + 0.5 + 3 = 7.5 s in all
  const r = robot(e, id);
  assert.deepEqual([r.state, r.activeTask, r.routeProgress], ['Completed', null, 1]);
  near(r.position.x, 4); near(r.position.y, 3); near(r.orientation, 90);
  assert.deepEqual([e.getTask({ taskID: t.taskID }).data.status, e.getTask({ taskID: t.taskID }).data.outcome], ['Completed', 'Reached Lab on L1.']);
  run(e, 2);
  assert.equal(robot(e, id).state, 'Idle');
});

test('a route to another level waits in the lift, then carries on there', () => {
  const { e, id } = setup();
  const t = go(e, { robotID: id, destination: 'Ward', lift: 'X' }).data;
  assert.deepEqual(t.steps.map((s) => s.type), ['drive', 'lift', 'drive']);
  assert.equal(t.steps[1].lift, 'X');
  run(e, 4.1);                                    // at the lift point B
  assert.deepEqual([robot(e, id).state, e.getTask({ taskID: t.taskID }).data.status], ['Waiting', 'Waiting']);
  run(e, LIFT_SECONDS);
  const r = robot(e, id);
  assert.deepEqual([r.level, r.position.z, r.state, e.getTask({ taskID: t.taskID }).data.status], ['L2', 1, 'Moving', 'Running']);
  run(e, 5);
  assert.equal(e.getTask({ taskID: t.taskID }).data.status, 'Completed');
  near(robot(e, id).position.y, 2);
});

test('without a lift choice the nearest lift is used; a chosen lift is honoured', () => {
  const a = setup();
  assert.equal(go(a.e, { robotID: a.id, destination: 'Ward' }).data.steps[1].lift, 'Y');
  const b = setup();
  assert.equal(go(b.e, { robotID: b.id, destination: 'Ward', lift: 'X' }).data.steps[1].lift, 'X');
});

test('no route: robot:navigate fails with 2003, the robot stays Idle and the job stays Queued', () => {
  const { e, id } = setup();
  const res = go(e, { robotID: id, destination: 'Island' });
  assert.equal(res.failureCode, 2003);
  const t = e.getTask({ taskID: 'task_1' }).data;
  assert.deepEqual([t.status, t.robotID], ['Queued', null]);
  assert.match(t.note, /robot_1 has no route/);
  assert.deepEqual([robot(e, id).state, robot(e, id).activeTask], ['Idle', null]);
});

test('emergency stop halts the robot where it is', () => {
  const { e, id } = setup();
  go(e, { robotID: id, destination: 'Lab' });
  run(e, 2);
  e.emergencyStop({ robotID: id });
  run(e, 3);
  const r = robot(e, id);
  assert.deepEqual([r.state, r.route.length], ['Error', 0]);
  near(r.position.x, 2);
});

test('pausing is just not ticking; reset puts robots back at their start', () => {
  const { e, id } = setup();
  go(e, { robotID: id, destination: 'Lab' });
  run(e, 2);
  e.reset();
  const r = robot(e, id);
  assert.deepEqual([r.state, r.position.x, r.activeTask], ['Idle', 0, null]);
  run(e, 2);
  assert.equal(robot(e, id).position.x, 0);
});

test('localize moves an idle robot (and its reset point), not one driving a task', () => {
  const { e, id } = setup();
  assert.equal(e.localize({ robotID: id, position: { x: 4, y: 0 } }).success, true);
  e.reset();
  assert.equal(robot(e, id).position.x, 4);
  go(e, { robotID: id, destination: 'Lab' });
  assert.equal(e.localize({ robotID: id, position: { x: 0, y: 0 } }).failureCode, 1004);
});
