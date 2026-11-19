const test = require('node:test'), assert = require('node:assert/strict');
const { Engine } = require('../src/engine');
const { parseNavGraph } = require('../src/navgraph');
const { TrafficManager, SAFE_DIST, corridorMap } = require('../src/traffic');
const { ROBOT_RADIUS, LIFT_SECONDS } = require('../src/simulation');

// A crossroads: W(-10,0) - X(0,0) - E(10,0) "East", and S(0,-10) - X - N(0,10) "North"; also a lift from W to L2.
const CROSS = { graphs: {
  L1: { W: { pose: { x: -10, y: 0 }, locationName: 'West', branches: ['X'] }, X: { pose: { x: 0, y: 0 }, branches: ['E', 'N', 'S', 'W'] },
    E: { pose: { x: 10, y: 0 }, locationName: 'East', branches: ['X'] }, S: { pose: { x: 0, y: -10 }, locationName: 'South', branches: ['X'] },
    N: { pose: { x: 0, y: 10 }, locationName: 'North', branches: ['X'] } },
  L2: { P: { pose: { x: 0, y: 0 }, branches: ['Q'] }, Q: { pose: { x: 0, y: 5 }, locationName: 'Ward', branches: ['P'] } } },
  liftConfig: { lifts: [{ id: 'K', details: [{ mapName: 'L1', liftPoint: 'W' }, { mapName: 'L2', liftPoint: 'P' }] }] } };
const setup = () => {
  const e = new Engine(); e.setNav(parseNavGraph(CROSS));
  const events = []; e.on('collision:event', (ev) => events.push(ev));
  return { e, events };
};
const robot = (e, id) => e.getRobot({ robotID: id }).data;
const place = (e, x, y, orientation = 0, level = 'L1', speed = 1) =>
  e.createRobot({ speed, startPosition: { level, x, y, orientation } }).data.robotID;
// a job for the robot, started (task:assign + robot:navigate)
const send = (e, id, destination) => e.navigateRobot({ robotID: id, taskID: e.assignTask({ destination }).data.taskID });
// runs the clock; returns the closest any two robots on the same level came (centre to centre) and every state seen
const run = (e, seconds) => {
  let closest = Infinity; const seen = {};
  for (let i = 0; i < Math.round(seconds * 10); i++) {
    e.tick(0.1);
    const rs = e.snapshot();
    for (const r of rs) (seen[r.robotID] ||= new Set()).add(r.state);
    for (let a = 0; a < rs.length; a++) for (let b = a + 1; b < rs.length; b++)
      if (rs[a].level === rs[b].level) closest = Math.min(closest, Math.hypot(rs[a].position.x - rs[b].position.x, rs[a].position.y - rs[b].position.y));
  }
  return { closest, seen };
};
const status = (e, taskID) => e.getTask({ taskID }).data.status;
const nodes = (e, id) => robot(e, id).route.map((p) => p.nodeID).join();
const turned = (heading, want) => Math.abs(((heading - want + 540) % 360) - 180) > 30;
const TOL = 0.06;   // reservations are checked every 5 cm along a path

test('plan(): the higher-priority robot reserves up to SAFE_DIST short of the other; the other, in its way head-on, waits', () => {
  const g = new TrafficManager().plan([
    { id: 'a', level: 'L', x: 0, y: 0, prio: 1, speed: 2, path: [{ x: 0, y: 0 }, { x: 10, y: 0 }], held: 0 },
    { id: 'b', level: 'L', x: 4, y: 0, prio: 2, speed: 1, path: [{ x: 4, y: 0 }, { x: -6, y: 0 }], held: 0 },
    { id: 'idle', level: 'L', x: 0, y: 5, prio: Infinity, speed: 1, path: null, held: 0 },
  ]);
  assert.ok(Math.abs(g.get('a').allow - (4 - SAFE_DIST)) < 1e-9, `a may drive ${g.get('a').allow}`);
  assert.deepEqual(g.get('a').waitFor, ['b']);
  assert.deepEqual([g.get('b').allow, g.get('b').waitFor], [0, ['a']]);
  assert.equal(g.has('idle'), false);   // not driving a route: only a footprint to keep clear of
});

test('crossroads: the job started first goes first; the other waits short of the crossing, then carries on', () => {
  const { e, events } = setup();
  const a = place(e, -10, 0, 0), b = place(e, 0, -10, 90);
  send(e, a, 'East'); send(e, b, 'North');               // both 10 m from the crossing at 1 m/s: they would meet there
  const { closest, seen } = run(e, 30);
  assert.ok(closest >= SAFE_DIST - TOL, `robots came within ${closest.toFixed(3)} m`);
  assert.deepEqual([status(e, 'task_1'), status(e, 'task_2')], ['Completed', 'Completed']);
  assert.equal(seen[a].has('Waiting'), false, `${a} has priority and never waits`);
  assert.equal(seen[b].has('Waiting'), true);
  const mine = events.filter((ev) => ev.robotIDs[0] === b).map((ev) => [ev.type, ev.response, ev.robotIDs[1]]);
  assert.deepEqual(mine, [['robot', 'wait', a], ['robot', 'resume', a]]);
  const waitAt = events.find((ev) => ev.response === 'wait').position;
  assert.ok(waitAt.y < -SAFE_DIST + TOL, `it waits short of the crossing (y ${waitAt.y.toFixed(2)})`);
});

test('crossroads the other way round: start the other job first and that robot goes first', () => {
  const { e } = setup();
  const a = place(e, -10, 0, 0), b = place(e, 0, -10, 90);
  send(e, b, 'North'); send(e, a, 'East');
  const { closest, seen } = run(e, 30);
  assert.ok(closest >= SAFE_DIST - TOL);
  assert.deepEqual([seen[a].has('Waiting'), seen[b].has('Waiting')], [true, false]);
});

test('following: a faster robot behind a slower one keeps SAFE_DIST and never passes through it', () => {
  const { e } = setup();
  const slow = place(e, -4, 0, 0, 'L1', 0.4), fast = place(e, -10, 0, 0, 'L1', 1);   // 6 m ahead on the same way
  send(e, fast, 'X'); send(e, slow, 'East');                  // the one behind even has priority: it still follows
  const { closest } = run(e, 60);                             // (and stops at the crossroads, so it is not after East,
                                                              //  where the slow one ends up parked)
  assert.ok(closest >= SAFE_DIST - TOL, `robots came within ${closest.toFixed(3)} m`);
  assert.deepEqual([status(e, 'task_1'), status(e, 'task_2')], ['Completed', 'Completed']);
});

test('head-on: the lower-priority robot gives way by going round the other on open floor, and both arrive', () => {
  const { e, events } = setup();
  const a = place(e, -10, 0, 0), b = place(e, 10, 0, 180);
  send(e, a, 'East'); send(e, b, 'West');
  const { closest, seen } = run(e, 60);
  assert.ok(closest >= SAFE_DIST - TOL, `robots came within ${closest.toFixed(3)} m`);
  assert.deepEqual([status(e, 'task_1'), status(e, 'task_2')], ['Completed', 'Completed']);
  assert.ok(seen[b].has('Waiting'));
  const replan = events.filter((ev) => ev.response === 'replan');
  assert.equal(replan.length, 1, 'only one of them gives way');
  assert.deepEqual([replan[0].robotIDs[0], replan[0].type], [b, 'robot']);
  assert.match(replan[0].message, new RegExp(`Giving way: going round ${a}`));
});

test('single-lane swap (no floor to go round on): one robot makes way in a free side arm, then both arrive', () => {
  const { e, events } = setup();
  e.nav.occupancy.L1 = { free: () => false };                // walls everywhere off the nav graph: every arm is one lane
  const b = place(e, 0, -10, 90), c = place(e, 10, 0, 180);
  send(e, b, 'East'); send(e, c, 'South');                   // each needs the arm the other is in
  const { closest } = run(e, 90);
  assert.ok(closest >= SAFE_DIST - TOL, `robots came within ${closest.toFixed(3)} m`);
  assert.deepEqual([status(e, 'task_1'), status(e, 'task_2')], ['Completed', 'Completed']);
  const made = events.find((ev) => /making way at/.test(ev.message));
  assert.ok(made, events.map((ev) => ev.message).join(' | '));
  assert.ok(events.some((ev) => ev.robotIDs[0] === made.robotIDs[0] && /has passed\. On the way to/.test(ev.message)));
});

test('three robots through a single-lane crossroads all arrive without coming within SAFE_DIST', () => {
  const { e } = setup();
  e.nav.occupancy.L1 = { free: () => false };
  const a = place(e, -10, 0, 0), b = place(e, 0, -10, 90), c = place(e, 10, 0, 180);
  send(e, a, 'North'); send(e, b, 'East'); send(e, c, 'South');
  const { closest } = run(e, 120);
  assert.ok(closest >= SAFE_DIST - TOL, `robots came within ${closest.toFixed(3)} m`);
  assert.deepEqual(['task_1', 'task_2', 'task_3'].map((id) => status(e, id)), ['Completed', 'Completed', 'Completed']);
});

// A 1.4 m wide corridor (too narrow to pass) between two junctions: J1(-7,0) - C1(-3,0) - C2(3,0) - J2(7,0), open
// floor beyond |x| = 6. W(-10,0) and NW(-9,4) off J1; NE(9,4) and SE(9,-4) off J2.
const CORRIDOR = { graphs: { L1: {
  W: { pose: { x: -10, y: 0 }, locationName: 'W', branches: ['J1'] }, NW: { pose: { x: -9, y: 4 }, locationName: 'NW', branches: ['J1'] },
  J1: { pose: { x: -7, y: 0 }, branches: ['W', 'NW', 'C1'] }, C1: { pose: { x: -3, y: 0 }, branches: ['J1', 'C2'] },
  C2: { pose: { x: 3, y: 0 }, branches: ['C1', 'J2'] }, J2: { pose: { x: 7, y: 0 }, branches: ['C2', 'NE', 'SE'] },
  NE: { pose: { x: 9, y: 4 }, locationName: 'NE', branches: ['J2'] }, SE: { pose: { x: 9, y: -4 }, locationName: 'SE', branches: ['J2'] } } } };
const corridorFloor = { free: (x, y, rad) => Math.abs(x) >= 6 || Math.abs(y) + rad <= 0.7 };

test('corridorMap: links too narrow to pass, joined through two-link nodes, make one corridor with one direction', () => {
  const nav = parseNavGraph(CORRIDOR); nav.occupancy.L1 = corridorFloor;
  const cm = corridorMap(nav, 'L1'), c = cm.get('J1|C1');
  assert.ok(c, 'J1-C1 is narrow');
  for (const k of ['C1|C2', 'C2|J2']) assert.deepEqual(cm.get(k), c);
  assert.deepEqual(cm.get('C1|J1'), { chain: c.chain, dir: -c.dir });
  assert.equal(cm.has('W|J1'), false);
  assert.equal(corridorMap(parseNavGraph(CORRIDOR), 'L1').size, 0);   // no floor plan: open floor, no corridors
});

test('narrow corridor: a robot coming the other way waits outside it until the one in it is out (no face-off inside)', () => {
  const e = new Engine(), nav = parseNavGraph(CORRIDOR); nav.occupancy.L1 = corridorFloor; e.setNav(nav);
  const events = []; e.on('collision:event', (ev) => events.push(ev));
  const a = place(e, -10, 0, 0), b = place(e, 9, -4, 120);
  send(e, a, 'NE'); send(e, b, 'NW');
  let closest = Infinity, bothIn = false;
  for (let i = 0; i < 600; i++) {
    e.tick(0.1);
    const [p, q] = [robot(e, a).position, robot(e, b).position];
    closest = Math.min(closest, Math.hypot(p.x - q.x, p.y - q.y));
    if (Math.abs(p.x) < 6 && Math.abs(q.x) < 6) bothIn = true;
  }
  assert.equal(bothIn, false, 'never both in the corridor');
  assert.ok(closest >= SAFE_DIST - TOL, `robots came within ${closest.toFixed(3)} m`);
  assert.deepEqual([status(e, 'task_1'), status(e, 'task_2')], ['Completed', 'Completed']);
  const wait = events.find((ev) => ev.robotIDs[0] === b && ev.response === 'wait');
  assert.ok(wait && wait.position.x >= 7, `it waits outside the corridor (at x ${wait && wait.position.x.toFixed(2)})`);
});

test('a robot parked on the route (no task) is waited for, then Stuck, never gone round; it goes on once the way clears', () => {
  const { e, events } = setup();                             // open floor: there would be room to go round
  const a = place(e, -10, 0, 0), parked = place(e, 5, 0);
  send(e, a, 'East');
  run(e, 23);                                                // waits from 14 s, 1 m short of it
  assert.deepEqual([robot(e, a).state, robot(e, a).blockedBy, status(e, 'task_1')], ['Waiting', [parked], 'Waiting']);
  run(e, 2);                                                 // STUCK_SECONDS after it stopped
  assert.deepEqual([robot(e, a).state, robot(e, a).blockedBy, status(e, 'task_1')], ['Stuck', [parked], 'Waiting']);
  assert.match(e.getTask({ taskID: 'task_1' }).data.note, /Stuck: robot_2 still in the way after 10 s\. Waiting here until the way is clear\./);
  run(e, 30);
  assert.ok(Math.abs(robot(e, a).position.x - (5 - SAFE_DIST)) < 1e-6 && robot(e, a).position.y === 0, 'still waiting in the same place');
  e.localize({ robotID: parked, position: { x: 0, y: 8 } });  // moved out of the way
  run(e, 15);
  assert.equal(status(e, 'task_1'), 'Completed');
  assert.deepEqual(events.map((ev) => ev.response), ['wait', 'stuck', 'resume']);
});

test('Stuck behind an idle robot that then gets a job and drives off: no turning round, it carries on along its route', () => {
  const { e, events } = setup();
  const a = place(e, 0, 10, -90), idle = place(e, 0, 0);    // idle robot parked on the crossroads
  send(e, a, 'South');
  const route = nodes(e, a);
  run(e, 25);
  assert.deepEqual([robot(e, a).state, robot(e, a).blockedBy], ['Stuck', [idle]]);
  send(e, idle, 'East');                                     // the idle robot gets a new job, away to the east
  let turn = false, closest = Infinity;
  for (let i = 0; i < 300; i++) {
    e.tick(0.1);
    turn ||= turned(robot(e, a).orientation, -90);
    const p = robot(e, a).position, q = robot(e, idle).position;
    closest = Math.min(closest, Math.hypot(p.x - q.x, p.y - q.y));
  }
  assert.equal(turn, false, 'it never turned round');
  assert.equal(nodes(e, a), route, 'same route');
  assert.ok(closest >= SAFE_DIST - TOL);
  assert.deepEqual([status(e, 'task_1'), status(e, 'task_2')], ['Completed', 'Completed']);
  assert.equal(events.some((ev) => ev.robotIDs[0] === a && ev.response === 'replan'), false);
});

test('a paused robot gives up its reservation; others keep clear of where it stands', () => {
  const { e } = setup();
  const a = place(e, -10, 0, 0), b = place(e, 0, -10, 90);
  send(e, a, 'East'); send(e, b, 'North');
  run(e, 9.5);                                               // a is about to cross
  e.stopRobot({ robotID: a });
  const { closest } = run(e, 30);
  assert.ok(closest >= SAFE_DIST - TOL, `came within ${closest.toFixed(3)} m`);
  assert.equal(robot(e, a).state, 'Paused');
  e.startRobot({ robotID: a });
  run(e, 30);
  assert.deepEqual([status(e, 'task_1'), status(e, 'task_2')], ['Completed', 'Completed']);
});

test('a robot paused in the way is just waited for: no going round, no turning back, no Stuck', () => {
  const { e, events } = setup();
  const a = place(e, -10, 0, 0), b = place(e, 0, -10, 90);
  send(e, a, 'East'); send(e, b, 'North');
  run(e, 10);                                                // a is in the middle of the crossing
  e.stopRobot({ robotID: a });
  const route = robot(e, b).route.map((p) => p.nodeID).join();
  const { closest } = run(e, 60);
  assert.ok(closest >= SAFE_DIST - TOL, `came within ${closest.toFixed(3)} m`);
  assert.deepEqual([robot(e, a).state, robot(e, b).state, robot(e, b).blockedBy], ['Paused', 'Waiting', [a]]);
  assert.equal(robot(e, b).route.map((p) => p.nodeID).join(), route, 'same route: it did not reroute or turn back');
  const mine = events.filter((ev) => ev.robotIDs[0] === b).map((ev) => ev.response);
  assert.equal(mine.some((x) => ['replan', 'stuck'].includes(x)), false, mine.join());
  assert.match(e.getTask({ taskID: 'task_2' }).data.note, new RegExp(`${a} is paused`));
  e.startRobot({ robotID: a });
  run(e, 30);
  assert.deepEqual([status(e, 'task_1'), status(e, 'task_2')], ['Completed', 'Completed']);
});

// A single corridor with walls: West(-10,0) - B(0,0) - East(10,0). Two robots head-on there cannot give way.
const LINE = { graphs: { L1: { A: { pose: { x: -10, y: 0 }, locationName: 'West', branches: ['B'] },
  B: { pose: { x: 0, y: 0 }, branches: ['A', 'C'] }, C: { pose: { x: 10, y: 0 }, locationName: 'East', branches: ['B'] } } } };

test('both Stuck head-on with no way to give way: drag one out of the way by hand; the other resumes, both arrive', () => {
  const e = new Engine(), nav = parseNavGraph(LINE); nav.occupancy.L1 = { free: () => false }; e.setNav(nav);
  const events = []; e.on('collision:event', (ev) => events.push(ev));
  const a = place(e, -10, 0, 0), b = place(e, 10, 0, 180);
  send(e, a, 'East'); send(e, b, 'West');
  run(e, 40);
  assert.deepEqual([robot(e, a).state, robot(e, b).state], ['Stuck', 'Stuck']);
  assert.match(e.getTask({ taskID: 'task_2' }).data.note, /no way to give way/);
  const moved = e.localize({ robotID: b, position: { x: -11, y: 2 } });   // carried past a, to near its destination
  assert.equal(moved.success, true);
  assert.deepEqual([moved.data.state, moved.data.activeTask, moved.data.route[0].nodeID], ['Moving', 'task_2', 'A']);   // its way on from there
  assert.match(e.getTask({ taskID: 'task_2' }).data.note, /Moved by hand to \(-11\.00, 2\.00\)\. Route planned again from there\./);
  const { closest } = run(e, 60);
  assert.ok(closest >= SAFE_DIST - TOL, `came within ${closest.toFixed(3)} m`);
  assert.deepEqual([status(e, 'task_1'), status(e, 'task_2')], ['Completed', 'Completed']);
  assert.ok(events.some((ev) => ev.robotIDs[0] === a && ev.response === 'resume'), 'a carried on by itself');
  e.reset();
  assert.deepEqual([robot(e, b).position.x, robot(e, b).position.y], [10, 0], 'sim:reset still puts it back at its start');
});

test('once both are Stuck nobody gives way, even when room frees up: they stay Stuck until one is moved by hand', () => {
  const { e, events } = setup();
  e.nav.occupancy.L1 = { free: () => false };                // single-lane arms
  const b = place(e, 0, -10, 90), c = place(e, 10, 0, 180);
  const w = place(e, -10, 0), n = place(e, 0, 10);           // idle robots parked in the free arms: nowhere to make way
  send(e, b, 'East'); send(e, c, 'South');                   // each needs the arm the other is in
  run(e, 40);
  assert.deepEqual([robot(e, b).state, robot(e, c).state], ['Stuck', 'Stuck']);
  const at = [robot(e, b).position, robot(e, c).position].map((p) => `${p.x.toFixed(3)},${p.y.toFixed(3)}`);
  e.localize({ robotID: w, position: { x: -30, y: -30 } }); e.localize({ robotID: n, position: { x: 30, y: 30 } });   // room again
  run(e, 60);
  assert.deepEqual([robot(e, b).state, robot(e, c).state], ['Stuck', 'Stuck'], 'still Stuck: they do not give way now');
  assert.deepEqual([robot(e, b).position, robot(e, c).position].map((p) => `${p.x.toFixed(3)},${p.y.toFixed(3)}`), at, 'and have not moved');
  assert.equal(events.some((ev) => ev.response === 'replan'), false, 'nobody turned round or went round');
  assert.equal(e.localize({ robotID: c, position: { x: 0, y: 9 } }).success, true);   // you help: carry c to the north arm
  const { closest } = run(e, 60);
  assert.ok(closest >= SAFE_DIST - TOL, `came within ${closest.toFixed(3)} m`);
  assert.deepEqual([status(e, 'task_1'), status(e, 'task_2')], ['Completed', 'Completed']);
});

test('moving a robot on a task by hand: only when it is not driving; a paused one stays paused', () => {
  const { e } = setup();
  const a = place(e, -10, 0, 0);
  send(e, a, 'East');
  run(e, 1);
  assert.equal(e.localize({ robotID: a, position: { x: 0, y: 5 } }).failureCode, 1004);   // driving: pause it first
  e.stopRobot({ robotID: a });
  const r = e.localize({ robotID: a, position: { x: 0, y: 5 } }).data;
  assert.deepEqual([r.state, r.stopped, r.position.x, r.position.y], ['Paused', true, 0, 5]);
  run(e, 3);
  assert.deepEqual([robot(e, a).position.x, robot(e, a).position.y], [0, 5], 'still paused where it was put');
  e.startRobot({ robotID: a });
  run(e, 25);
  assert.equal(status(e, 'task_1'), 'Completed');
});

test('one active job per robot still holds with traffic', () => {
  const { e } = setup();
  const a = place(e, -10, 0, 0);
  send(e, a, 'East');
  assert.equal(send(e, a, 'North').failureCode, 1004);
});

test('a robot coming out of a lift waits in it while another robot stands at the lift door', () => {
  const { e } = setup();
  const a = place(e, -10, 0, 0), door = place(e, 0, 0.5, 0, 'L2');   // standing at P on L2
  send(e, a, 'Ward');
  run(e, LIFT_SECONDS + 2);
  assert.deepEqual([robot(e, a).level, robot(e, a).state], ['L1', 'Waiting']);
  e.localize({ robotID: door, position: { x: 4, y: 0 } });
  const { closest } = run(e, 10);
  assert.ok(closest >= SAFE_DIST - TOL);
  assert.equal(status(e, 'task_1'), 'Completed');
});

test('robot:move stops SAFE_DIST short of another robot', () => {
  const { e, events } = setup();
  const a = place(e, -10, 0, 0); place(e, -7, 0);
  e.moveRobot({ robotID: a, command: 'forward', distance: 5 });
  run(e, 6);
  assert.ok(Math.abs(robot(e, a).position.x - (-7 - SAFE_DIST)) < 1e-6, `x ${robot(e, a).position.x}`);
  assert.deepEqual([events[0].type, events[0].robotIDs], ['robot', [a, 'robot_2']]);
  assert.ok(SAFE_DIST > 2 * ROBOT_RADIUS);
});
