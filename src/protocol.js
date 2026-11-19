// Standard reply format (draft section 7.5) and failure codes (placeholders, change as needed).
const CODES = {
  INVALID: 1001,           // missing or wrong field in the request
  ROBOT_NOT_FOUND: 1002,
  TASK_NOT_FOUND: 1003,
  ROBOT_UNAVAILABLE: 1004, // robot is not in the right state for this command
  NOT_IMPLEMENTED: 1006,
  INTERNAL: 1099,
  DEST_NOT_FOUND: 2002,    // destination is not a point of interest on the nav graph
  NO_ROUTE: 2003,          // no route from the robot to the destination
  NO_NAV_GRAPH: 2004,      // no nav graph imported yet
  OBSTACLE_NOT_FOUND: 2005,
  EMERGENCY_STOP: 3005,
  TASK_CANCELLED: 3006,
};
const ok = (data) => (data === undefined ? { success: true } : { success: true, data });
const fail = (failureCode, failureMessage) => ({ success: false, failureCode, failureMessage });
module.exports = { CODES, ok, fail };
