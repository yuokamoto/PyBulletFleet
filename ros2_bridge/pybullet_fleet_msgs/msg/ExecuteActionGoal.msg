# Goal payload for the ExecuteAction interface.
#
# Shared by both the /{robot}/execute_action topic (fire-and-forget)
# and the ExecuteAction.action goal (blocking with feedback).
#
# action_type values:
#   "move"  — MoveAction   (path via waypoints)
#   "pick"  — PickAction   (attach nearest / named object)
#   "drop"  — DropAction   (detach to pose)
#   "wait"  — WaitAction   (pause for duration)
#   "joint" — JointAction  (set joint positions)
#
# action_params_json encodes parameters as a JSON object.
# See robot_handler.py _parse_action_goal() for the schema per type.
#
# Examples:
#
#   # Move to (3, 2)
#   action_type: "move"
#   action_params_json: '{"path": [[3.0, 2.0, 0.0]]}'
#
#   # Pick nearest object within 1m
#   action_type: "pick"
#   action_params_json: '{"search_radius": 1.0}'
#
#   # Drop at position
#   action_type: "drop"
#   action_params_json: '{"drop_pose": [5.0, 3.0, 0.1]}'
#
#   # Wait 3 seconds
#   action_type: "wait"
#   action_params_json: '{"duration": 3.0}'
#
#   # Set joints
#   action_type: "joint"
#   action_params_json: '{"target_joint_positions": [0.0, 1.57, 0.0, -1.57]}'

string action_type          # "move" | "pick" | "drop" | "wait" | "joint"
string action_params_json   # JSON-encoded parameters
