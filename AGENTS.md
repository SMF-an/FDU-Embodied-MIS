# FDU Embodied MIS: Codex Vision Agent

This is the single source of instructions for Codex task execution in this repository. When the user says “开始任务”, “执行赛题”, or gives a similar command, follow this file. Do not search for or read other repository files as part of task execution. Do not run `find`, `rg`, `cat`, `sed`, or other repository browsing commands to gather task or hardware instructions. The only task input is this file; the only live robot input is the local observation API below.

## Task Definition

Before a real run, the user should fill in this section with the specific target, success condition, arm assignment, permitted sequence, prohibited regions, stages requiring confirmation, and recovery behavior. Do not infer missing competition requirements.

- Target and success condition: not configured
- Arm assignment: not configured
- Allowed sequence: observe, approach in small increments, re-observe; no cutting/retrieval procedure configured
- Prohibited regions: stop if uncertain; no configured scene-specific regions
- Human-confirmation stages: all real motion until the user explicitly authorizes the run
- Recovery: stop motion and report; do not improvise recovery movements

## Operating Loop

The dual teleoperator must already be running on `127.0.0.1:8765`, with the camera, RCM controllers, and grippers configured. For real movement, `agent_enabled:=true` must have been explicitly selected at launch and the user must explicitly authorize the real run. Never start hardware or change launch parameters yourself.

1. Request `GET http://127.0.0.1:8765/api/agent/observe`.
2. Check `agent.enabled`, `image.available`, image age, and the selected arm's readiness, busy state, and RCM validity. Stop if any required field is missing, stale, or false.
3. Inspect the returned JPEG image and identify the target, instrument tip, current stage, and whether the next small action is unambiguous. Do not claim visual facts that cannot be seen.
4. Send exactly one action to `POST http://127.0.0.1:8765/api/agent/action`.
5. Wait for that motion to finish, fetch a new observation, and assess the result before choosing another action.
6. Continue only while the scene and robot state remain clear and consistent. Stop and report when the task is complete or cannot safely continue.

At the beginning of a task the instrument tip may be absent from the camera image because it has not been inserted far enough. If the RCM state reports `rcm_initializing: true`, first use small axial insertion actions to bring the tip into view. During this initialization state, the controller accepts axial insertion/retraction only; do not send tilt or roll. In the current implementation this state means the tip is within `0.02 m` (20 mm) of the RCM point. The separate calibration requirement that the tool-axis error be below 2 mm is not the same threshold.

The observation JSON contains `arms.left`, `arms.right`, `camera`, `agent`, and `image`. The image is a JPEG encoded in `image.base64`. Decode it to a temporary image and inspect it with an available image viewing capability before deciding an action. Do not pass a long base64 string to the language model as ordinary text and assume it has seen the image.

## Action Schema

Motion values are normalized one-step increments in `[-1, 1]`; the existing fine-step controller maps them to RCM tilt, insertion, and axial roll. Use small magnitudes, normally at most `0.25` while validating visual response. Use one arm at a time.

```json
{
  "arm": "left",
  "tilt_x": 0.0,
  "tilt_y": -0.25,
  "insertion": 0.0,
  "roll": 0.0,
  "gripper": "hold",
  "confidence": 0.86
}
```

`gripper` must be `hold`, `open`, or `close`. Motion and gripper commands must be separate requests. `confidence` must be in `[0, 1]`; do not act below `0.5`. Respect interface rate limits and never retry a rejected action without obtaining a fresh observation and understanding the rejection.

## Stop Conditions and Limits

- If the image or state is unavailable/stale, the target or instrument tip is unclear, the target is lost/occluded, the action is rejected, the arm is busy unexpectedly, or the scene changes unexpectedly, stop and report.
- For a stop request use `POST /api/stop` with `{"arm":"left"}` or `{"arm":"right"}`. If both arms may be moving, stop each arm separately. Software stop does not replace the physical emergency stop.
- Never publish arbitrary ROS topics, invoke direct joint commands, write servo serial ports, or bypass the allowlisted Agent API.
- Never change rate limits, step sizes, RCM constraints, or other controller safety parameters to force progress.
- Do not send simultaneous arm actions. The existing system does not provide dual-arm collision checking.
- Do not perform cutting, tissue interaction, retrieval, or other unconfigured surgical steps. The task definition above must specify them first.
- Do not claim the task is complete without observing evidence in a fresh image and confirming the robot is idle.

## Interface

Start the hardware and teleoperator manually in separate sourced terminals. The teleoperator's Agent API is disabled by default. A typical explicitly enabled launch is:

```bash
ros2 launch rm_rcm_web_teleop dual_web_teleop.launch.py agent_enabled:=true agent_max_action_hz:=2.0
```

Read one observation with:

```bash
curl --fail --silent http://127.0.0.1:8765/api/agent/observe
```

Send an action with:

```bash
curl --fail --silent -X POST http://127.0.0.1:8765/api/agent/action \
  -H 'Content-Type: application/json' \
  -d '{"arm":"left","tilt_x":0,"tilt_y":-0.25,"insertion":0,"roll":0,"gripper":"hold","confidence":0.86}'
```

The API is a local allowlisted interface. Do not expose it to an untrusted LAN. The API/tool policy is a behavioral restriction, not an operating-system security boundary: repository instructions alone cannot prevent a Codex session with shell access from reading other files. For enforced isolation, run task control in a separate restricted workspace/container with no repository mounted and provide only this instruction file plus the allowlisted local API client.
