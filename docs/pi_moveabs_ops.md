# Coordinated Motion Console Interface (Pi)

This document describes the UART console interface for coordinated moves and a safe test workflow. It does not prescribe any Raspberry Pi implementation details.

## Requirements
- Teensy firmware with `moveabs` and `coordstatus`.
- UART configured to 1,000,000 baud on `/dev/ttyAMA0`.

## Setup
1. Configure UART parameters:
   ```bash
   stty -F /dev/ttyAMA0 1000000 cs8 -cstopb -parenb -ixon -ixoff
   ```
2. Open a read console in one terminal:
   ```bash
   cat /dev/ttyAMA0
   ```
3. Send commands from another terminal with `echo -e "...\n" > /dev/ttyAMA0`.

## State Model (Console)
- `coordstatus` reports: `state` (`idle|queued|running`), `homed`, `queue`, `err`, and `axes`.
- Homing is required for X/Z/P targets (`homed=1`), and is invalidated if Z/X1/X2 disconnect or Z hits its limit outside the homing sequence.
- `stop` aborts an active coordinated move and clears the queue.

## Coordinated Move Command
- `moveabs [x <steps>] [z <steps>] [p <steps>] [r <steps>]`
  - Absolute positions are full steps, matching the `dist` field in telemetry.
  - Unspecified axes hold position.
  - Requests queue if the system is already moving (FIFO, max 8).

## Position Snapshot
- `pos` prints a single line with the current X/Z/P/R positions and a `homed` flag.

## Configuration Commands
- `maxvelocity [axis] [sps]` and `maxaccel [axis] [sps^2]`
  - `axis` may be `x`, `z`, `p`, `r`, or a physical axis name (e.g. `X1`).
  - `x`/`p` set virtual caps used by coordinated planning.

## Test Workflow
1. Confirm homing is complete:
   ```bash
   echo -e "coordstatus\n" > /dev/ttyAMA0
   ```
   Ensure `homed=1`. If `homed=0`, issue `home z` (or press the Z home button) and wait for `HOME -> done (soft limits enabled)` before re-checking.
2. Send a small coordinated move:
   ```bash
   echo -e "moveabs x 100 z -100 p 0 r 0\n" > /dev/ttyAMA0
   ```
   Watch for `COORD -> start` followed by `COORD -> done`.
3. Queue behavior:
   ```bash
   echo -e "moveabs x 200\n" > /dev/ttyAMA0
   echo -e "moveabs z -200\n" > /dev/ttyAMA0
   ```
   The second request should report `COORD -> queued`.
4. Stop behavior:
   ```bash
   echo -e "stop\n" > /dev/ttyAMA0
   ```
   Queue clears and any active move aborts.

## Safe Iteration Tips
- Keep the read console open at all times while testing.
- Start with small deltas before running long moves.
- Use `stop` immediately if motion looks wrong or telemetry drops.
- Re-run homing after any `COORD err=DISCONNECT` or `err=NOT_HOMED`.
