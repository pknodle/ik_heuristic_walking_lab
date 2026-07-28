# IK & Heuristic Walking Lab

CS 123 lab code: inverse kinematics on one leg, then a heuristic (Raibert-style)
trotting gait on all four. Follow the lab spec on the course website — this repo is
where you write the code.

## Layout

| File | Purpose |
|------|---------|
| `part_1_ik.py` | Part 1: FK, gradient-descent IK, and triangle trajectory tracking on the front right leg |
| `part_1.yaml` / `part_1.launch.py` | controller stack for Part 1 (commands 3 joints) |
| `part_2_walking.py` | Parts 2–5: FK for all four legs, trot keyframes, and the gait loop |
| `part_2.yaml` / `part_2.launch.py` | controller stack for Part 2 (commands all 12 joints) |
| `extension/` | Part 6: live gait tuning with `pupper-gait-tuner` |

## Running

Each part needs two terminals. Put Pupper on its stand before running anything.

**Part 1**

```bash
cd ~/ik_heuristic_walking_lab
ros2 launch part_1.launch.py     # terminal 1
python3 part_1_ik.py             # terminal 2
```

**Part 2**

```bash
cd ~/ik_heuristic_walking_lab
ros2 launch part_2.launch.py     # terminal 1
python3 part_2_walking.py        # terminal 2
```

Part 2 solves IK for the whole gait cycle at startup, which takes a few seconds.
Once your gait works you can save that result and skip the solve:

```bash
python3 part_2_walking.py --save-cache   # solve, then write joint_positions_cache.npz
python3 part_2_walking.py --use-cache    # load it back and start immediately
```

Regenerate the cache whenever you change the keyframes — `--use-cache` will happily
replay a stale gait.

Only one process may drive the motors at a time. Stop `part_1_ik.py` before starting
`part_2_walking.py`, and stop both before running the gait tuner.

## TODOs

Part 1 has TODOs 1–6, Part 2 has TODOs 7–10, and the extension has TODOs 11–13. TODOs
marked `[already done in Part 1]` are places to bring forward code you already wrote.
