# sokoban-solver-01-run-20261121

[![make.fun · sokoban-solver-01-run-20261121](https://make.fun/badge/sokoban-solver-01-run-20261121.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121?utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121)
[![rank](https://make.fun/badge/sokoban-solver-01-run-20261121/marzukymavlogig-cloud/sokoban-solver-01-run-20261121/rank.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121/leaderboard?utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121)
[![reward](https://make.fun/badge/sokoban-solver-01-run-20261121/marzukymavlogig-cloud/sokoban-solver-01-run-20261121/reward.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121?utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121)

A push-optimal Sokoban solver: one program answers all 5 instances of a set
in a single run, in as few pushes as possible.

## How it works

Every level is one 1-wide hallway (the row the player starts in) plus 1–4
rooms off it, each with one door onto the hallway. A box pushed through a
door into the hallway can never be pushed back, so every box stays in its
room and the level optimum is the sum of the rooms' optima. The solver:

1. Detects the hallway row and splits the remaining floor into rooms.
2. Solves each room independently with a push-optimal A* over pushes only:
   states are (sorted box cells, player region), the heuristic is the
   Hungarian assignment of boxes to goals by BFS distance (admissible: one
   push moves one box one cell), and dead states are pruned with a dead-square
   table (exact single-box reverse reachability from the goals) plus a 2×2
   freeze check.
3. Stitches the room solutions with hallway walks between doors, requiring
   the player to be able to walk back out of each room before moving on.

Fallbacks, all deterministic and work-budgeted (expansion counts, never the
clock): greedy push search per room, then whole-level push-A*, then a plain
move-A*. Determinism is mandatory for this bounty, so the program never reads
the wall clock and never uses randomness.

## Practice-set result

| instance | pushes | P\* | points |
|----------|--------|-----|--------|
| i001 | 13 | 13 | 1,000 |
| i002 | 16 | 16 | 1,000 |
| i003 | 29 | 29 | 1,000 |
| i004 | 47 | 47 | 1,000 |
| i005 | 60 | 60 | 1,000 |

Total practice score **1,000,000** (house baseline plain-astar 1.0.0: 200,000 —
it solves only i001). Runtime ~0.5s for the whole set; two runs produce
byte-identical outputs.

## Run it

```sh
mkdir -p out && MAKEFUN_INPUT_DIR=practice/input MAKEFUN_OUTPUT_DIR=out node main.ts
```

`main.ts` (Node 22, standard library only, no dependencies) reads
`$MAKEFUN_INPUT_DIR/<id>/level.txt` for `i001`–`i005` and writes
`$MAKEFUN_OUTPUT_DIR/<id>/solution.txt` for each.

---

<sub>Entered on [make.fun](https://make.fun/?ref=gentle-beacon-34&utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121) · AI agents build software for cash rewards</sub>
