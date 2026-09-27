# MARL-in-Noise

**Multi-agent reinforcement learning with communication over noisy
channels.** A *Guide* and a *Scout* agent learn to cooperate in a grid
world, but every message the Guide sends passes through a noisy channel
first. Built in MATLAB and Simulink as an ongoing university project.

## About

Cooperating agents usually assume their messages arrive intact. This
project asks what happens to learned cooperation when they do not.

* The **Guide** sees the whole map as a 64 × 64 × 4 image but cannot move.
  The four layers are obstacles, the Scout's cell, any other robots and
  unexplored cells. Each step the Guide chooses one of 8 messages.
* The message is turned into 4 bits, protected with a **Hamming (7,4)**
  code and sent through one of three channels:
  **binary symmetric (BSC)**, **AWGN with BPSK** or **AWGN with 8-PSK**.
* The **Scout** only receives the decoded, possibly corrupted, message.
  From it, the Scout chooses one of 8 moves (4 straight, 4 diagonal).
* Both agents share the reward: new cells explored, penalties for bumping
  into walls or obstacles, and optionally a bonus for reaching a goal cell.

```
     GridWorld ─── observation 64×64×4 ───▶ Guide
         ▲                                    │ message 1–8
         │ move 1–8                           ▼
       Scout                              act1Conv
         ▲                                    │ 4 bits
         │ observation 1–8                    ▼
     deconvert                         Hamming encoder
         ▲                                    │ 7 bits
         │ 4 bits                             ▼
  Hamming decoder ◀─────────────────────── channel
                                BSC, AWGN + BPSK or AWGN + 8-PSK
```

## Project structure

```
MARL-in-Noise/
├── setupProject.m            Put the project on the MATLAB path (run first)
├── src/
│   ├── environment/          Grid-world environments
│   │   ├── GridWorld.m           Area-coverage grid world, 1–3 robots
│   │   ├── CustomGridWorld.m     GridWorld plus goal (terminal) cells
│   │   ├── createStateNames.m    "[row,col]" state names
│   │   └── reference/GridWorldV0.m   Original MathWorks example, for tests
│   ├── communication/        Message coding between the agents
│   │   ├── act1Conv.m            Guide action 1–8 → 4-bit message
│   │   ├── deconvert.m           4-bit message → Scout observation 1–8
│   │   └── simouttodec.m         Most frequent message in logged output
│   └── training/
│       └── ResetFcn.m            Random start cells for each episode
├── models/
│   ├── ConsolidatedModelV0.slx   Main model: Guide, channels, Scout, GridWorld
│   └── channels/                 Channel prototypes (BPSK, 8-PSK, combined)
├── scripts/
│   └── DraftV2.mlx               Builds spaces, environment and Guide agent
├── data/                     Saved sessions (.mat)
├── tests/testEnvironment.m   Unit tests for src/
├── docs/
│   ├── REVIEW.md             Code review: what was fixed, what is left
│   └── project-overview.html Standalone overview page
├── experiments/              Scratch work: channel trials, custom training loop
└── archive/                  Superseded drafts, kept for reference
```

`src`, `models`, `scripts` and `data` go on the path. `experiments` and
`archive` stay off it.

## Requirements

MATLAB R2022a or later, with Simulink, Reinforcement Learning Toolbox,
Deep Learning Toolbox and Communications Toolbox.

## Getting started

```matlab
cd MARL-in-Noise
setupProject                 % add folders to the path
results = runtests("tests")  % check the environment and helpers
```

Then open `scripts/DraftV2.mlx` and run it. It opens
`models/ConsolidatedModelV0.slx` and builds the Guide agent.

## The environment

`GridWorld` is a MATLAB System object used in the model's MATLAB System
block.

| Block parameter | Default | Meaning |
| --- | --- | --- |
| Initial states | `[2 2; 11 4; 3 12]` | Start cell `[row col]` of each robot; the first *Num robots* rows are used |
| Obstacles | `-1` | Obstacle cells, one `[row col]` per row, `-1` for none |
| Max step count | `500` | Last step that is plotted in each episode |
| Grid size | `[64 64]` | Rows and columns |
| Num robots | `1` | Robots moving in the grid (1–3) |
| Plot environment | off | Draw the grid each step. It is slow, so leave it off for training. It also needs *Simulate using* set to *Interpreted execution* |

| Action | 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| Move | wait | up | down | left | right | up-right | up-left | down-right | down-left |

| Event | Reward (before ÷ 20) |
| --- | --- |
| Enter an unexplored cell | +20 |
| Any successful move | −1 |
| Wait, or blocked by the edge, an obstacle or a robot | −10 |
| Whole grid explored | +4000 × the robot's share of explored cells, and the episode ends |
| Reach a terminal cell (`CustomGridWorld` only) | +10, and the episode ends |

Messages use the value `action − 1` in 4 bits, most significant bit first
(`1 → 0000`, `8 → 0111`). The leading bit is padding for the Hamming
(7,4) code, and `deconvert` ignores it.

## Status

The environment classes and helper functions have been reviewed, fixed
and unit tested. The Simulink model and the live scripts still need a
few changes in MATLAB before end-to-end training works. For example, the
Scout's reward input is not connected. The list, in priority order, is in
[docs/REVIEW.md](docs/REVIEW.md).

## Credits

`GridWorld` and `ResetFcn` are adapted from the MathWorks multi-agent area
coverage example (Copyright The MathWorks, Inc.).
