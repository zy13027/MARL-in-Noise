# Project review

Review of MARL-in-Noise, September 2026. Part 1 covers what was changed.
Part 2 covers what still needs doing in MATLAB and Simulink. Part 3 lists
optional tidying.

**Summary.** Before this review, the project could not run end to end.

* `GridWorld` crashed on its first step with one robot.
* Diagonal moves could leave the grid.
* The Simulink model had wiring faults: the Scout never got a reward, and
  the Guide's action reached the Hamming encoder as a number, not bits.

The MATLAB code is now fixed, deduplicated and tested. The Simulink model
and the live scripts are binary files that could not be opened here to
check any edits. Their fixes are listed in part 2, in priority order.

---

## 1. Changed in this review

### `src/environment/GridWorld.m`

Bugs fixed:

* **Crash on the first step with one robot.** The friend channel always
  read a first and second "other" robot, even when there were none.
* **Diagonal moves left the grid.** Actions 5 to 8 only checked the
  right-hand edge. A robot could step off the top or bottom, which
  crashed, and actions 6 and 8 could pass the left edge.
* **Output sizes were hard-coded** to the 12 × 12, three-robot example
  (`[12 12 4 3]` and `[3 1]`). The grid is 64 × 64 with one robot, so the
  sizes did not match the `[64 64 4]` observation spec.
* **The error message for an unknown state name** transposed the name
  (`name.'`), so building the message threw its own error.

Optimisations:

* Eight near-identical `case` branches, about 90 lines, became one move
  table plus one bounds check.
* The obstacle layout and free-cell count are now computed once, in
  `setupImpl`, not every step. The collision check reads the obstacle
  mask instead of joining the obstacle list every step.
* **Plotting is now off by default** (new `PlotEnvironment` parameter).
  The old code drew the grid and called `drawnow` on each of the first
  500 steps of every episode. That was the largest cost during training.
  With plotting on, the new code uses `drawnow limitrate`.
* New block parameters `GridSize` and `NumRobots`. Their defaults,
  64 × 64 and 1, match the old behaviour. The output sizes follow them.

Behaviour changes. Please check these suit the project:

1. Only the first `NumRobots` rows of `InitialStates` are used. Before,
   robots 2 and 3 were still placed on the grid and counted as explored,
   even though they never moved.
2. The full-coverage bonus is now split by each robot's share of explored
   cells, as the code comment ("coverage contribution") describes. Before,
   every robot got 4000 × the total. With one robot the result is the
   same.
3. An action outside 0 to 8 now costs −10, like a blocked move. Before,
   the robot stayed put with no penalty. This cannot happen with the
   1-to-8 action spec.
4. The colour limits are fixed at `[0 1]`, so cell colours stay correct
   whatever values are on the grid.
5. The header comment said the lazy (wait) penalty is −2. The code has
   always used −10, so the comment now says −10. The code is unchanged.

### `src/environment/CustomGridWorld.m`

Before: a full copy of `GridWorld` that could not run. It used
`NumExploredCells` without declaring it and declared `InitialStates` as
1 × 2 but read three rows. Its grid size was `[0 0]`, and it had the same
friend-channel crash.

Now: a subclass of `GridWorld` of about 50 lines. It adds
`TerminalStates`, the behaviour its header comment described. Reaching a
terminal cell gives +10 (+0.5 after the usual scaling) and ends the
episode. `TerminalStates` is tunable, so `ResetFcn` can move the goal each
episode. The block dialog shows the new parameter.

### Helper functions

| File | Before | Now |
| --- | --- | --- |
| `act1Conv.m` | 8-case `switch`; input outside 1–8 gave "output argument not assigned" | One arithmetic line; accepts vectors; clear error for bad input; works in MATLAB Function blocks |
| `deconvert.m` (was `.mlx`) | Entirely commented out | Inverse of `act1Conv`; ignores the padding bit, so an error in it cannot give a value outside 1–8; works in MATLAB Function blocks |
| `simouttodec.m` (was two `.mlx`) | Could not run: invalid signature `simouttodec(out.simout1)`, undefined `trnFlat`, `for i = lenData` looped once; converted bits through text | One reshape, one matrix product and `mode`; accepts every logging layout |
| `ResetFcn.m` (was `.mlx`) | Undefined `L`, typo `raanperm`, `s0` built transposed (`[rows'cols]'`), compared cells with text | Picks two different obstacle-free cells directly, with no retry loop; `L` is an argument; clear error when given text |
| `createStateNames.m` | Grew a string array inside two loops | Vectorised |
| `reference/GridWorldV0.m` | Class named `GridWorld`, so MATLAB could not load the file | Renamed; kept unmodified as the MathWorks reference for the tests |

### Project layout and housekeeping

* Files are sorted into `src/`, `models/`, `scripts/`, `data/`, `tests/`,
  `docs/`, `experiments/` and `archive/`, all moved with `git mv` so
  history follows them. `setupProject.m` puts the right folders on the
  path. It also sends Simulink's generated files to an ignored `work/`
  folder.
* Removed, but still in git history:
  * `*.slxc`: Simulink cache files, rebuilt automatically.
  * `Exported-2022-06-21*.vssettings`: Visual Studio settings exports,
    unrelated to the project, about 600 KB.
  * `gridMap.m`: empty, and it shadowed the `gridMap` variable name.
  * `sim_and_train_differences.m`: contained only `help sim; help train`.
  * The four `.mlx` function files listed above. They are now plain `.m`
    files, so changes show up in diffs.
* Added `.gitignore`, `.gitattributes` (stops git treating `.slx`,
  `.mlx` and `.mat` as text), `README.md` and `tests/testEnvironment.m`.

### How this was checked

MATLAB is not available where this review was done. The `.m` code was run
in GNU Octave 8.4 with a small stand-in for `matlab.System`.

* `tests/testEnvironment.m` passes, and it fails when a move is
  deliberately broken.
* The new `GridWorld` gave the same observations, rewards, done flags and
  positions as the unmodified MathWorks `GridWorldV0` over about 16,000
  random three-robot steps.

**Not verified:** Simulink code generation for `GridWorld`, the block
dialogs, `createStateNames` (Octave has no `string` type) and everything
inside the `.slx` and `.mlx` files. Please run `setupProject` and then
`runtests("tests")` in MATLAB.

---

## 2. Still to do in MATLAB and Simulink

### `models/ConsolidatedModelV0.slx`: these block training

1. **The Guide's action goes straight into the Hamming Encoder.** The
   encoder needs 4 bits per message, but the Guide outputs one number from
   1 to 8. Add a MATLAB Function block between the Guide and the
   subsystem's `In2`:
   ```matlab
   function bits = fcn(a)
   bits = act1Conv(a).';
   end
   ```
2. **The Scout never gets a reward.** The *Scout Agent2* block's reward
   and isdone inputs are not connected, so its reward is always 0 and it
   cannot learn. Wire `GridWorld`'s reward and isdone outputs to the Scout
   as well, so the agents share a reward.
3. **The subsystem's MATLAB Function block cannot compile.** It reads
   `out.simout2.Data` from a numeric signal, uses an undefined `trnFlat`,
   and calls text functions that MATLAB Function blocks do not support.
   Replace its body with:
   ```matlab
   function act2 = fcn(u)
   act2 = deconvert(u);
   end
   ```
   Do not call `simouttodec` from this block. The block's own function is
   named `simouttodec`, so the call would recurse.
4. **The channel selector switches with the data.** The Switch's control
   input (`u2 > 0`) is the decoded 8-PSK bits. The chosen channel changes
   bit by bit, and BPSK-AWGN bits get mixed with BSC bits. Use a Multiport
   Switch driven by a Constant `channelMode`, or a Variant Subsystem. A
   variant also stops Simulink from simulating all three channels every
   step.
5. **The 8-PSK path is miswired.**
   * The M-PSK Modulator (M = 8, *Integer* input) receives code bits
     0 and 1. It therefore only uses two neighbouring constellation
     points, 45° apart, which is far weaker than BPSK at the same SNR.
   * The demodulator can output 2 to 7, which the Hamming Decoder cannot
     take as bits.

   Set both M-PSK blocks to *Bit* input, or drop this path. With *Bit*
   input, the frame length must be a multiple of 3, so pad the 7-bit
   codeword to 9.
6. **Every episode starts in the same cell.** `GridWorld`'s *Initial
   states* is the literal `[2 2;11 4;3 12]`, and nothing uses the `s0`
   that `ResetFcn` sets. Set *Initial states* to `s0(2,:)`, the Scout's
   cell. For a goal-reaching task, switch the block to `CustomGridWorld`
   and set *Terminal states* to `s0(1,:)`.
7. **Episodes are capped at 100 steps.** The model's StopTime is 10 s.
   At Ts = 0.1 that is 100 steps, not the 1000 that `maxsteps` in
   `DraftV2` assumes. Set StopTime to `Tf`.
8. **`GridWorld` block settings.**
   * Set *Grid size* to `[L L]` so it follows the script.
   * *Simulate using* is *Code generation*. If you turn plotting on,
     switch it to *Interpreted execution*.
9. **Both AWGN blocks use seed 67.** Every run gets the same noise, and
   both channels get the same noise sequence. That is fine for debugging.
   For training, use different or random seeds.

### `scripts/DraftV2.mlx`

1. `rng(0)` runs after `initPos = randi(...)`, so the start positions
   cannot be reproduced. Move `rng(0)` to the top.
2. `obstacleLoca = [num2str(r) num2str(c)]` builds text such as `'3517'`.
   That could mean (3,51) or (35,17), and it cannot be compared with cell
   positions. Use `obstacleLoca = initPos(3,:);` and
   `env.ResetFcn = @(in) ResetFcn(in,obstacleLoca,L);`. Also,
   `randi([1,L],3,2)` can put the guide, scout and obstacle in the same
   cell.
3. The Scout agent (`agent2`), the training options and the `train` and
   `sim` sections are missing. They exist only in `archive/Draft.mlx`,
   where they do not run.
4. `inputdlg` for the agent type blocks unattended runs. Set
   `RLAgentType = "PPO";` at the top instead.
5. DQN case: `criticOptions` (learning rate 1e-4, L2 1e-4) is built but
   never used. The agent gets `DQNAgentOptOpts` (learning rate 1e-3).
   Pick one.
6. DDPG case: its action is continuous in [0, 9], but the channel needs a
   discrete message from 1 to 8. Drop this case.
7. RL (Q-table) case: `rlTable` needs a finite observation set, and a
   64 × 64 × 4 image is not one, so this case will error. Drop it.
8. ACAgent case: `lstmLayer` after `imageInputLayer` needs a
   `sequenceInputLayer` instead. `'UseDevice','gpu'` also needs a GPU.
9. **The PPO networks are larger than needed.** The two stride-1
   convolutions leave 59 × 59 × 8 = 27,848 features. The critic's first
   fully connected layer alone therefore has about 7.1 M weights (the
   actor's has about 1.8 M). Stride 2 in both convolutions gives
   15 × 15 × 8 = 1,800 features and about 0.46 M weights: roughly 15×
   fewer, and much faster to train.
10. `episodeMax = 5e5` has no stop criterion, so training will run for a
    very long time. Add `StopTrainingCriteria`, for example on average
    reward.
11. Smaller points:
    * `obs2Info = rlFiniteSetSpec([1 1]); obs2Info.Elements = ...` is
      simpler as `rlFiniteSetSpec(1:8)`.
    * `obs1Info.Description` says 12 × 12 × 4, but the image is
      64 × 64 × 4.

### Experiments and archive

* `archive/Draft.mlx`:
  * Many variables are undefined: `msg`, `data`, `decData`, `numObs`,
    `statePath`, `obsInfo2`, `nO`, `actor2`, `critic2` and others.
  * `ExperienceBufferLength` is `1e65`, probably meant to be 1e5 or 1e6.
  * It calls `resetFcn` but the file is `ResetFcn`; the case differs.
  * `clear all` is slow because it also clears compiled functions. Use
    `clear`.
  * It asks for the noise mode twice.
* `experiments/trial.mlx`: the last line decodes the noisy complex
  symbols `nData`. It should decode the demodulated bits `data`.
* `experiments/customTrainingLoop.mlx`: a copied example with undefined
  variables. It loads `CustomActor.mat`, which is not in the repo. Its
  discounted-return double loop is O(N²); one backward pass is O(N):
  ```matlab
  G = 0;
  for t = batchSize:-1:1
      G = rewardBatch(t) + discountFactor*G;
      discountedReturn(t) = G;
  end
  ```

---

## 3. Optional tidying (not done)

* `archive/AbstractGridWorld.m` is an unused, verbatim copy of MathWorks'
  internal `rl.env.GridWorld`, including its copyright notice. Deleting it
  also avoids publishing toolbox source code in a public repository.
* `archive/CustomGridWorldV0.m` is an unfinished stub with no reset and
  no outputs.
* `archive/expri0.slx` differs from `experiments/expri.slx` only in
  window-layout metadata.
* Git already keeps history, so one current copy of each file with clear
  commit messages is easier to follow than `V0`, `V1` and `V2` copies.
* `data/Multi Agent.mat` has a space in its name, so `load` needs quotes.
* `L = 64` is set in several places. Define it once.
* Next modularisation step: move the agent-building `switch` in
  `DraftV2.mlx` into `src/agents/createGuideAgent.m` and
  `createScoutAgent.m` functions. The main script then only sets
  parameters and calls functions.
