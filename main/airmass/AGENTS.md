# AirMass Rules

## Start Every Task

- Run `git status --short` before editing. Treat existing modified or untracked files as user work unless you know you created them.
- When starting a new feature, if the current worktree has other work in progress (uncommitted changes or an unmerged feature branch), create a new worktree for it and work there instead of mixing it into the existing one. In Claude Code, use the `EnterWorktree` tool: it creates the worktree under `.claude/worktrees/` and switches the session into it; `ExitWorktree` switches back. Elsewhere, use `git worktree add .claude/worktrees/<feature> -b <feature> <base branch>`.
- Identify the affected surfaces before changing code: the device code here, the display and setup menus that call into it, the host simulator and replay, the browser replay, the analysis scripts, CI, and the READMEs.
- Read nearby code and tests first. Reuse existing project patterns, helpers, data models, and menu structure before adding new abstractions.
- For cross-surface behavior, update the contract and all consumers together. The same estimator sources are compiled into the firmware, the host tools, and the WebAssembly module, so do not leave them inconsistent.
- Before finishing, inspect the actual diff and run the checklist below against the diff, not from memory.

## Core Rules

Treat these as active checks to run, not background facts to recall.

### 1. Keep firmware and simulators synchronized

Every change to the firmware's AirMass behavior must land in the host simulator, the host replay, and the browser replay in the same change, and every simulator change that models device behavior must match the firmware. The estimator sources are shared, but the code around them is not: input conversion, sampling, settings and defaults, initialization, and output handling exist separately on the device (`main/airmass/`), in `tools/airmass_sim/` (`sim.cpp`, `replay_core.h`, `replay_settings.h`, `sens2rec.py`), and in `tools/airmass_sim/web/` (`airmass_wasm.cpp`, `convert.js`).

- Active check: for each firmware file in the diff, name its counterpart in the simulator, the replay, and the browser replay, and confirm it was updated or needs no change. Rebuild `airmass.js` and run the host and browser replay tests.

### 2. Check every EKF tuning change on synthetic and real flights

Any change to EKF tuning (an `AirMassEKF::Config` default, a noise or gate value, a time constant, a blend, or logic that changes how the estimator weighs its inputs) must go through performance analysis on both synthetic and real flight data before it counts as done. Passing tests is not enough.

- Synthetic: run the simulator scenarios (`ctest`) and compare vario and wind errors against the previous values, not only the pass/fail thresholds.
- Real: rerun the relevant `tools/airmass_sim/analysis/` scripts (at least `batch.js` over the real logs) before and after the change, and record the comparison in a dated `RESULTS-*.md` there.
- Air mass vario accuracy takes priority over instant wind accuracy when the two trade off.
- Active check: before finishing, state the before/after numbers for both data sets. If either was not run, say so instead of reporting the change as verified.

### 3. Stay inside AirMass

Avoid changes outside `main/airmass/` and `tools/airmass_sim/` unless absolutely necessary. Shared firmware code (display, setup menus, sensors, wind, protocols, components) serves the whole instrument and tracks upstream, so solve the problem inside AirMass first.

- When an outside change cannot be avoided, keep it to the smallest hook that calls into `main/airmass/`, and put the logic here.
- Active check: before finishing, list every file in the diff outside those two directories and state why each change could not be made inside AirMass. Revert the ones without a reason.

### 4. Never mention AI/agent authorship in git metadata

Do not mention the agent, AI assistant, or automated authorship in branch names, commit messages, PR titles, PR descriptions, or any other project metadata.

- The default Claude Code commit template appends a `Co-Authored-By: Claude ...` trailer. Strip it before every commit.
- Active check: before running `git commit`, read back the composed message and confirm it contains no `Co-Authored-By`, `Claude`, `AI`, or `agent` attribution.

### 5. Avoid comments unless the WHY is non-obvious

Do not add explanatory comments that just restate what the code does. Only comment when the reasoning behind a choice is not obvious from the code itself.

- Active check: before declaring a coding task done, search the diff of files just touched for comment syntax such as `^\s*//` and `#`, then remove anything that is not explaining a genuinely non-obvious WHY.

### 6. Reuse existing patterns; avoid over-engineering

Prefer existing components, utilities, and standard approaches over bespoke new code. When touching a flow, prefer simplifying it and covering corner cases over adding surface area.

- Active check: before writing a new menu entry, status page, screen element, or filter, search for an existing one that already serves the same role (`AirMassMenu.cpp`, `main/setup/`, `main/screen/element/`, `main/sensor/Filters.h`) and reuse or extract it instead of copying its structure.
- Active check: if a new setting or output parallels an existing one, compare them side by side and match naming, units, and menu placement, not just wording.

### 7. Keep user-facing docs aligned

When updating `README.md` or `tools/airmass_sim/web/README.md`, prefer existing wording over introducing new phrasing, avoid over-specification, and remove mentions of settings or flows that no longer exist.

### 8. Match existing wording style for new UI text

New menu entries, help texts, and status labels should follow the phrasing of the existing setup menus, and the browser replay should use the same name as the device for the same setting or value.

- Active check: before finishing, grep `AirMassMenu.cpp` and `main/setup/` for a similar existing entry and align the new title and help text to it.

### 9. Keep commit messages short and to the point

One line, no body, ending with a period, prefixed `AirMass:` for AirMass changes, as in `git log`.

- Active check: before running `git commit`, re-read the drafted message and cut it down to one line.

## Before You Finish

Before reporting a task done, re-read this checklist against the actual diff and check off each item explicitly.

- [ ] Ran the comment-syntax check from Core Rule #5 over the files changed in this task and removed any comment that does not explain a non-obvious WHY.
- [ ] Read back the commit message, if committing, and confirmed no AI/agent attribution per Core Rule #4 and that it's a one-liner per Core Rule #9.
- [ ] If a new menu entry, status page, or screen element was added, confirmed per Core Rule #6 that it reuses an existing one rather than duplicating it.
- [ ] If new user-facing strings were added, confirmed per Core Rule #8 that wording matches existing phrasing and is consistent between the device and the browser replay.
- [ ] Per Core Rule #1, is every firmware change mirrored in the simulator, the replay, and the browser replay, and the reverse?
- [ ] Per Core Rule #2, if EKF tuning changed, were before/after results produced on both synthetic and real flights and recorded?
- [ ] Per Core Rule #3, is every changed file outside `main/airmass/` and `tools/airmass_sim/` absolutely necessary and reduced to a minimal hook?
- [ ] Are tests updated to cover the changes?
- [ ] Did the host tests pass (Reference section 1)?
- [ ] If device code outside the estimator changed, did the firmware build pass?
- [ ] If a source file was added or an include path changed, are `tools/airmass_sim/CMakeLists.txt` and `tools/airmass_sim/web/build.sh` still correct?
- [ ] Is `.github/workflows/cmake.yml` still valid?
- [ ] Are the AirMass section of `README.md` and `tools/airmass_sim/web/README.md` accurate with these changes?
- [ ] If an `AirMassEKF::Config` field was added, renamed, or removed, is `tools/airmass_sim/replay_settings.h` updated?
- [ ] If the recording format changed, do old recordings still replay (Reference section 4)?
- [ ] If sampling, input conversion, or gating changed, were sensor gaps, garbled readings, missing GNSS, and an airborne start considered or tested?
- [ ] Before `git push`, does `git branch --show-current` match the branch your new commit actually landed on? A push can report "Everything up-to-date" while pushing the wrong local branch, silently leaving the real commit unpushed elsewhere.

## Reference

### 1. Tests

- Build and test only in Docker, never with the host toolchain. Host tests, as CI runs them:

      docker run --rm -v "$PWD":/project -w /project espressif/idf:release-v5.4 bash -c ". \$IDF_PATH/export.sh > /dev/null && cmake -S tools/airmass_sim -B build_sim -DCMAKE_BUILD_TYPE=Release && cmake --build build_sim -j && ctest --test-dir build_sim --output-on-failure"

- Firmware build: the same image, with the ESP-IDF patches of `patches/esp-idf/` applied before `idf.py build` (`patches/esp-idf/README.md`). Flashing stays outside the container.

      docker run --rm -v "$PWD":/project -w /project espressif/idf:release-v5.4 bash -c ". \$IDF_PATH/export.sh > /dev/null && git -C \$IDF_PATH apply /project/patches/esp-idf/*.patch && idf.py build"

- Add simulator scenarios and unit tests in `tools/airmass_sim/` (`add_test` in its `CMakeLists.txt`) when modifying estimator logic.
- Browser replay tests: `node --test tools/airmass_sim/web/settings_test.js`, `node --test tools/airmass_sim/web/serial_test.js`, and after `build.sh` `node tools/airmass_sim/web/replay_start_test.js`. Log conversion parity between `sens2rec.py` and `convert.js`: `python3 tools/airmass_sim/conversion_test.py`.

### 2. Browser Replay

- `tools/airmass_sim/web/build.sh` compiles the estimator to `airmass.js` in the `emscripten/emsdk` Docker image. `airmass.js` is gitignored, so rebuild it after any estimator change before running the page, its tests, or the analysis scripts.
- `tools/airmass_sim/web/deploy.sh` publishes the page publicly. Run it only when asked.

### 3. Documentation

- `README.md` has the user-facing AirMass description: settings, status page, recording, and tuning. Update it whenever a change affects what a pilot sees or sets.
- `tools/airmass_sim/web/README.md` describes the browser replay, and `tools/airmass_sim/analysis/README.md` the comparison scripts and results.

### 4. Recordings & Input Robustness

- `AirMassRecord.h` defines the recording (`$PXCVA` output, `$PXCVS` initialization, `$PXCVC` setup, and the inputs). `airmass_replay` and the browser replay read it, and the replay checks itself against the recorded `$PXCVA`. Recordings from earlier versions and plain `$SENS` logs exist in the field, so a format change must keep them replaying.
- Sensor and GNSS input can have gaps, repeated or garbled readings, and can start in flight. When changing sampling, conversion, gating, or initialization, account for these and keep `robustness_test`, `sampling_test`, `input_test`, and `replay_start_test` passing.

