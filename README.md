# SANA path

An interactive React + TypeScript route composer for the 2026–2027 FTC BIOBUZZ field. Draw or place waypoints on a Pedro-coordinate field, tune headings and cubic Bézier handles, inspect automatic interpolation choices, and export a Pedro Pathing 3 Java OpMode.

## Run locally

These commands work in **Windows Command Prompt or PowerShell, macOS Terminal, and Linux terminals**. Install Node.js 22.12 or newer (with npm) and Git first, then open a terminal in the folder where you want to download the project:

```sh
git clone https://github.com/seveneqq1/biobuzz-path-studio.git
cd biobuzz-path-studio
npm install
npm run dev
```

The `cd` command uses the repository folder name, not a personal username or operating-system-specific path. If the repository is private, GitHub access is required to clone it.

Already downloaded the project? Open a terminal inside its folder (the one containing `package.json`) and run:

```sh
npm install
npm run dev
```

Open the localhost URL printed by Vite (usually http://localhost:5173). Keep the terminal running; press Ctrl+C to stop. On Windows, if PowerShell blocks `npm.ps1`, use Command Prompt for the same commands. Use `npm run test`, `npm run lint`, and `npm run build` to check the project.

## Quick start: Auto-build

1. Pick the **Draw** tool and sketch your route (press `1`–`5` while drawing to drop commands).
2. When you release, **Auto-build** runs automatically (toggle "Auto-build after drawing" to turn it off). You can also click **Auto-build path** anytime.
3. Choose a style in the header: **Follow my drawing**, **Balanced** or **Fastest**.
4. Click **Export Java** for a Pedro 3 + Ivy OpMode.

Auto-build works like the Pedro Pathing visualizer: the route becomes one path between each pair of *major points* (start, every command point, any pose whose heading you edited, and the end). Each path gets the fewest control points that stay close to your drawing. The planner then checks the robot's rotated footprint (+0.5 in buffer) against walls and hive supports and bends the path away from anything it hits. Unsafe major points are moved to the nearest safe spot. Among the clear candidates, it picks the best trade-off between simulated drive time (motor RPM, wheel size, mass, traction, curvature and turn limits) and closeness to your sketch. **Best headings** simulates every valid heading interpolation (constant, linear, tangent, piecewise) for each path and keeps the fastest collision-free option. The inspector explains why each option was chosen.

Select a waypoint to see the purple control points of its paths. Drag them to bend the path, double-click to delete one, or use **Add control / Remove / Straight** in the inspector. Control points export as `Paths.curve(start, p.of(...), ..., end)`; zero control points export as `Paths.line`. The generated class also has a `MIRROR_FOR_OTHER_ALLIANCE` switch that mirrors every pose through `PoseFactory.mirrorY(72)`.

## Features

- 144 × 144 inch BIOBUZZ field schematic with Pedro's bottom-left origin
- Freehand smoothing, waypoint placement, draggable nodes, heading handles, zoom and pan
- Select a point with the Select/move tool to edit X, Y and Rotation directly in the 2D coordinate overlay. Type values or use Up/Down in a focused field (0.1 inch for position, 1° for rotation); hold Shift for 10× steps. Enter finishes editing. The inspector and Java preview update live.
- Constant, linear, tangent and piecewise heading optimization with per-segment explanations
- Schneider-style cubic Bézier fitting with iterative Newton reparameterization and a reverse-distance overshoot check: a 0.2-inch default error target, adjustable from 0.1 to 0.5 inches. Shared tangents retain smooth semicircles/S-curves; command markers and manual interpolation boundaries survive reduction. Fitted handles are used by both rendering and export. Freehand capture now retains finer detail before optimization.
- Draw-time action bindings: `1` shooter, `2` intake, `3` transfer, `4` flower intake, and hold `5` for a wait rounded to 100 ms
- Manual interpolation overrides and Bézier tension controls
- Inspector action to snap a waypoint to a nearby minimum-distance safe pose with a 0.5-inch footprint buffer, preserving heading. Uses a 0.25-inch radial search with local refinement; boxes are conservatively inflated. Adjacent curves are rechecked, not silently rerouted. Unsafe curved segments are red in both views, even when their endpoints are clear.
- Heading rehearsal in 3D: A/D or Left/Right turn counterclockwise/clockwise at the configured maximum rotation rate; W/S step clockwise/counterclockwise by 90°. Playback pauses, Enter applies to the selected waypoint, and Esc cancels. On-screen controls and a waypoint selector are available. Rotation sweeps stop at obstacles; explicit heading edits remain independent of tangent auto-selection and preserve curve handles.
- Pedro 3 `PoseFactory.degrees()`, `Paths.line/curve/path`, and Ivy command-based Java output
- Sequential actions, parallel joins with the next path, path-scoped deadlines, and optional action timeout races; instant intake and millisecond wait commands
- Multiple independent actions per path: select a waypoint, use **Path actions → Add path action**, then drag each numbered marker along the curve or enter its curve position (0–100%). Markers can move between path segments without moving waypoints. Each has its own type, sequential/parallel/deadline mode, wait duration or timeout. Existing waypoint commands can be converted with **Make draggable path action**.
- Live Java preview, clipboard copy and `.java` download
- Refined robotics workbench UI: edit in 2D, then run in a lazy-loaded Three.js 3D scene with orbit, robot-follow and hive cameras
- Persistent light/dark theme toggle in the header, including inspector, settings and 3D stage
- Fully light planning surfaces, controls, overlays and Java editor (field artwork remains unchanged), responsive desktop/sidebar sizing and dock-aware field fitting; header credit: Presented by SANA #24697
- Swept wall collision checks for the rotated square robot footprint, including pure rotation and large timesteps; invalid starts are constrained and blocked, and contact stops playback before later commands execute. Unsafe exports refuse to initialize until the route is corrected.
- Shared render/physics geometry for hive pillars, ground rails, feet, sign panels and the overhead axle. Robot support contacts stop the route; swept ball contacts rebound off the frame. Planning markers and Java initialization guards flag unsafe support clearance, while open space under the frame remains traversable where the template fits.
- Shot-aware follow camera: widens before a shooting waypoint, frames the robot, alliance hive and airborne shots (including their predicted apex), then smoothly returns to the close chase view after shooting/flight finishes. Framing accounts for portrait and landscape screens.
- Independent, rate-limited yaw/pitch turret with a turntable bearing, adjustable elevation hood and twin flywheels; alignment gates firing without changing the route's chassis headings
- Opt-in **Shoot while moving** capability converts shoot commands with a following path to Ivy parallel groups (explicit deadlines remain deadlines); the last shoot remains sequential. The next command waits for the group to join. Automatic elevation estimates constant-velocity lead, and launched pieces inherit field-frame robot velocity. Manual elevation does not compensate flight. This is not a calibrated moving-shot controller.
- Selectable gecko/silicone wheel-stack intake, front-facing collection, opposed transfer rollers and a gated indexer; queued pieces advance one at a time while preserving type/color
- Recreated mecanum robot, triangular hive frame, paired transparent cells, flowers, perforated pollen and alliance-colored nectar; shadows and physically based materials
- Smooth hollow polyethylene-style game pieces with analytic circular holes and rounded lips, nonmetallic nectar surfaces and shared reusable geometry. Wheel animation uses actual distance traveled; stationary chassis contact no longer adds speed on every tick.
- Fixed-step ball gravity, rolling resistance, bounce, wall and mass-weighted ball contacts, intake, flower release, ballistic shooting, and damped hive tipping
- Cell contents preserve their actual piece type, mass and alliance color through spills, collection and re-shooting; energetic spills leave the tilted opening's lower lip with gravity, pivot velocity and a decaying stop jolt. Reproducible packing/lip deflections spread pieces in a forward fan, followed by livelier floor bounces and longer rolls—not a radial explosion.
- Drive-time prediction from motor output RPM, wheel size, gearing, mass, drive force, traction, loaded speed, curvature, and rotation limits; action delays and inventory-dependent shooting give a runtime range
- Red/blue autonomous scoreboard and route-end breakdown, with a 30-second cutoff. AUTO estimates include hive tips (20), LEAVE (3) and AUTO PARK (5) for this single robot. Cell, owned-flower, bottom-nectar and garden values are separately labeled end-state potential, not AUTO points. No partner, fouls, RP or 8-second transition settling is modeled. Values follow FIRST TU04 §10.5; flower geometry assessment is approximate. This match-style display is not an exact DSIM replica.

## Preview and calibration

Click **Run in 3D**, or **Preview 3D** to inspect without playing. **Plan 2D** returns to editing. The robot follows the same Bézier handles used by the editor and exporter. Drag to orbit, scroll to zoom, or use the **Follow robot** / **Hives** cameras. Drag floor balls while paused, or toggle intake directly to explore collection. **Robot setup** configures drive timing, inventory, shooter and alliance. Editing geometry or configuration resets playback.

By default, the intake command instantly starts the wheels and stays latched until shooting or the route finishes. Only balls entering the front mouth are collected; side/back contacts push them. Collection uses hopper capacity for the whole stored/queued inventory. Each queued piece takes the configured transfer time through the indexer. Transfer waits for that queue to empty and its configured minimum delay; flower intake releases the nearest flower's pollen toward the intake mouth and leaves the intake latched. Choosing Flower intake (or Auto-build) moves the waypoint square to the wall, facing the flower. Shooting waits for turret yaw/pitch alignment, feeds each indexed piece, then waits for flight/settling before the next path. Parallel actions start alongside the following uninterrupted path and join at the next command marker. Deadline actions are cancelled when that path finishes (deadline intake stays active for the entire path). A timeout races a bounded action against `waitMs()` without cancelling the drive branch. Concurrent actions on the last waypoint fall back to sequential. The Java exporter and simulator share route partitioning. Runtime ranges are planning estimates, not guarantees; transferred inventory and mechanism behavior can increase them.

The independent turret aims at the raised alliance cell, with a configurable yaw motor rate and a modeled 90°/s pitch rate. Automatic elevation solves a ballistic high arc from the barrel muzzle when launch speed can reach it. Disable automatic elevation to explore manual angles and misses. The template assumes continuous turret yaw; your hardware controller must enforce encoder, cable and mechanical limits. Gecko/silicone changes the recreated tread, not an experimentally verified friction model. The transfer template is a starting architecture, not a claim that one mechanism is optimal for every robot.

**Shoot while moving** is off by default. It overrides sequential shoot composition for nodes with a following path; manually selected concurrent compositions still behave as specified when the toggle is off. Use this capability only if your real robot supports target tracking while driving. Moving-shot Java output includes fail-fast `robotFieldVelocityX/Y()` hooks, which must be wired to your tuned localizer's measured field velocity in inches/second. Hardware validation is required. A safe snapped waypoint is not proof that its incoming/outgoing curves, arbitrary rotations or your robot's moving mechanisms are safe.

This is a planning simulation, not a validated digital twin or official CAD model. Robot motion follows the prescribed route until a field wall or solid hive support blocks it; the run then stops instead of clipping or sliding to later actions. Support checks use a conservative rotated box up to the template's maximum turret height, capsules for tubes and boxes for feet/panels (the sign panel collider conservatively bounds its trapezoid). The default demo uses a clear lane around the supports. Hive-envelope overlap remains an additional dynamic-cell/height-clearance warning. Hive geometry uses nominal FIRST dimensions, with approximate latch torque, inertia, damping and contact restitution. Loads use actual pollen/nectar mass; openings are swept-contact targets, with approximate cell-skin contacts. It does not adjudicate official scoring, simulate other robots, or model exact moving robot/cell CAD contacts, drivetrain slip, jams or compliant-wheel dynamics. Calibrate force, speed and indexer delays from measured runs. The 3D robot is a recreated template, not your team's CAD. The Java clearance guards are sampled planning checks, not real-robot collision sensors or safety guarantees.

## Pedro 3 + Ivy export setup

Path markers are independent of the editor's waypoints. Execution/export subdivides the original Bézier exactly at each marker; it does not refit or move the drawn curve. At a shared position, sequential actions run in list order before parallel/deadline branches start with the following path. Parallel branches join at the next command marker; deadline branches cancel there. End-of-route actions finish sequentially. Per-marker modes are explicit, even with the global moving-shot preset enabled. Simultaneous actions that share an Ivy subsystem requirement (for example, shoot and transfer both using the feeder) are rejected before playback and by a generated initialization guard; move one marker or make it sequential. Combined shooting and intake may keep collecting, so completion predicates on real hardware must define when feeding finishes.

Install and tune [Pedro 3](https://pedropathing.com/docs/pathing/installation) in your FTC Android project. Install [Ivy's Pedro integration](https://pedropathing.com/docs/ivy) in `build.dependencies.gradle`:

```groovy
implementation 'com.pedropathing.ivy:pedro:1.1.1'
```

Export `SanaAuto.java` into your TeamCode package. Adjust the `pedro.Constants` import to your team's actual package and initialize your subsystems in `init()`. Wire each generated start/update/completion/stop hook to hardware. Unwired start hooks deliberately throw; completion hooks deliberately return false instead of silently claiming a mechanism finished. Implement safe stop hooks before running. Shared resource identities prevent conflicts. The scheduler resets on init/stop, runs in `loop()`, and follow commands wait for the follower's complete end constraints. Shoot commands first aim the turret, then run shooter feeding as the deadline alongside continuous turret tracking. The feeder only advances when yaw/pitch are aligned and measured flywheel speed is ready. `startShooter` should only spin up; `updateShooter` maintains its controller; `advanceFeeder` actuates the indexer. Use break-beam/encoder feedback to finish transfers and count shots. The exported alliance, launch speed, elevation, intake material and footprint come from Robot setup; update the hive target hooks from observed cell state after tipping.

The exporter targets Pedro core **3.0.0** and Ivy **1.1.1**. Linear headings use arc length. Its piecewise compatibility helper compensates for Pedro 3.0.0's child reparameterization to keep tangent headings aligned with the curve and continuous into the fixed-heading transition. Reverify this helper when upgrading the library. See [PoseFactory](https://pedropathing.com/docs/pathing/guide/pose-creation), [Paths](https://pedropathing.com/docs/pathing/reference/api), and [Ivy compositions](https://pedropathing.com/docs/ivy/command-compositions).

`npm run test:java` requires a JDK (`javac` + `jar`) and Internet access. It downloads the official Maven Central binaries and compiles three exported route variants against real Pedro/Ivy APIs. Only the FTC OpMode/telemetry and your team's Constants are compile-only stubs: this is not a complete Android build or hardware verification.

The regression suite checks smooth semicircles/S-curves, bidirectional fit error, aligned join tangents, command preservation, timing, waits/compositions/timeouts, front intake and queued transfer, nectar conservation, turret gating, energetic/reproducible lower-lip spills, translation/rotation wall and support sweeps, blocked starts, large timesteps, projectile rebounds, shot-camera frustum fitting/damping, elapsed-time continuity, buffered snapping, unsafe interiors with safe endpoints, independent headings, rehearsal turns, moving-shot joins, AUTO score categories/time limits, stationary contact energy and reusable plastic models. Path-action tests additionally cover exact higher-order curve subdivision, independent marker movement, shared-position command bundles, per-action modes, resource conflicts, optimizer preservation and close-marker join barriers. The API compile check includes multi-action and mixed-composition exports, moving-shot velocity hooks, automatic and manual pitch, red/blue alliance targets, and both intake templates.

The included 4096 × 4096 MeepMeep-compatible BIOBUZZ field image was created by [FTC Team Juice 16236](https://www.reddit.com/r/FTC/comments/1weleaj/biobuzz_custom_field_images_meepmeep_compatible/) and is used with the attribution requested by its creator. This asset places blue at the top and red at the bottom. Field placement was checked against the official FIRST Event Field Setup Guide. Game-piece and hive assumptions were checked against the [FIRST BIOBUZZ Competition Manual TU04](https://ftc-resources.firstinspires.org/ftc/game/cm-html/BIOBUZZ%20Competition%20Manual%20-%20TU04.htm). Use official FIRST materials for construction measurements and rules.
