import type { PathAction, SegmentDecision, Waypoint } from '../types'
import { segmentPoints } from './geometry'
import { compileRoute, effectiveAction, nodeActions } from './route'
import { materializePathActions } from './pathActions'
import { defaultConfig, profile } from './simulation'
import type { RobotConfig } from './simulation'
import { cellOpening, createHive } from './hivePhysics'

const n = (value: number) => Number(value.toFixed(3)).toString()
const poseName = (index: number) => index === 0 ? 'startPose' : `pose${index}`
const milliseconds = (value: number | undefined, fallback = 100) => Math.max(100, Math.round((value ?? fallback) / 100) * 100)

function interpolation(decision: SegmentDecision, start: number, end: number, points: Waypoint[]) {
  switch (decision.type) {
    case 'constant': return `.constant(${poseName(start)})`
    case 'tangent': return '.tangent()'
    case 'piecewise': return `.heading(smoothPiecewise(Math.toRadians(${n(points[end].heading)})))`
    default: return `.linear(${poseName(start)}, ${poseName(end)})`
  }
}

export function generateJava(points: Waypoint[], decisions: SegmentDecision[],config:RobotConfig=defaultConfig) {
  if (points.length < 2) return '// Add at least two waypoints to generate a path.'
  const expanded = materializePathActions(points, decisions)
  points = expanded.points; decisions = expanded.decisions
  const route = compileRoute(points,config.shootWhileMoving)
  const paths = route.flatMap(step => step.kind === 'path' ? [step] : step.kind === 'group' ? [step.path] : [])
  const actions = points.flatMap((p, i) => nodeActions(p).map(a => effectiveAction(a,i,points.length,config.shootWhileMoving)))
  const used = new Set(actions.map(action => action.type))
  const safety=profile(points,config),collision=safety.wallCollision,support=safety.supportCollision
  const actionConflict=safety.actionConflicts[0]
  const opening=cellOpening(createHive(config.alliance==='red'?1:0,config.alliance),config.alliance==='red'?1:0)
  const fields = points.map((point, index) => `    private final Pose ${poseName(index)} = p.of(${n(point.x)}, ${n(point.y)}, ${n(point.heading)});`).join('\n')
  const pathMethods = paths.map(item => {
    const segments = points.slice(item.startIndex, item.endIndex).map((a, offset) => {
      const index = item.startIndex + offset, b = points[index + 1], inner = segmentPoints(points, index).slice(1, -1)
      const cross = (p: {x: number; y: number}) => Math.abs((b.x-a.x)*(p.y-a.y)-(b.y-a.y)*(p.x-a.x))
      const span=(b.x-a.x)**2+(b.y-a.y)**2
      const along=(p:{x:number;y:number})=>(p.x-a.x)*(b.x-a.x)+(p.y-a.y)*(b.y-a.y)
      const straight = a.controlPoints ? !a.controlPoints.length
        : inner.every(c => cross(c) < 1e-8 && along(c) >= 0 && along(c) <= span) && along(inner[1]) >= along(inner[0]) && !a.curve
      const geometry = straight ? `Paths.line(${poseName(index)}, ${poseName(index + 1)})`
        : `Paths.curve(${poseName(index)},\n${inner.map(c => `                p.of(${n(c.x)}, ${n(c.y)}, 0),`).join('\n')}\n                ${poseName(index + 1)})`
      return `            ${geometry}${interpolation(decisions[index], index, index + 1, points)}`
    })
    return `    private Path ${item.name}() {\n        return Paths.path(\n${segments.join(',\n')}\n        );\n    }`
  }).join('\n\n')
  const actionCode = (action: PathAction, deadline = false, keepIntake = false) => {
    const expression = action.type === 'wait' ? `waitMs(${milliseconds(action.durationMs)})`
      : action.type === 'intake' ? deadline ? 'intakeUntilCancelled()' : 'instant(this::startIntake).requiring(intakeResource)'
      : action.type === 'shoot' && keepIntake ? 'shootCommand(true)' : `${action.type}Command()`
    return action.timeoutMs && action.type !== 'wait' && action.type !== 'intake'
      ? `race(${expression}, waitMs(${milliseconds(action.timeoutMs)}))` : expression
  }
  const commands = route.map(step => {
    if (step.kind === 'path') return `            followPath(${step.name}())`
    if (step.kind === 'action') return `            ${actionCode(step.action)}`
    const branches = [step.action, ...(step.additionalActions ?? [])]
    const keepIntake = branches.some(a => a.type === 'intake' || a.type === 'flowerIntake')
    const deadlines = branches.filter(a => a.composition === 'deadline')
    const parallel = branches.filter(a => a.composition !== 'deadline')
    const drive = deadlines.length ? `deadline(followPath(${step.path.name}()), ${deadlines.map(a=>actionCode(a,true,keepIntake)).join(', ')})` : `followPath(${step.path.name}())`
    return `            ${parallel.length ? `parallel(${drive}, ${parallel.map(a=>actionCode(a,false,keepIntake)).join(', ')})` : drive}`
  })
  commands.push('            instant(this::stopAllSubsystems)')
  const boundedTypes = ['shoot', 'transfer', 'flowerIntake'] as const
  const commandsMethods = boundedTypes.filter(type => used.has(type)).map(type => {
    const suffix = type === 'shoot' ? 'Shooter' : type === 'transfer' ? 'Transfer' : 'FlowerIntake'
    const startHook=type==='shoot' && used.has('intake') ? '() -> { if (!keepIntakeRunning) stopIntake(); startShooter(); }' : `this::start${suffix}`
    const command=`Command.build()\n            .setStart(${startHook})\n            .setExecute(${type==='shoot'?'() -> { updateShooter(); if (isTurretAtTarget() && isShooterAtSpeed()) advanceFeeder(); else holdFeeder(); }':`this::update${suffix}`})\n            .setDone(this::is${suffix}Finished)\n            .setEnd(reason -> stop${suffix}())\n            .requiring(${type === 'shoot' ? 'shooterResource, feedResource' : type === 'transfer' ? 'feedResource' : 'intakeResource'})`
    return `${type==='shoot'?`    private Command shootCommand() { return shootCommand(false); }\n\n`:''}    private Command ${type}Command(${type==='shoot'?'boolean keepIntakeRunning':''}) {\n        return ${type==='shoot'?`sequential(\n            aimTurretCommand(),\n            deadline(${command}, trackTurretCommand())\n        )`:command};\n    }`
  })
  if(used.has('shoot'))commandsMethods.push(`    // Turret yaw is independent of every path's chassis heading.
    private Command aimTurretCommand() {
        return Command.build()
            .setStart(this::updateTurretTarget)
            .setExecute(this::updateTurretTarget)
            .setDone(this::isTurretAtTarget)
            .setEnd(reason -> stopTurret())
            .requiring(turretResource);
    }

    private Command trackTurretCommand() {
        return Command.build()
            .setExecute(this::updateTurretTarget)
            .setDone(() -> false)
            .setEnd(reason -> stopTurret())
            .requiring(turretResource);
    }

    private void updateTurretTarget() {
        Pose robot = follower.pose();
        double dx = hiveTargetX() - robot.x(), dy = hiveTargetY() - robot.y();
        double angle = Math.atan2(dy, dx) - robot.heading();
        double yaw = Math.atan2(Math.sin(angle), Math.cos(angle));
        double elevation = Math.toRadians(${n(config.shotAngle)});
        ${config.autoAim?`double range = Math.max(0.01, Math.hypot(dx, dy));
        double speedSquared = ${n(config.shotSpeed**2)}; // calibrate measured exit speed
        for (int i = 0; i < 10; i++) {
            double horizontal = Math.max(0.01, range - ${n(config.size*.42)} * Math.cos(elevation));
            double rise = hiveTargetHeight() - (18 + ${n(config.size*.42)} * Math.sin(elevation));
            double discriminant = speedSquared * speedSquared - 386.09 *
                (386.09 * horizontal * horizontal + 2 * rise * speedSquared);
            if (discriminant < 0) throw new IllegalStateException("Hive is outside calibrated shooter range");
            elevation = Math.atan((speedSquared + Math.sqrt(discriminant)) / (386.09 * horizontal));
            ${config.shootWhileMoving?`double vertical = ${n(config.shotSpeed)} * Math.sin(elevation);
            double riseFromMuzzle = hiveTargetHeight() - 18 - ${n(config.size*.42)} * Math.sin(elevation);
            double flight = (vertical + Math.sqrt(Math.max(0, vertical * vertical - 2 * 386.09 * riseFromMuzzle))) / 386.09;
            dx = hiveTargetX() - robot.x() - robotFieldVelocityX() * flight;
            dy = hiveTargetY() - robot.y() - robotFieldVelocityY() * flight;
            range = Math.max(0.01, Math.hypot(dx, dy));
            angle = Math.atan2(dy, dx) - robot.heading();
            yaw = Math.atan2(Math.sin(angle), Math.cos(angle));`:''}
        }`: '// Manual launch elevation selected in Robot setup.'}
        setTurretTarget(yaw, elevation); // chassis-relative yaw and elevation in RADIANS
        updateTurretController();
    }${config.shootWhileMoving?`

    // Moving shots need measured FIELD-frame velocity in INCHES/SECOND.
    // These hooks must use your tuned localizer; do not return a guessed speed.
    private double robotFieldVelocityX() { throw new IllegalStateException("Wire measured field X velocity"); }
    private double robotFieldVelocityY() { throw new IllegalStateException("Wire measured field Y velocity"); }
    `:''}`)
  if (actions.some(action => action.type === 'intake' && action.composition === 'deadline')) commandsMethods.push(`    private Command intakeUntilCancelled() {\n        return Command.build()\n            .setStart(this::startIntake)\n            .setDone(() -> false) // the following path is the deadline\n            .setEnd(reason -> stopIntake())\n            .requiring(intakeResource);\n    }`)
  const hooks = boundedTypes.filter(type => used.has(type)).map(type => {
    const suffix = type === 'shoot' ? 'Shooter' : type === 'transfer' ? 'Transfer' : 'FlowerIntake'
    return `    private void start${suffix}() {\n        throw new IllegalStateException("Wire start${suffix} to your robot hardware before running");\n    }\n    private void update${suffix}() { /* TODO: update subsystem controller / sensors */ }\n    private boolean is${suffix}Finished() { return false; /* TODO: measured completion */ }\n    private void stop${suffix}() { /* TODO: motor power zero / safe servo state */ }`
  })
  if (used.has('intake')) hooks.push(`    private void startIntake() {\n        throw new IllegalStateException("Wire startIntake to your robot hardware before running");\n    }\n    private void stopIntake() { /* TODO: motor power zero */ }`)
  if(used.has('shoot'))hooks.push(`    // Raised ${config.alliance} cell at reset. Update these from observed hive state after a tip.
    private double hiveTargetX() { return ${n(72+opening.x)}; }
    private double hiveTargetY() { return ${config.alliance==='red'?'59.25':'84.75'}; }
    private double hiveTargetHeight() { return ${n(opening.z)}; }
    private void setTurretTarget(double yawRadians, double elevationRadians) {
        throw new IllegalStateException("Wire turret PID, encoder, pitch servo and cable limits before running");
    }
    private void updateTurretController() { /* TODO: closed-loop yaw/pitch control; never rotate chassis to aim */ }
    private boolean isTurretAtTarget() { return false; /* TODO: yaw encoder AND pitch tolerance */ }
    private boolean isShooterAtSpeed() { return false; /* TODO: measured flywheel velocity */ }
    private void advanceFeeder() { /* TODO: feed one piece, then close gate until next shot */ }
    private void holdFeeder() { /* TODO: indexer power zero; close feed gate while aiming */ }
    private void stopTurret() { /* TODO: safe yaw/pitch outputs */ }`)
  const stops = [...boundedTypes.filter(type => used.has(type)).map(type => `        stop${type === 'shoot' ? 'Shooter' : type === 'transfer' ? 'Transfer' : 'FlowerIntake'}();`), ...(used.has('intake') ? ['        stopIntake();'] : []),...(used.has('shoot')?['        holdFeeder();','        stopTurret();']:[])]

  return `package org.firstinspires.ftc.teamcode;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.api.Paths;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
${decisions.some(d => d.type === 'piecewise') ? 'import com.pedropathing.paths.interpolator.Interpolator;\n' : ''}import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.*;
import static com.pedropathing.ivy.pedro.PedroCommands.*;

// Use the package containing YOUR Pedro 3 Constants file.
import org.firstinspires.ftc.teamcode.pedro.Constants;

// Requires Pedro 3 + Ivy (com.pedropathing.ivy:pedro:1.1.1).
// Positions are inches; PoseFactory headings are DEGREES.
// Wire the fail-fast subsystem hooks below before operating a real robot.
// Intake template: ${config.intakeMaterial} wheel stacks -> opposed rollers -> break-beam indexer.
// Transfer must finish on a sensor/encoder, not an unconditional timer.
@Autonomous(name = "SANA path Ivy Auto")
public class SanaAuto extends OpMode {
    private Follower follower;
    private Command auto;
    // Flip to true to run this route on the opposite alliance: every pose and
    // control point is mirrored across the field midline (y = 72).
    // Hive target hooks below are NOT mirrored; update them too.
    private static final boolean MIRROR_FOR_OTHER_ALLIANCE = false;
    private final PoseFactory p = MIRROR_FOR_OTHER_ALLIANCE
        ? PoseFactory.degrees().mirrorY(72).mapHeading(h -> -h)
        : PoseFactory.degrees();
${fields}

    // Shared requirement identities; replace with your subsystem objects if desired.
    private final Object shooterResource = new Object();
    private final Object intakeResource = new Object();
    private final Object feedResource = new Object();
    private final Object turretResource = new Object();
    private final boolean routeClearsWalls = ${!collision}; // ${config.size}-inch rotated square footprint
    private final boolean routeClearsSupports = ${!support}; // conservative template height; recheck against your CAD
${actionConflict?'    private final boolean actionsCompatible = false;\n':''}

    @Override
    public void init() {
        Scheduler.reset();
${actionConflict?`        if (!actionsCompatible) throw new IllegalArgumentException("${actionConflict.message}");\n`:''}
        if (!routeClearsWalls) throw new IllegalArgumentException("${collision?`Route intersects a wall near (${n(collision.x)}, ${n(collision.y)}). Edit the route before running.`:'Recheck wall clearance after changing the robot footprint.'}");
        if (!routeClearsSupports) throw new IllegalArgumentException("${support?`Route intersects ${support.name} near (${n(support.sample.x)}, ${n(support.sample.y)}). Edit the route before running.`:'Recheck hive support clearance against your robot geometry.'}");
        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
        // TODO: initialize your hardware/subsystems here.
        auto = sequential(
${commands.join(',\n')}
        );
    }

    @Override
    public void start() {
        Scheduler.schedule(auto);
    }

    @Override
    public void loop() {
        follower.update();
        Scheduler.execute();
        telemetry.addData("Auto running", Scheduler.isScheduled(auto));
        telemetry.update();
    }

    @Override
    public void stop() {
        Scheduler.reset(); // cancels groups and invokes command cleanup
        if (follower != null) follower.stop();
        stopAllSubsystems();
    }

    private Command followPath(Path route) {
        return follow(follower, route)
            // Wait for all follower end constraints, not just parametric progress.
            .setDone(() -> !follower.following())
            .requiring(follower);
    }

${pathMethods}
${decisions.some(d=>d.type==='piecewise') ? `
    // Pedro 3.0.0 reparameterizes piecewise children. Undo that remapping for
    // geometric tangent headings, and rotate by arc length after the 68% join.
    private Interpolator smoothPiecewise(double endHeading) {
        return Interpolator.piecewise()
            .until(0.68, (curve, localT) -> Interpolator.tangent.interpolate(
                curve, localT * curve.parameter(0.68)))
            .until(1.0, (curve, localT) -> {
                double join = curve.parameter(0.68);
                double actualT = join + localT * (1.0 - join);
                double progress = (curve.pathCompletion(actualT) - 0.68) / 0.32;
                double heading = Interpolator.tangent.interpolate(curve, join);
                return Interpolator.linear(heading, endHeading).interpolate(
                    curve, curve.parameter(Math.max(0, Math.min(1, progress))));
            });
    }
` : ''}
${commandsMethods.length ? '\n' + commandsMethods.join('\n\n') + '\n' : ''}
    private void stopAllSubsystems() {
${stops.length ? stops.join('\n') : '        // No subsystem actions in this route.'}
    }
${hooks.length ? '\n    // Completion predicates must be sensor-based; never return true unconditionally.\n' + hooks.join('\n\n') + '\n' : ''}}
`
}

export function downloadJava(code: string) {
  const url = URL.createObjectURL(new Blob([code], { type: 'text/x-java' }))
  const anchor = document.createElement('a')
  anchor.href = url
  anchor.download = 'SanaAuto.java'
  anchor.click()
  URL.revokeObjectURL(url)
}
