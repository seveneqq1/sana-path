import type { Point2D, ResolvedInterpolation, Waypoint } from '../types'
import { bezierAt, distance, segmentPoints, shortestAngle, tangentDegrees } from './geometry'
import { bufferedPoseSafe, nearestSafePose } from './safeSpot'
import { flowers, profile } from './simulation'
import type { RobotConfig } from './simulation'

export type Closeness = 'close' | 'balanced' | 'fast'
export const closenessOptions: { value: Closeness; label: string }[] = [
  { value: 'close', label: 'Follow my drawing' },
  { value: 'balanced', label: 'Balanced' },
  { value: 'fast', label: 'Fastest' },
]
// tolerance: preferred max deviation (in); deviation: seconds of cost per inch of mean deviation.
const SETTINGS: Record<Closeness, { tolerance: number; deviation: number }> = {
  close: { tolerance: 1.5, deviation: .4 },
  balanced: { tolerance: 3, deviation: .12 },
  fast: { tolerance: 10, deviation: .03 },
}
export const BUFFER = .5
const MAX_CONTROLS = 5

const binomial = (n: number, k: number) => { let r = 1; for (let i = 1; i <= k; i++) r = r * (n - k + i) / i; return r }
const bernstein = (n: number, i: number, t: number) => binomial(n, i) * t ** i * (1 - t) ** (n - i)

function solve(matrix: number[][], rhs: number[]) {
  const n = rhs.length, a = matrix.map((row, i) => [...row, rhs[i]])
  for (let c = 0; c < n; c++) {
    let pivot = c
    for (let r = c + 1; r < n; r++) if (Math.abs(a[r][c]) > Math.abs(a[pivot][c])) pivot = r;
    [a[c], a[pivot]] = [a[pivot], a[c]]
    const d = a[c][c] || 1e-12
    for (let r = 0; r < n; r++) if (r !== c) { const f = a[r][c] / d; for (let k = c; k <= n; k++) a[r][k] -= f * a[c][k] }
  }
  return a.map((row, i) => row[n] / (row[i] || 1e-12))
}

function resample(poly: Point2D[], count: number) {
  const lengths = [0]
  for (let i = 1; i < poly.length; i++) lengths.push(lengths[i - 1] + distance(poly[i - 1], poly[i]))
  const total = lengths.at(-1) || 1, out: Point2D[] = []
  let k = 0
  for (let j = 0; j < count; j++) {
    const target = total * j / (count - 1)
    while (k < poly.length - 2 && lengths[k + 1] < target) k++
    const span = lengths[k + 1] - lengths[k] || 1, f = Math.min(1, Math.max(0, (target - lengths[k]) / span))
    out.push({ x: poly[k].x + (poly[k + 1].x - poly[k].x) * f, y: poly[k].y + (poly[k + 1].y - poly[k].y) * f })
  }
  return out
}

function segmentDistance(p: Point2D, a: Point2D, b: Point2D) {
  const dx = b.x - a.x, dy = b.y - a.y, t = Math.max(0, Math.min(1, ((p.x - a.x) * dx + (p.y - a.y) * dy) / (dx * dx + dy * dy || 1)))
  return Math.hypot(p.x - a.x - t * dx, p.y - a.y - t * dy)
}
const polylineDistance = (p: Point2D, poly: Point2D[]) => {
  let best = Infinity
  for (let i = 1; i < poly.length; i++) best = Math.min(best, segmentDistance(p, poly[i - 1], poly[i]))
  return best
}

// Least-squares Bézier with fixed endpoints and `k` free control points,
// refined by re-projecting each sample onto the curve (Schneider-style).
function fitBezier(samples: Point2D[], start: Point2D, end: Point2D, k: number): Point2D[] {
  if (k === 0) return [start, end]
  const n = k + 1
  const lengths = [0]
  for (let i = 1; i < samples.length; i++) lengths.push(lengths[i - 1] + distance(samples[i - 1], samples[i]))
  let ts = lengths.map(l => l / (lengths.at(-1) || 1))
  let ctrl: Point2D[] = []
  for (let round = 0; round < 4; round++) {
    const ata = Array.from({ length: k }, () => Array(k).fill(0)), bx = Array(k).fill(0), by = Array(k).fill(0)
    ts.forEach((t, j) => {
      const w = Array.from({ length: k }, (_, i) => bernstein(n, i + 1, t))
      const b0 = bernstein(n, 0, t), bn = bernstein(n, n, t)
      const rx = samples[j].x - b0 * start.x - bn * end.x, ry = samples[j].y - b0 * start.y - bn * end.y
      for (let r = 0; r < k; r++) { bx[r] += w[r] * rx; by[r] += w[r] * ry; for (let c = 0; c < k; c++) ata[r][c] += w[r] * w[c] }
    })
    for (let i = 0; i < k; i++) ata[i][i] += 1e-6
    const xs = solve(ata, bx), ys = solve(ata.map(row => [...row]), by)
    ctrl = [start, ...xs.map((x, i) => ({ x, y: ys[i] })), end]
    const dense = Array.from({ length: 201 }, (_, i) => bezierAt(ctrl, i / 200))
    ts = samples.map((p, j) => {
      if (j === 0) return 0
      if (j === samples.length - 1) return 1
      let best = 0, bestD = Infinity
      dense.forEach((q, i) => { const d = (q.x - p.x) ** 2 + (q.y - p.y) ** 2; if (d < bestD) { bestD = d; best = i } })
      return best / 200
    })
  }
  return ctrl
}

const headingAt = (h0: number, h1: number, t: number) => h0 + shortestAngle(h0, h1) * t
function collisions(ctrl: Point2D[], h0: number, h1: number, size: number, count = 48) {
  const hits: { t: number; pose: Point2D & { heading: number } }[] = []
  for (let j = 1; j < count; j++) {
    const t = j / count, p = bezierAt(ctrl, t), pose = { ...p, heading: headingAt(h0, h1, t) }
    if (!bufferedPoseSafe(pose, size, BUFFER)) hits.push({ t, pose })
  }
  return hits
}

// Push the curve out of walls / hive supports. Each step moves the control
// points by the minimum-norm amount that relocates the worst hit to its
// nearest safe pose; Bézier curves are linear in their control points.
function repair(input: Point2D[], h0: number, h1: number, size: number) {
  const ctrl = input.map(p => ({ ...p })), n = ctrl.length - 1
  if (n < 2) return { ctrl, clear: !collisions(ctrl, h0, h1, size).length }
  for (let iteration = 0; iteration < 60; iteration++) {
    const hits = collisions(ctrl, h0, h1, size).filter(hit => hit.t > .03 && hit.t < .97)
    if (!hits.length) return { ctrl, clear: !collisions(ctrl, h0, h1, size).length }
    const probes = hits.filter((_, i) => i % Math.max(1, Math.ceil(hits.length / 5)) === 0)
    let worst: { t: number; d: Point2D } | null = null
    for (const hit of probes) {
      const safe = nearestSafePose(hit.pose, size, BUFFER + .5)
      if (!safe) continue
      const d = { x: (safe.x - hit.pose.x) * 1.15, y: (safe.y - hit.pose.y) * 1.15 }
      if (!worst || Math.hypot(d.x, d.y) > Math.hypot(worst.d.x, worst.d.y)) worst = { t: hit.t, d }
    }
    if (!worst) break
    const weights = Array.from({ length: n - 1 }, (_, i) => bernstein(n, i + 1, worst!.t))
    const norm = weights.reduce((sum, w) => sum + w * w, 0) || 1
    weights.forEach((w, i) => {
      let dx = w / norm * worst!.d.x, dy = w / norm * worst!.d.y
      const m = Math.hypot(dx, dy)
      if (m > 15) { dx *= 15 / m; dy *= 15 / m }
      ctrl[i + 1].x += dx; ctrl[i + 1].y += dy
    })
  }
  return { ctrl, clear: !collisions(ctrl, h0, h1, size).length }
}

function deviation(ctrl: Point2D[], intent: Point2D[]) {
  const curve = Array.from({ length: 61 }, (_, i) => bezierAt(ctrl, i / 60))
  const there = curve.map(p => polylineDistance(p, intent)), back = intent.map(p => polylineDistance(p, curve))
  const all = [...there, ...back]
  return { mean: all.reduce((s, d) => s + d, 0) / all.length, max: Math.max(...all) }
}

// Flowers sit against the walls: drive square to the wall, intake mouth on
// the flower, with the safety buffer between bumper and wall.
export function alignToFlower(point: Waypoint, size: number): Waypoint {
  const flower = flowers.reduce((a, b) => distance(a, point) <= distance(b, point) ? a : b)
  const wallX = flower.x < 10 ? 0 : flower.x > 134 ? 144 : null
  const offset = size / 2 + BUFFER
  const pose = wallX !== null
    ? { x: wallX === 0 ? offset : 144 - offset, y: flower.y, heading: wallX === 0 ? 180 : 0 }
    : { x: flower.x, y: flower.y > 72 ? 144 - offset : offset, heading: flower.y > 72 ? 90 : 270 }
  return { ...point, ...pose, headingLocked: true }
}

const isFlower = (p: Waypoint) => p.action?.type === 'flowerIntake'

function snapSafe(point: Waypoint, size: number): Waypoint {
  if (isFlower(point)) return alignToFlower(point, size)
  if (bufferedPoseSafe(point, size, BUFFER)) return point
  const safe = nearestSafePose(point, size, BUFFER)
  return safe ? { ...point, x: safe.x, y: safe.y } : point
}

export interface BuildReport { sections: number; controls: number; before: number; after: number; clear: boolean; drive: number; snapped: number }

// Pedro-visualizer structure: one Bézier path between consecutive major
// points (start, command markers, user-locked headings, end).
export function autoBuild(points: Waypoint[], config: RobotConfig, closeness: Closeness = 'balanced'): { points: Waypoint[]; report: BuildReport } {
  const settings = SETTINGS[closeness], size = config.size
  if (points.length < 2) return { points, report: { sections: 0, controls: 0, before: points.length, after: points.length, clear: true, drive: 0, snapped: 0 } }
  const majors = points.flatMap((p, i) => i === 0 || i === points.length - 1 || p.action || p.headingLocked || p.pathActions?.length || points[i-1]?.pathActions?.length ? [i] : [])
  let snapped = 0, clear = true
  const snap = (p: Waypoint) => { const s = snapSafe(p, size); if (distance(s, p) > 1e-6) snapped++; return { ...s, curve: undefined, controlPoints: undefined } }
  const out: Waypoint[] = [snap(points[0])]
  for (let s = 1; s < majors.length; s++) {
    const first = majors[s - 1], last = majors[s]
    const raw: Point2D[] = []
    for (let i = first; i < last; i++) { const pts = segmentPoints(points, i); for (let j = 0; j < 40; j++) raw.push(bezierAt(pts, j / 40)) }
    raw.push(points[last])
    const intent = resample(raw, 60)
    const start = out.at(-1)!, end = snap(points[last])
    intent[0] = start; intent[intent.length - 1] = end
    const straightish = deviation([start, end], intent).max < .75
    type Candidate = { ctrl: Point2D[]; clear: boolean; mean: number; max: number; time: number; score: number }
    const candidates: Candidate[] = []
    for (let k = 0; k <= MAX_CONTROLS; k++) {
      if (k > 0 && straightish && candidates[0]?.clear) break
      const fitted = fitBezier(intent, start, end, k)
      const { ctrl, clear: ok } = repair(fitted, start.heading, end.heading, size)
      const dev = deviation(ctrl, intent)
      const trial: Waypoint[] = [{ ...start, controlPoints: ctrl.slice(1, -1) }, { ...end, interpolation: 'linear' }]
      const time = profile(trial, config).drive
      candidates.push({ ctrl, clear: ok, ...dev, time, score: time + settings.deviation * dev.mean + .05 * k })
      if (ok && dev.max <= settings.tolerance * .35) break
    }
    const feasible = candidates.filter(c => c.clear)
    const close = feasible.filter(c => c.max <= settings.tolerance)
    const pool = close.length ? close : feasible.length ? feasible : candidates
    const best = pool.reduce((a, b) => (feasible.length ? a.score <= b.score : collisions(a.ctrl, start.heading, end.heading, size).length <= collisions(b.ctrl, start.heading, end.heading, size).length) ? a : b)
    if (!best.clear) clear = false
    out[out.length - 1] = { ...start, controlPoints: best.ctrl.slice(1, -1).map(p => ({ x: +p.x.toFixed(3), y: +p.y.toFixed(3) })) }
    out.push(end)
  }
  const final = bestHeadings(out, config)
  const result = profile(final, config)
  return { points: final, report: {
    sections: majors.length - 1, before: points.length, after: final.length, snapped,
    controls: final.reduce((sum, p) => sum + (p.controlPoints?.length ?? 0), 0),
    clear: clear && !result.wallCollision && !result.supportCollision, drive: result.drive,
  } }
}

const labels: Record<ResolvedInterpolation, string> = { constant: 'Constant', linear: 'Linear', tangent: 'Tangent', piecewise: 'Piecewise' }

// Choose each segment's heading interpolation by simulating every valid
// option with the robot's drive/turn limits and footprint collisions.
export function bestHeadings(points: Waypoint[], config: RobotConfig): Waypoint[] {
  const next = points.map(p => ({ ...p }))
  for (let i = 1; i < next.length; i++) {
    const a = next[i - 1], b = next[i], pts = segmentPoints(next, i - 1)
    const startTangent = tangentDegrees(bezierAt(pts, 0), bezierAt(pts, .02)), endTangent = tangentDegrees(bezierAt(pts, .98), bezierAt(pts, 1))
    const turn = Math.abs(shortestAngle(a.heading, b.heading))
    const startAligned = Math.abs(shortestAngle(a.heading, startTangent)) < 12, endAligned = Math.abs(shortestAngle(b.heading, endTangent)) < 12
    const options: ResolvedInterpolation[] = ['linear']
    if (turn < 3) options.push('constant')
    if (startAligned && endAligned) options.push('tangent')
    else if (startAligned && Math.abs(shortestAngle(startTangent, endTangent)) > 20) options.push('piecewise')
    const results = options.map(type => {
      const run = profile([{ ...a, interpolation: 'auto' }, { ...b, interpolation: type }], config)
      return { type, time: run.drive, clear: !run.wallCollision && !run.supportCollision }
    })
    const preference = (r: typeof results[number]) => r.time - (r.type === 'tangent' ? .03 : r.type === 'constant' ? .02 : 0)
    const clearResults = results.filter(r => r.clear)
    const pick = (clearResults.length ? clearResults : results).reduce((x, y) => preference(x) <= preference(y) ? x : y)
    const others = results.filter(r => r !== pick).map(r => `${labels[r.type].toLowerCase()} ${r.time.toFixed(2)} s${r.clear ? '' : ' (hits obstacle)'}`)
    next[i] = { ...b, interpolation: pick.type, autoReason: `${pick.clear ? 'Fastest collision-free' : 'Best available (still touches an obstacle)'} heading: ${labels[pick.type].toLowerCase()} ${pick.time.toFixed(2)} s${others.length ? ` vs ${others.join(', ')}` : ''}.` }
  }
  return next
}

// Keep the route's structure: move unsafe points to the nearest safe pose,
// then bend any still-unsafe path with its control points.
export function makeSafe(points: Waypoint[], config: RobotConfig): { points: Waypoint[]; moved: number; clear: boolean } {
  let moved = 0
  const next = points.map(p => { const s = snapSafe(p, config.size); if (distance(s, p) > 1e-6) moved++; return s })
  for (let i = 0; i < next.length - 1; i++) {
    const a = next[i], b = next[i + 1]
    if (!collisions(segmentPoints(next, i), a.heading, b.heading, config.size).length) continue
    let pts = segmentPoints(next, i)
    if (pts.length < 4) pts = fitBezier(Array.from({ length: 30 }, (_, j) => bezierAt(pts, j / 29)), a, b, 2)
    const { ctrl } = repair(pts, a.heading, b.heading, config.size)
    next[i] = { ...a, curve: undefined, controlPoints: ctrl.slice(1, -1) }
  }
  const run = profile(next, config)
  return { points: next, moved, clear: !run.wallCollision && !run.supportCollision }
}

// Pedro-visualizer style editing helpers for the segment leaving `index`.
export function addControlPoint(points: Waypoint[], index: number): Waypoint[] {
  if (index < 0 || index >= points.length - 1) return points
  const pts = segmentPoints(points, index), n = pts.length - 1
  // Degree elevation keeps the curve shape while adding a handle.
  const elevated = Array.from({ length: n + 2 }, (_, i) => i === 0 ? pts[0] : i === n + 1 ? pts[n] : {
    x: i / (n + 1) * pts[i - 1].x + (1 - i / (n + 1)) * pts[i].x,
    y: i / (n + 1) * pts[i - 1].y + (1 - i / (n + 1)) * pts[i].y,
  })
  return points.map((p, i) => i === index ? { ...p, curve: undefined, controlPoints: elevated.slice(1, -1) } : p)
}

export function removeControlPoint(points: Waypoint[], index: number): Waypoint[] {
  if (index < 0 || index >= points.length - 1) return points
  const inner = segmentPoints(points, index).slice(1, -1)
  if (!inner.length) return points
  const samples = Array.from({ length: 40 }, (_, i) => bezierAt(segmentPoints(points, index), i / 39))
  const ctrl = fitBezier(samples, points[index], points[index + 1], inner.length - 1)
  return points.map((p, i) => i === index ? { ...p, curve: undefined, controlPoints: ctrl.slice(1, -1) } : p)
}

export function setControlPoints(points: Waypoint[], index: number, controlPoints: Point2D[]): Waypoint[] {
  return points.map((p, i) => i === index ? { ...p, curve: undefined, controlPoints } : p)
}
