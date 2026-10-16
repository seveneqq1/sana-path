import type { PathActionMarker, Point2D, SegmentDecision, Waypoint } from '../types'
import { bezierAt, clamp, distance, normalizeDegrees, segmentPoints, shortestAngle, tangentDegrees } from './geometry'
import { curveArc } from './curveFit'

// De Casteljau subdivision keeps arbitrary-degree Pedro curves exactly intact.
export function splitBezier(polygon: Point2D[], t: number): [Point2D[], Point2D[]] {
  let row = polygon.map(p => ({ x: p.x, y: p.y }))
  const left = [row[0]], right = [row.at(-1)!]
  while (row.length > 1) {
    row = row.slice(1).map((p, i) => ({ x: row[i].x + (p.x - row[i].x) * t, y: row[i].y + (p.y - row[i].y) * t }))
    left.push(row[0]); right.unshift(row.at(-1)!)
  }
  return [left, right]
}

export function nearestPathPosition(points: Waypoint[], target: Point2D, onlySegment?: number) {
  let best = { segment: onlySegment ?? 0, position: 0, distance: Infinity }
  for (let segment = 0; segment < points.length - 1; segment++) {
    if (onlySegment !== undefined && segment !== onlySegment) continue
    const polygon = segmentPoints(points, segment)
    let t = 0, error = Infinity
    for (let i = 0; i <= 160; i++) {
      const d = distance(target, bezierAt(polygon, i / 160))
      if (d < error) { error = d; t = i / 160 }
    }
    let lo = Math.max(0, t - 1 / 160), hi = Math.min(1, t + 1 / 160)
    for (let i = 0; i < 28; i++) {
      const a = lo + (hi - lo) / 3, b = hi - (hi - lo) / 3
      if (distance(target, bezierAt(polygon, a)) < distance(target, bezierAt(polygon, b))) hi = b
      else lo = a
    }
    const refined = (lo + hi) / 2, refinedError = distance(target, bezierAt(polygon, refined))
    if (refinedError < error) { t = refined; error = refinedError }
    if (error < best.distance) best = { segment, position: t, distance: error }
  }
  return best
}

export function movePathAction(points: Waypoint[], id: string, segment: number, position: number): Waypoint[] {
  const marker = points.flatMap(p => p.pathActions ?? []).find(a => a.id === id)
  if (!marker || segment < 0 || segment >= points.length - 1) return points
  return points.map((p, i) => ({ ...p, pathActions: i === segment && p.pathActions?.some(a=>a.id===id)
    ? p.pathActions.map(a=>a.id===id?{...a,position:clamp(position,0,1)}:a) : [
    ...(p.pathActions ?? []).filter(a => a.id !== id),
    ...(i === segment ? [{ ...marker, position: clamp(position, 0, 1) }] : []),
  ] }))
}

export function pathActionMarkers(points: Waypoint[]) {
  return points.slice(0, -1).flatMap((p, segment) => (p.pathActions ?? []).map(marker => ({
    ...marker, segment, point: bezierAt(segmentPoints(points, segment), marker.position),
  })))
}

// The editor's anchors are never changed. Only execution/export use these exact
// subdivisions; markers at the same position share one stop and command bundle.
export function materializePathActions(points: Waypoint[], decisions: SegmentDecision[]) {
  if (!points.some(p => p.pathActions?.length)) return { points, decisions, segments: points.slice(1).map((_, i) => i) }
  const clean = (p: Waypoint): Waypoint => ({ ...p, pathActions: undefined, actionQueue: undefined, curve: undefined, controlPoints: undefined })
  const out = points.length ? [clean(points[0])] : []
  const resolved: SegmentDecision[] = [], segments: number[] = []
  const append = (point: Waypoint, markers: PathActionMarker[]) => {
    const actions = [...(point.action ? [point.action] : []), ...(point.actionQueue ?? []), ...markers.map(m => ({ ...m.action, anchored: true }))]
    point.action = actions[0]; point.actionQueue = actions.slice(1)
  }
  for (let i = 0; i < points.length - 1; i++) {
    const polygon = segmentPoints(points, i), arc = curveArc(...polygon), decision = decisions[i]
    const markers = (points[i].pathActions ?? []).map(m => ({ ...m, position: clamp(m.position, 0, 1) }))
    append(out.at(-1)!, markers.filter(m => m.position === 0))
    const join = arc.parameterAt(.68)
    const cuts = [...new Set([...markers.filter(m => m.position > 0 && m.position < 1).map(m => m.position), ...(decision.type === 'piecewise' ? [join] : []), 1])].sort((a, b) => a - b)
    const tangent = (t: number) => tangentDegrees(bezierAt(polygon, Math.max(0, t - .000001)), bezierAt(polygon, Math.min(1, t + .000001)))
    const heading = (t: number) => decision.type === 'constant' ? points[i].heading
      : decision.type === 'tangent' ? tangent(t)
      : decision.type === 'piecewise' ? t < join ? tangent(t) : tangent(join) + shortestAngle(tangent(join), points[i + 1].heading) * (arc.completionAt(t) - .68) / .32
      : points[i].heading + shortestAngle(points[i].heading, points[i + 1].heading) * arc.completionAt(t)
    let rest = polygon, previous = 0
    for (const t of cuts) {
      const [part, tail] = splitBezier(rest, (t - previous) / (1 - previous))
      const type = decision.type === 'piecewise' ? previous < join - 1e-12 ? 'tangent' : 'linear' : decision.type
      const next: Waypoint = t === 1 ? clean(points[i + 1]) : {
        ...bezierAt(polygon, t), id: `${points[i].id}:action:${t}`, heading: normalizeDegrees(heading(t)), interpolation: type, controlWeight: 1,
      }
      next.interpolation = type
      append(next, markers.filter(m => m.position === t))
      out[out.length - 1] = { ...out.at(-1)!, controlPoints: part.slice(1, -1) }
      out.push(next); resolved.push({ ...decision, type }); segments.push(i)
      rest = tail; previous = t
    }
  }
  return { points: out, decisions: resolved, segments }
}
