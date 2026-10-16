import type { PathAction, Waypoint } from '../types'

export interface RoutePath { kind: 'path'; name: string; startIndex: number; endIndex: number }
export type RouteStep = RoutePath | { kind: 'action'; action: PathAction; node: number }
  | { kind: 'group'; action: PathAction; additionalActions?: PathAction[]; node: number; mode: 'parallel' | 'deadline'; path: RoutePath }

export const nodeActions = (point: Waypoint) => [...(point.action ? [point.action] : []), ...(point.actionQueue ?? [])]
export function actionResourceConflict(actions: PathAction[]) {
  const resources: Record<PathAction['type'], string[]> = { shoot:['shooter','feeder','turret'], transfer:['feeder'], intake:['intake'], flowerIntake:['intake'], wait:[] }
  const used=new Set<string>()
  for(const action of actions)for(const resource of resources[action.type]){
    if(used.has(resource))return `Concurrent actions share the ${resource} at the same position. Move one marker or set it to Sequential.`
    used.add(resource)
  }
  return null
}
export const effectiveAction = (action: PathAction, node: number, count: number, moving: boolean): PathAction =>
  action.type === 'shoot' && moving && !action.anchored && node < count - 1
    ? { ...action, composition: action.composition === 'deadline' ? 'deadline' : 'parallel' } : action
export const actionPlan = (point: Waypoint, node: number, count: number, moving = false) => {
  const actions = nodeActions(point).map(a => effectiveAction(a, node, count, moving))
  return {
    sequential: actions.filter(a => !a.composition || a.composition === 'sequential' || node === count - 1),
    concurrent: node < count - 1 ? actions.filter(a => a.composition && a.composition !== 'sequential') : [],
  }
}

// The next command marker is a join barrier, preventing silent action overlap.
export function nextPathEnd(points: Waypoint[], node: number) {
  for (let i = node + 1; i < points.length; i++) if (nodeActions(points[i]).length) return i
  return points.length - 1
}

export const routeActions=(points:Waypoint[],shootWhileMoving=false)=>points.map((point,node)=>
  point.action ? effectiveAction(point.action,node,points.length,shootWhileMoving) : undefined)

export function compileRoute(points: Waypoint[],shootWhileMoving=false): RouteStep[] {
  const raw: (RoutePath | Extract<RouteStep, {kind: 'action'}>)[] = []
  let start = 0, number = 0
  points.forEach((point, node) => {
    const plan = actionPlan(point, node, points.length, shootWhileMoving)
    if (!plan.sequential.length && !plan.concurrent.length) return
    if (node > start) raw.push({kind: 'path', name: `path${number++}`, startIndex: start, endIndex: node})
    for (const action of [...plan.sequential, ...plan.concurrent]) raw.push({kind: 'action', action, node})
    start = node
  })
  if (start < points.length - 1) raw.push({kind: 'path', name: `path${number++}`, startIndex: start, endIndex: points.length - 1})
  const steps: RouteStep[] = []
  for (let i = 0; i < raw.length; i++) {
    const step = raw[i]
    const together: PathAction[] = []
    let end = i
    if (step.kind === 'action' && step.action.composition && step.action.composition !== 'sequential') {
      while (raw[end]?.kind === 'action') {
        const item = raw[end]
        if (item.kind !== 'action' || item.node !== step.node || !item.action.composition || item.action.composition === 'sequential') break
        together.push(item.action); end++
      }
    }
    const next = raw[end]
    if (step.kind === 'action' && step.action.composition && step.action.composition !== 'sequential' && next?.kind === 'path') {
      steps.push({kind: 'group', action: step.action, additionalActions: together.slice(1), node: step.node, mode: step.action.composition, path: next})
      i = end
    } else steps.push(step)
  }
  return steps
}
