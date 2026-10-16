export type Interpolation = 'auto' | 'constant' | 'linear' | 'tangent' | 'piecewise'
export type ResolvedInterpolation = Exclude<Interpolation, 'auto'>

export interface Point2D { x: number; y: number }

export type ActionType = 'shoot' | 'intake' | 'transfer' | 'flowerIntake' | 'wait'

export interface PathAction {
  type: ActionType
  durationMs?: number
  composition?: 'sequential' | 'parallel' | 'deadline'
  timeoutMs?: number
  // Internal: a path marker's explicit composition takes precedence over presets.
  anchored?: boolean
}

export interface PathActionMarker { id: string; position: number; action: PathAction }

export interface Waypoint extends Point2D {
  id: string
  heading: number
  interpolation: Interpolation
  controlWeight: number
  headingLocked?:boolean
  action?: PathAction
  pathActions?: PathActionMarker[]
  // Internal commands at a shared, materialized path position.
  actionQueue?: PathAction[]
  curve?: { endId: string; c1: Point2D; c2: Point2D }
  // Pedro-visualizer control points for the path leaving this waypoint.
  // [] is a straight line; when present it overrides `curve` and auto handles.
  controlPoints?: Point2D[]
  autoReason?: string
}

export interface SegmentDecision {
  type: ResolvedInterpolation
  reason: string
  curvature: number
  rotationRate: number
}

export type CanvasTool = 'select' | 'draw' | 'waypoint' | 'pan'
