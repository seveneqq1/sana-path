import { Gauge, Info, MapPin, Minus, Plus, RotateCw, Slash, SlidersHorizontal, Spline, Timer, Trash2, Zap } from 'lucide-react'
import type { ActionType, Interpolation, SegmentDecision, Waypoint } from '../types'
import { clamp } from '../lib/geometry'
import type { ReactNode } from 'react'

interface Props {
  point: Waypoint | null
  index: number
  decision: SegmentDecision | null
  onChange: (changes: Partial<Waypoint>) => void
  onDelete: () => void
  onSnapSafe:()=>void
  snapMessage:string
  shootWhileMoving:boolean
  isLast:boolean
  controlCount:number|null
  onAddControl:()=>void
  onRemoveControl:()=>void
  onStraighten:()=>void
  onAlignFlower:()=>void
  pathActionsEditor?: ReactNode
  onMoveCommandToPath: () => void
}

const interpolationOptions: { value: Interpolation; label: string }[] = [
  { value: 'auto', label: 'Auto select' },
  { value: 'linear', label: 'Linear' },
  { value: 'constant', label: 'Constant' },
  { value: 'tangent', label: 'Tangent' },
  { value: 'piecewise', label: 'Piecewise' },
]

export function Inspector({ point, index, decision, onChange, onDelete, onSnapSafe,snapMessage,shootWhileMoving,isLast,controlCount,onAddControl,onRemoveControl,onStraighten,onAlignFlower,pathActionsEditor,onMoveCommandToPath }: Props) {
  if (!point) return <div className="inspector-empty">
    <div className="empty-orbit"><MapPin size={23} /></div>
    <h3>No waypoint selected</h3>
    <p>Select a node on the field to tune its pose and arrival heading.</p>
  </div>

  const numberChange = (key: 'x' | 'y' | 'heading' | 'controlWeight', value: string) => {
    const parsed = Number(value)
    if (!Number.isFinite(parsed)) return
    const safe = key === 'heading' ? clamp(parsed, 0, 360)
      : key === 'controlWeight' ? clamp(parsed, .25, 1.75)
      : clamp(parsed, 0, 144)
    onChange({ [key]: safe })
  }

  return <div className="inspector-content">
    <div className="inspector-heading">
      <div><span>Waypoint {index + 1}</span><h2>Pose inspector</h2></div>
      <button className="icon-danger" onClick={onDelete} aria-label="Delete waypoint" title="Delete waypoint"><Trash2 size={17} /></button>
    </div>

    <div className="field-group command-inspector">
      <div className="group-title"><Zap size={15} /><span>Route command</span><small>runs after arrival</small></div>
      <select aria-label="Route command" value={point.action?.type ?? 'none'} onChange={event => {
        const type = event.target.value as ActionType | 'none'
        onChange({ action: type === 'none' ? undefined : { type, ...(type === 'wait' ? { durationMs: 500 } : {}) } })
      }}>
        <option value="none">No command</option>
        <option value="shoot">1 · Shoot</option>
        <option value="intake">2 · Intake</option>
        <option value="transfer">3 · Transfer</option>
        <option value="flowerIntake">4 · Flower intake</option>
        <option value="wait">5 · Wait</option>
      </select>
      {point.action?.type === 'wait' && <label className="wait-input"><Timer size={14} /><input aria-label="Wait duration" type="number" min="100" step="100" value={point.action.durationMs ?? 100} onChange={event => onChange({ action: { ...point.action!, durationMs: Math.max(100, Math.round(Number(event.target.value) / 100) * 100) } })} /><span>ms</span></label>}
      {point.action && <>
        <button className="safe-snap-button" onClick={onMoveCommandToPath}><Zap size={14}/>Make draggable path action</button>
        <label className="command-label">Ivy composition
          <select aria-label="Ivy composition" value={point.action.type==='shoot'&&shootWhileMoving?point.action.composition==='deadline'?'deadline':'parallel':point.action.composition ?? 'sequential'} onChange={event => onChange({action: {...point.action!, composition: event.target.value as 'sequential' | 'parallel' | 'deadline'}})}>
            <option value="sequential" disabled={point.action.type==='shoot'&&shootWhileMoving}>Sequential · then drive</option>
            <option value="parallel">Parallel · with next path</option>
            <option value="deadline">Deadline · until next path ends</option>
          </select>
        </label>
        {point.action.type==='shoot'&&shootWhileMoving&&<p>Shoot while moving is on. This command runs alongside the next path; turn the toggle off for a stopped shot.</p>}
        {point.action.type==='flowerIntake'&&<button className="safe-snap-button" onClick={onAlignFlower}><MapPin size={14}/>Align to nearest flower <small>face it, mouth on the pollen</small></button>}
        {point.action.type==='flowerIntake'&&<p>Removing pollen from a flower does not award points. Flower scoring is an end-of-match achievement, not AUTO scoring.</p>}
        {point.action.type !== 'wait' && point.action.type !== 'intake' && <label className="command-label">Timeout race (0 = none)
          <div className="wait-input"><Timer size={14}/><input aria-label="Command timeout" type="number" min="0" step="100" value={point.action.timeoutMs ?? 0} onChange={event => onChange({action: {...point.action!, timeoutMs: Math.max(0, Math.round(Number(event.target.value) / 100) * 100)}})}/><span>ms</span></div>
        </label>}
        <p>{point.action.composition === 'parallel' ? 'Starts here alongside the next path. Both must finish before the next command.' : point.action.composition === 'deadline' ? 'The next path is the deadline; unfinished actions are cancelled and cleaned up.' : point.action.type === 'intake' ? 'Instant command enables intake, then driving continues. Intake stays on until shooting or the end.' : 'The next path starts after this command finishes.'} Concurrent modes on the final waypoint run sequentially.</p>
      </>}
    </div>

    {pathActionsEditor}
    <div className="field-group">
      <div className="group-title"><MapPin size={15} /><span>Field position</span><small>inches</small></div>
      <div className="input-grid">
        <label><span>X</span><input type="number" min="0" max="144" step="0.1" value={Number(point.x.toFixed(1))} onChange={e => numberChange('x', e.target.value)} /></label>
        <label><span>Y</span><input type="number" min="0" max="144" step="0.1" value={Number(point.y.toFixed(1))} onChange={e => numberChange('y', e.target.value)} /></label>
      </div>
      <button className="safe-snap-button" onClick={onSnapSafe}><MapPin size={14}/>Snap to nearest safe spot <small>+0.5 in buffer</small></button>
      {snapMessage&&<p className="inspector-help" role="status">{snapMessage}</p>}
    </div>

    <div className="field-group">
      <div className="group-title"><RotateCw size={15} /><span>Target heading</span><small>degrees</small></div>
      <div className="heading-control">
        <input className="heading-range" type="range" min="0" max="360" value={point.heading} onChange={e => numberChange('heading', e.target.value)} />
        <label><input type="number" min="0" max="360" value={Number(point.heading.toFixed(1))} onChange={e => numberChange('heading', e.target.value)} /><span>°</span></label>
      </div>
      <div className="heading-compass" aria-hidden="true">
        {[0, 90, 180, 270].map(deg => <i key={deg} style={{ transform: `rotate(${-deg}deg)` }}>{deg}°</i>)}
      </div>
      <p className="inspector-help">Heading is independent of the curve. Editing it switches adjacent segments to linear headings. In 3D, use A/D or ←/→, W/S for ±90°, then Enter to apply.</p>
    </div>

    <div className="field-group">
      <div className="group-title"><SlidersHorizontal size={15} /><span>Arrival interpolation</span>{index === 0 && <small>start node</small>}</div>
      <select value={point.interpolation} disabled={index === 0} onChange={e => onChange({ interpolation: e.target.value as Interpolation, autoReason: undefined })}>
        {interpolationOptions.map(option => <option key={option.value} value={option.value}>{option.label}</option>)}
      </select>
      {index > 0 && decision && <div className={`decision-note ${point.interpolation !== 'auto' ? 'manual' : ''}`}>
        <Info size={15} />
        <div><strong>{point.interpolation === 'auto' ? `Optimizer chose ${decision.type}` : point.autoReason ? `Simulator chose ${decision.type}` : `${decision.type} override`}</strong><p>{decision.reason}</p></div>
      </div>}
    </div>

    {!isLast && <div className="field-group">
      <div className="group-title"><Spline size={15} /><span>Path to next point</span><small>{controlCount === null ? 'auto handles' : controlCount === 0 ? 'straight line' : `${controlCount} control point${controlCount === 1 ? '' : 's'}`}</small></div>
      <div className="control-buttons">
        <button onClick={onAddControl}><Plus size={14} />Add control</button>
        <button onClick={onRemoveControl} disabled={controlCount === 0}><Minus size={14} />Remove</button>
        <button onClick={onStraighten} disabled={controlCount === 0}><Slash size={14} />Straight</button>
      </div>
      <p className="inspector-help">Like the Pedro visualizer: drag the square control points on the field to bend this path. Double-click a control point to delete it.</p>
    </div>}

    {controlCount === null && !isLast && <div className="field-group">
      <div className="group-title"><Gauge size={15} /><span>Bézier tension</span><small>{point.controlWeight.toFixed(2)}×</small></div>
      <input className="weight-range" type="range" min="0.25" max="1.75" step="0.05" value={point.controlWeight} onChange={e => numberChange('controlWeight', e.target.value)} />
      <div className="range-labels"><span>Tighter</span><span>Longer handles</span></div>
    </div>}
  </div>
}
