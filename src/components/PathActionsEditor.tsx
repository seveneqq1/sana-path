import { Plus, Trash2, Zap } from 'lucide-react'
import type { ActionType, PathActionMarker, Waypoint } from '../types'
import { bezierAt, clamp, segmentPoints } from '../lib/geometry'

export function PathActionsEditor({ points, segment, selectedId, onSelect, onChange }: {
  points: Waypoint[]
  segment: number
  selectedId: string | null
  onSelect: (id: string) => void
  onChange: (markers: PathActionMarker[]) => void
}) {
  if (segment < 0 || segment >= points.length - 1) return null
  const markers = points[segment].pathActions ?? []
  const update = (id: string, changes: Partial<PathActionMarker>) => onChange(markers.map(m => m.id === id ? { ...m, ...changes } : m))
  return <div className="field-group path-actions-editor">
    <div className="group-title"><Zap size={15} /><span>Path actions</span><small>segment {segment + 1}</small></div>
    <p className="inspector-help">Add independent markers to this path. Drag their numbered badges along the field, or type a curve position below.</p>
    {markers.map((marker, index) => {
      const position = bezierAt(segmentPoints(points, segment), marker.position)
      return <div key={marker.id} className={`path-action-row${selectedId === marker.id ? ' selected' : ''}`} onFocusCapture={() => onSelect(marker.id)}>
        <div className="path-action-title"><button onClick={() => onSelect(marker.id)} aria-label={`Select action ${index + 1}`}>Action {index + 1}</button><small>X {position.x.toFixed(1)} · Y {position.y.toFixed(1)}</small><button onClick={() => onChange(markers.filter(m => m.id !== marker.id))} aria-label={`Delete action ${index + 1}`}><Trash2 size={13} /></button></div>
        <label>Action<select aria-label={`Action ${index + 1} type`} value={marker.action.type} onChange={e => update(marker.id, { action: { ...marker.action, type: e.target.value as ActionType } })}>
          <option value="shoot">Shoot</option><option value="intake">Intake</option><option value="transfer">Transfer</option><option value="flowerIntake">Flower intake</option><option value="wait">Wait</option>
        </select></label>
        <label>Curve position (%)<input aria-label={`Action ${index + 1} position (%)`} type="number" min={0} max={100} step={.1} value={Number((marker.position * 100).toFixed(3))} onChange={e => { if (e.target.value && Number.isFinite(e.target.valueAsNumber)) update(marker.id, { position: clamp(e.target.valueAsNumber / 100, 0, 1) }) }} /></label>
        <label>Run mode<select aria-label={`Action ${index + 1} mode`} value={marker.action.composition ?? 'sequential'} onChange={e => update(marker.id, { action: { ...marker.action, composition: e.target.value as 'sequential' | 'parallel' | 'deadline' } })}>
          <option value="sequential">Sequential · stop here</option><option value="parallel">Parallel · with next path</option><option value="deadline">Deadline · until next marker</option>
        </select></label>
        {marker.action.type === 'wait'
          ? <label>Wait (ms)<input aria-label={`Action ${index + 1} wait (ms)`} type="number" min={100} step={100} value={marker.action.durationMs ?? 500} onChange={e => update(marker.id, { action: { ...marker.action, durationMs: Math.max(100, Math.round(Number(e.target.value) / 100) * 100) } })} /></label>
          : marker.action.type !== 'intake' && <label>Timeout (ms, 0 = none)<input aria-label={`Action ${index + 1} timeout (ms)`} type="number" min={0} step={100} value={marker.action.timeoutMs ?? 0} onChange={e => update(marker.id, { action: { ...marker.action, timeoutMs: Math.max(0, Math.round(Number(e.target.value) / 100) * 100) } })} /></label>}
      </div>
    })}
    <button className="safe-snap-button" onClick={() => {
      const id = crypto.randomUUID()
      let position = .5
      for (let i = 1; markers.some(m => Math.abs(m.position - position) < .02) && i < 20; i++) position = (i % 2 ? .5 + Math.ceil(i / 2) * .05 : .5 - i / 2 * .05)
      onChange([...markers, { id, position: clamp(position, 0, 1), action: { type: 'intake', composition: 'sequential' } }]); onSelect(id)
    }}><Plus size={14} />Add path action</button>
    {!!markers.length && <p className="inspector-help">Sequential actions at a shared position run first; parallel actions start together with the following path and join before the next marker. At the route end, all actions finish before stopping. Intake stays on until shooting or the route ends. Shared-subsystem actions cannot run together in Ivy.</p>}
  </div>
}
