import { useState } from 'react'
import { clamp } from '../lib/geometry'
import type { Waypoint } from '../types'

function PoseValue({ label, value, max, step, unit, onChange }: {
  label: string
  value: number
  max: number
  step: number
  unit: string
  onChange: (value: number) => void
}) {
  const [draft, setDraft] = useState<string | null>(null)
  const formatted = (number: number) => String(Number(number.toFixed(3)))
  return <label className="pose-readout-value">
    <span>{label}</span>
    <div>
      <input
        aria-label={`Selected point ${label}`}
        title="Type a value or use ↑/↓. Hold Shift for 10× steps."
        type="number" min={0} max={max} step={step}
        value={draft ?? formatted(value)}
        onChange={event => {
          const text = event.target.value
          setDraft(text)
          if (text.trim() && Number.isFinite(Number(text))) onChange(clamp(Number(text), 0, max))
        }}
        onBlur={() => setDraft(null)}
        onKeyDown={event => {
          if (event.key === 'Enter') { event.currentTarget.blur(); return }
          if (event.key !== 'ArrowUp' && event.key !== 'ArrowDown') return
          event.preventDefault()
          const parsed = draft?.trim() ? Number(draft) : value
          const current = clamp(Number.isFinite(parsed) ? parsed : value, 0, max)
          const next = clamp(current + (event.key === 'ArrowUp' ? 1 : -1) * step * (event.shiftKey ? 10 : 1), 0, max)
          setDraft(formatted(next))
          onChange(Number(next.toFixed(3)))
        }}
      />
      <small>{unit}</small>
    </div>
  </label>
}

export function PoseReadout({ point, index, onChange }: {
  point: Waypoint
  index: number
  onChange: (changes: Partial<Waypoint>) => void
}) {
  return <div className="coordinate-readout pose-readout" role="group" aria-label="Selected point pose">
    <div className="pose-readout-heading"><span>Point {index + 1}</span><small>↑/↓ adjust · Shift 10×</small></div>
    <div className="pose-readout-fields">
      <PoseValue label="X" value={point.x} max={144} step={.1} unit="in" onChange={x => onChange({ x })} />
      <PoseValue label="Y" value={point.y} max={144} step={.1} unit="in" onChange={y => onChange({ y })} />
      <PoseValue label="Rotation" value={point.heading} max={360} step={1} unit="°" onChange={heading => onChange({ heading })} />
    </div>
  </div>
}
