import { useEffect, useMemo, useRef, useState } from 'react'
import type { CSSProperties } from 'react'
import { useSimulation } from './Simulation'
import { PoseReadout } from './PoseReadout'
import { Arrow, Circle, Group, Image, Layer, Line, Rect, Shape, Stage, Text } from 'react-konva'
import type Konva from 'konva'
import { Crosshair, Hand, MousePointer2, PenTool, RotateCcw, ScanLine, ZoomIn, ZoomOut } from 'lucide-react'
import type { ActionType, CanvasTool, PathAction, Point2D, SegmentDecision, Waypoint } from '../types'
import { setControlPoints } from '../lib/smartPath'
import { bezierAt, canvasToWorld, clamp, editWaypoint, segmentPoints, FIELD_PIXELS, normalizeDegrees, simplify, tangentDegrees, worldToCanvas } from '../lib/geometry'
import type { RobotConfig } from '../lib/simulation'
import { clearsWalls } from '../lib/walls'
import { robotSupportCollision } from '../lib/supportCollisions'

interface Props {
  points: Waypoint[]
  decisions: SegmentDecision[]
  selectedId: string | null
  tool: CanvasTool
  snap: boolean
  config:RobotConfig
  onConfigChange:(config:RobotConfig)=>void
  theme:'light'|'dark'
  onToolChange: (tool: CanvasTool) => void
  onSnapChange: (snap: boolean) => void
  onPointsChange: (points: Waypoint[]) => void
  onSelect: (id: string | null) => void
  onStroke: (points: Waypoint[]) => void
}

const toolItems: { id: CanvasTool; label: string; icon: typeof MousePointer2 }[] = [
  { id: 'select', label: 'Select', icon: MousePointer2 },
  { id: 'draw', label: 'Draw', icon: PenTool },
  { id: 'waypoint', label: 'Point', icon: Crosshair },
  { id: 'pan', label: 'Pan', icon: Hand },
]

const actionKeys: { key: string; type: ActionType; label: string; color: string }[] = [
  { key: '1', type: 'shoot', label: 'Shoot', color: '#ff6f77' },
  { key: '2', type: 'intake', label: 'Intake', color: '#6ee7f2' },
  { key: '3', type: 'transfer', label: 'Transfer', color: '#b69cff' },
  { key: '4', type: 'flowerIntake', label: 'Flower', color: '#f2c94c' },
  { key: '5', type: 'wait', label: 'Hold: wait', color: '#ffffff' },
]

const actionMeta = Object.fromEntries(actionKeys.map(action => [action.type, action])) as Record<ActionType, typeof actionKeys[number]>
interface PendingAction { point: Point2D; sampleIndex: number; action: PathAction }

export function FieldCanvas({ points, decisions, selectedId, tool, snap, config,onConfigChange,theme,onToolChange, onSnapChange, onPointsChange, onSelect, onStroke }: Props) {
  const wrapRef = useRef<HTMLDivElement>(null)
  const simulation=useSimulation(points,config,onConfigChange,theme,{selectedId,onSelect,onPointsChange})
  const supportMarker=simulation.route.supportCollision?worldToCanvas(simulation.route.supportCollision.sample):null
  const stageRef = useRef<Konva.Stage>(null)
  const [size, setSize] = useState({ width: 760, height: 720 })
  const [scale, setScale] = useState(0.86)
  const [position, setPosition] = useState({ x: 32, y: 28 })
  const [cursor, setCursor] = useState<Point2D>({ x: 0, y: 0 })
  const [stroke, setStroke] = useState<Point2D[]>([])
  const [drawing, setDrawing] = useState(false)
  const [pendingActions, setPendingActions] = useState<PendingAction[]>([])
  const [activeWait, setActiveWait] = useState(false)
  const [actionFlash, setActionFlash] = useState<string | null>(null)
  const [fieldImage, setFieldImage] = useState<HTMLImageElement | null>(null)
  const strokeRef = useRef<Point2D[]>([])
  const drawingRef = useRef(false)
  const pendingActionsRef = useRef<PendingAction[]>([])
  const waitStartRef = useRef<{ startedAt: number; point: Point2D; sampleIndex: number } | null>(null)

  const registerAction = (entry: PendingAction) => {
    pendingActionsRef.current = [...pendingActionsRef.current, entry]
    setPendingActions(pendingActionsRef.current)
    const meta = actionMeta[entry.action.type]
    setActionFlash(entry.action.type === 'wait' ? `Wait ${entry.action.durationMs} ms added` : `${meta.label} added`)
    window.setTimeout(() => setActionFlash(null), 900)
  }

  const finishWait = () => {
    const waiting = waitStartRef.current
    if (!waiting) return
    const durationMs = Math.max(100, Math.round((Date.now() - waiting.startedAt) / 100) * 100)
    registerAction({ point: waiting.point, sampleIndex: waiting.sampleIndex, action: { type: 'wait', durationMs } })
    waitStartRef.current = null
    setActiveWait(false)
  }

  useEffect(() => {
    const editableTarget = (target: EventTarget | null) => target instanceof HTMLInputElement || target instanceof HTMLSelectElement || target instanceof HTMLTextAreaElement
    const keyDown = (event: KeyboardEvent) => {
      if (!drawingRef.current || editableTarget(event.target)) return
      const binding = actionKeys.find(action => action.key === event.key)
      if (!binding || event.repeat) return
      const currentStroke = strokeRef.current
      const point = currentStroke.at(-1)
      if (!point) return
      event.preventDefault()
      if (binding.type === 'wait') {
        waitStartRef.current = { startedAt: Date.now(), point, sampleIndex: currentStroke.length - 1 }
        setActiveWait(true)
      } else {
        registerAction({ point, sampleIndex: currentStroke.length - 1, action: { type: binding.type } })
      }
    }
    const keyUp = (event: KeyboardEvent) => {
      if (event.key === '5' && waitStartRef.current) { event.preventDefault(); finishWait() }
    }
    window.addEventListener('keydown', keyDown)
    window.addEventListener('keyup', keyUp)
    return () => { window.removeEventListener('keydown', keyDown); window.removeEventListener('keyup', keyUp) }
  })

  useEffect(() => {
    const image = new window.Image()
    image.src = '/biobuzz-field.png'
    image.onload = () => setFieldImage(image)
  }, [])

  useEffect(() => {
    if (!wrapRef.current) return
    const element=wrapRef.current
    const fitView=()=>{
      const {width,height}=element.getBoundingClientRect()
      element.style.setProperty('--field-height',`${height}px`)
      setSize({ width, height })
      const dockHeight=element.querySelector('.simulation-ui')?.getBoundingClientRect().height??100
      const available=Math.max(90,height-dockHeight-142)
      const fit=clamp(Math.min((width-56)/FIELD_PIXELS,available/FIELD_PIXELS),.12,1.4)
      setScale(fit)
      setPosition({x:(width-FIELD_PIXELS*fit)/2,y:72+Math.max(0,(available-FIELD_PIXELS*fit)/2)})
    }
    const resize = new ResizeObserver(fitView)
    resize.observe(element)
    // Warning text can resize the dock during a drag; only resize the view
    // when the drawing workspace itself changes, preserving its zoom/pan.
    fitView()
    return () => resize.disconnect()
  }, [])

  const pointerInField = () => {
    const stage = stageRef.current
    const pointer = stage?.getPointerPosition()
    if (!stage || !pointer) return null
    return { x: (pointer.x - position.x) / scale, y: (pointer.y - position.y) / scale }
  }

  const updateCursor = () => {
    const local = pointerInField()
    if (local) setCursor(canvasToWorld(local))
  }

  const handleDown = (event: Konva.KonvaEventObject<MouseEvent | TouchEvent>) => {
    updateCursor()
    if (event.target !== event.target.getStage()) return
    const local = pointerInField()
    if (!local) return
    if (tool === 'draw') {
      setDrawing(true)
      drawingRef.current = true
      setStroke([local])
      strokeRef.current = [local]
      setPendingActions([])
      pendingActionsRef.current = []
      onSelect(null)
    } else if (tool === 'waypoint') {
      const world = canvasToWorld(local, snap)
      const previous = points.at(-1)
      onPointsChange([...points, {
        ...world,
        id: crypto.randomUUID(),
        heading: previous ? tangentDegrees(previous, world) : 0,
        interpolation: 'auto',
        controlWeight: 1,
      }])
    } else if (tool === 'select') onSelect(null)
  }

  const handleMove = () => {
    updateCursor()
    if (!drawing || tool !== 'draw') return
    const local = pointerInField()
    if (!local) return
    const last = stroke.at(-1)
    if (!last || Math.hypot(local.x - last.x, local.y - last.y) > 3) setStroke(current => {
      const next = [...current, local]
      strokeRef.current = next
      return next
    })
  }

  const handleUp = () => {
    if (!drawing) return
    if (waitStartRef.current) finishWait()
    setDrawing(false)
    drawingRef.current = false
    const endpoint=pointerInField(),last=strokeRef.current.at(-1)
    const rawStroke = endpoint && last && Math.hypot(endpoint.x-last.x,endpoint.y-last.y)>.5 ? [...strokeRef.current,endpoint] : strokeRef.current
    const actions = pendingActionsRef.current.sort((a, b) => a.sampleIndex - b.sampleIndex)
    const sampled: { point: Point2D; action?: PathAction }[] = []
    let startIndex = 0
    for (const marker of actions) {
      const endIndex = clamp(marker.sampleIndex, startIndex, rawStroke.length - 1)
      const section = simplify(rawStroke.slice(startIndex, endIndex + 1), 1)
      sampled.push(...section.slice(sampled.length ? 1 : 0).map(point => ({ point })))
      if (sampled.length) sampled[sampled.length - 1].action = marker.action
      startIndex = endIndex
    }
    const tail = simplify(rawStroke.slice(startIndex), 1)
    sampled.push(...tail.slice(sampled.length ? 1 : 0).map(point => ({ point })))
    const worldSamples = sampled.map(sample => ({ ...sample, point: canvasToWorld(sample.point, snap) }))
    if (worldSamples.length >= 2) {
      const next = worldSamples.map((sample, index) => ({
        ...sample.point,
        action: sample.action,
        id: crypto.randomUUID(),
        heading: index===worldSamples.length-1 ? tangentDegrees(worldSamples[index-1].point,sample.point) : tangentDegrees(sample.point,worldSamples[index+1].point),
        interpolation: 'auto' as const,
        controlWeight: 1,
      }))
      onStroke(next)
    }
    setStroke([])
    strokeRef.current = []
    setPendingActions([])
    pendingActionsRef.current = []
  }

  const resetView = () => {
    const fit = Math.min((size.width - 56) / FIELD_PIXELS, (size.height - (size.width<560?210:170)) / FIELD_PIXELS)
    setScale(fit)
    setPosition({ x: Math.max(28, (size.width - FIELD_PIXELS * fit) / 2), y: 24 })
  }

  const pathShapes = useMemo(() => points.slice(0, -1).map((point, index) => {
    const next = points[index + 1]
    const pts = segmentPoints(points, index)
    return { index, point, next, pts, canvas: pts.map(worldToCanvas), mid: worldToCanvas(bezierAt(pts, .5)) }
  }), [points])
  const selectedIndex = points.findIndex(point => point.id === selectedId)
  const editableSegments = selectedIndex < 0 ? [] : [selectedIndex - 1, selectedIndex].filter(i => i >= 0 && i < points.length - 1)
  const traceSegment = (ctx: Konva.Context, canvas: Point2D[]) => {
    ctx.beginPath(); ctx.moveTo(canvas[0].x, canvas[0].y)
    if (canvas.length === 2) ctx.lineTo(canvas[1].x, canvas[1].y)
    else if (canvas.length === 3) ctx.quadraticCurveTo(canvas[1].x, canvas[1].y, canvas[2].x, canvas[2].y)
    else if (canvas.length === 4) ctx.bezierCurveTo(canvas[1].x, canvas[1].y, canvas[2].x, canvas[2].y, canvas[3].x, canvas[3].y)
    else for (let i = 1; i <= 64; i++) { const p = bezierAt(canvas, i / 64); ctx.lineTo(p.x, p.y) }
  }

  const updatePoint = (id: string, changes: Partial<Waypoint>) => onPointsChange(editWaypoint(points,id,changes))

  return <section className="field-panel" ref={wrapRef} data-view={simulation.threeD?'3d':'2d'} aria-label="Interactive BIOBUZZ field">
    <div className="canvas-tools" role="toolbar" aria-label="Canvas tools">
      <div className="tool-cluster">
        {toolItems.map(item => <button key={item.id} className={tool === item.id ? 'tool active' : 'tool'} onClick={() => onToolChange(item.id)} title={item.label} aria-label={item.label}><item.icon size={17} /></button>)}
      </div>
      <div className="tool-cluster">
        <button className={snap ? 'tool active' : 'tool'} onClick={() => onSnapChange(!snap)} title="Snap to 6-inch grid"><ScanLine size={17} /></button>
        <button className="tool" onClick={() => setScale(value => clamp(value * 1.15, .35, 3))} title="Zoom in"><ZoomIn size={17} /></button>
        <button className="tool" onClick={() => setScale(value => clamp(value / 1.15, .35, 3))} title="Zoom out"><ZoomOut size={17} /></button>
        <button className="tool" onClick={resetView} title="Reset view"><RotateCcw size={17} /></button>
      </div>
    </div>
    <div className="action-rail" aria-label="Drawing action shortcuts">
      {actionKeys.map(action => <div key={action.key} className={activeWait && action.key === '5' ? 'action-key recording' : 'action-key'} style={{ '--action-color': action.color } as CSSProperties}><kbd>{action.key}</kbd><span>{action.label}</span></div>)}
    </div>
    {actionFlash && <div className="action-flash">{actionFlash}</div>}
    {tool === 'select' && selectedIndex >= 0
      ? <PoseReadout key={selectedId} point={points[selectedIndex]} index={selectedIndex} onChange={changes => updatePoint(points[selectedIndex].id, changes)} />
      : <div className="coordinate-readout"><span>X {cursor.x.toFixed(1)}</span><span>Y {cursor.y.toFixed(1)}</span><small>in</small></div>}
    <Stage
      ref={stageRef} width={size.width} height={size.height}
      onMouseDown={handleDown} onTouchStart={handleDown}
      onMouseMove={handleMove} onTouchMove={handleMove}
      onMouseUp={handleUp} onTouchEnd={handleUp}
      onWheel={event => {
        event.evt.preventDefault()
        const stage = stageRef.current
        const pointer = stage?.getPointerPosition()
        if (!pointer) return
        const mouse = { x: (pointer.x - position.x) / scale, y: (pointer.y - position.y) / scale }
        const next = clamp(event.evt.deltaY > 0 ? scale / 1.08 : scale * 1.08, .35, 3)
        setScale(next)
        setPosition({ x: pointer.x - mouse.x * next, y: pointer.y - mouse.y * next })
      }}
      draggable={tool === 'pan'} x={0} y={0}
      onDragEnd={event => {
        if (tool !== 'pan') return
        setPosition({ x: position.x + event.target.x(), y: position.y + event.target.y() })
        event.target.position({ x: 0, y: 0 })
      }}
      style={{ display:simulation.threeD?'none':undefined, cursor: tool === 'draw' ? 'crosshair' : tool === 'pan' ? 'grab' : 'default' }}
    >
      <Layer x={position.x} y={position.y} scaleX={scale} scaleY={scale}>
        <Image image={fieldImage ?? undefined} width={FIELD_PIXELS} height={FIELD_PIXELS} listening={false} />
        {Array.from({ length: 25 }).map((_, i) => <Line key={`v${i}`} points={[i * 30, 0, i * 30, FIELD_PIXELS]} stroke="rgba(160,190,195,.18)" strokeWidth={i % 4 === 0 ? 1.3 : .55} listening={false} />)}
        {Array.from({ length: 25 }).map((_, i) => <Line key={`h${i}`} points={[0, i * 30, FIELD_PIXELS, i * 30]} stroke="rgba(160,190,195,.18)" strokeWidth={i % 4 === 0 ? 1.3 : .55} listening={false} />)}
        {pathShapes.map(({ index, point, next, canvas, mid }) => {
          const selected = selectedId === point.id || selectedId === next.id
          const unsafe=simulation.route.unsafeSegments.includes(index)
          return <Group key={next.id}>
            <Shape sceneFunc={(ctx, shape) => { traceSegment(ctx, canvas); ctx.fillStrokeShape(shape) }} stroke="#09141b" strokeWidth={11} lineCap="round" listening={false} />
            <Shape sceneFunc={(ctx, shape) => { traceSegment(ctx, canvas); ctx.fillStrokeShape(shape) }} stroke={unsafe?'#ff6b72':selected ? '#6ee7f2' : '#f2c94c'} strokeWidth={unsafe?7:5} lineCap="round" dash={unsafe?[12,5]:decisions[index]?.type === 'piecewise' ? [16, 7] : undefined} listening={false} />
            <Circle x={mid.x} y={mid.y} radius={12} fill="#f2c94c" stroke="#0d1e28" strokeWidth={2} listening={false} />
            <Text x={mid.x - 13} y={mid.y - 12} text={`${index + 1}`} width={26} height={24} align="center" verticalAlign="middle" fill="#08161f" fontSize={13} fontStyle="bold" listening={false} />
          </Group>
        })}
        {tool === 'select' && editableSegments.map(index => {
          const { canvas, pts } = pathShapes[index]
          const inner = canvas.slice(1, -1)
          return <Group key={`ctrl-${points[index].id}`}>
            <Line points={canvas.flatMap(p => [p.x, p.y])} stroke="#b69cff" strokeWidth={1.5} dash={[6, 5]} listening={false} />
            {inner.map((handle, j) => <Rect
              key={j} x={handle.x} y={handle.y} width={13} height={13} offsetX={6.5} offsetY={6.5} rotation={45}
              fill="#b69cff" stroke="#08161f" strokeWidth={2} draggable
              onDragMove={event => {
                const world = canvasToWorld({ x: event.target.x(), y: event.target.y() }, snap)
                onPointsChange(setControlPoints(points, index, pts.slice(1, -1).map((c, k) => k === j ? world : c)))
              }}
              onDblClick={() => onPointsChange(setControlPoints(points, index, pts.slice(1, -1).filter((_, k) => k !== j)))}
              onDblTap={() => onPointsChange(setControlPoints(points, index, pts.slice(1, -1).filter((_, k) => k !== j)))}
            />)}
          </Group>
        })}
        {supportMarker && <Group x={supportMarker.x} y={supportMarker.y} listening={false}><Circle radius={14} fill="#253039" stroke="#ff9380" strokeWidth={3}/><Line points={[-5,-5,5,5,0,0,-5,5,5,-5]} stroke="#ff9380" strokeWidth={2}/><Text text="Support clearance" x={18} y={-7} fill="#ffc1a4" fontSize={12}/></Group>}
        {stroke.length > 1 && <Line points={stroke.flatMap(point => [point.x, point.y])} stroke="#f2c94c" strokeWidth={5} lineCap="round" lineJoin="round" dash={[10, 6]} listening={false} />}
        {pendingActions.map((marker, index) => {
          const meta = actionMeta[marker.action.type]
          return <Group key={`${marker.sampleIndex}-${index}`} x={marker.point.x} y={marker.point.y} listening={false}>
            <Circle radius={14} fill="#07161f" stroke={meta.color} strokeWidth={3} />
            <Text text={meta.key} x={-10} y={-8} width={20} height={16} align="center" fill={meta.color} fontSize={12} fontStyle="bold" />
          </Group>
        })}
        {points.map((point, index) => {
          const canvas = worldToCanvas(point)
          const radians = -point.heading * Math.PI / 180
          const arrowEnd = { x: canvas.x + Math.cos(radians) * 42, y: canvas.y + Math.sin(radians) * 42 }
          const selected = point.id === selectedId
          const safe=clearsWalls(point,config.size)&&!robotSupportCollision(point,config.size)
          const action = point.action ? actionMeta[point.action.type] : null
          return <Group key={point.id}>
            {selected && <Rect x={canvas.x} y={canvas.y} offsetX={config.size*2.5} offsetY={config.size*2.5} width={config.size*5} height={config.size*5} rotation={-point.heading} fill={safe?'#6ee7f215':'#ff6f7728'} stroke={safe?'#6ee7f2':'#ff6f77'} strokeWidth={1.5} dash={[6,4]} listening={false}/>}
            {selected && <Line points={[canvas.x, canvas.y, arrowEnd.x, arrowEnd.y]} stroke="#6ee7f2" strokeWidth={2} dash={[5, 4]} listening={false} />}
            <Arrow points={[canvas.x, canvas.y, arrowEnd.x, arrowEnd.y]} stroke={selected ? '#6ee7f2' : '#fff3c6'} fill={selected ? '#6ee7f2' : '#fff3c6'} pointerLength={8} pointerWidth={8} strokeWidth={3} listening={false} />
            <Circle
              x={canvas.x} y={canvas.y} radius={selected ? 10 : 8}
              fill={selected ? '#6ee7f2' : '#f8faf9'} stroke="#08161f" strokeWidth={3}
              draggable={tool === 'select'}
              onClick={() => onSelect(point.id)} onTap={() => onSelect(point.id)}
              onDragStart={() => onSelect(point.id)}
              onDragMove={event => {
                const world = canvasToWorld({ x: event.target.x(), y: event.target.y() }, snap)
                updatePoint(point.id, world)
              }}
            />
            {selected && <Circle
              x={arrowEnd.x} y={arrowEnd.y} radius={7} fill="#07161e" stroke="#6ee7f2" strokeWidth={3}
              draggable onDragMove={event => {
                const dx = event.target.x() - canvas.x
                const dy = event.target.y() - canvas.y
                updatePoint(point.id, { heading: normalizeDegrees(-Math.atan2(dy, dx) * 180 / Math.PI) })
              }}
              onDragEnd={event => event.target.position(arrowEnd)}
            />}
            <Text x={canvas.x - 22} y={canvas.y + 14} width={44} align="center" text={`${index + 1}`} fill="#ffffff" fontSize={11} fontStyle="bold" listening={false} />
            {action && <Group x={canvas.x - 14} y={canvas.y - 22} listening={false}>
              <Circle radius={11} fill="#07161f" stroke={action.color} strokeWidth={2.5} />
              <Text text={action.key} x={-8} y={-6} width={16} align="center" fill={action.color} fontSize={10} fontStyle="bold" />
            </Group>}
          </Group>
        })}
        {simulation.overlay}
      </Layer>
    </Stage>
    {simulation.scene}
    {simulation.hud}
    <a className="field-credit" href="https://www.reddit.com/r/FTC/comments/1weleaj/biobuzz_custom_field_images_meepmeep_compatible/" target="_blank" rel="noreferrer">Field art: Team Juice 16236</a>
    <div className="canvas-hint">{tool === 'draw' ? 'Drag to sketch a new path' : tool === 'waypoint' ? 'Click the field to add a waypoint' : tool === 'pan' ? 'Drag to pan · scroll to zoom' : 'Drag nodes · drag the cyan handle to rotate'}</div>
  </section>
}
