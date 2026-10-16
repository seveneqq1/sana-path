import { useEffect, useMemo, useRef, useState } from 'react'
import { Download, Hexagon, Moon, Route, RotateCcw, ShieldCheck, Sparkles, Sun, Undo2, Wand2 } from 'lucide-react'
import { FieldCanvas } from './components/FieldCanvas'
import { Inspector } from './components/Inspector'
import { CodePanel } from './components/CodePanel'
import { PathActionsEditor } from './components/PathActionsEditor'
import { editWaypoint, seedWaypoints } from './lib/geometry'
import { analyzePath } from './lib/optimizer'
import { downloadJava, generateJava } from './lib/codegen'
import type { CanvasTool, Waypoint } from './types'
import { defaultConfig, profile } from './lib/simulation'
import { addControlPoint, alignToFlower, autoBuild, BUFFER, bestHeadings, closenessOptions, makeSafe, removeControlPoint, setControlPoints } from './lib/smartPath'
import type { Closeness } from './lib/smartPath'
import { nearestSafePose } from './lib/safeSpot'

export default function App() {
  const [points, setPoints] = useState<Waypoint[]>(seedWaypoints)
  const [history, setHistory] = useState<Waypoint[][]>([])
  const [selectedId, setSelectedId] = useState<string | null>(points[1]?.id ?? null)
  const [selectedActionId, setSelectedActionId] = useState<string | null>(null)
  const [tool, setTool] = useState<CanvasTool>('select')
  const [snap, setSnap] = useState(false)
  const [config,setConfig]=useState(defaultConfig)
  const [theme,setTheme]=useState<'light'|'dark'>(()=>{
    try { const saved=localStorage.getItem('sana-theme');if(saved==='light'||saved==='dark')return saved } catch { /* storage may be disabled */ }
    return window.matchMedia('(prefers-color-scheme: dark)').matches?'dark':'light'
  })
  useEffect(()=>{document.documentElement.dataset.theme=theme;try{localStorage.setItem('sana-theme',theme)}catch{/* optional persistence */}},[theme])
  const [optimized, setOptimized] = useState(false)
  const [closeness, setCloseness] = useState<Closeness>('balanced')
  const [autoAfterDraw, setAutoAfterDraw] = useState(true)
  const [toast, setToast] = useState('')
  const toastTimer = useRef<number | undefined>(undefined)
  const [snapMessage,setSnapMessage]=useState('')
  const [snappedId,setSnappedId]=useState<string|null>(null)
  const decisions = useMemo(() => analyzePath(points), [points])
  const code = useMemo(() => generateJava(points, decisions,config), [points, decisions,config])
  const unsafe = useMemo(() => { const run = profile(points, config); return !!(run.wallCollision || run.supportCollision) }, [points, config])
  const selectedIndex = points.findIndex(point => point.id === selectedId)
  const selectedPoint = selectedIndex >= 0 ? points[selectedIndex] : null
  const selectedDecision = selectedIndex > 0 ? decisions[selectedIndex - 1] : null
  const selectedPathIndex = selectedIndex === points.length - 1 ? selectedIndex - 1 : selectedIndex

  const updatePoints = (next: Waypoint[]) => {
    setHistory(current => [...current.slice(-19), points])
    setPoints(next)
    setOptimized(false)
  }
  const updateSelected = (changes: Partial<Waypoint>) => {
    if (!selectedId) return
    const old = points.find(p => p.id === selectedId)
    if (old && changes.action?.type === 'flowerIntake' && old.action?.type !== 'flowerIntake') {
      const { x, y, heading } = alignToFlower(old, config.size)
      changes = { ...changes, x, y, heading }
    }
    updatePoints(editWaypoint(points,selectedId,changes))
  }
  const undo = () => {
    const previous = history.at(-1)
    if (!previous) return
    setPoints(previous)
    setHistory(current => current.slice(0, -1))
  }
  const flash = (message: string) => {
    setToast(message)
    window.clearTimeout(toastTimer.current)
    toastTimer.current = window.setTimeout(() => setToast(''), 4500)
  }
  const optimize = () => {
    updatePoints(bestHeadings(points, config))
    setOptimized(true)
    flash('Headings chosen by simulating each option against drive/turn limits and obstacles.')
    window.setTimeout(() => setOptimized(false), 1700)
  }
  const fixSafety = () => {
    const result = makeSafe(points, config)
    updatePoints(result.points)
    flash(`${result.moved} point${result.moved === 1 ? '' : 's'} moved to the closest safe spot · ${result.clear ? 'route is clear ✓' : 'some contact remains — try Auto-build path'}`)
  }
  const build = (source: Waypoint[], fromDrawing = false) => {
    const { points: next, report } = autoBuild(source, config, closeness)
    if (fromDrawing) setHistory(current => [...current.slice(-19), points, source])
    else setHistory(current => [...current.slice(-19), points])
    setPoints(next)
    setSelectedId(next[0]?.id ?? null)
    flash(`${report.before} → ${report.after} points · ${report.sections} path${report.sections === 1 ? '' : 's'} · ${report.controls} control points · ${report.drive.toFixed(1)} s drive${report.snapped ? ` · ${report.snapped} point${report.snapped === 1 ? '' : 's'} moved to safety` : ''} · ${report.clear ? 'clear of obstacles ✓' : 'still touches an obstacle — adjust it'}`)
  }

  return <div className="app-shell">
    <header className="topbar">
      <div className="brand">
        <div className="brand-mark"><Hexagon size={30} /><Route size={15} /></div>
        <div><h1>SANA <span>path</span><small className="presenter">(Presented by SANA #24697)</small></h1><p>FTC autonomous workbench</p></div>
      </div>
      <div className="header-controls">
        <span className="season-select">BIOBUZZ 2026–27</span>
        <button className="theme-toggle" aria-label={theme==='light'?'Switch to dark theme':'Switch to light theme'} title={theme==='light'?'Dark theme':'Light theme'} onClick={()=>setTheme(theme==='light'?'dark':'light')}>{theme==='light'?<Moon size={17}/>:<Sun size={17}/>}</button>
        <select className="fit-precision" aria-label="Path style" value={closeness} onChange={e=>setCloseness(e.target.value as Closeness)} title="How closely Auto-build should follow your drawing versus driving faster">{closenessOptions.map(o=><option key={o.value} value={o.value}>{o.label}</option>)}</select>
        <button className="primary-button auto-build-button" onClick={()=>build(points)} title="Turn the route into start/end points with control points, avoid walls and hive supports, and pick the fastest headings"><Wand2 size={16} />Auto-build path</button>
        <button className="secondary-button interpolation-optimize-button" aria-label="Pick best headings" onClick={optimize} title="Simulate every heading interpolation and keep the fastest collision-free one"><Sparkles size={16} /><span className="desktop-label">{optimized ? 'Done' : 'Best headings'}</span><span className="mobile-label">{optimized ? 'Done' : 'Headings'}</span></button>
        <button className="secondary-button" onClick={() => downloadJava(code)}><Download size={16} />Export Java</button>
      </div>
    </header>

    <main className="workspace">
      <div className="canvas-column">
        <div className="section-bar">
          <div><Route size={15}/>Your route <small>{Math.max(0, points.length - 1)} segments · {points.reduce((sum,point) => sum + (point.action ? 1 : 0) + (point.pathActions?.length ?? 0),0)} commands</small></div>
          <div className="section-actions">
            {unsafe && <button className="make-safe-button" onClick={fixSafety} title="Move unsafe points to the closest safe spot and bend unsafe paths clear of obstacles"><ShieldCheck size={15} />Make route safe</button>}
            <label className="auto-draw-toggle" title="Automatically clean up freehand drawings into Pedro paths"><input type="checkbox" checked={autoAfterDraw} onChange={e=>setAutoAfterDraw(e.target.checked)}/>Auto-build after drawing</label>
            <button onClick={undo} disabled={!history.length} title="Undo"><Undo2 size={15} />Undo</button>
            <button onClick={() => { updatePoints(seedWaypoints()); setSelectedId(null) }} title="Reset demo path"><RotateCcw size={15} />Reset</button>
          </div>
        </div>
        <FieldCanvas
          points={points} decisions={decisions} selectedId={selectedId} tool={tool} snap={snap}
          config={config} onConfigChange={setConfig} theme={theme}
          onToolChange={setTool} onSnapChange={setSnap} onPointsChange={updatePoints} onSelect={id=>{setSelectedId(id);setSelectedActionId(null)}}
          selectedActionId={selectedActionId} onSelectAction={(pointId,actionId)=>{setSelectedId(pointId);setSelectedActionId(actionId)}}
          onStroke={stroke => autoAfterDraw ? build(stroke, true) : updatePoints(stroke)}
        />
        {toast && <div className="build-toast" role="status">{toast}</div>}
      </div>

      <aside className="right-column">
        <section className="inspector-panel">
          <Inspector
            point={selectedPoint} index={selectedIndex} decision={selectedDecision}
            onChange={updateSelected}
            onMoveCommandToPath={()=>{
              if(!selectedPoint?.action||points.length<2)return
              const segment=selectedIndex===points.length-1?selectedIndex-1:selectedIndex,id=crypto.randomUUID()
              const marker={id,position:selectedIndex===points.length-1?1:0,action:{...selectedPoint.action,composition:selectedPoint.action.composition??'sequential' as const}}
              updatePoints(points.map((p,i)=>({...p,...(i===selectedIndex?{action:undefined}:{}),...(i===segment?{pathActions:[...(p.pathActions??[]),marker]}:{})})))
              setSelectedId(points[segment].id);setSelectedActionId(id)
            }}
            pathActionsEditor={<PathActionsEditor points={points} segment={selectedPathIndex} selectedId={selectedActionId}
              onSelect={id=>{setSelectedId(points[selectedPathIndex].id);setSelectedActionId(id)}}
              onChange={pathActions=>updatePoints(editWaypoint(points,points[selectedPathIndex].id,{pathActions}))} />}
            isLast={selectedIndex===points.length-1}
            controlCount={selectedIndex>=0&&selectedIndex<points.length-1?(points[selectedIndex].controlPoints?.length??null):null}
            onAddControl={()=>updatePoints(addControlPoint(points,selectedIndex))}
            onRemoveControl={()=>updatePoints(removeControlPoint(points,selectedIndex))}
            onAlignFlower={()=>{if(!selectedPoint)return;const {x,y,heading}=alignToFlower(selectedPoint,config.size);updateSelected({x,y,heading})}}
            onStraighten={()=>updatePoints(setControlPoints(points,selectedIndex,[]))}
            snapMessage={snappedId===selectedId?snapMessage:''}
            shootWhileMoving={config.shootWhileMoving&&selectedIndex<points.length-1}
            onSnapSafe={()=>{
              if(!selectedPoint)return
              setSnappedId(selectedPoint.id)
              const safe=nearestSafePose(selectedPoint,config.size,BUFFER)
              if(!safe){setSnapMessage('No buffered safe spot found for this footprint.');return}
              updateSelected({x:safe.x,y:safe.y})
              setSnapMessage(`Snapped to (${safe.x.toFixed(2)}, ${safe.y.toFixed(2)}) with ${BUFFER} in clearance. Adjacent curves are checked separately.`)
            }}
            onDelete={() => {
              if (!selectedId) return
              const remaining=points.filter(point => point.id !== selectedId)
              if(remaining.length)remaining[remaining.length-1]={...remaining.at(-1)!,pathActions:undefined}
              updatePoints(remaining)
              setSelectedId(null)
            }}
          />
        </section>
        <CodePanel code={code} onExport={() => downloadJava(code)} />
      </aside>
    </main>
    <footer><span>Pedro coordinates</span><b>(0, 0)</b> bottom-left <i /> <b>+X</b> right <i /> <b>+Y</b> up <i /> heading is counter-clockwise</footer>
  </div>
}
