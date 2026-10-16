import { useEffect, useMemo, useRef, useState } from 'react'
import { Circle, Group, Image, Line, Rect, Text } from 'react-konva'
import { Box, Map, Pause, Play, RotateCcw, Settings2, X } from 'lucide-react'
import type { Waypoint } from '../types'
import { worldToCanvas } from '../lib/geometry'
import { flowers, hiveCenters, profile, resetSimulation, stepSimulation } from '../lib/simulation'
import type { RobotConfig } from '../lib/simulation'
import { cellOpening, pieceRadius } from '../lib/hivePhysics'
import { ThreePreview } from './ThreePreview'
import { usePreviewHeading } from './usePreviewHeading'
import { AutoScoreboard } from './AutoScoreboard'

const fields: {key:Exclude<keyof RobotConfig,'alliance'|'autoAim'|'intakeMaterial'|'shootWhileMoving'>;label:string;min:number;max:number;step:number}[]=[
  {key:'rpm',label:'Motor output RPM',min:1,max:6000,step:1},
  {key:'wheel',label:'Wheel diameter (in)',min:1,max:8,step:.1},
  {key:'gearing',label:'Motor : wheel ratio',min:.1,max:20,step:.1},
  {key:'mass',label:'Robot mass (kg)',min:1,max:40,step:.5},
  {key:'force',label:'Drive force (N)',min:1,max:500,step:1},
  {key:'grip',label:'Traction coefficient',min:.1,max:2,step:.05},
  {key:'efficiency',label:'Loaded speed factor',min:.1,max:1,step:.05},
  {key:'size',label:'Robot footprint (in)',min:12,max:24,step:1},
  {key:'turnRate',label:'Max rotation (°/s)',min:10,max:720,step:10},
  {key:'preload',label:'Preloaded pollen',min:0,max:20,step:1},
  {key:'capacity',label:'Hopper capacity',min:1,max:20,step:1},
  {key:'shotSpeed',label:'Launch speed (in/s)',min:50,max:600,step:5},
  {key:'shotAngle',label:'Launch elevation (°)',min:10,max:85,step:1},
  {key:'shotInterval',label:'Seconds per shot',min:.1,max:5,step:.05},
  {key:'transferTime',label:'Transfer time (s)',min:.1,max:5,step:.1},
  {key:'turretRate',label:'Turret rotation (°/s)',min:10,max:720,step:10},
]

function robotPNG() {
  const canvas=document.createElement('canvas');canvas.width=160;canvas.height=160
  const ctx=canvas.getContext('2d')!
  ctx.fillStyle='#050c12';ctx.fillRect(15,12,28,54);ctx.fillRect(15,94,28,54);ctx.fillRect(117,12,28,54);ctx.fillRect(117,94,28,54)
  ctx.strokeStyle='#8397a4';ctx.lineWidth=3
  for(let i=0;i<5;i++)for(const x of [18,120])for(const y of [18,100]){ctx.beginPath();ctx.moveTo(x,y+i*9);ctx.lineTo(x+20,y+i*9+7);ctx.stroke()}
  ctx.fillStyle='#287d94';ctx.fillRect(30,22,100,116);ctx.strokeStyle='#b0ebf5';ctx.lineWidth=4;ctx.strokeRect(30,22,100,116)
  ctx.fillStyle='#122635';ctx.fillRect(49,46,62,68);ctx.fillStyle='#f2c94c';ctx.fillRect(62,13,36,34)
  ctx.beginPath();ctx.moveTo(80,2);ctx.lineTo(68,19);ctx.lineTo(92,19);ctx.closePath();ctx.fill()
  ctx.fillStyle='#c2e9f0';for(const x of [39,121])for(const y of [31,129]){ctx.beginPath();ctx.arc(x,y,3,0,Math.PI*2);ctx.fill()}
  ctx.fillStyle='#d9f9ff';ctx.font='bold 17px sans-serif';ctx.textAlign='center';ctx.fillText('FTC',80,88)
  return canvas.toDataURL('image/png')
}

export function useSimulation(points:Waypoint[],config:RobotConfig,setConfig:(config:RobotConfig)=>void,theme:'light'|'dark',editor:{selectedId:string|null;onSelect:(id:string|null)=>void;onPointsChange:(points:Waypoint[])=>void}) {
  const dock=useRef<HTMLDivElement>(null)
  const [enabled,setEnabled]=useState(false),[running,setRunning]=useState(false),[settings,setSettings]=useState(false)
  const [threeD,setThreeD]=useState(false)
  const [speed,setSpeed]=useState(1)
  const route=useMemo(()=>profile(points,config),[points,config])
  const engine=useRef(resetSimulation(points,config))
  const [view,setView]=useState(()=>resetSimulation(points,config))
  const heading=usePreviewHeading({enabled:threeD&&!settings,points,config,view,...editor,onPause:()=>setRunning(false)})
  const [sprite,setSprite]=useState<HTMLImageElement>()
  useEffect(()=>{
    const element=dock.current
    if(!element)return
    const resize=()=>element.parentElement?.style.setProperty('--dock-height',`${element.offsetHeight}px`)
    const observer=new ResizeObserver(resize);observer.observe(element);resize()
    return()=>observer.disconnect()
  },[])
  useEffect(()=>{const img=new window.Image();img.onload=()=>setSprite(img);img.src=robotPNG()},[])
  useEffect(()=>{
    engine.current=resetSimulation(points,config)
    // Reset runs when geometry/config changes; never continue on a stale trajectory.
    const frame=requestAnimationFrame(()=>{setView(structuredClone(engine.current));setRunning(false)})
    return ()=>cancelAnimationFrame(frame)
  },[points,config])
  useEffect(()=>{
    if(!running || !enabled)return
    let frame=0,last=0,accumulator=0
    const tick=(now:number)=>{
      if(last)accumulator+=Math.min((now-last)/1000,.1)*speed
      last=now
      while(accumulator>=1/120){stepSimulation(engine.current,points,config,route,1/120);accumulator-=1/120}
      setView(structuredClone(engine.current))
      if(engine.current.finished || engine.current.blocked){setRunning(false);return}
      frame=requestAnimationFrame(tick)
    }
    frame=requestAnimationFrame(tick)
    return ()=>cancelAnimationFrame(frame)
  },[running,enabled,speed,points,config,route])
  const reset=()=>{heading.cancel();engine.current=resetSimulation(points,config);setView(structuredClone(engine.current));setRunning(false)}
  const plan=()=>{heading.cancel();setThreeD(false);setRunning(false);setEnabled(false)}
  const moveBall=(id:number,x:number,y:number)=>{
    const ball=engine.current.balls.find(b=>b.id===id)
    if(ball && !running){ball.x=x;ball.y=y;ball.vx=0;ball.vy=0;ball.vz=0;ball.z=pieceRadius(ball.kind);setView(structuredClone(engine.current))}
  }
  const play=()=>{heading.cancel();if(view.finished)reset();setEnabled(true);setThreeD(true);setRunning(!running)}
  const robot=worldToCanvas(view)
  const clearance=route.wallCollision?{sample:route.wallCollision,name:'Field wall'}:route.supportCollision
  const hud=<div className="simulation-ui" ref={dock}>
    <div className="simulation-toolbar">
      <div className="view-switch" aria-label="Workspace view">
        <button aria-pressed={!threeD} onClick={plan}><Map size={16}/>Plan 2D</button>
        <button aria-pressed={threeD} onClick={()=>{setThreeD(true);setEnabled(true)}}><Box size={16}/>Preview 3D</button>
      </div>
      <div className="runtime-estimate" title="Drive time plus inventory-dependent shooting delays"><span>{clearance?'Obstacle clearance':'Estimated run'}</span><strong>{clearance?'Unsafe route':<>{route.min.toFixed(1)}{route.max>route.min?`–${route.max.toFixed(1)}`:''}<small> s</small></>}</strong></div>
      <div className="playback-controls">
        <label className="moving-shot-toggle"><input type="checkbox" checked={config.shootWhileMoving} onChange={e=>setConfig({...config,shootWhileMoving:e.target.checked})}/>Shoot while moving</label>
        <button className="robot-setup-button" aria-label="Robot configuration" onClick={()=>setSettings(!settings)}><Settings2 size={16}/><span>Robot setup</span></button>
        <select aria-label="Playback speed" value={speed} onChange={e=>setSpeed(Number(e.target.value))}><option value={.5}>0.5×</option><option value={1}>1×</option><option value={2}>2×</option></select>
        <button className="reset-playback" onClick={reset} aria-label="Reset simulation"><RotateCcw size={15}/></button>
        <button className="run-button" disabled={points.length<2||view.blocked} onClick={play} aria-label={running?'Pause simulation':'Play simulation'}>{running?<Pause size={16}/>:<Play size={16}/>}<span>{view.blocked?'Collision stop':running?'Pause':view.finished?'Replay':'Run in 3D'}</span></button>
      </div>
    </div>
    {enabled && <div className="simulation-status" aria-live="off">
      <span className="time-chip">{view.time.toFixed(1)} / 30 s</span><span className="action-status">{view.blocked?'Blocked':view.finished?'Finished':running?view.action==='Drive'?'Driving':view.action:'Paused'}</span>
      <span>Hopper <b>{view.inventory}/{config.capacity}</b></span>
      <span>Indexer <b>{view.feeder.length}</b></span><span>Turret <b>{view.turretReady?'Aligned':'Aiming'}</b></span>
      <button aria-pressed={view.intake} onClick={()=>{engine.current.intake=!engine.current.intake;setView(structuredClone(engine.current))}}>Intake {view.intake?'on':'off'}</button>
      <span className="tip-count red">Red tips <b>{view.hives[0].tips}</b></span><span className="tip-count blue">Blue tips <b>{view.hives[1].tips}</b></span>
      {view.warning && <strong>{view.warning}</strong>}
    </div>}
    {clearance && !view.blocked && <div className="route-safety" role="status">Unsafe path segments: {route.unsafeSegments.map(i=>i+1).join(', ')}. {clearance.name} near ({clearance.sample.x.toFixed(1)}, {clearance.sample.y.toFixed(1)}). Safe waypoints do not guarantee a safe curve. Red segments will stop the robot at contact.</div>}
    {!!route.actionConflicts.length && <div className="route-safety" role="status">{route.actionConflicts[0].message}</div>}
    {threeD&&heading.controls}
    {enabled&&<AutoScoreboard state={view} config={config}/>}
    {settings && <div className="robot-settings">
      <h3>Robot setup <button aria-label="Close robot configuration" onClick={()=>setSettings(false)}><X size={17}/></button></h3>
      <p>Use measured drive force and loaded speed to calibrate timing. Changing these settings resets the run.</p>
      <label>Alliance<select value={config.alliance} onChange={e=>setConfig({...config,alliance:e.target.value as 'red'|'blue'})}><option value="red">Red</option><option value="blue">Blue</option></select></label>
      <label>Intake wheels<select value={config.intakeMaterial} onChange={e=>setConfig({...config,intakeMaterial:e.target.value as RobotConfig['intakeMaterial']})}><option value="gecko">Gecko</option><option value="silicone">Silicone</option></select></label>
      <label><input type="checkbox" checked={config.autoAim} onChange={e=>setConfig({...config,autoAim:e.target.checked})}/> Automatically aim launch elevation at the raised cell</label>
      <p>Moving shots overlap the next path and join before the next command. Use only if your real robot can track targets while driving. Automatic elevation includes a constant-velocity lead estimate; manual elevation does not compensate flight.</p>
      <div className="robot-config-grid">{fields.map(f=><label key={f.key}>{f.label}<input type="number" min={f.min} max={f.max} step={f.step} value={config[f.key]} onChange={e=>{const n=Number(e.target.value);if(Number.isFinite(n))setConfig({...config,[f.key]:Math.max(f.min,Math.min(f.max,n))})}}/></label>)}</div>
      <div className="model-summary"><span>Loaded speed <b>{route.maxSpeed.toFixed(1)} in/s</b></span><span>Acceleration <b>{route.acceleration.toFixed(1)} in/s²</b></span></div>
      <p>Front wheel intake feeds a timed indexer. The independent turret gates shots on alignment. Walls and hive supports stop the route; balls rebound from support tubes, feet and panels. Support checks use a conservative box up to the template’s maximum turret height. This is approximate physics; validate clearances and timings on your robot.</p>
    </div>}
  </div>
  const overlay=enabled && !threeD && <Group>
    {/* Hide baked-in pieces in this map layer and replace them with movable entities. */}
    <Rect x={0} y={0} width={15} height={62} fill="#292929" listening={false}/>
    <Rect x={705} y={658} width={15} height={62} fill="#292929" listening={false}/>
    {flowers.map((f,i)=>{const p=worldToCanvas(f);return <Group key={i} listening={false}><Circle x={p.x} y={p.y} radius={14} fill="#292929" stroke="#e9bb51" strokeWidth={3}/><Text x={p.x-8} y={p.y-6} text={String(view.balls.filter(b=>b.flower===i).length)} fill="white" fontSize={12}/></Group>})}
    {view.hives.map((h,i)=>{const p=worldToCanvas(hiveCenters[i]);return <Group key={i} x={p.x} y={p.y} listening={false}>
      <Rect x={-118} y={-59} width={236} height={118} fill="#27323a" stroke={i===0?'#f36b76':'#6f95ff'} strokeWidth={3}/>
      <Line points={[-75,0,75,0]} stroke="#b5cad0" strokeWidth={6}/>
      {[0,1].map(side=><Group key={side} x={cellOpening(h,side).x*5}><Rect x={-30} y={-50} width={60} height={100} fill={side===h.side?'#52616c':'#17262e'} stroke={i===0?'#f36b76':'#6f95ff'} strokeWidth={3}/><Text x={-26} y={-10} text={`${side===h.side?'UP':'DOWN'}\n${h.contents[side].length} pieces`} fill="white" fontSize={10}/></Group>)}
    </Group>})}
    {view.balls.filter(b=>b.flower===undefined).map(b=>{const p=worldToCanvas(b);return <Group key={b.id}>
      <Circle x={p.x} y={p.y} radius={7} fill="black" opacity={.25} listening={false}/>
      <Circle x={p.x} y={p.y-b.z*.65} radius={pieceRadius(b.kind)*5} fill={b.kind==='pollen'?'#ffcf3e':b.color==='blue'?'#6f95ff':'#f36b76'} stroke="#796019" strokeWidth={1} draggable={!running && b.z<=pieceRadius(b.kind)+.05} onMouseDown={e=>{e.cancelBubble=true}} onTouchStart={e=>{e.cancelBubble=true}} onDragEnd={e=>{e.cancelBubble=true;moveBall(b.id,Math.max(1.8,Math.min(142.2,e.target.x()/5)),Math.max(1.8,Math.min(142.2,144-e.target.y()/5)))}}/>
    </Group>})}
    <Circle x={robot.x} y={robot.y} radius={5} fill="#6ee7f2" stroke="white" strokeWidth={2} listening={false}/>
    <Group x={robot.x} y={robot.y} rotation={90-view.heading} listening={false}>
      <Image image={sprite} x={-config.size*2.5} y={-config.size*2.5} width={config.size*5} height={config.size*5} opacity={.94}/>
      {view.intake && <Rect x={-config.size*2} y={-config.size*2.8} width={config.size*4} height={5} fill="#70efbd"/>}
    </Group>
  </Group>
  const scene=threeD && <ThreePreview view={heading.pose} config={config} route={route} running={running&&!heading.editing} onBallMove={moveBall} onBack={plan} theme={theme}/>
  return {hud,overlay,scene,threeD,route}
}
