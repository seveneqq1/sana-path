import test from 'node:test'
import assert from 'node:assert/strict'
import type { Waypoint } from '../types'
import { addControlPoint, autoBuild, bestHeadings, makeSafe } from './smartPath'
import { defaultConfig, profile, resetSimulation, stepSimulation } from './simulation'
import { bezierAt, segmentPoints, seedWaypoints } from './geometry'
import { generateJava } from './codegen'
import { analyzePath } from './optimizer'
import { HIVE } from './hivePhysics'
import { materializePathActions, movePathAction, nearestPathPosition, pathActionMarkers } from './pathActions'
import { actionPlan, compileRoute, nodeActions } from './route'

const wp = (x: number, y: number, extra: Partial<Waypoint> = {}): Waypoint => ({ id: crypto.randomUUID(), x, y, heading: 90, interpolation: 'auto', controlWeight: 1, ...extra })

test('auto-build collapses a dense drawing into major points with control points', () => {
  const drawn = Array.from({ length: 25 }, (_, i) => wp(20 + i * 2, 20 + Math.sin(i / 6) * 18))
  drawn[12].action = { type: 'intake' }
  const { points, report } = autoBuild(drawn, defaultConfig)
  assert.equal(points.length, 3)
  assert.equal(report.sections, 2)
  assert.ok(points[0].controlPoints && points[1].controlPoints)
  assert.equal(points[1].action?.type, 'intake')
  assert.ok(report.clear)
})

test('auto-build stays close to the drawing when the drawing is clear', () => {
  const drawn = Array.from({ length: 20 }, (_, i) => wp(20 + i * 4, 20 + i * .8))
  const { points } = autoBuild(drawn, defaultConfig, 'close')
  const route = profile(points, defaultConfig)
  for (const p of drawn) assert.ok(Math.min(...route.samples.map(s => Math.hypot(s.x - p.x, s.y - p.y))) < 1.5)
})

test('auto-build bends a drawing that cuts through the hive away from supports', () => {
  const drawn = Array.from({ length: 13 }, (_, i) => wp(30 + i * 7, 72 - HIVE.frameDepth / 2, { heading: 0 }))
  const before = profile(drawn, defaultConfig)
  assert.ok(before.supportCollision || before.unsafeSegments.length, 'fixture should collide')
  const { points, report } = autoBuild(drawn, defaultConfig)
  const after = profile(points, defaultConfig)
  assert.ok(report.clear, 'route should be clear')
  assert.equal(after.supportCollision, undefined)
  assert.equal(after.wallCollision, undefined)
})

test('best headings records a simulator reason and keeps the route clear', () => {
  const next = bestHeadings(seedWaypoints(), defaultConfig)
  assert.ok(next.slice(1).every(p => p.interpolation !== 'auto' && p.autoReason))
  assert.equal(profile(next, defaultConfig).wallCollision, undefined)
})

test('adding a control point keeps the curve shape and exports a higher-order Pedro curve', () => {
  const points = [wp(20, 20, { controlPoints: [{ x: 40, y: 60 }] }), wp(80, 30)]
  const elevated = addControlPoint(points, 0)
  assert.equal(elevated[0].controlPoints!.length, 2)
  for (const t of [.2, .5, .8]) {
    const a = bezierAt(segmentPoints(points, 0), t), b = bezierAt(segmentPoints(elevated, 0), t)
    assert.ok(Math.hypot(a.x - b.x, a.y - b.y) < 1e-9)
  }
  const code = generateJava(elevated, analyzePath(elevated), defaultConfig)
  assert.match(code, /Paths\.curve\(startPose,\n(\s+p\.of\([^)]*\),\n){2}\s+pose1\)/)
  assert.match(generateJava([wp(20, 20, { controlPoints: [] }), wp(60, 20)], analyzePath([wp(20, 20), wp(60, 20)])), /Paths\.line\(startPose, pose1\)/)
})

test('make safe moves unsafe points and bends unsafe paths clear', () => {
  const route = [wp(30, 72 - HIVE.frameDepth / 2, { heading: 0 }), wp(72, 72 - HIVE.frameDepth / 2, { heading: 0 }), wp(114, 72 - HIVE.frameDepth / 2, { heading: 0 })]
  const result = makeSafe(route, defaultConfig)
  assert.equal(result.points.length, 3)
  assert.ok(result.moved >= 1)
  assert.ok(result.clear)
})

test('flower intake: auto-build aligns to the flower and the simulator collects its pollen', () => {
  const drawn = [wp(30, 100, { heading: 90 }), wp(50, 120), wp(47, 132, { action: { type: 'flowerIntake' } }), wp(40, 110)]
  const { points, report } = autoBuild(drawn, defaultConfig)
  const flower = points.find(p => p.action?.type === 'flowerIntake')!
  assert.deepEqual([flower.x, flower.y, flower.heading], [48, 144 - defaultConfig.size / 2 - .5, 90])
  assert.ok(report.clear)
  const config = { ...defaultConfig, preload: 0 }
  const route = profile(points, config), state = resetSimulation(points, config)
  for (let i = 0; i < 3000 && !state.finished && !state.blocked; i++) stepSimulation(state, points, config, route, 1 / 120)
  assert.equal(state.blocked, false)
  assert.equal(state.inventory, 4)
})

test('independent action markers split higher-order curves exactly without changing editor anchors', () => {
  const points=[wp(20,20,{heading:350,controlPoints:[{x:28,y:35},{x:35,y:40},{x:42,y:30},{x:60,y:20}],pathActions:[
    {id:'a',position:.2,action:{type:'intake',composition:'parallel'}},
    {id:'b',position:.6,action:{type:'wait',durationMs:500,composition:'sequential'}},
  ]}),wp(90,25,{heading:10,interpolation:'linear'})]
  const before=JSON.stringify(points),expanded=materializePathActions(points,analyzePath(points)),cuts=[0,.2,.6,1]
  assert.equal(JSON.stringify(points),before);assert.equal(expanded.points.length,4)
  for(let i=0;i<3;i++)for(let j=0;j<=20;j++){
    const a=bezierAt(segmentPoints(points,0),cuts[i]+(cuts[i+1]-cuts[i])*j/20)
    const b=bezierAt(segmentPoints(expanded.points,i),j/20)
    assert.ok(Math.hypot(a.x-b.x,a.y-b.y)<1e-9)
  }
  assert.deepEqual(expanded.segments,[0,0,0])
  assert.deepEqual(expanded.points.flatMap(nodeActions).map(a=>a.type),['intake','wait'])
})

test('action dragging projects onto the path and can cross segments without moving any waypoint',()=>{
  const points=[wp(20,20,{controlPoints:[],pathActions:[{id:'a',position:.2,action:{type:'shoot',composition:'parallel'}}]}),wp(100,20,{controlPoints:[]}),wp(100,100)]
  const target=nearestPathPosition(points,{x:103,y:70}),moved=movePathAction(points,'a',target.segment,target.position)
  assert.equal(target.segment,1);assert.ok(Math.abs(target.position-.625)<1e-6)
  assert.equal(moved[0].pathActions?.length,0);assert.equal(moved[1].pathActions?.[0].id,'a')
  assert.deepEqual(moved.map(({x,y,id,heading,controlPoints})=>({x,y,id,heading,controlPoints})),points.map(({x,y,id,heading,controlPoints})=>({x,y,id,heading,controlPoints})))
  assert.equal(points[0].pathActions?.[0].position,.2)
})

test('coincident shoot and intake markers export one parallel group and keep intake running',()=>{
  const points=[wp(85,20,{heading:0,controlPoints:[],pathActions:[
    {id:'shoot',position:.25,action:{type:'shoot',composition:'parallel'}},
    {id:'intake',position:.25,action:{type:'intake',composition:'parallel'}},
    {id:'wait',position:.75,action:{type:'wait',durationMs:300,composition:'sequential'}},
  ]}),wp(115,20,{heading:0,interpolation:'linear'})]
  const expanded=materializePathActions(points,analyzePath(points)),steps=compileRoute(expanded.points)
  const group=steps.find(s=>s.kind==='group')
  assert.equal(group?.kind,'group');if(group?.kind==='group')assert.equal(group.additionalActions?.[0].type,'intake')
  const code=generateJava(points,analyzePath(points))
  assert.match(code,/parallel\(followPath\(path1\(\)\), shootCommand\(true\), instant\(this::startIntake\)/)
  assert.match(code,/waitMs\(300\)/);assert.match(code,/if \(!keepIntakeRunning\) stopIntake\(\)/)
  const config={...defaultConfig,preload:1},route=profile(points,config),state=resetSimulation(points,config)
  let combined=false
  for(let i=0;i<3000&&!state.finished&&!state.blocked;i++){
    stepSimulation(state,points,config,route,1/120)
    if(state.shooting&&state.intake&&state.driveVx>0)combined=true
  }
  assert.ok(combined);assert.ok(state.finished);assert.equal(state.blocked,false)
})

test('multiple sequential path waits stop at their own independent curve positions',()=>{
  const points=[wp(20,20,{heading:0,controlPoints:[],pathActions:[
    {id:'first',position:.25,action:{type:'wait',durationMs:500,composition:'sequential'}},
    {id:'second',position:.75,action:{type:'wait',durationMs:700,composition:'sequential'}},
  ]}),wp(120,20,{heading:0,interpolation:'linear'})]
  const route=profile(points,defaultConfig),state=resetSimulation(points,defaultConfig),stops=new Set<number>()
  for(let i=0;i<2000&&!state.finished;i++){
    stepSimulation(state,points,defaultConfig,route,1/120)
    if(state.activeType==='wait')stops.add(Math.round(state.x))
  }
  assert.deepEqual([...stops],[45,95]);assert.ok(state.finished&&!state.blocked)
  assert.ok(Math.abs(state.time-route.drive-1.2)<.08)
})

test('coincident parallel actions overlap while endpoint actions fall back to sequential',()=>{
  const points=[wp(20,20,{heading:0,controlPoints:[],pathActions:[
    {id:'a',position:.2,action:{type:'wait',durationMs:1000,composition:'parallel'}},
    {id:'b',position:.2,action:{type:'wait',durationMs:1500,composition:'parallel'}},
    {id:'c',position:1,action:{type:'wait',durationMs:500,composition:'parallel'}},
    {id:'d',position:1,action:{type:'wait',durationMs:700,composition:'parallel'}},
  ]}),wp(120,20,{heading:0,interpolation:'linear'})]
  const route=profile(points,defaultConfig),state=resetSimulation(points,defaultConfig)
  assert.equal(actionPlan(route.points.at(-1)!,route.points.length-1,route.points.length).sequential.length,2)
  for(let i=0;i<2000&&!state.finished;i++)stepSimulation(state,points,defaultConfig,route,1/120)
  assert.ok(state.finished&&!state.blocked);assert.ok(Math.abs(state.time-route.max)<.08)
})

test('individually chosen modes are honored even when the global moving-shot preset is on',()=>{
  const points=[wp(85,20,{controlPoints:[],pathActions:[{id:'shoot',position:.5,action:{type:'shoot',composition:'sequential'}}]}),wp(115,20)]
  const route=profile(points,{...defaultConfig,shootWhileMoving:true})
  assert.equal(route.actions[1]?.composition,'sequential')
  const code=generateJava(points,analyzePath(points),{...defaultConfig,shootWhileMoving:true})
  assert.doesNotMatch(code,/parallel\(followPath/)
})

test('auto-build retains path markers and safety warnings retain original segment indices',()=>{
  const points=[wp(20,20,{heading:0,controlPoints:[],pathActions:[{id:'a',position:.3,action:{type:'intake'}}]}),wp(60,20,{heading:0}),wp(100,20,{heading:0})]
  const built=autoBuild(points,defaultConfig).points
  assert.equal(pathActionMarkers(built)[0].id,'a');assert.equal(built[0].pathActions?.[0].position,.3)
  points[0].pathActions!.push({id:'b',position:.6,action:{type:'wait'}})
  points[1].controlPoints=[{x:220,y:20}]
  assert.deepEqual(profile(points,defaultConfig).unsafeSegments,[1])
})

test('splitting a piecewise heading path preserves tangent-to-fixed interpolation',()=>{
  const points=[wp(20,20,{heading:0,controlPoints:[{x:30,y:70},{x:90,y:70}],pathActions:[{id:'a',position:.2,action:{type:'intake',composition:'parallel'}}]}),wp(110,20,{heading:180,interpolation:'piecewise'})]
  const expanded=materializePathActions(points,analyzePath(points))
  assert.deepEqual(expanded.decisions.map(d=>d.type),['tangent','tangent','linear'])
  assert.equal(expanded.points.at(-1)?.heading,180)
  const code=generateJava(points,analyzePath(points))
  assert.match(code,/\.tangent\(\)/);assert.match(code,/\.linear\(/)
})

test('conflicting concurrent subsystem actions are rejected before simulation or robot execution',()=>{
  const points=[wp(85,20,{controlPoints:[],pathActions:[
    {id:'a',position:.5,action:{type:'shoot',composition:'parallel'}},
    {id:'b',position:.5,action:{type:'transfer',composition:'parallel'}},
  ]}),wp(115,20)]
  const route=profile(points,defaultConfig),state=resetSimulation(points,defaultConfig)
  assert.match(route.actionConflicts[0].message,/share the feeder/)
  stepSimulation(state,points,defaultConfig,route,1/120)
  assert.ok(state.blocked);assert.equal(state.time,0)
  assert.match(generateJava(points,analyzePath(points)),/if \(!actionsCompatible\) throw/)
  points[0].pathActions![1].action.composition='sequential'
  assert.equal(profile(points,defaultConfig).actionConflicts.length,0)
})

test('a parallel join cannot skip a nearby subsequent action marker',()=>{
  const points=[wp(20,20,{heading:0,controlPoints:[],pathActions:[
    {id:'a',position:.5,action:{type:'wait',durationMs:1000,composition:'parallel'}},
    {id:'b',position:.5001,action:{type:'wait',durationMs:500,composition:'sequential'}},
    {id:'c',position:.9,action:{type:'wait',durationMs:700,composition:'sequential'}},
  ]}),wp(120,20,{heading:0,interpolation:'linear'})]
  const route=profile(points,defaultConfig),state=resetSimulation(points,defaultConfig)
  for(let i=0;i<2000&&!state.finished;i++)stepSimulation(state,points,defaultConfig,route,1/120)
  assert.ok(state.finished&&!state.blocked)
  assert.deepEqual(state.tasks?.map(t=>t.action.durationMs),[1000,500,700])
  assert.ok(Math.abs(state.time-route.max)<.08)
})

test('mixed deadline and parallel branches at one marker cancel and join independently',()=>{
  const points=[wp(20,20,{heading:0,controlPoints:[],pathActions:[
    {id:'a',position:.3,action:{type:'intake',composition:'deadline'}},
    {id:'b',position:.3,action:{type:'wait',durationMs:1500,composition:'parallel'}},
    {id:'c',position:.6,action:{type:'wait',durationMs:300,composition:'sequential'}},
  ]}),wp(120,20,{heading:0,interpolation:'linear'})]
  const route=profile(points,defaultConfig),state=resetSimulation(points,defaultConfig)
  let stoppedIntake=false
  for(let i=0;i<2000&&!state.finished;i++){
    stepSimulation(state,points,defaultConfig,route,1/120)
    if(state.activeType==='wait'&&state.x>=79.99&&!state.intake)stoppedIntake=true
  }
  assert.ok(state.finished&&!state.blocked&&stoppedIntake)
  assert.ok(Math.abs(state.time-route.max)<.08)
  assert.match(generateJava(points,analyzePath(points)),/parallel\(deadline\(followPath\(path1\(\)\), intakeUntilCancelled\(\)\), waitMs\(1500\)\)/)
})
