import type { PathAction, Point2D, Waypoint } from '../types'
import { bezierAt, clamp, distance, segmentPoints, shortestAngle, tangentDegrees } from './geometry'
import { curveArc } from './curveFit'
import { analyzePath } from './optimizer'
import { addCellPiece, advanceHive, cellOpening, createHive, GRAVITY, HIVE, hiveToLocal, pieceMass, pieceRadius } from './hivePhysics'
import type { Hive } from './hivePhysics'
import type { CellPiece } from './hivePhysics'
import { actionPlan, actionResourceConflict, compileRoute, nextPathEnd, nodeActions, routeActions } from './route'
import { materializePathActions } from './pathActions'
import { clearsWalls, constrainPose, sweepWalls } from './walls'
import type { RobotPose } from './walls'
import { bounceOffSupports, constrainSupports, robotSupportCollision, sweepSupports } from './supportCollisions'
export { HIVE } from './hivePhysics'
export type { Hive } from './hivePhysics'

export interface RobotConfig {
  rpm:number; wheel:number; gearing:number; mass:number; force:number; grip:number; efficiency:number
  size:number; turnRate:number; capacity:number; preload:number; shotSpeed:number; shotAngle:number; shotInterval:number; transferTime:number
  alliance:'red'|'blue'
  autoAim:boolean; turretRate:number; intakeMaterial:'gecko'|'silicone'
  shootWhileMoving:boolean
}
export const defaultConfig: RobotConfig={rpm:312,wheel:3.78,gearing:1,mass:12,force:65,grip:.8,efficiency:.75,size:16,turnRate:180,capacity:8,preload:4,shotSpeed:240,shotAngle:55,shotInterval:.45,transferTime:.6,alliance:'red',autoAim:true,turretRate:240,intakeMaterial:'gecko',shootWhileMoving:false}
export interface RouteSample extends Point2D {heading:number; s:number; time:number; node?:number; segment:number}
export function profile(points:Waypoint[],config:RobotConfig) {
  const samples:RouteSample[]=[]
  const expanded=materializePathActions(points,analyzePath(points))
  points=expanded.points
  const decisions=expanded.decisions
  const actions=routeActions(points,config.shootWhileMoving)
  const actionConflicts=points.flatMap((p,node)=>{const message=actionResourceConflict(actionPlan(p,node,points.length,config.shootWhileMoving).concurrent);return message?[{node,message}]:[]})
  for(let i=0;i<points.length-1;i++) {
    const pts=segmentPoints(points,i)
    const arc=curveArc(...pts),joinT=arc.parameterAt(.68)
    const joinHeading=tangentDegrees(bezierAt(pts,joinT-.001),bezierAt(pts,joinT+.001))
    for(let j= i===0 ? 0:1;j<=80;j++) {
      const t=j/80, p=bezierAt(pts,t)
      const before=bezierAt(pts,Math.max(0,t-.001)),after=bezierAt(pts,Math.min(1,t+.001))
      const tangent=tangentDegrees(before,after), type=decisions[i].type
      const progress=arc.completionAt(t)
      let heading=points[i].heading+shortestAngle(points[i].heading,points[i+1].heading)*progress
      if(type==='constant') heading=points[i].heading
      if(type==='tangent') heading=tangent
      if(type==='piecewise') {
        heading=progress<.68 ? tangent : joinHeading+shortestAngle(joinHeading,points[i+1].heading)*(progress-.68)/.32
      }
      const prev=samples.at(-1)
      samples.push({...p,heading,segment:expanded.segments[i],s:(prev?.s??0)+(prev?distance(prev,p):0),time:0,...(j===80?{node:i+1}:i===0&&j===0?{node:0}:{})})
    }
  }
  const maxSpeed=config.rpm/config.gearing*Math.PI*config.wheel/60*config.efficiency
  const acceleration=Math.min(config.force/config.mass,config.grip*9.81)*39.3701
  const speeds=samples.map((sample,i)=>{
    if(i===0 || i===samples.length-1 || sample.node!==undefined && actionPlan(points[sample.node],sample.node,points.length,config.shootWhileMoving).sequential.length) return 0
    const prev=samples[i-1],next=samples[i+1],ds=distance(prev,next)
    const bend=Math.abs(shortestAngle(tangentDegrees(prev,sample),tangentDegrees(sample,next)))*Math.PI/180
    const lateral=Math.sqrt(acceleration*Math.max(ds/2,.01)/Math.max(bend,.0001))
    const angular=config.turnRate*Math.max(distance(prev,sample),.001)/Math.max(Math.abs(shortestAngle(prev.heading,sample.heading)),.001)
    return Math.min(maxSpeed,lateral,angular)
  })
  for(let i=1;i<speeds.length;i++) speeds[i]=Math.min(speeds[i],Math.sqrt(speeds[i-1]**2+2*acceleration*(samples[i].s-samples[i-1].s)))
  for(let i=speeds.length-2;i>=0;i--) speeds[i]=Math.min(speeds[i],Math.sqrt(speeds[i+1]**2+2*acceleration*(samples[i+1].s-samples[i].s)))
  for(let i=1;i<samples.length;i++) samples[i].time=samples[i-1].time+2*(samples[i].s-samples[i-1].s)/Math.max(.01,speeds[i]+speeds[i-1])
  const drive=samples.at(-1)?.time??0
  const nodeTime=(node:number)=>samples.find(s=>s.node===node)?.time??0
  let min=0,max=0
  for(const step of compileRoute(points,config.shootWhileMoving)) {
    if(step.kind==='path'){const time=nodeTime(step.endIndex)-nodeTime(step.startIndex);min+=time;max+=time;continue}
    const duration=(action:PathAction)=>{
      const fixed=action.type==='wait'?(action.durationMs??100)/1000:action.type==='transfer'?config.transferTime:action.type==='flowerIntake'?1:0
      const upper=action.type==='shoot'?config.capacity*Math.max(config.shotInterval,config.transferTime)+2*config.shotSpeed/GRAVITY+2.5+180/config.turretRate:action.type==='transfer'?config.capacity*config.transferTime:fixed
      const cap=action.timeoutMs ? action.timeoutMs/1000:Infinity
      return {min:Math.min(fixed,cap),max:Math.min(upper,cap)}
    }
    if(step.kind==='action'){const d=duration(step.action);min+=d.min;max+=d.max}
    else {
      const time=nodeTime(step.path.endIndex)-nodeTime(step.path.startIndex)
      const branches=[step.action,...(step.additionalActions??[])].filter(a=>a.composition!=='deadline').map(duration)
      min+=Math.max(time,...branches.map(d=>d.min));max+=Math.max(time,...branches.map(d=>d.max))
    }
  }
  let wallCollision:RouteSample|undefined
  let supportCollision:{sample:RouteSample;name:string}|undefined
  const unsafeSegments=new Set<number>()
  let previous:RobotPose|undefined=points[0]
  for(const sample of samples){
    if(previous){
      // Profiling needs a collision flag, not the expensive invalid-start
      // projection used by playback. Dense obstructed strokes stay responsive.
      const initial=robotSupportCollision(previous,config.size)
      const wall=sweepWalls(previous,sample,config.size),support=initial?{solid:initial}:sweepSupports(previous,sample,config.size)
      if(wall.hit||support.solid)unsafeSegments.add(sample.segment)
      if(!wallCollision&&wall.hit)wallCollision=sample
      if(!supportCollision&&support.solid)supportCollision={sample,name:support.solid.name}
    }
    previous=sample
  }
  const shotStops=points.flatMap((point,node)=>nodeActions(point).some(a=>a.type==='shoot')?[{node,time:nodeTime(node)}]:[])
  return {samples,drive,min,max,maxSpeed,acceleration,wallCollision,supportCollision,shotStops,unsafeSegments:[...unsafeSegments],actions,points,actionConflicts}
}
export interface Ball extends Point2D {id:number;vx:number;vy:number;z:number;vz:number;kind:'pollen'|'nectar';color?:'red'|'blue';flower?:number;released?:boolean;shot?:boolean}
export interface SimState {x:number;y:number;heading:number;time:number;driveTime:number;index:number;inventory:number;hopper:CellPiece[];intake:boolean;balls:Ball[];hives:Hive[];action:string;remaining:number;shotClock:number;shooting:boolean;finished:boolean;warning:string;processed:number[];nextId:number;
  tasks?:{action:PathAction;remaining:number;timeout:number;done:boolean}[]; pendingActions?:PathAction[]; pendingConcurrent?:PathAction[];
  activeType?:string; timeout:number; concurrent?:{endNode:number;mode:'parallel'|'deadline'};
  blocked:boolean;turretYaw:number;turretElevation:number;turretReady:boolean;feeder:{piece:CellPiece;remaining:number}[];
  driveVx:number;driveVy:number;travelled:number;leftWall:boolean;autoTimedOut:boolean }
// Asset orientation: blue alliance at top, red at bottom; no rotation is applied.
export const flowers=[{x:48,y:141},{x:141,y:96},{x:96,y:3},{x:3,y:48}]
export const hiveCenters=[{x:72,y:59.25},{x:72,y:84.75}]
export function resetSimulation(points:Waypoint[],config:RobotConfig):SimState {
  const balls:Ball[]=[]
  const add=(x:number,y:number,flower?:number,z=1.4)=>balls.push({id:balls.length,x,y,z,vx:0,vy:0,vz:0,kind:'pollen',flower})
  for(let i=0;i<4;i++){add(1.5,142.5-i*2.8);add(142.5,1.5+i*2.8)}
  flowers.forEach((f,i)=>{for(let j=0;j<4;j++)add(f.x,f.y,i,1.4+j*2.8)})
  const hopper=Array.from({length:Math.floor(Math.min(config.capacity,config.preload))},()=>({kind:'pollen' as const}))
  const start={x:points[0]?.x??18,y:points[0]?.y??18,heading:points[0]?.heading??0},wallBlocked=!clearsWalls(start,config.size),support=robotSupportCollision(start,config.size),blocked=wallBlocked||!!support
  return {...constrainSupports(constrainPose(start,config.size),config.size),time:0,driveTime:0,index:0,inventory:hopper.length,hopper,intake:false,balls,hives:[createHive(1,'red'),createHive(0,'blue')],action:blocked?'Blocked':'Ready',remaining:0,shotClock:0,shooting:false,finished:false,warning:wallBlocked?'Start footprint intersects a wall. Move the start inward, then reset.':support?`Start overlaps ${support.name}. Move the start clear, then reset.`:'',processed:[],nextId:24,timeout:Infinity,blocked,turretYaw:start.heading,turretElevation:config.shotAngle,turretReady:false,feeder:[],driveVx:0,driveVy:0,travelled:0,leftWall:false,autoTimedOut:false}
}
export function turretSolution(s:Pick<SimState,'x'|'y'|'hives'>&Partial<Pick<SimState,'driveVx'|'driveVy'>>,config:RobotConfig) {
  const index=config.alliance==='red'?0:1,opening=cellOpening(s.hives[index],s.hives[index].side)
  const target={x:hiveCenters[index].x+opening.x,y:hiveCenters[index].y}
  let yaw=Math.atan2(target.y-s.y,target.x-s.x),range=Math.max(.01,distance(s,target))
  const length=config.size*.42,v2=config.shotSpeed**2
  let elevation=config.shotAngle*Math.PI/180,reachable=true
  if(config.autoAim)for(let i=0;i<10;i++){
    const horizontal=Math.max(.01,range-length*Math.cos(elevation)),rise=opening.z-(18+length*Math.sin(elevation))
    const discriminant=v2*v2-GRAVITY*(GRAVITY*horizontal*horizontal+2*rise*v2)
    reachable=discriminant>=0
    if(!reachable)break
    elevation=Math.atan((v2+Math.sqrt(discriminant))/(GRAVITY*horizontal))
    if(config.shootWhileMoving){
      const vz=config.shotSpeed*Math.sin(elevation),d=vz*vz-2*GRAVITY*(opening.z-18-length*Math.sin(elevation))
      const flight=(vz+Math.sqrt(Math.max(0,d)))/GRAVITY
      const dx=target.x-s.x-(s.driveVx??0)*flight,dy=target.y-s.y-(s.driveVy??0)*flight
      yaw=Math.atan2(dy,dx);range=Math.max(.01,Math.hypot(dx,dy))
    }
  }
  return {yaw:yaw*180/Math.PI,elevation:elevation*180/Math.PI,reachable}
}
export function stepSimulation(s:SimState,points:Waypoint[],config:RobotConfig,route:ReturnType<typeof profile>,dt:number) {
  if(s.finished || s.blocked)return
  if(route.actionConflicts.length){s.blocked=true;s.intake=false;s.shooting=false;s.action='Blocked';s.warning=route.actionConflicts[0].message;return}
  points=route.points
  dt=Math.min(dt,Math.max(0,30-s.time));s.time+=dt
  const current=route.samples[s.index]
  if(!current){s.finished=true;return}
  s.tasks??=[]
  // Keep the public simulation state's direct shooting control compatible.
  if(s.shooting&&!s.tasks.some(t=>!t.done&&t.action.type==='shoot')){
    s.tasks.forEach(t=>{t.done=true})
    s.tasks.push({action:{type:'shoot'},remaining:0,timeout:s.timeout,done:false})
  }
  const activeBusy=()=>s.tasks!.some(t=>!t.done)
  const busy=()=>activeBusy()||!!s.pendingActions?.length||!!s.pendingConcurrent?.length
  const finishTask=(task:NonNullable<SimState['tasks']>[number])=>{
    task.done=true
    if(task.action.type==='intake'&&task.action.composition==='deadline')s.intake=false
  }
  const sync=()=>{
    const active=s.tasks!.filter(t=>!t.done)
    s.shooting=active.some(t=>t.action.type==='shoot')
    s.remaining=Math.max(0,...active.map(t=>t.remaining));s.timeout=Math.min(Infinity,...active.map(t=>t.timeout))
    s.activeType=active[0]?.action.type
  }
  const endAction=()=>{s.tasks!.forEach(finishTask);s.pendingActions=[];s.pendingConcurrent=[];sync()}
  // A parallel group joins both branches. A deadline cancels its action branch.
  if(s.concurrent && current.node===s.concurrent.endNode) {
    s.tasks.filter(t=>t.action.composition==='deadline').forEach(finishTask)
    if(!busy())s.concurrent=undefined
  }
  if(!s.concurrent && !busy() && current.node!==undefined && !s.processed.includes(current.node)) {
    s.processed.push(current.node)
    const plan=actionPlan(points[current.node],current.node,points.length,config.shootWhileMoving)
    s.pendingActions=plan.sequential;s.pendingConcurrent=plan.concurrent
  }
  const startAction=(original:PathAction,concurrent:boolean)=>{
      const action={...original,composition:concurrent?original.composition:'sequential' as const}
      const task={action,remaining:action.type==='wait'?(action.durationMs??100)/1000:action.type==='transfer'?config.transferTime:action.type==='flowerIntake'?1:0,
        timeout:action.timeoutMs&&action.type!=='intake'&&action.type!=='wait'?action.timeoutMs/1000:Infinity,done:false}
      s.tasks!.push(task);s.action=action.type
      if(action.type==='intake')s.intake=true
      if(action.type==='intake'&&action.composition!=='deadline')task.done=true
      if(action.type==='shoot'){s.shotClock=0;s.intake=false}
      if(action.type==='flowerIntake') {
        s.intake=true
        // Pollen drops from the nearest flower; the intake stays latched (like
        // the intake command) so pieces that land or roll late are still collected.
        const nearest=flowers.reduce((a,b)=>distance(a,s)<=distance(b,s)?a:b),index=flowers.indexOf(nearest)
        if(distance(nearest,s)<config.size/2+10)for(const ball of s.balls)if(ball.flower===index){
          ball.flower=undefined;const h=s.heading*Math.PI/180;ball.vx=-Math.cos(h)*6;ball.vy=-Math.sin(h)*6
        }
      }
  }
  while(!activeBusy()&&s.pendingActions?.length)startAction(s.pendingActions.shift()!,false)
  if(!activeBusy()&&!s.pendingActions?.length&&s.pendingConcurrent?.length&&current.node!==undefined){
    const actions=s.pendingConcurrent;s.pendingConcurrent=[]
    s.concurrent={mode:actions.some(a=>a.composition==='parallel')?'parallel':'deadline',endNode:nextPathEnd(points,current.node)}
    for(const action of actions)startAction(action,true)
    if(actions.some(a=>a.type==='intake'||a.type==='flowerIntake'))s.intake=true
  }
  for(const task of s.tasks)if(!task.done){task.timeout-=dt;if(task.timeout<=0)finishTask(task)}
  sync()
  // A two-stage roller handoff feeds a sensor-indexed queue, one piece at a time.
  if(s.feeder.length){s.feeder[0].remaining-=dt;if(s.feeder[0].remaining<=0)s.hopper.push(s.feeder.shift()!.piece)}
  const aim=turretSolution(s,config),yawError=shortestAngle(s.turretYaw,aim.yaw),pitchError=aim.elevation-s.turretElevation
  s.turretYaw+=clamp(yawError,-config.turretRate*dt,config.turretRate*dt)
  s.turretElevation+=clamp(pitchError,-90*dt,90*dt)
  s.turretReady=Math.abs(shortestAngle(s.turretYaw,aim.yaw))<1 && Math.abs(s.turretElevation-aim.elevation)<1
  if(s.shooting) {
    s.shotClock-=dt
    if(s.hopper.length>0 && s.shotClock<=0 && s.turretReady){
      const angle=s.turretYaw*Math.PI/180,elevation=s.turretElevation*Math.PI/180,length=config.size*.42
      const piece=s.hopper.shift()!
      s.balls.push({...piece,id:s.nextId++,shot:true,x:s.x+Math.cos(angle)*length*Math.cos(elevation),y:s.y+Math.sin(angle)*length*Math.cos(elevation),z:18+length*Math.sin(elevation),vx:Math.cos(angle)*config.shotSpeed*Math.cos(elevation)+(config.shootWhileMoving?s.driveVx:0),vy:Math.sin(angle)*config.shotSpeed*Math.cos(elevation)+(config.shootWhileMoving?s.driveVy:0),vz:config.shotSpeed*Math.sin(elevation)})
      s.inventory=s.hopper.length+s.feeder.length;s.shotClock=config.shotInterval
    }
    if(s.inventory===0 && !s.balls.some(b=>b.z>pieceRadius(b.kind)+.2 && b.flower===undefined) && !s.hives.some(h=>h.tipping||h.spilling!==null))s.tasks.filter(t=>t.action.type==='shoot').forEach(finishTask)
  }
  for(const task of s.tasks)if(!task.done&&task.action.type!=='shoot'){
    task.remaining=Math.max(0,task.remaining-dt)
    if(task.remaining===0&&!(task.action.type==='transfer'&&s.feeder.length)&&!(task.action.type==='intake'&&task.action.composition==='deadline'))finishTask(task)
  }
  sync()
  const groupAtEnd=s.concurrent && current.node===s.concurrent.endNode
  const markerPending=current.node!==undefined&&nodeActions(points[current.node]).length>0&&!s.processed.includes(current.node)
  if(!markerPending&&(!busy() || s.concurrent && !groupAtEnd)) {
    s.action=busy()?`${s.activeType} + Drive`:'Drive'
    const old={x:s.x,y:s.y,heading:s.heading},oldTime=s.driveTime,oldIndex=s.index
    s.driveTime+=dt
    while(s.index<route.samples.length-1 && route.samples[s.index+1].time<=s.driveTime){
      s.index++
      const next=route.samples[s.index]
      if(next.node!==undefined && (nodeActions(points[next.node]).length && !s.processed.includes(next.node) || s.concurrent?.endNode===next.node)){s.driveTime=next.time;break}
    }
    const a=route.samples[s.index],b=route.samples[s.index+1]??a
    const atAction=a.node!==undefined && nodeActions(points[a.node]).length && !s.processed.includes(a.node)
    const t=atAction?0:clamp((s.driveTime-a.time)/Math.max(.00001,b.time-a.time),0,1)
    const target={x:a.x+(b.x-a.x)*t,y:a.y+(b.y-a.y)*t,heading:a.heading+shortestAngle(a.heading,b.heading)*t}
    let previous=old,previousTime=oldTime
    const sweepTargets=[...route.samples.slice(oldIndex+1,s.index+1),{...target,time:s.driveTime}]
    for(const next of sweepTargets){
      const wall=sweepWalls(previous,next,config.size),support=sweepSupports(previous,next,config.size)
      const contact=support.solid&&support.fraction<=wall.fraction?support:wall
      // A swept target may be a RouteSample with its own drive-only time.
      // Copy geometry only; the elapsed clock must retain action delays.
      s.x=contact.pose.x;s.y=contact.pose.y;s.heading=contact.pose.heading
      if('solid' in contact?contact.solid:contact.hit){s.driveTime=previousTime+(next.time-previousTime)*contact.fraction;s.index=oldIndex;s.blocked=true;s.finished=false;s.intake=false;endAction();s.concurrent=undefined;s.action='Blocked';s.warning='solid' in contact?`${contact.solid?.name} contact: route stopped. Move the route clear and reset.`:'Wall contact: route stopped. Move the route inward and reset.';return}
      previous=next;previousTime=next.time
    }
    s.driveVx=(s.x-old.x)/dt;s.driveVy=(s.y-old.y)/dt;s.travelled+=distance(old,s)
    if(s.travelled>.05&&clearsWalls(s,config.size+.02))s.leftWall=true
    s.turretYaw+=shortestAngle(old.heading,s.heading) // chassis motion carries the turret; motor counter-tracks next tick
    if(s.index===route.samples.length-1 && !busy() && !s.concurrent && !(a.node!==undefined && nodeActions(points[a.node]).length && !s.processed.includes(a.node)) && !s.balls.some(ball=>ball.flower===undefined && ball.z>pieceRadius(ball.kind)+.2) && !s.hives.some(h=>h.tipping || h.spilling!==null)){s.finished=true;s.intake=false}
  } else {s.driveVx=0;s.driveVy=0}
  const remove=new Set<number>()
  for(const ball of s.balls){
    if(ball.flower!==undefined)continue
    const old={x:ball.x,y:ball.y,z:ball.z},r=pieceRadius(ball.kind)
    ball.x+=ball.vx*dt;ball.y+=ball.vy*dt
    if(ball.z>r || ball.vz>0){ball.z+=ball.vz*dt-GRAVITY*dt*dt/2;ball.vz-=GRAVITY*dt}
    s.hives.forEach((h,i)=>{
      const center=hiveCenters[i],local=hiveToLocal(h,ball.x-center.x,ball.z),before=hiveToLocal(h,old.x-center.x,old.z)
      const sign=h.side===0?-1:1
      const oldPlane=sign*before.x-HIVE.outer,newPlane=sign*local.x-HIVE.outer
      if(!ball.released && oldPlane>0 && newPlane<=0 && cellOpening(h,h.side).nz>0) {
        const t=oldPlane/(oldPlane-newPlane),y=old.y+(ball.y-old.y)*t-center.y,z=before.z+(local.z-before.z)*t
        if(Math.abs(y)<HIVE.openingWidth/2-r && Math.abs(z-HIVE.openingCenter)<HIVE.openingHeight/2-r){
          addCellPiece(h,h.side,{kind:ball.kind,color:ball.color});remove.add(ball.id);return
        }
      }
      // Approximate rigid cell skins: balls striking backs / side walls rebound
      // instead of passing through the basket. Opening entry is handled above.
      if(Math.abs(ball.y-center.y)<HIVE.openingWidth/2+r && local.z>-1.2-r && local.z<19.8+r) {
        for(const side of [-1,1]) {
          const u=side*local.x,previous=side*before.x
          if(Math.abs(u-HIVE.inner)<r+.3 && (previous-HIVE.inner)*(u-HIVE.inner)<=0){
            const nx=side*Math.cos(h.angle),nz=side*Math.sin(h.angle),dot=ball.vx*nx+ball.vz*nz
            ball.vx-=1.4*dot*nx;ball.vz-=1.4*dot*nz
            ball.x=old.x;ball.z=old.z
          }
        }
      }
    })
    if(remove.has(ball.id))continue
    bounceOffSupports(ball,old,r)
    if(ball.z<r){ball.z=r;ball.vz=Math.abs(ball.vz)>16?-ball.vz*(ball.released ? .5 : .32):0}
    if(ball.z<=r+.01 && ball.vz===0){const speed=Math.hypot(ball.vx,ball.vy),next=Math.max(0,speed-(ball.released?6:9)*dt);ball.vx*=speed?next/speed:0;ball.vy*=speed?next/speed:0}
    for(const axis of ['x','y'] as const){const velocity=axis==='x'?'vx':'vy';if(ball[axis]<r || ball[axis]>144-r){ball[axis]=clamp(ball[axis],r,144-r);ball[velocity]*=-.48}}
    const h=s.heading*Math.PI/180,dx=ball.x-s.x,dy=ball.y-s.y,forward=dx*Math.cos(h)+dy*Math.sin(h),lateral=-dx*Math.sin(h)+dy*Math.cos(h),half=config.size/2
    const inMouth=forward>=half-3 && forward<=half+r+1 && Math.abs(lateral)<half-2
    if(ball.z<4 && inMouth && s.intake && s.inventory<config.capacity){
      remove.add(ball.id);s.feeder.push({piece:{kind:ball.kind,color:ball.color},remaining:config.transferTime});s.inventory=s.hopper.length+s.feeder.length
    } else if(ball.z<4 && Math.abs(forward)<half+r && Math.abs(lateral)<half+r){
      // Oriented chassis contact: balls at the side/back are pushed, not collected.
      const f=clamp(forward,-half,half),l=clamp(lateral,-half,half)
      let nx=forward-f,ny=lateral-l,d=Math.hypot(nx,ny)
      let surfaceF=f,surfaceL=l
      if(d<1e-6){if(half-Math.abs(forward)<half-Math.abs(lateral)){nx=Math.sign(forward)||1;ny=0;surfaceF=nx*half}else{nx=0;ny=Math.sign(lateral)||1;surfaceL=ny*half}d=1}
      if(d<r || Math.abs(forward)<=half&&Math.abs(lateral)<=half){nx/=d;ny/=d;const wx=nx*Math.cos(h)-ny*Math.sin(h),wy=nx*Math.sin(h)+ny*Math.cos(h)
        ball.x=s.x+(surfaceF+nx*r)*Math.cos(h)-(surfaceL+ny*r)*Math.sin(h);ball.y=s.y+(surfaceF+nx*r)*Math.sin(h)+(surfaceL+ny*r)*Math.cos(h)
        const closing=(ball.vx-s.driveVx)*wx+(ball.vy-s.driveVy)*wy
        if(closing<0){ball.vx-=1.35*closing*wx;ball.vy-=1.35*closing*wy}}
    }
  }
  s.balls=s.balls.filter(b=>!remove.has(b.id))
  for(let i=0;i<s.balls.length;i++)for(let j=i+1;j<s.balls.length;j++){
    const a=s.balls[i],b=s.balls[j],d=Math.hypot(b.x-a.x,b.y-a.y,b.z-a.z),r=pieceRadius(a.kind)+pieceRadius(b.kind)
    if(a.flower!==undefined || b.flower!==undefined || d>=r || d<.001)continue
    const nx=(b.x-a.x)/d,ny=(b.y-a.y)/d,nz=(b.z-a.z)/d,ia=1/pieceMass(a.kind),ib=1/pieceMass(b.kind),overlap=r-d
    a.x-=nx*overlap*ia/(ia+ib);a.y-=ny*overlap*ia/(ia+ib);a.z=Math.max(pieceRadius(a.kind),a.z-nz*overlap*ia/(ia+ib))
    b.x+=nx*overlap*ib/(ia+ib);b.y+=ny*overlap*ib/(ia+ib);b.z=Math.max(pieceRadius(b.kind),b.z+nz*overlap*ib/(ia+ib))
    const speed=(a.vx-b.vx)*nx+(a.vy-b.vy)*ny+(a.vz-b.vz)*nz
    if(speed>0){const impulse=1.38*speed/(ia+ib);a.vx-=nx*impulse*ia;a.vy-=ny*impulse*ia;a.vz-=nz*impulse*ia;b.vx+=nx*impulse*ib;b.vy+=ny*impulse*ib;b.vz+=nz*impulse*ib}
  }
  // Ball/chassis or ball/ball separation can push a piece into a static solid.
  // Re-project those contacts before rendering, including compound tube joints.
  for(const ball of s.balls)if(ball.flower===undefined){
    for(let i=0;i<3;i++)if(!bounceOffSupports(ball,{x:ball.x,y:ball.y,z:ball.z},pieceRadius(ball.kind)))break
    ball.z=Math.max(pieceRadius(ball.kind),ball.z)
    ball.x=clamp(ball.x,pieceRadius(ball.kind),144-pieceRadius(ball.kind));ball.y=clamp(ball.y,pieceRadius(ball.kind),144-pieceRadius(ball.kind))
  }
  s.hives.forEach((h,i)=>{
    const center=hiveCenters[i]
    for(const spill of advanceHive(h,dt))s.balls.push({...spill,id:s.nextId++,x:center.x+spill.x,y:center.y+spill.y,released:true})
  })
  const half=config.size/2
  s.warning=!aim.reachable&&s.shooting?'Launch speed cannot reach the cell; manual-angle shots may miss':Math.abs(s.x-72)<23+half && Math.abs(s.y-72)<19+half?'HIVE envelope overlap: check height and frame clearance':''
  if(s.time>=30&&!s.finished){s.finished=true;s.autoTimedOut=true;s.intake=false;s.shooting=false;s.activeType=undefined;s.concurrent=undefined;s.driveVx=0;s.driveVy=0;s.warning='30-second AUTO limit reached. Later actions were not run; transition settling is not modeled.'}
}
