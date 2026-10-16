import type { Point2D, Waypoint } from '../types'
import { bezierAt, distance, segmentPoints } from './geometry'

export function cubic(a: Point2D, b: Point2D, c: Point2D, d: Point2D, t: number): Point2D {
  const u = 1 - t
  return { x: u*u*u*a.x + 3*u*u*t*b.x + 3*u*t*t*c.x + t*t*t*d.x,
    y: u*u*u*a.y + 3*u*u*t*b.y + 3*u*t*t*c.y + t*t*t*d.y }
}

// Pedro 3 linear/piecewise interpolation uses travelled arc length, not raw
// Bézier parameter. This table keeps the planner's heading/timing estimate aligned.
export function curveArc(...pts: Point2D[]) {
  const count=240,lengths=[0]
  let previous=pts[0]
  for(let i=1;i<=count;i++){
    const point=bezierAt(pts,i/count);lengths.push(lengths[i-1]+distance(previous,point));previous=point
  }
  const total=lengths[count]||1
  const completionAt=(t:number)=>{
    const at=Math.max(0,Math.min(count,t*count)),index=Math.min(count-1,Math.floor(at))
    return (lengths[index]+(lengths[index+1]-lengths[index])*(at-index))/total
  }
  const parameterAt=(completion:number)=>{
    const target=Math.max(0,Math.min(1,completion))*total
    let low=0,high=count
    while(high-low>1){const mid=Math.floor((high+low)/2);if(lengths[mid]<target)low=mid;else high=mid}
    return (low+(target-lengths[low])/Math.max(1e-12,lengths[high]-lengths[low]))/count
  }
  return {completionAt,parameterAt}
}
const unit = (a: Point2D, b: Point2D) => {
  const length = distance(a, b) || 1
  return { x: (b.x-a.x)/length, y: (b.y-a.y)/length }
}
type Fit = { first: number; last: number; c1: Point2D; c2: Point2D }

const segmentDistance=(p:Point2D,a:Point2D,b:Point2D)=>{
  const dx=b.x-a.x,dy=b.y-a.y,t=Math.max(0,Math.min(1,((p.x-a.x)*dx+(p.y-a.y)*dy)/(dx*dx+dy*dy||1)))
  return Math.hypot(p.x-a.x-t*dx,p.y-a.y-t*dy)
}

// Adaptive least-squares cubic fitting. Shared split tangents guarantee G1 continuity.
function fit(samples: Point2D[], first: number, last: number, left: Point2D, right: Point2D, tolerance: number): Fit[] {
  const a = samples[first], d = samples[last]
  const lengths = [0]
  for (let i = first+1; i <= last; i++) lengths.push(lengths.at(-1)! + distance(samples[i-1], samples[i]))
  const total = lengths.at(-1)! || 1
  let ts = lengths.map(length => length/total)
  const solve=()=>{
  let aa=0, ab=0, bb=0, ax=0, bx=0
  ts.forEach((t, i) => {
    const u=1-t, b1=3*u*u*t, b2=3*u*t*t
    const v1={x:left.x*b1,y:left.y*b1}, v2={x:right.x*b2,y:right.y*b2}
    const base=cubic(a,a,d,d,t), p=samples[first+i]
    const residual={x:p.x-base.x,y:p.y-base.y}
    aa+=v1.x*v1.x+v1.y*v1.y; ab+=v1.x*v2.x+v1.y*v2.y; bb+=v2.x*v2.x+v2.y*v2.y
    ax+=v1.x*residual.x+v1.y*residual.y; bx+=v2.x*residual.x+v2.y*residual.y
  })
  const determinant=aa*bb-ab*ab
  let alpha=determinant ? (ax*bb-bx*ab)/determinant : total/3
  let beta=determinant ? (bx*aa-ax*ab)/determinant : total/3
  if (alpha<=0 || beta<=0 || alpha>total*2 || beta>total*2) alpha=beta=distance(a,d)/3
  const c1={x:a.x+alpha*left.x,y:a.y+alpha*left.y}, c2={x:d.x+beta*right.x,y:d.y+beta*right.y}
  return {c1,c2}
  }
  let {c1,c2}=solve(),worst=Infinity,split=Math.floor((first+last)/2)
  // Schneider-style Newton reparameterization lets a cubic describe a circular
  // sweep without adding anchors just because chord-length t was inaccurate.
  for(let iteration=0;iteration<9;iteration++) {
    worst=0
    ts.forEach((t,i)=>{const error=distance(cubic(a,c1,c2,d,t),samples[first+i]);if(error>worst){worst=error;split=first+i}})
    if(worst<=tolerance || iteration===8)break
    const next=ts.map((t,i)=>{
      if(i===0 || i===ts.length-1)return t
      const u=1-t,q=cubic(a,c1,c2,d,t),p=samples[first+i]
      const v={x:3*u*u*(c1.x-a.x)+6*u*t*(c2.x-c1.x)+3*t*t*(d.x-c2.x),y:3*u*u*(c1.y-a.y)+6*u*t*(c2.y-c1.y)+3*t*t*(d.y-c2.y)}
      const acc={x:6*u*(c2.x-2*c1.x+a.x)+6*t*(d.x-2*c2.x+c1.x),y:6*u*(c2.y-2*c1.y+a.y)+6*t*(d.y-2*c2.y+c1.y)}
      const denominator=v.x*v.x+v.y*v.y+(q.x-p.x)*acc.x+(q.y-p.y)*acc.y
      return Math.max(0,Math.min(1,t-((q.x-p.x)*v.x+(q.y-p.y)*v.y)/(denominator||1)))
    })
    if(next.some((t,i)=>i>0 && t<=next[i-1]))break
    ts=next;({c1,c2}=solve())
  }
  // Reverse-distance guard rejects lobes/overshoot between source samples.
  if(worst<=tolerance)for(let j=1;j<120;j++) {
    const p=cubic(a,c1,c2,d,j/120)
    let nearest=Infinity
    for(let k=first;k<last;k++)nearest=Math.min(nearest,segmentDistance(p,samples[k],samples[k+1]))
    if(nearest>tolerance){worst=nearest;split=Math.floor((first+last)/2);break}
  }
  if(worst<=tolerance || last-first===1) return [{first,last,c1,c2}]
  split=Math.max(first+1,Math.min(last-1,split))
  const tangent=unit(samples[split-1],samples[split+1])
  return [...fit(samples,first,split,left,{x:-tangent.x,y:-tangent.y},tolerance), ...fit(samples,split,last,tangent,right,tolerance)]
}

export function optimizeCurves(points: Waypoint[], tolerance=.2): Waypoint[] {
  if(points.length<3) return points
  const boundaries=[0,...points.flatMap((p,i)=>i>0 && (p.action || p.interpolation!=='auto' || p.pathActions?.length || points[i-1]?.pathActions?.length) ? [i]:[])]
  if(boundaries.at(-1)!==points.length-1) boundaries.push(points.length-1)
  const result: Waypoint[]=[]
  for(let b=1;b<boundaries.length;b++) {
    const first=boundaries[b-1], last=boundaries[b], samples: Point2D[]=[]
    for(let i=first;i<last;i++) {
      const pts=segmentPoints(points,i)
      for(let j=0;j<80;j++) samples.push(bezierAt(pts,j/80))
    }
    samples.push(points[last])
    const curves=fit(samples,0,samples.length-1,unit(samples[0],samples[1]),unit(samples.at(-1)!,samples.at(-2)!),tolerance)
    const chunk: Waypoint[]=[{...points[first],curve:undefined,controlPoints:undefined}]
    for(const curve of curves) {
      const end: Waypoint=curve.last===samples.length-1 ? {...points[last],curve:undefined,controlPoints:undefined} : {
        ...samples[curve.last],id:crypto.randomUUID(),heading:points[Math.min(last,first+Math.round(curve.last/80))].heading,interpolation:'auto',controlWeight:1,
      }
      chunk[chunk.length-1].curve={endId:end.id,c1:curve.c1,c2:curve.c2}
      chunk.push(end)
    }
    if(result.length) result[result.length-1]=chunk[0]
    result.push(...chunk.slice(result.length ? 1:0))
  }
  // Never add nodes just to meet a fitting tolerance: the source is already exact.
  return result.length<points.length ? result : points
}
