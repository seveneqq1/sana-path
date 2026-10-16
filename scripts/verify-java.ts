// API compatibility smoke test using the REAL Pedro/Ivy binaries. FTC SDK and
// team-specific Constants are minimal compile-only stubs, not a hardware test.
import { mkdtemp, mkdir, writeFile, readdir } from 'node:fs/promises'
import { tmpdir } from 'node:os'
import { join, delimiter } from 'node:path'
import { spawnSync } from 'node:child_process'
import { generateJava } from '../src/lib/codegen'
import { analyzePath } from '../src/lib/optimizer'
import type { Waypoint } from '../src/types'
import { defaultConfig } from '../src/lib/simulation'

const root=await mkdtemp(join(tmpdir(),'biobuzz-java-'))
const run=(command:string,args:string[])=>{
  const result=spawnSync(command,args,{encoding:'utf8',cwd:root})
  if(result.status!==0)throw new Error(`${command} failed: ${result.stderr || result.error || result.stdout}`)
  return result.stdout
}
const artifacts=[
  ['pedro-core.jar','com/pedropathing/core/3.0.0/core-3.0.0.jar'],
  ['ivy-core.jar','com/pedropathing/ivy/core/1.1.1/core-1.1.1.jar'],
  ['ivy-pedro.aar','com/pedropathing/ivy/pedro/1.1.1/pedro-1.1.1.aar'],
]
await Promise.all(artifacts.map(async([name,path])=>{
  const response=await fetch(`https://repo.maven.apache.org/maven2/${path}`)
  if(!response.ok)throw new Error(`Maven download failed (${response.status}): ${path}`)
  await writeFile(join(root,name),new Uint8Array(await response.arrayBuffer()))
}))
run('jar',['xf','ivy-pedro.aar','classes.jar'])
const source=join(root,'source'),out=join(root,'classes')
await mkdir(source);await mkdir(out)
const files:Record<string,string>={
  'OpMode.java':`package com.qualcomm.robotcore.eventloop.opmode;
public abstract class OpMode {
  public Object hardwareMap;
  public Telemetry telemetry = new Telemetry();
  public abstract void init();
  public void start() {} public abstract void loop(); public void stop() {}
  public static class Telemetry { public void addData(String key, Object value) {} public void update() {} }
}`,
  'Autonomous.java':`package com.qualcomm.robotcore.eventloop.opmode;
public @interface Autonomous { String name(); }`,
  'Constants.java':`package org.firstinspires.ftc.teamcode.pedro;
import com.pedropathing.follower.Follower;
public class Constants { public static Follower create(Object hardwareMap) { return null; } }`,
  'VerifyHeadings.java':`package org.firstinspires.ftc.teamcode;
import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.paths.curves.Curve;
import com.pedropathing.paths.interpolator.Interpolator;
public class VerifyHeadings {
  private static double error(double a, double b) { return Math.atan2(Math.sin(b-a), Math.cos(b-a)); }
  public static void main(String[] args) throws Exception {
    PoseFactory p = PoseFactory.degrees();
    Curve curve = Paths.curve(p.of(0,0,0),p.of(0,70,0),p.of(15,100,0),p.of(100,100,0)).curve;
    java.lang.reflect.Method method = SanaAuto0.class.getDeclaredMethod("smoothPiecewise", double.class);
    method.setAccessible(true);
    double target = Math.toRadians(250);
    Interpolator heading = (Interpolator) method.invoke(new SanaAuto0(), target);
    double join = curve.parameter(0.68), start = curve.tangent(join).theta();
    for(int i=0;i<=100;i++) {
      double t=i/100.0, progress=curve.pathCompletion(t);
      double expected=progress<0.68 ? curve.tangent(t).theta() : start+error(start,target)*(progress-0.68)/0.32;
      if(Math.abs(error(expected,heading.interpolate(curve,t)))>1e-7)
        throw new AssertionError("Piecewise heading differs at t="+t);
    }
    if(Math.abs(error(heading.interpolate(curve,join-1e-7),heading.interpolate(curve,join+1e-7)))>1e-4)
      throw new AssertionError("Piecewise join is discontinuous");
    System.out.println("Pedro 3.0.0 piecewise runtime checks passed (tangent alignment, arc-length transition, continuity).");
  }
}`,
}
const node=(i:number):Waypoint=>({id:String(i),x:20+i*15,y:30+i*8,heading:i*30,interpolation:'auto',controlWeight:1})
for(let variant=0;variant<3;variant++){
  const points=Array.from({length:6},(_,i)=>node(i))
  points[1].interpolation='linear';points[2].interpolation='constant';points[3].interpolation='tangent';points[4].interpolation='piecewise'
  points[0].curve={endId:'1',c1:{x:25,y:42},c2:{x:29,y:47}}
  if(variant===2){points[2].controlPoints=[{x:55,y:60},{x:62,y:40},{x:70,y:58}];points[4].controlPoints=[]}
  points[0].action={type:'intake',composition:variant===0?'sequential':variant===1?'parallel':'deadline'}
  points[1].action={type:'shoot',timeoutMs:5000}
  points[2].action={type:'transfer',composition:'parallel'}
  points[3].action={type:'flowerIntake',composition:'deadline'}
  points[4].action={type:'wait',durationMs:700}
  points[5].action={type:'shoot',composition:'parallel'}
  if(variant===1)points[1].pathActions=[
    {id:'moving-shot',position:.25,action:{type:'shoot',composition:'parallel'}},
    {id:'intake',position:.25,action:{type:'intake',composition:'parallel'}},
    {id:'wait',position:.8,action:{type:'wait',durationMs:300,composition:'sequential'}},
  ]
  if(variant===1)points[2].pathActions=[
    {id:'deadline',position:.4,action:{type:'wait',durationMs:5000,composition:'deadline'}},
    {id:'parallel',position:.4,action:{type:'wait',durationMs:700,composition:'parallel'}},
  ]
  const name=`SanaAuto${variant}`
  files[`${name}.java`]=generateJava(points,analyzePath(points),{...defaultConfig,shootWhileMoving:variant===1,alliance:variant===2?'blue':'red',autoAim:variant!==2,intakeMaterial:variant===2?'silicone':'gecko'}).replace('public class SanaAuto ',`public class ${name} `)
}
for(const [name,text] of Object.entries(files))await writeFile(join(source,name),text)
const classpath=['pedro-core.jar','ivy-core.jar','classes.jar'].map(name=>join(root,name)).join(delimiter)
run('javac',['-Xlint:all','-cp',classpath,'-d',out,...(await readdir(source)).map(name=>join(source,name))])
console.log(run('java',['-cp',out+delimiter+classpath,'org.firstinspires.ftc.teamcode.VerifyHeadings']).trim())
console.log('Generated Java compiled against Pedro core 3.0.0 and Ivy 1.1.1 (3 route variants).')
console.log('FTC SDK/team Constants are compile-only stubs; validate hardware in your Android project.')
console.log(`Compatibility artifacts: ${root}`)
