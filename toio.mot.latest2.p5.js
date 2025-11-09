const sketch = (p) => {
  const W = 1200, H = 1200, NUM = 30;
  const ROBOT_W = 70, ROBOT_H = 30, PAD = 10;
  const HALF_A = ROBOT_W * 0.5, HALF_B = ROBOT_H * 0.5;
  
  // 速度・操舵
  const MAX_SPEED = 3.0, MIN_SPEED = 1.2, MAX_FORCE = 0.08, TURN_NOISE = 0.01;
  const STALL_BIAS_FORCE = 0.12, STALL_BIAS_JITTER = 0.7, MAX_FORCE_STALL = 0.14;
  
  // Boidsゾーン
  const R_SEP_BASE = 60, R_SEP = R_SEP_BASE + PAD * 0.5, R_ALIGN = 130, R_COH = 220;
  const W_SEP = 1.6, W_ALI = 0.9, W_COH = 0.55;
  
  // 密度・優先度
  const SEP_CROWD_THRESH = 6, SEP_DENSITY_GAIN = 0.25, SEP_CENTROID_PUSH = 0.8;
  const W_PRI = 1.3, PRIORITY_GAIN = 1.0;
  
  // 衝突・壁
  const RESOLVE_PASSES = 3, WALL_RESTITUTION = 1.0, CELL = Math.max(R_COH, ROBOT_H);
  
  // 停滞判定
  const STALL_WINDOW_MS = 2000, STALL_SAMPLE_MS = 500, STALL_THRESHOLD_A = 20;
  
  // リング水流
  const FLOW_ENABLED = true, FLOW_R0 = 500, FLOW_SIGMA = 160;
  const FLOW_MAX_FORCE = 0.06, FLOW_RADIAL_PULL = 0.08;

  class SpatialHash {
    constructor(cell){ this.cell=cell; this.map=new Map(); }
    key(ix,iy){ return `${ix},${iy}`; }
    clear(){ this.map.clear(); }
    insert(idx,pos){
      const ix = Math.floor(p.constrain(pos.x,0,W-1)/this.cell);
      const iy = Math.floor(p.constrain(pos.y,0,H-1)/this.cell);
      const k = this.key(ix,iy);
      if(!this.map.has(k)) this.map.set(k,[]);
      this.map.get(k).push(idx);
    }
    neighbors(pos){
      const ix = Math.floor(p.constrain(pos.x,0,W-1)/this.cell);
      const iy = Math.floor(p.constrain(pos.y,0,H-1)/this.cell);
      const out = [];
      for(let dy=-1; dy<=1; dy++){
        for(let dx=-1; dx<=1; dx++){
          const k = this.key(ix+dx, iy+dy);
          if(this.map.has(k)) out.push(...this.map.get(k));
        }
      }
      return out;
    }
  }

  class Robot {
    constructor(x,y,id){
      this.id = id; this.pri = id;
      this.pos = p.createVector(x,y);
      const a = p.random(p.TWO_PI);
      this.vel = p.createVector(Math.cos(a), Math.sin(a)).mult(p.random(MIN_SPEED, MAX_SPEED));
      this.acc = p.createVector(0,0);
      this.traj = []; this.lastSampleAt = 0; this.isStalled = false;
    }
    heading(){ return Math.atan2(this.vel.y, this.vel.x); }
    fwd(){ return this.vel.copy().normalize(); }
  }

  let bots = [], hash = new SpatialHash(CELL);

  p.setup = () => {
    p.createCanvas(W,H); p.rectMode(p.CENTER); p.noStroke();
    const ids = [...Array(NUM).keys()];
    for(let i=ids.length-1;i>0;i--){ const j=Math.floor(Math.random()*(i+1)); [ids[i],ids[j]]=[ids[j],ids[i]]; }
    for(let i=0;i<NUM;i++) bots.push(new Robot(p.random(W), p.random(H), ids[i]));
  };

  p.draw = () => {
    p.background(245);
    drawWalls();

    const now = p.millis();
    for(const b of bots){
      updateTrajectory(b, now);
      b.isStalled = isStalledByArea(b);
    }

    hash.clear();
    for(let i=0;i<bots.length;i++) hash.insert(i, bots[i].pos);
    
    for(const b of bots) applyBoidsPriority(b);
    for(const b of bots) integrateWithWallBounce(b);
    resolveCollisionsWithPadding();
    for(const b of bots) clampInsideOBB(b);
    for(const b of bots) drawRobot(b);

    p.fill(20); p.textSize(14);
    p.text(`N=${NUM}  Padding=${PAD}  Priority+Flow`, 16, 20);
    if(FLOW_ENABLED){ p.noFill(); p.stroke(120,120,180,120); p.strokeWeight(1.5); p.circle(W*0.5, H*0.5, FLOW_R0*2); }
  };

  function applyBoidsPriority(self){
    const neighIdx = hash.neighbors(self.pos);
    let sep = p.createVector(0,0), ali = p.createVector(0,0), coh = p.createVector(0,0);
    let nSep=0, nAli=0, nCoh=0, sepCentroid = p.createVector(0,0), leader = null;

    for(const j of neighIdx){
      const other = bots[j];
      if(other===self) continue;
      const d = p5.Vector.sub(other.pos, self.pos);
      const r2 = d.magSq(); if(r2 === 0) continue;
      const r = Math.sqrt(r2);

      if(r < R_SEP){ sep.add(d.copy().mult(-1/r2)); sepCentroid.add(other.pos); nSep++; }
      if(r < R_ALIGN){ ali.add(other.fwd()); nAli++; if(!leader || other.pri > leader.pri) leader = other; }
      if(r < R_COH){ coh.add(other.pos); nCoh++; }
    }

    if(nAli>0){ ali.div(nAli); ali.sub(self.fwd()); }
    if(nCoh>0){ coh.div(nCoh); coh.sub(self.pos); }

    let sepGain = 1.0 + (nSep > SEP_CROWD_THRESH ? SEP_DENSITY_GAIN * (nSep - SEP_CROWD_THRESH) : 0);
    let sepOut = p.createVector(0,0);
    if(nSep > 0){ sepCentroid.div(nSep); sepOut = p5.Vector.sub(self.pos, sepCentroid).normalize().mult(SEP_CENTROID_PUSH * Math.min(1, nSep/12)); }

    let priAlign = p.createVector(0,0);
    if(leader && leader.pri > self.pri){
      priAlign = p5.Vector.sub(leader.fwd(), self.fwd());
      const priScale = W_PRI * (1 + PRIORITY_GAIN * (leader.pri - self.pri) / NUM);
      if(priAlign.magSq() > 0) priAlign.setMag(priScale);
    }

    let steer = p.createVector(0,0);
    if(sep.magSq()>0) steer.add(sep.normalize().mult(W_SEP * sepGain));
    if(sepOut.magSq()>0) steer.add(sepOut);
    if(ali.magSq()>0) steer.add(ali.normalize().mult(W_ALI));
    if(coh.magSq()>0) steer.add(coh.normalize().mult(W_COH));
    if(priAlign.magSq()>0) steer.add(priAlign);
    steer.add(p.createVector(p.randomGaussian()*TURN_NOISE, p.randomGaussian()*TURN_NOISE));

    const fs = flowSteer(self);
    if(fs.magSq() > 0) steer.add(fs);

    let limit = MAX_FORCE;
    if(self.isStalled){ steer.add(outwardRandomBias(self, neighIdx)); limit = MAX_FORCE_STALL; }
    if(steer.mag() > limit) steer.setMag(limit);
    self.acc.add(steer);
  }

  function flowSteer(self){
    if(!FLOW_ENABLED) return p.createVector(0,0);
    const cx = W*0.5, cy = H*0.5, dx = self.pos.x - cx, dy = self.pos.y - cy, r = Math.hypot(dx, dy);
    let tang = p.createVector(dy, -dx);
    if(tang.magSq() === 0) tang = p.createVector(0, -1);
    tang.normalize();

    const dr = r - FLOW_R0, g = Math.exp(-(dr*dr)/(2*FLOW_SIGMA*FLOW_SIGMA));
    const desired = tang.copy().mult(MAX_SPEED), steer = desired.sub(self.vel);
    const fmax = FLOW_MAX_FORCE * (0.2 + 0.8*g);
    if(steer.mag() > fmax) steer.setMag(fmax);

    if(Math.abs(dr) > 1){
      const rdir = p.createVector(dx, dy).normalize();
      const pull = rdir.mult(-Math.tanh(dr / FLOW_SIGMA) * FLOW_RADIAL_PULL * MAX_SPEED);
      steer.add(pull);
    }
    return steer;
  }

  function outwardRandomBias(self, neighIdx){
    let centroid = p.createVector(0,0), cnt=0;
    for(const j of neighIdx){
      const o = bots[j]; if(o===self) continue;
      const d = p5.Vector.sub(o.pos, self.pos);
      if(d.magSq() <= R_ALIGN*R_ALIGN){ centroid.add(o.pos); cnt++; }
    }
    let dir = cnt>0 ? (centroid.div(cnt), p5.Vector.sub(self.pos, centroid)) : p.createVector(Math.cos(p.random(p.TWO_PI)), Math.sin(p.random(p.TWO_PI)));
    if(dir.magSq()===0) dir = p.createVector(1,0);
    dir.normalize();
    const h = Math.atan2(dir.y, dir.x) + p.random(-STALL_BIAS_JITTER, STALL_BIAS_JITTER);
    return p.createVector(Math.cos(h), Math.sin(h)).mult(STALL_BIAS_FORCE);
  }

  function integrateWithWallBounce(b){
    b.vel.add(b.acc);
    if(b.vel.mag() > MAX_SPEED) b.vel.setMag(MAX_SPEED);
    if(b.vel.mag() < MIN_SPEED) b.vel.setMag(MIN_SPEED);
    b.pos.add(b.vel); b.acc.mult(0);

    const {ex, ey} = obbAxisExtents(b);
    if(b.pos.x < ex){ b.pos.x = ex; b.vel.x = Math.abs(b.vel.x) * WALL_RESTITUTION; }
    else if(b.pos.x > W - ex){ b.pos.x = W - ex; b.vel.x = -Math.abs(b.vel.x) * WALL_RESTITUTION; }
    if(b.pos.y < ey){ b.pos.y = ey; b.vel.y = Math.abs(b.vel.y) * WALL_RESTITUTION; }
    else if(b.pos.y > H - ey){ b.pos.y = H - ey; b.vel.y = -Math.abs(b.vel.y) * WALL_RESTITUTION; }

    const sp = b.vel.mag();
    if(sp > MAX_SPEED) b.vel.setMag(MAX_SPEED);
    if(sp < MIN_SPEED) b.vel.setMag(MIN_SPEED);
  }

  function clampInsideOBB(b){
    const {ex, ey} = obbAxisExtents(b);
    b.pos.x = p.constrain(b.pos.x, ex, W - ex);
    b.pos.y = p.constrain(b.pos.y, ey, H - ey);
  }

  function obbAxisExtents(b){
    const th = b.heading(), ux = Math.cos(th), uy = Math.sin(th), vx = -Math.sin(th), vy = Math.cos(th);
    return {ex: Math.abs(ux)*HALF_A + Math.abs(vx)*HALF_B, ey: Math.abs(uy)*HALF_A + Math.abs(vy)*HALF_B};
  }

  function obbOf(b, aHalf = HALF_A, bHalf = HALF_B){
    const th = b.heading();
    return {c: b.pos.copy(), u: p.createVector(Math.cos(th), Math.sin(th)), v: p.createVector(-Math.sin(th), Math.cos(th)), a:aHalf, b:bHalf};
  }

  function satOverlapMTV(A,B){
    const axes = [A.u, A.v, B.u, B.v], dC = p5.Vector.sub(B.c, A.c);
    let minOverlap = Infinity, bestAxis = null, bestSign = 1;
    for(const ax0 of axes){
      const ax = ax0.copy().normalize();
      const RA = A.a*Math.abs(A.u.dot(ax)) + A.b*Math.abs(A.v.dot(ax));
      const RB = B.a*Math.abs(B.u.dot(ax)) + B.b*Math.abs(B.v.dot(ax));
      const dist = Math.abs(dC.dot(ax)), overlap = RA + RB - dist;
      if(overlap <= 0) return {intersect:false};
      if(overlap < minOverlap){ minOverlap = overlap; bestAxis = ax; bestSign = Math.sign(dC.dot(ax)) || 1; }
    }
    return {intersect:true, mtv: bestAxis.copy().mult(minOverlap * bestSign)};
  }

  function resolveCollisionsWithPadding(){
    const aPad = HALF_A + PAD, bPad = HALF_B + PAD;
    for(let pass=0; pass<RESOLVE_PASSES; pass++){
      hash.clear();
      for(let i=0;i<bots.length;i++) hash.insert(i, bots[i].pos);
      for(let i=0;i<bots.length;i++){
        const A = bots[i], obbA = obbOf(A, aPad, bPad);
        for(const j of hash.neighbors(A.pos)){
          if(j <= i) continue;
          const B = bots[j], res = satOverlapMTV(obbA, obbOf(B, aPad, bPad));
          if(!res.intersect) continue;
          const half = res.mtv.copy().mult(0.5);
          A.pos.sub(half); B.pos.add(half);
          A.vel.mult(0.98); B.vel.mult(0.98);
        }
      }
    }
  }

  function updateTrajectory(b, nowMs){
    if(b.traj.length === 0){ b.traj.push({t: nowMs, x: b.pos.x, y: b.pos.y}); b.lastSampleAt = nowMs; return; }
    const cutoff = nowMs - STALL_WINDOW_MS;
    while(b.traj.length && b.traj[0].t < cutoff) b.traj.shift();
    if(nowMs - b.lastSampleAt >= STALL_SAMPLE_MS){ b.traj.push({t: nowMs, x: b.pos.x, y: b.pos.y}); b.lastSampleAt = nowMs; }
  }

  function isStalledByArea(b){
    if(b.traj.length < 3) return false;
    const hull = convexHull(b.traj.map(e => ({x:e.x, y:e.y})));
    return polygonArea(hull) < STALL_THRESHOLD_A;
  }

  function convexHull(pts){
    const P = pts.slice().sort((a,b)=> a.x===b.x ? a.y-b.y : a.x-b.x);
    if(P.length <= 1) return P;
    const cross = (o,a,b)=> (a.x-o.x)*(b.y-o.y) - (a.y-o.y)*(b.x-o.x);
    const lower=[]; for(const p of P){ while(lower.length>=2 && cross(lower.at(-2), lower.at(-1), p)<=0) lower.pop(); lower.push(p); }
    const upper=[]; for(let i=P.length-1;i>=0;i--){ const p=P[i]; while(upper.length>=2 && cross(upper.at(-2), upper.at(-1), p)<=0) upper.pop(); upper.push(p); }
    return lower.slice(0,-1).concat(upper.slice(0,-1));
  }

  function polygonArea(poly){
    if(!poly || poly.length<3) return 0;
    let s=0; for(let i=0;i<poly.length;i++){ const j=(i+1)%poly.length; s += poly[i].x*poly[j].y - poly[j].x*poly[i].y; }
    return Math.abs(s)*0.5;
  }

  function drawWalls(){
    p.push(); p.noStroke(); p.fill(60); p.rectMode(p.CORNER);
    p.rect(0,0,W,8); p.rect(0,H-8,W,8); p.rect(0,0,8,H); p.rect(W-8,0,8,H); p.pop();
  }

  function drawRobot(b){
    const th = b.heading();
    p.push(); p.translate(b.pos.x, b.pos.y); p.rotate(th);
    p.fill(220); p.rect(2,2,ROBOT_W+6,ROBOT_H+6,6);
    p.fill(b.isStalled ? [230,60,60] : [70,130,255]);
    p.rect(0,0,ROBOT_W,ROBOT_H,5);
    p.fill(255,240,180); p.rect(ROBOT_W*0.5-3, 0, 6, ROBOT_H*0.30, 2);
    p.pop();
  }
};

new p5(sketch);
