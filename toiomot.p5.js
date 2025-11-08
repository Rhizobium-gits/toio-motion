// === Boids Rect OBB + Wall Bounce + Padding + Stagnation + Anti-Stall
// === Priority Alignment（優先度で向きを揃える）+ Density-Boosted Separation
// === Clockwise Ring Flow Field（時計回りリング水流を追加）
const sketch = (p) => {
  // ---- フィールド・台数 ----
  const W = 1200, H = 1200;
  const NUM = 30;

  // ---- ロボット寸法（短辺=進行方向）----
  const ROBOT_W = 30, ROBOT_H = 70;
  const HALF_A = ROBOT_W * 0.5;     // 進行方向 半幅
  const HALF_B = ROBOT_H * 0.5;     // 横方向 半幅

  // ---- Robot間Padding ----
  const PAD = 10;

  // ---- 速度・操舵 ----
  const MAX_SPEED = 3.0;
  const MIN_SPEED = 1.2;
  const MAX_FORCE = 0.08;            // 通常時の操舵上限
  const TURN_NOISE = 0.01;

  // 停滞時の追加
  const STALL_BIAS_FORCE = 0.12;
  const STALL_BIAS_JITTER = 0.7;
  const MAX_FORCE_STALL  = 0.14;

  // ---- 三ゾーン半径 ----
  const R_SEP_BASE = 60;
  const R_SEP   = R_SEP_BASE + PAD * 0.5; // 分離
  const R_ALIGN = 130;                    // 整列
  const R_COH   = 220;                    // 結合

  // ---- 三則の基礎重み ----
  const W_SEP = 1.6;
  const W_ALI = 0.9;
  const W_COH = 0.55;

  // ---- 高密度時の分離ブースト ----
  const SEP_CROWD_THRESH = 6;        // R_SEP内の人数がこれを超えると強化
  const SEP_DENSITY_GAIN = 0.25;     // 1人超過ごとに分離重みを+25%
  const SEP_CENTROID_PUSH = 0.8;     // 局所重心から外向きの押し出し

  // ---- 優先整列（高優先度リーダーに従う）----
  const W_PRI = 1.3;                 // 優先整列の基礎重み
  const PRIORITY_GAIN = 1.0;         // (Δpriority/NUM)でスケール

  // ---- 衝突解消・壁 ----
  const RESOLVE_PASSES = 3;
  const WALL_RESTITUTION = 1.0;

  // ---- 近傍探索 ----
  const CELL = Math.max(R_COH, ROBOT_H);

  // ---- 停滞判定 ----
  const STALL_WINDOW_MS   = 2000;
  const STALL_SAMPLE_MS   = 500;
  const STALL_THRESHOLD_A = 20;

  // ==== 追加：リング水流（時計回り） ===========================
  const FLOW_ENABLED   = true;   // 水流の有効/無効
  const FLOW_R0        = 500;    // 目標半径（≈600だが壁8px＋車体厚を避け少し内側）
  const FLOW_SIGMA     = 160;    // リング幅（大きいほど緩やかに効く）
  const FLOW_MAX_FORCE = 0.06;   // 水流による操舵上限
  const FLOW_RADIAL_PULL = 0.08;// リングへ戻す弱いラジアル引力
  // =============================================================

  class SpatialHash {
    constructor(cell){ this.cell=cell; this.map=new Map(); }
    key(ix,iy){ return ix + "," + iy; }
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
      this.id  = id;                 // 0..N-1（数値大ほど高優先）
      this.pri = id;
      this.pos = p.createVector(x,y);
      const a  = p.random(p.TWO_PI);
      this.vel = p.createVector(Math.cos(a), Math.sin(a)).mult(p.random(MIN_SPEED, MAX_SPEED));
      this.acc = p.createVector(0,0);

      // 停滞
      this.traj = [];
      this.lastSampleAt = 0;
      this.isStalled = false;
    }
    heading(){ return Math.atan2(this.vel.y, this.vel.x); }
    fwd(){ return this.vel.copy().normalize(); }
  }

  let bots = [];
  let hash = new SpatialHash(CELL);

  p.setup = () => {
    p.createCanvas(W,H);
    p.rectMode(p.CENTER);
    p.noStroke();

    // 0..NUM-1 をシャッフルして優先度を付与
    const ids = [...Array(NUM).keys()];
    for(let i=ids.length-1;i>0;i--){ const j=Math.floor(Math.random()*(i+1)); [ids[i],ids[j]]=[ids[j],ids[i]]; }

    for(let i=0;i<NUM;i++){
      bots.push(new Robot(p.random(W), p.random(H), ids[i]));
    }
  };

  p.draw = () => {
    p.background(245);
    drawWalls();

    // 停滞状態更新
    const now = p.millis();
    for(const b of bots){
      updateTrajectory(b, now);
      b.isStalled = isStalledByArea(b);
    }

    // 空間ハッシュ
    hash.clear();
    for(let i=0;i<bots.length;i++) hash.insert(i, bots[i].pos);

    // Boids＋優先整列＋停滞バイアス＋水流
    for(const b of bots) applyBoidsPriority(b);

    // 移動＋壁反射
    for(const b of bots) integrateWithWallBounce(b);

    // 非貫通（Padding込み）
    resolveCollisionsWithPadding();

    // クランプ
    for(const b of bots) clampInsideOBB(b);

    // 描画
    for(const b of bots) drawRobot(b);

    // HUD
    p.fill(20); p.textSize(14);
    p.text(`N=${NUM}  Field=${W}x${H}  Padding=${PAD}  Priority Align ON`, 16, 20);
    p.text(`Flow CW ring r≈${FLOW_R0}  sigma=${FLOW_SIGMA}  Fmax=${FLOW_MAX_FORCE}`, 16, 38);
    drawFlowGuide(); // 視覚ガイド（任意）
  };

  // ===== Boids＋密度ブースト分離＋優先整列＋停滞バイアス＋水流 =====
  function applyBoidsPriority(self){
    const neighIdx = hash.neighbors(self.pos);

    let sep = p.createVector(0,0);
    let ali = p.createVector(0,0);
    let coh = p.createVector(0,0);

    let nSep=0, nAli=0, nCoh=0;
    let sepCentroid = p.createVector(0,0);

    // 近傍で最も高優先度のリーダー（R_ALIGN内）
    let leader = null;

    for(const j of neighIdx){
      const other = bots[j];
      if(other===self) continue;

      const d = p5.Vector.sub(other.pos, self.pos);
      const r2 = d.magSq();
      if(r2 === 0) continue;
      const r  = Math.sqrt(r2);

      if(r < R_SEP){
        // 1/r^2 反発
        sep.add( d.copy().mult(-1/(r2)) );
        sepCentroid.add(other.pos);
        nSep++;
      }
      if(r < R_ALIGN){
        ali.add(other.fwd());
        nAli++;
        // リーダー更新
        if(!leader || other.pri > leader.pri) leader = other;
      }
      if(r < R_COH){
        coh.add(other.pos);
        nCoh++;
      }
    }

    // 整列（平均方向）
    if(nAli>0){
      ali.div(nAli);
      ali.sub(self.fwd());
    }

    // 結合（近傍重心へ）
    if(nCoh>0){
      coh.div(nCoh);
      coh.sub(self.pos);
    }

    // 分離：密度で重み強化＋局所重心から外向き押し出し
    let sepGain = 1.0;
    if(nSep > SEP_CROWD_THRESH){
      sepGain += SEP_DENSITY_GAIN * (nSep - SEP_CROWD_THRESH);
    }
    let sepOut = p.createVector(0,0);
    if(nSep > 0){
      sepCentroid.div(nSep);
      sepOut = p5.Vector.sub(self.pos, sepCentroid).normalize().mult(SEP_CENTROID_PUSH * Math.min(1, nSep/12));
    }

    // 優先整列：自分より優先度が高いリーダーの向きを強く採用
    let priAlign = p.createVector(0,0);
    if (leader && leader.pri > self.pri){
      const leaderDir = leader.fwd();
      priAlign = p5.Vector.sub(leaderDir, self.fwd());
      const priScale = W_PRI * (1 + PRIORITY_GAIN * (leader.pri - self.pri) / NUM);
      if (priAlign.magSq() > 0) priAlign.setMag(priScale);
    }

    // 合力
    let steer = p.createVector(0,0);
    if(sep.magSq()>0) steer.add(sep.normalize().mult(W_SEP * sepGain));
    if(sepOut.magSq()>0) steer.add(sepOut);
    if(ali.magSq()>0) steer.add(ali.normalize().mult(W_ALI));
    if(coh.magSq()>0) steer.add(coh.normalize().mult(W_COH));
    if(priAlign.magSq()>0) steer.add(priAlign);

    // 微小ノイズ
    steer.add(p.createVector(p.randomGaussian()*TURN_NOISE, p.randomGaussian()*TURN_NOISE));

    // ★ 追加：水流による操舵（時計回りリング）
    const fs = flowSteer(self);
    if (fs.magSq() > 0) steer.add(fs);

    // 停滞中は外向きランダムバイアス＆操舵上限緩和
    let limit = MAX_FORCE;
    if (self.isStalled){
      const bias = outwardRandomBias(self, neighIdx);
      steer.add(bias);
      limit = MAX_FORCE_STALL;
    }

    if(steer.mag() > limit) steer.setMag(limit);
    self.acc.add(steer);
  }

  // ★ 追加：リング水流ベクトル場と操舵
  function flowSteer(self){
    if(!FLOW_ENABLED) return p.createVector(0,0);

    const cx = W*0.5, cy = H*0.5;
    const dx = self.pos.x - cx, dy = self.pos.y - cy;
    const r  = Math.hypot(dx, dy);

    // 時計回り接線（(dy,-dx)）
    let tang = p.createVector(dy, -dx);
    if (tang.magSq() === 0) tang = p.createVector(0, -1);
    tang.normalize();

    // リング半径からの距離でガウシアン減衰
    const dr = r - FLOW_R0;
    const g  = Math.exp(-(dr*dr)/(2*FLOW_SIGMA*FLOW_SIGMA)); // 0..1

    // 目標速度は接線方向のMAX_SPEED，現在速度との差分で操舵
    const desired = tang.copy().mult(MAX_SPEED);
    const steer = desired.sub(self.vel);

    // 強さはgに応じて，遠くでも僅かに効かせる
    const fmax = FLOW_MAX_FORCE * (0.2 + 0.8*g);
    if (steer.mag() > fmax) steer.setMag(fmax);

    // リングへ戻す弱いラジアル項（外なら内向き，内なら外向き）
    if (Math.abs(dr) > 1){
      const rdir = p.createVector(dx, dy).normalize();
      const pull = rdir.mult(-Math.tanh(dr / FLOW_SIGMA) * FLOW_RADIAL_PULL * MAX_SPEED);
      steer.add(pull);
    }
    return steer;
  }

  // 停滞用：局所重心から外向き＋ランダム回転
  function outwardRandomBias(self, neighIdx){
    let centroid = p.createVector(0,0), cnt=0;
    for(const j of neighIdx){
      const o = bots[j];
      if(o===self) continue;
      const d = p5.Vector.sub(o.pos, self.pos);
      if(d.magSq() <= R_ALIGN*R_ALIGN){ centroid.add(o.pos); cnt++; }
    }
    let dir;
    if(cnt>0){
      centroid.div(cnt);
      dir = p5.Vector.sub(self.pos, centroid);
      if(dir.magSq()===0) dir = p.createVector(1,0);
      dir.normalize();
    }else{
      const a = p.random(p.TWO_PI);
      dir = p.createVector(Math.cos(a), Math.sin(a));
    }
    const jitter = p.random(-STALL_BIAS_JITTER, STALL_BIAS_JITTER);
    const h = Math.atan2(dir.y, dir.x) + jitter;
    return p.createVector(Math.cos(h), Math.sin(h)).mult(STALL_BIAS_FORCE);
  }

  // ===== 物理更新・壁反射 =====
  function integrateWithWallBounce(b){
    b.vel.add(b.acc);
    if(b.vel.mag() > MAX_SPEED) b.vel.setMag(MAX_SPEED);
    if(b.vel.mag() < MIN_SPEED) b.vel.setMag(MIN_SPEED);

    b.pos.add(b.vel);
    b.acc.mult(0);

    const {ex, ey} = obbAxisExtents(b);

    if (b.pos.x < ex){ b.pos.x = ex; b.vel.x = Math.abs(b.vel.x) * WALL_RESTITUTION; }
    else if (b.pos.x > W - ex){ b.pos.x = W - ex; b.vel.x = -Math.abs(b.vel.x) * WALL_RESTITUTION; }

    if (b.pos.y < ey){ b.pos.y = ey; b.vel.y = Math.abs(b.vel.y) * WALL_RESTITUTION; }
    else if (b.pos.y > H - ey){ b.pos.y = H - ey; b.vel.y = -Math.abs(b.vel.y) * WALL_RESTITUTION; }

    const sp = b.vel.mag();
    if(sp > MAX_SPEED) b.vel.setMag(MAX_SPEED);
    if(sp < MIN_SPEED) b.vel.setMag(MIN_SPEED);
  }
