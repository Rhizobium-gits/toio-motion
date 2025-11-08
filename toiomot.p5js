// === Toio群シミュレーション: 進行方向を90°回転 ===

const GRID_W = 1200;
const GRID_H = 1200;
const ROBOT_W = 30;
const ROBOT_H = 70;
const ROBOT_SPEED = 1.0;
const SAFE_RADIUS = 80;
const NEIGHBOR_DIST = 100;
const FOOT_W = 210;
const FOOT_H = 80;

let robots = [];
let groups = [];
let feet = [];

function setup() {
  createCanvas(GRID_W, GRID_H);
  rectMode(CENTER);
  noStroke();

  // 初期配置（安全距離確保）
  let attempts = 0;
  while (robots.length < 60 && attempts < 5000) {
    let x = random(200, width - 200);
    let y = random(200, height - 200);
    let ok = true;
    for (let r of robots) {
      if (dist(x, y, r.pos.x, r.pos.y) < SAFE_RADIUS * 1.5) {
        ok = false;
        break;
      }
    }
    if (ok) robots.push(new Robot(x, y, -HALF_PI)); // 上向き
    attempts++;
  }

  groups = [{ id: 0, color: color(70, 130, 255), members: robots }];

  // 足（障害物）
  feet.push(new Foot(1000, 1000, createVector(-0.8, -0.7)));
}

function draw() {
  background(240);

  // 足を動かす
  for (let f of feet) {
    f.move();
    f.display();
  }

  // 群更新
  updateGroups();

  // 各群の動作
  for (let g of groups) {
    g.direction = computeGroupDirection(g);
    for (let r of g.members) {
      r.update(g.direction, feet, robots);
      r.display(g.color);
    }
  }
}

// ------------------------------------
// ロボットクラス
class Robot {
  constructor(x, y, angle) {
    this.pos = createVector(x, y);
    this.angle = angle;
  }

  update(groupDir, feet, allRobots) {
    // === 群方向に合わせて角度をスムーズに変化 ===
    let targetAngle = atan2(groupDir.y, groupDir.x);
    this.angle = lerpAngle(this.angle, targetAngle, 0.05);

    // === 進行方向を90°回転（右に） ===
    //   ※左にしたい場合は -PI/2 に変更
    let heading = p5.Vector.fromAngle(this.angle + HALF_PI).setMag(ROBOT_SPEED);

    // === 他トイオとの距離保持 ===
    let repel = createVector(0, 0);
    for (let other of allRobots) {
      if (other === this) continue;
      let d = p5.Vector.dist(this.pos, other.pos);
      if (d < SAFE_RADIUS && d > 0) {
        let diff = p5.Vector.sub(this.pos, other.pos);
        diff.normalize();
        diff.mult((SAFE_RADIUS - d) * 0.03);
        repel.add(diff);
      }
    }

    // === 足（障害物）回避 ===
    for (let f of feet) {
      if (
        abs(this.pos.x - f.pos.x) < FOOT_W / 2 + ROBOT_W &&
        abs(this.pos.y - f.pos.y) < FOOT_H / 2 + ROBOT_H
      ) {
        let diff = p5.Vector.sub(this.pos, f.pos);
        diff.normalize();
        repel.add(diff.mult(1.5));
      }
    }

    // === 位置更新 ===
    this.pos.add(p5.Vector.add(heading, repel));

    // 壁反射
    if (this.pos.x < SAFE_RADIUS || this.pos.x > width - SAFE_RADIUS)
      this.angle = PI - this.angle;
    if (this.pos.y < SAFE_RADIUS || this.pos.y > height - SAFE_RADIUS)
      this.angle = -this.angle;
  }

  display(col) {
    push();
    translate(this.pos.x, this.pos.y);
    rotate(this.angle); // 表示向きは元のまま
    fill(col);
    rect(0, 0, ROBOT_W, ROBOT_H, 5);
    fill(255, 255, 180);
    rect(ROBOT_W / 2 - 3, 0, 6, ROBOT_H * 0.3, 2);
    pop();
  }
}

// ------------------------------------
// 足クラス
class Foot {
  constructor(x, y, dir) {
    this.pos = createVector(x, y);
    this.vel = dir.copy().setMag(1.0);
  }

  move() {
    this.pos.add(this.vel);
    if (this.pos.x < 0 || this.pos.x > width) this.vel.x *= -1;
    if (this.pos.y < 0 || this.pos.y > height) this.vel.y *= -1;
  }

  display() {
    fill(255, 100, 100, 100);
    rect(this.pos.x, this.pos.y, FOOT_W, FOOT_H);
  }
}

// ------------------------------------
// 群の平均方向
function computeGroupDirection(group) {
  let avg = createVector(0, 0);
  for (let r of group.members) avg.add(p5.Vector.fromAngle(r.angle));
  avg.div(group.members.length);
  return avg.setMag(ROBOT_SPEED);
}

// ------------------------------------
// 群の分断・再結合
function updateGroups() {
  let unvisited = [...robots];
  let newGroups = [];
  let nextID = 0;

  while (unvisited.length > 0) {
    let cluster = [];
    let stack = [unvisited.pop()];
    while (stack.length > 0) {
      let current = stack.pop();
      cluster.push(current);
      for (let i = unvisited.length - 1; i >= 0; i--) {
        let other = unvisited[i];
        if (p5.Vector.dist(current.pos, other.pos) < NEIGHBOR_DIST) {
          stack.push(other);
          unvisited.splice(i, 1);
        }
      }
    }
    newGroups.push({
      id: nextID++,
      members: cluster,
      color: color(70, 130, 255),
    });
  }

  // 群再結合（人数優先）
  for (let i = 0; i < newGroups.length; i++) {
    for (let j = i + 1; j < newGroups.length; j++) {
      let g1 = newGroups[i];
      let g2 = newGroups[j];
      let d = dist(
        averagePos(g1.members).x,
        averagePos(g1.members).y,
        averagePos(g2.members).x,
        averagePos(g2.members).y
      );
      if (d < NEIGHBOR_DIST * 2) {
        if (g1.members.length > g2.members.length) {
          g1.members = g1.members.concat(g2.members);
          newGroups.splice(j, 1);
          j--;
        } else if (g1.members.length < g2.members.length) {
          g2.members = g2.members.concat(g1.members);
          newGroups.splice(i, 1);
          i--;
          break;
        }
      }
    }
  }

  groups = newGroups;
}

function averagePos(list) {
  let sum = createVector(0, 0);
  for (let r of list) sum.add(r.pos);
  sum.div(list.length);
  return sum;
}

function lerpAngle(a, b, t) {
  let diff = (b - a + PI) % (TWO_PI) - PI;
  return a + diff * t;
}
