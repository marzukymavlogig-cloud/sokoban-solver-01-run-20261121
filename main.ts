// Push-optimal Sokoban solver for make.fun sokoban-solver-01-run-20261121.
//
// Idea: every level is one 1-wide hallway (the row the player starts in) plus
// 1-4 rooms off it, each with one door onto the hallway. A box pushed through
// a door into the hallway can never be pushed back, so every box stays in its
// room and the level optimum is the sum of the rooms' optima. The solver
// therefore detects the hallway row, splits the remaining floor into rooms,
// solves each room independently with a push-optimal A* (Hungarian assignment
// heuristic over BFS distances, dead-square + 2x2-freeze pruning), and stitches
// the room solutions together with hallway walks. Whole-level push-A* and a
// plain move-A* are fallbacks if the decomposition ever fails.
//
// Determinism: no clock, no randomness; every budget is an expansion count,
// every tie-break is a fixed order, so two runs write identical bytes.
// Node 22, standard library only, plain JavaScript (no TypeScript-only syntax).
import { mkdirSync, readdirSync, readFileSync, writeFileSync } from 'node:fs';
import { join } from 'node:path';

const I = process.env.MAKEFUN_INPUT_DIR ?? '/input';
const O = process.env.MAKEFUN_OUTPUT_DIR ?? '/output';

const DX = [0, 0, -1, 1];
const DY = [-1, 1, 0, 0];
const LET = ['u', 'd', 'l', 'r'];

// Letter for a single step from cell `from` to adjacent cell `to`.
function dirLetter(from, to, w) {
  const d = to - from;
  if (d === 1) return 'r';
  if (d === -1) return 'l';
  if (d === w) return 'd';
  return 'u';
}

function parseLevel(text) {
  const rows = text.split('\n').filter((r) => r.length > 0);
  const h = rows.length;
  let w = 0;
  for (const r of rows) if (r.length > w) w = r.length;
  const N = w * h;
  const wall = new Uint8Array(N);
  const goals = [];
  const boxes = [];
  let player = 0;
  for (let y = 0; y < h; y++) {
    const row = rows[y];
    for (let x = 0; x < w; x++) {
      const ch = x < row.length ? row[x] : '#';
      const i = y * w + x;
      if (ch === '#') wall[i] = 1;
      else {
        if (ch === '.' || ch === '*' || ch === '+') goals.push(i);
        if (ch === '$' || ch === '*') boxes.push(i);
        if (ch === '@' || ch === '+') player = i;
      }
    }
  }
  boxes.sort((a, b) => a - b);
  goals.sort((a, b) => a - b);
  return { w, h, N, wall, goals, boxes, player };
}

// Flood fill from `start` over cells where pass[c] is true and occ[c] is false.
// Returns dist (-1 unreached), parent (for path reconstruction) and minCell
// (the smallest visited cell, used as the player-region key).
function flood(start, occ, pass, w, h) {
  const N = w * h;
  const dist = new Int32Array(N).fill(-1);
  const parent = new Int32Array(N).fill(-1);
  const q = [start];
  dist[start] = 0;
  let qi = 0;
  let minCell = start;
  while (qi < q.length) {
    const c = q[qi++];
    if (c < minCell) minCell = c;
    const x = c % w;
    const y = (c / w) | 0;
    for (let d = 0; d < 4; d++) {
      const nx = x + DX[d];
      const ny = y + DY[d];
      if (nx < 0 || nx >= w || ny < 0 || ny >= h) continue;
      const n = ny * w + nx;
      if (!pass[n] || occ[n] || dist[n] >= 0) continue;
      dist[n] = dist[c] + 1;
      parent[n] = c;
      q.push(n);
    }
  }
  return { dist, parent, minCell };
}

// Walk string (lowercase letters) from `from` to `to` using flood parents.
function walkString(fl, from, to, w) {
  if (from === to) return '';
  const letters = [];
  let c = to;
  while (c !== from) {
    const p = fl.parent[c];
    if (p < 0) return '';
    letters.push(dirLetter(p, c, w));
    c = p;
  }
  letters.reverse();
  return letters.join('');
}

// BFS distances from `src` over passable cells (boxes ignored).
function bfsDist(src, pass, w, h) {
  return flood(src, new Uint8Array(w * h), pass, w, h).dist;
}

// Dead squares: a box cell is dead when no sequence of pushes can ever move a
// lone box from it onto a goal. Computed exactly for one box by a reverse BFS
// over (box, player) states, starting from every (goal, any player cell) and
// undoing walks and pushes. The player may start anywhere, so this is an
// over-approximation of live cells: a cell marked dead is truly dead, and
// pruning it is sound. Corner deadlocks fall out of this automatically.
function deadSquares(boxFloor, playerFloor, goals, wall, w, h) {
  const N = w * h;
  const dead = new Uint8Array(N);
  const live = new Uint8Array(N);
  const goalSet = new Uint8Array(N);
  for (const g of goals) goalSet[g] = 1;
  const playerCells = [];
  for (let c = 0; c < N; c++) if (playerFloor[c]) playerCells.push(c);
  const visited = new Set();
  const qb = [];
  const qp = [];
  for (const g of goals) {
    for (const p of playerCells) {
      if (p === g) continue;
      const k = g * N + p;
      if (visited.has(k)) continue;
      visited.add(k);
      qb.push(g);
      qp.push(p);
      live[g] = 1;
    }
  }
  let qi = 0;
  while (qi < qb.length) {
    const b = qb[qi];
    const p = qp[qi];
    qi++;
    live[b] = 1;
    const px = p % w;
    const py = (p / w) | 0;
    const bx = b % w;
    const by = (b / w) | 0;
    // Undo a walk: the player came from an adjacent player-floor cell.
    for (let d = 0; d < 4; d++) {
      const nx = px + DX[d];
      const ny = py + DY[d];
      if (nx < 0 || nx >= w || ny < 0 || ny >= h) continue;
      const p2 = ny * w + nx;
      if (!playerFloor[p2] || p2 === b) continue;
      const k = b * N + p2;
      if (visited.has(k)) continue;
      visited.add(k);
      qb.push(b);
      qp.push(p2);
    }
    // Undo a push: the player stands at p (the box's old cell), the box came
    // from p, and the player before that stood one step further back.
    const ddx = bx - px;
    const ddy = by - py;
    if (Math.abs(ddx) + Math.abs(ddy) === 1) {
      const qx = px - ddx;
      const qy = py - ddy;
      if (qx >= 0 && qx < w && qy >= 0 && qy < h) {
        const preP = qy * w + qx;
        if (playerFloor[preP] && boxFloor[p]) {
          const k = p * N + preP;
          if (!visited.has(k)) {
            visited.add(k);
            qb.push(p);
            qp.push(preP);
          }
        }
      }
    }
  }
  for (let c = 0; c < N; c++) if (boxFloor[c] && !live[c] && !goalSet[c]) dead[c] = 1;
  return dead;
}

// Admissible push heuristic: the minimum assignment of boxes to goals by BFS
// distance (exact Hungarian by permutation for <= 5 boxes; sum of each box's
// nearest-goal distance otherwise, which is still a lower bound). One push
// moves one box one cell, so it changes any box's distance to a fixed goal by
// at most 1, and the assignment total lower-bounds the remaining pushes.
// distFromGoal[gi] is the BFS distance table from goal gi (boxes ignored);
// -1 means unreachable, which makes the assignment invalid (Infinity).
function heuristic(boxes, goals, distFromGoal) {
  const n = boxes.length;
  if (n === 0) return 0;
  if (n <= 5 && goals.length === n) {
    let best = Infinity;
    const perm = [];
    for (let i = 0; i < n; i++) perm.push(i);
    const cnt = new Array(n).fill(0);
    const evalPerm = () => {
      let s = 0;
      for (let i = 0; i < n; i++) {
        const d = distFromGoal[perm[i]][boxes[i]];
        if (d < 0) return;
        s += d;
        if (s >= best) return;
      }
      if (s < best) best = s;
    };
    evalPerm();
    let i = 0;
    while (i < n) {
      if (cnt[i] < i) {
        const j = i % 2 === 0 ? 0 : cnt[i];
        const t = perm[j];
        perm[j] = perm[i];
        perm[i] = t;
        evalPerm();
        cnt[i]++;
        i = 0;
      } else {
        cnt[i] = 0;
        i++;
      }
    }
    return best;
  }
  let s = 0;
  for (const b of boxes) {
    let m = Infinity;
    for (let gi = 0; gi < goals.length; gi++) {
      const d = distFromGoal[gi][b];
      if (d >= 0 && d < m) m = d;
    }
    if (m === Infinity) return Infinity;
    s += m;
  }
  return s;
}

// True when the 2x2 window containing `tgt` (after the box moved there) is
// entirely wall/box and holds a box that is not on a goal: every push of any
// box in such a block needs a stance or target cell inside the block, so all
// of them are frozen forever and the state is dead.
function frozen2x2(tgt, occ, goalSet, wall, w, h) {
  const tx = tgt % w;
  const ty = (tgt / w) | 0;
  for (let ox = -1; ox <= 0; ox++) {
    for (let oy = -1; oy <= 0; oy++) {
      const x0 = tx + ox;
      const y0 = ty + oy;
      if (x0 < 0 || y0 < 0 || x0 + 1 >= w || y0 + 1 >= h) continue;
      const cells = [y0 * w + x0, y0 * w + x0 + 1, (y0 + 1) * w + x0, (y0 + 1) * w + x0 + 1];
      let allFilled = true;
      let badBox = false;
      for (const c of cells) {
        if (wall[c]) continue;
        if (occ[c]) {
          if (!goalSet[c]) badBox = true;
          continue;
        }
        allFilled = false;
      }
      if (allFilled && badBox) return true;
    }
  }
  return false;
}

// Push-optimal A* over pushes only. A state is (sorted box cells, player
// region); expanding a state flood-fills the player's reachable region and
// tries every push whose stance cell is reachable and whose target cell is a
// free box-floor cell. Cost is pushes; the heuristic above is admissible and
// consistent, so with weight 1 the first goal state popped is push-optimal.
//   playerFloor: cells the player may stand on (room floor + door entries)
//   boxFloor:    cells boxes may occupy (room floor, or the whole floor)
//   exit:        null (goal = boxes on goals) or door cells the player must
//                still be able to reach once the boxes are home, so the
//                player can walk out to the next room.
//   maxExp:      expansion budget (deterministic, never the clock).
//   weight:      1 = optimal A*; larger = weighted/greedy fallback.
// Returns { moves, pushes, endPlayer, endBoxes } or null on exhaustion.
function pushSearch(o) {
  const { w, h, N, wall, playerFloor, boxFloor, boxes, goals, startPlayer, exit, maxExp, weight } = o;
  const goalSet = new Uint8Array(N);
  for (const g of goals) goalSet[g] = 1;
  const distFromGoal = goals.map((g) => bfsDist(g, boxFloor, w, h));
  const dead = deadSquares(boxFloor, playerFloor, goals, wall, w, h);
  const nB = boxes.length;

  const heap = [];
  const less = (a, b) => a[0] < b[0] || (a[0] === b[0] && (a[1] > b[1] || (a[1] === b[1] && a[2] < b[2])));
  let seq = 0;
  function heapPush(e) {
    let i = heap.length;
    heap.push(e);
    while (i > 0) {
      const p = (i - 1) >> 1;
      if (!less(heap[i], heap[p])) break;
      const t = heap[i];
      heap[i] = heap[p];
      heap[p] = t;
      i = p;
    }
  }
  function heapPop() {
    const top = heap[0];
    const last = heap.pop();
    if (heap.length > 0) {
      heap[0] = last;
      let i = 0;
      for (;;) {
        let c = 2 * i + 1;
        if (c >= heap.length) break;
        if (c + 1 < heap.length && less(heap[c + 1], heap[c])) c++;
        if (!less(heap[c], heap[i])) break;
        const t = heap[i];
        heap[i] = heap[c];
        heap[c] = t;
        i = c;
      }
    }
    return top;
  }

  const nodePlayer = [startPlayer];
  const nodeBoxes = [boxes.slice()];
  const nodeG = [0];
  const nodeParent = [-1];
  const nodeEdge = [''];
  const startOcc = new Uint8Array(N);
  for (const b of boxes) startOcc[b] = 1;
  const startFl = flood(startPlayer, startOcc, playerFloor, w, h);
  const nodeRegion = [startFl.minCell];
  const keyOf = (bs, region) => bs.join(',') + '|' + region;
  const seen = new Map();
  seen.set(keyOf(boxes, startFl.minCell), 0);

  function finish(idx) {
    const parts = [];
    let cur = idx;
    while (cur > 0) {
      parts.push(nodeEdge[cur]);
      cur = nodeParent[cur];
    }
    parts.reverse();
    return { moves: parts.join(''), pushes: nodeG[idx], endPlayer: nodePlayer[idx], endBoxes: nodeBoxes[idx].slice() };
  }

  const h0 = heuristic(boxes, goals, distFromGoal);
  if (h0 === Infinity) return null;
  heapPush([weight * h0, 0, seq++, 0]);

  let expanded = 0;
  while (heap.length > 0) {
    const top = heapPop();
    const idx = top[3];
    const g = nodeG[idx];
    const bs = nodeBoxes[idx];
    if (seen.get(keyOf(bs, nodeRegion[idx])) !== g) continue;
    let solved = true;
    for (const b of bs) {
      if (!goalSet[b]) {
        solved = false;
        break;
      }
    }
    if (solved) {
      if (!exit) return finish(idx);
      const occ = new Uint8Array(N);
      for (const b of bs) occ[b] = 1;
      const fl = flood(nodePlayer[idx], occ, playerFloor, w, h);
      let canExit = false;
      for (const e of exit) {
        if (fl.dist[e] >= 0) {
          canExit = true;
          break;
        }
      }
      if (canExit) return finish(idx);
    }
    expanded++;
    if (expanded > maxExp) return null;
    const p = nodePlayer[idx];
    const occ = new Uint8Array(N);
    for (const b of bs) occ[b] = 1;
    const fl = flood(p, occ, playerFloor, w, h);
    for (let bi = 0; bi < nB; bi++) {
      const b = bs[bi];
      const bx = b % w;
      const by = (b / w) | 0;
      for (let d = 0; d < 4; d++) {
        const tx = bx + DX[d];
        const ty = by + DY[d];
        const sx = bx - DX[d];
        const sy = by - DY[d];
        if (tx < 0 || tx >= w || ty < 0 || ty >= h || sx < 0 || sx >= w || sy < 0 || sy >= h) continue;
        const tgt = ty * w + tx;
        const stance = sy * w + sx;
        if (fl.dist[stance] < 0) continue;
        if (!boxFloor[tgt] || occ[tgt]) continue;
        if (dead[tgt] && !goalSet[tgt]) continue;
        const nb = bs.slice();
        nb[bi] = tgt;
        nb.sort((a, b2) => a - b2);
        const nocc = new Uint8Array(N);
        for (const x of nb) nocc[x] = 1;
        if (frozen2x2(tgt, nocc, goalSet, wall, w, h)) continue;
        const hh = heuristic(nb, goals, distFromGoal);
        if (hh === Infinity) continue;
        const nfl = flood(b, nocc, playerFloor, w, h);
        const ng = g + 1;
        const key = keyOf(nb, nfl.minCell);
        const prev = seen.get(key);
        if (prev !== undefined && prev <= ng) continue;
        seen.set(key, ng);
        const ni = nodePlayer.length;
        nodePlayer.push(b);
        nodeBoxes.push(nb);
        nodeG.push(ng);
        nodeParent.push(idx);
        nodeEdge.push(walkString(fl, p, stance, w) + LET[d].toUpperCase());
        nodeRegion.push(nfl.minCell);
        heapPush([ng + weight * hh, ng, seq++, ni]);
      }
    }
  }
  return null;
}

// Shortest walk (lowercase moves) from `from` to `to` over passable floor
// with `occ` cells blocked, or null when unreachable.
function walkBetween(from, to, occ, pass, w, h) {
  if (from === to) return '';
  const fl = flood(from, occ, pass, w, h);
  if (fl.dist[to] < 0) return null;
  return walkString(fl, from, to, w);
}

// Solve one room push-optimally, starting with the player on its hallway
// entry cell. Returns the room's moves plus the walk back out through the
// door, and the player's hallway cell afterwards, or null on failure.
function solveRoom(room, entry, L) {
  const { w, h, N, wall } = L;
  const boxFloor = new Uint8Array(N);
  for (const c of room.cells) boxFloor[c] = 1;
  const playerFloor = new Uint8Array(N);
  for (const c of room.cells) playerFloor[c] = 1;
  for (const e of room.entryCells) playerFloor[e] = 1;
  let res = pushSearch({
    w, h, N, wall, playerFloor, boxFloor,
    boxes: room.boxes.slice(), goals: room.goals.slice(),
    startPlayer: entry, exit: room.doorCells, maxExp: 100000, weight: 1,
  });
  if (!res) {
    res = pushSearch({
      w, h, N, wall, playerFloor, boxFloor,
      boxes: room.boxes.slice(), goals: room.goals.slice(),
      startPlayer: entry, exit: room.doorCells, maxExp: 100000, weight: 1000,
    });
  }
  if (!res) return null;
  if (room.entryCells.indexOf(res.endPlayer) >= 0) {
    return { moves: res.moves, endPlayer: res.endPlayer, endBoxes: res.endBoxes };
  }
  const occ = new Uint8Array(N);
  for (const b of res.endBoxes) occ[b] = 1;
  const fl = flood(res.endPlayer, occ, playerFloor, w, h);
  let door = -1;
  let bestD = Infinity;
  for (const dc of room.doorCells) {
    const d = fl.dist[dc];
    if (d >= 0 && (d < bestD || (d === bestD && dc < door))) {
      bestD = d;
      door = dc;
    }
  }
  if (door < 0) return null;
  const outEntry = room.doorToEntry.get(door);
  const walk = walkString(fl, res.endPlayer, door, w);
  const step = dirLetter(door, outEntry, w);
  return { moves: res.moves + walk + step, endPlayer: outEntry, endBoxes: res.endBoxes };
}

// Whole-level fallback: push-A* over the full floor (optimal, then greedy),
// then a plain move-A* in the style of the house baseline.
function solveWhole(L) {
  const { w, h, N, wall, goals, boxes, player } = L;
  const floor = new Uint8Array(N);
  for (let c = 0; c < N; c++) if (!wall[c]) floor[c] = 1;
  let res = pushSearch({
    w, h, N, wall, playerFloor: floor, boxFloor: floor,
    boxes: boxes.slice(), goals: goals.slice(),
    startPlayer: player, exit: null, maxExp: 50000, weight: 1,
  });
  if (res) return res.moves;
  res = pushSearch({
    w, h, N, wall, playerFloor: floor, boxFloor: floor,
    boxes: boxes.slice(), goals: goals.slice(),
    startPlayer: player, exit: null, maxExp: 50000, weight: 1000,
  });
  if (res) return res.moves;
  return moveAStar(L);
}

// Plain move-A* (fewest moves, not fewest pushes): the last-resort fallback
// so a level is at least solved with some pushes instead of left empty.
function moveAStar(L) {
  const { w, h, goals, boxes, player, wall } = L;
  const goalSet = new Set(goals);
  const EXPANSIONS = 120000;
  const dist = (b) => {
    let m = Infinity;
    for (const g of goals) {
      const d = Math.abs((g % w) - (b % w)) + Math.abs(((g / w) | 0) - ((b / w) | 0));
      if (d < m) m = d;
    }
    return m;
  };
  const est = (bs) => {
    let s = 0;
    for (const b of bs) s += dist(b);
    return s;
  };
  const key = (p, bs) => p + '|' + bs.join(',');
  const sp = [player];
  const sb = [boxes.slice()];
  const sg = [0];
  const parent = [-1];
  const step = [''];
  const best = new Map();
  best.set(key(player, boxes), 0);
  const heap = [[est(boxes), 0, 0]];
  const less = (a, b) => a[0] < b[0] || (a[0] === b[0] && (a[1] > b[1] || (a[1] === b[1] && a[2] < b[2])));
  function hpush(e) {
    let i = heap.length;
    heap.push(e);
    while (i > 0) {
      const p = (i - 1) >> 1;
      if (!less(heap[i], heap[p])) break;
      const t = heap[i];
      heap[i] = heap[p];
      heap[p] = t;
      i = p;
    }
  }
  function hpop() {
    const top = heap[0];
    const last = heap.pop();
    if (heap.length > 0) {
      heap[0] = last;
      let i = 0;
      for (;;) {
        let c = 2 * i + 1;
        if (c >= heap.length) break;
        if (c + 1 < heap.length && less(heap[c + 1], heap[c])) c++;
        if (!less(heap[c], heap[i])) break;
        const t = heap[i];
        heap[i] = heap[c];
        heap[c] = t;
        i = c;
      }
    }
    return top;
  }
  let expanded = 0;
  while (heap.length > 0 && expanded < EXPANSIONS) {
    const top = hpop();
    const g = top[1];
    const s = top[2];
    if (best.get(key(sp[s], sb[s])) !== s) continue;
    const bs = sb[s];
    if (bs.every((b) => goalSet.has(b))) {
      let moves = '';
      for (let t = s; t > 0; t = parent[t]) moves = step[t] + moves;
      return moves;
    }
    expanded++;
    const p = sp[s];
    const px = p % w;
    const py = (p / w) | 0;
    for (let d = 0; d < 4; d++) {
      const nx = px + DX[d];
      const ny = py + DY[d];
      if (nx < 0 || nx >= w || ny < 0 || ny >= h) continue;
      const q = ny * w + nx;
      if (wall[q]) continue;
      let nb = bs;
      let move = LET[d];
      const at = bs.indexOf(q);
      if (at >= 0) {
        const tx = nx + DX[d];
        const ty = ny + DY[d];
        if (tx < 0 || tx >= w || ty < 0 || ty >= h) continue;
        const tq = ty * w + tx;
        if (wall[tq] || bs.includes(tq)) continue;
        nb = bs.slice();
        nb[at] = tq;
        nb.sort((a, b) => a - b);
        move = LET[d].toUpperCase();
      }
      const k = key(q, nb);
      const seenIdx = best.get(k);
      if (seenIdx !== undefined && sg[seenIdx] <= g + 1) continue;
      const t = sp.length;
      sp.push(q);
      sb.push(nb);
      sg.push(g + 1);
      parent.push(s);
      step.push(move);
      best.set(k, t);
      hpush([g + 1 + est(nb), g + 1, t]);
    }
  }
  return '';
}

// Solve one level: split the floor (minus the hallway row the player starts
// in) into rooms, solve each room from its door, and stitch the solutions
// with hallway walks. Any surprise in the layout falls back to whole-level
// search, never to an empty answer when a solve exists.
function solveLevel(text) {
  const L = parseLevel(text);
  const { w, h, N, wall, goals, boxes, player } = L;
  const goalSet = new Uint8Array(N);
  for (const g of goals) goalSet[g] = 1;
  const boxSet = new Uint8Array(N);
  for (const b of boxes) boxSet[b] = 1;
  const floor = new Uint8Array(N);
  for (let c = 0; c < N; c++) if (!wall[c]) floor[c] = 1;
  if (boxes.length > 0 && boxes.every((b) => goalSet[b])) return '';
  const hallY = (player / w) | 0;
  const rooms = [];
  const compSeen = new Uint8Array(N);
  let decompOk = true;
  let roomBoxTotal = 0;
  for (let c = 0; c < N; c++) {
    if (wall[c] || ((c / w) | 0) === hallY || compSeen[c]) continue;
    const cells = [];
    const q = [c];
    compSeen[c] = 1;
    let qi = 0;
    while (qi < q.length) {
      const cur = q[qi++];
      cells.push(cur);
      const x = cur % w;
      const y = (cur / w) | 0;
      for (let d = 0; d < 4; d++) {
        const nx = x + DX[d];
        const ny = y + DY[d];
        if (nx < 0 || nx >= w || ny < 0 || ny >= h || ny === hallY) continue;
        const n = ny * w + nx;
        if (wall[n] || compSeen[n]) continue;
        compSeen[n] = 1;
        q.push(n);
      }
    }
    cells.sort((a, b) => a - b);
    const rBoxes = cells.filter((x) => boxSet[x]);
    const rGoals = cells.filter((x) => goalSet[x]);
    if (rBoxes.length === 0) {
      if (rGoals.length > 0) decompOk = false;
      continue;
    }
    roomBoxTotal += rBoxes.length;
    rooms.push({ cells, boxes: rBoxes, goals: rGoals });
  }
  if (!decompOk || roomBoxTotal !== boxes.length) return solveWhole(L);
  for (const r of rooms) {
    if (r.boxes.length !== r.goals.length || r.boxes.length > 4) return solveWhole(L);
    const doorCells = [];
    const entrySet = new Set();
    const doorToEntry = new Map();
    for (const c of r.cells) {
      const x = c % w;
      const y = (c / w) | 0;
      for (let d = 0; d < 4; d++) {
        const nx = x + DX[d];
        const ny = y + DY[d];
        if (ny !== hallY) continue;
        if (nx < 0 || nx >= w) continue;
        const n = ny * w + nx;
        if (wall[n]) continue;
        doorCells.push(c);
        entrySet.add(n);
        if (!doorToEntry.has(c) || n < doorToEntry.get(c)) doorToEntry.set(c, n);
      }
    }
    r.doorCells = [...new Set(doorCells)].sort((a, b) => a - b);
    r.entryCells = [...entrySet].sort((a, b) => a - b);
    r.doorToEntry = doorToEntry;
    if (r.doorCells.length === 0) return solveWhole(L);
  }
  rooms.sort((a, b) => a.doorCells[0] - b.doorCells[0]);
  let curPlayer = player;
  let curBoxes = boxes.slice();
  let out = '';
  for (const room of rooms) {
    const entry = room.entryCells[0];
    const occ = new Uint8Array(N);
    for (const b of curBoxes) occ[b] = 1;
    const walk = walkBetween(curPlayer, entry, occ, floor, w, h);
    if (walk === null) return solveWhole(L);
    out += walk;
    curPlayer = entry;
    const res = solveRoom(room, entry, L);
    if (!res) return solveWhole(L);
    out += res.moves;
    curPlayer = res.endPlayer;
    const roomBoxSet = new Set(room.boxes);
    curBoxes = curBoxes.filter((b) => !roomBoxSet.has(b)).concat(res.endBoxes);
    curBoxes.sort((a, b) => a - b);
  }
  return out;
}

for (const id of readdirSync(I).sort()) {
  const text = readFileSync(join(I, id, 'level.txt'), 'utf8');
  let moves = '';
  try {
    moves = solveLevel(text);
  } catch (e) {
    try {
      moves = moveAStar(parseLevel(text));
    } catch (e2) {
      moves = '';
    }
  }
  mkdirSync(join(O, id), { recursive: true });
  writeFileSync(join(O, id, 'solution.txt'), moves + '\n');
}
