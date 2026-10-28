// Dijkstra vs A* 공통 코드: 그래프 정의, 실제 탐색을 돌려 단계 기록, SVG 그리기

const GRAPH = {
  nodes: ['S', 'A', 'B', 'C', 'D', 'F', 'G'],
  coord: { S: [0, 0], A: [3, 0], G: [6, 0], B: [-2, 0], D: [0, -2], C: [0, 3], F: [3, 3] },
  edges: [['S', 'A', 4], ['A', 'G', 4], ['S', 'B', 2], ['S', 'D', 2], ['S', 'C', 3], ['C', 'F', 3], ['F', 'G', 5]],
  start: 'S',
  goal: 'G',
};

// 인접 리스트 (이웃은 알파벳순으로 확인)
const ADJ = (() => {
  const adj = {};
  GRAPH.nodes.forEach(v => (adj[v] = []));
  GRAPH.edges.forEach(([a, b, w]) => { adj[a].push([b, w]); adj[b].push([a, w]); });
  Object.values(adj).forEach(list => list.sort((x, y) => x[0].localeCompare(y[0])));
  return adj;
})();

// h(v) = v에서 목표 G까지 직선거리. 모든 간선 가중치가 직선거리 이상이므로 실제 남은 거리를 넘지 않음
const H_TRUE = (() => {
  const [gx, gy] = GRAPH.coord[GRAPH.goal], h = {};
  for (const v of GRAPH.nodes) { const [x, y] = GRAPH.coord[v]; h[v] = Math.hypot(x - gx, y - gy); }
  return h;
})();
// 각 정점에서 목표까지 실제 최단 거리 (h가 과대평가인지 판정할 때 사용)
const REMAIN = (() => {
  const d = { [GRAPH.goal]: 0 }, done = new Set();
  for (;;) {
    const u = GRAPH.nodes.filter(v => d[v] !== undefined && !done.has(v)).sort((a, b) => d[a] - d[b])[0];
    if (!u) return d;
    done.add(u);
    for (const [v, w] of ADJ[u]) if (d[v] === undefined || d[u] + w < d[v]) d[v] = d[u] + w;
  }
})();
// h(A)만 바꾼 휴리스틱. 반례 기본값은 실제 남은 거리 4보다 큰 10
const makeH = hA => ({ ...H_TRUE, A: hA });
const H_OVER = makeH(10);

const EPS = 1e-9;
const fmt = x => (x === undefined ? '∞' : Math.abs(x - Math.round(x)) < EPS ? String(Math.round(x)) : x.toFixed(1));
const join = arr => arr.join(', ');
// 숫자 뒤 조사 (끝자리 0 1 3 6 7 8은 받침 있음). 정점 이름 S A B C D F G는 모두 받침 없음
const hasB = x => [0, 1, 3, 6, 7, 8].includes(Number(fmt(x).slice(-1)));
const 은 = x => fmt(x) + (hasB(x) ? '은' : '는');
const 이 = x => fmt(x) + (hasB(x) ? '이' : '가');
const 이라 = x => fmt(x) + (hasB(x) ? '이라' : '라');
const 로 = x => fmt(x) + (hasB(x) && ![1, 7, 8].includes(Number(fmt(x).slice(-1))) ? '으로' : '로');

function pathFrom(parent, t) {
  const path = [];
  for (let v = t; v !== undefined; v = parent[v]) path.unshift(v);
  return path;
}

// kind: 'dijkstra' | 'astar'. 두 알고리즘은 우선순위 계산 한 줄만 다름
function trace(kind, h = H_TRUE) {
  const s = GRAPH.start, t = GRAPH.goal;
  const isA = kind === 'astar', over = isA && h.A > REMAIN.A + EPS;
  const g = { [s]: 0 }, parent = {};
  const closed = new Set(), seen = new Set([s]);
  let pq = [s];
  const steps = [];
  const pri = v => (isA ? g[v] + h[v] : g[v]);
  const sorted = () => pq.slice().sort((a, b) => pri(a) - pri(b) || a.localeCompare(b));
  const snap = info => steps.push({
    kind, h, pq: sorted().map(v => ({ v, p: pri(v) })), g: { ...g }, parent: { ...parent },
    closed: [...closed], seen: [...seen], ...info,
  });
  const priText = v => (isA ? `f = ${fmt(g[v])} + ${fmt(h[v])} = ${로(pri(v))}` : `g = ${로(g[v])}`);

  snap({
    current: null, lines: [1], title: '초기화',
    what: `출발 정점 S의 g를 0으로 두고 우선순위 큐에 넣는다${isA ? ` (f = 0 + h(S) = ${fmt(h.S)})` : ''}.`,
    why: isA
      ? `h(S) = ${은(h.S)} S에서 목표 G까지의 직선거리이다. A*는 처음부터 "목표까지 대략 얼마 남았는지"를 알고 시작한다.`
      : 'Dijkstra가 아는 정보는 출발점에서의 실제 거리 g뿐이다. 다른 정점은 아직 g = ∞이다.',
  });

  while (pq.length) {
    const order = sorted(), u = order[0];
    const tie = order[1] && Math.abs(pri(order[1]) - pri(u)) < EPS ? order[1] : null;
    pq = pq.filter(x => x !== u);
    closed.add(u);
    const tieNote = tie ? ` ${tie}도 우선순위가 ${로(pri(u))} 같지만 알파벳순 규칙에 따라 ${u}가 먼저다.` : '';

    if (u === t) {
      const path = pathFrom(parent, t);
      const best = isA ? trace('dijkstra').dist : g[t];
      const rest = sorted();
      let why;
      if (!isA) {
        why = `우선순위가 g이므로 ${t}를 꺼낸 순간 큐에 남은 정점은 모두 g가 ${fmt(g[t])} 이상이다. 그보다 짧은 길은 있을 수 없어서 여기서 끝낸다.`;
      } else if (Math.abs(g[t] - best) < EPS && over) {
        why = `h(A) = ${은(h.A)} 과대평가라 정답이 보장되지는 않았다. 그래도 이번에는 부풀려진 A가 ${t}보다 먼저 꺼내져서 최단 경로를 찾았다. 과대평가가 작으면 탐색량만 늘고 답은 맞을 수 있다.`;
      } else if (Math.abs(g[t] - best) < EPS) {
        why = `h가 실제 남은 거리보다 크지 않으므로 f는 그 정점을 지나는 경로 길이의 하한이다. 큐에 남은 정점의 f가 모두 ${fmt(g[t])} 이상${rest.length ? `(최소 ${fmt(pri(rest[0]))})` : ''}이라 더 짧은 길은 없다.`;
      } else {
        why = `틀린 답이다. 실제 최단은 ${trace('dijkstra').path.join(' → ')}, 거리 ${fmt(best)}이다. A를 지나는 실제 경로는 ${fmt(best)}인데 A의 f는 ${fmt(g.A + h.A)}로 부풀려져 큐 뒤에 남아 있었다. h가 과대평가하면 f가 하한 역할을 못 해서 최단 경로를 보기 전에 끝나 버린다.`;
      }
      snap({
        current: u, lines: [3, 4], path, done: true, wrong: isA && Math.abs(g[t] - best) > EPS,
        title: `${u} 꺼냄 → 도착`,
        what: `큐에서 ${priText(u)} 가장 작은 ${t}를 꺼냈다.${tieNote} parent를 거꾸로 따라가면 경로는 ${path.join(' → ')}, 거리 ${fmt(g[t])}이다.`,
        why,
      });
      return { steps, path, dist: g[t], popped: closed.size, discovered: seen.size };
    }

    const updated = [], ignored = [], skipped = [];
    for (const [v, w] of ADJ[u]) {
      const cand = g[u] + w;
      if (g[v] === undefined || cand < g[v] - EPS) {
        updated.push({ v, old: g[v], w });
        g[v] = cand; parent[v] = u; seen.add(v); closed.delete(v);
        if (!pq.includes(v)) pq.push(v);
      } else if (closed.has(v)) skipped.push(v);
      else ignored.push({ v, cand, cur: g[v] });
    }

    let what = `큐에서 ${priText(u)} 가장 작은 ${u}를 꺼낸다.${tieNote}`;
    if (updated.length) {
      what += ' 갱신: ' + updated.map(({ v, old, w }) =>
        `${v}의 g ${fmt(old)} → ${fmt(g[v])}${isA ? ` (f = ${fmt(g[v])} + ${fmt(h[v])} = ${fmt(pri(v))})` : ` (${fmt(g[u])} + ${w})`}`).join(', ') + '.';
    }
    for (const { v, cand, cur } of ignored) what += ` ${v}: ${fmt(g[u])} + ${fmt(cand - g[u])} = ${은(cand)} 현재 ${fmt(cur)}보다 크므로 무시한다.`;
    if (!updated.length && !ignored.length) what += ` 이웃 ${join(skipped)}는 이미 꺼냈으므로 갱신할 것이 없다.`;

    const deadEnd = !updated.length && !ignored.length;
    let why;
    if (!isA) {
      if (u === s) why = 'Dijkstra는 목표가 어느 쪽인지 모른다. 이웃을 모두 후보로 넣고, 출발점에서 가까운 순서대로 꺼낸다.';
      else if (deadEnd) why = `${u}는 목표와 상관없는 방향의 막다른 정점이다. 그런데도 g가 작다는 이유만으로 먼저 확인한다. 목표 방향 정보가 없어서 생기는 불필요한 탐색이다.`;
      else if (ignored.length) why = `이미 더 짧은 길이 있는 정점은 바꾸지 않는다. ${ignored[0].v}에는 ${fmt(ignored[0].cur)}짜리 길이 있으므로 ${u}를 거치는 길(${fmt(ignored[0].cand)})은 버린다.`;
      else why = `출발점을 중심으로 동심원처럼 퍼져 나간다. 지금 큐에서 ${u}의 g가 가장 작으므로 ${u}까지의 최단 거리 ${이(g[u])} 확정된다.`;
    } else if (u === s) {
      const byG = updated.slice().sort((a, b) => g[a.v] - g[b.v] || a.v.localeCompare(b.v))[0].v;
      const byF = updated.slice().sort((a, b) => pri(a.v) - pri(b.v) || a.v.localeCompare(b.v))[0].v;
      why = byG !== byF
        ? `${byG}는 g = ${fmt(g[byG])}로 가장 가깝지만 h = ${이라(h[byG])} f = ${이(pri(byG))} 되어 ${byF}(f = ${fmt(pri(byF))})보다 뒤로 밀린다. 목표에서 멀다는 정보가 h에 담겨 있기 때문이다.`
        : 'f = g + h가 작은 순서로 꺼낸다. h가 목표까지 남은 거리를 알려 주므로 목표 쪽 정점이 앞에 선다.';
      if (over) why += ` 단, 이번 h(A) = ${은(h.A)} 실제 남은 거리 ${fmt(REMAIN.A)}보다 크다(과대평가). A의 f가 ${로(pri('A'))} 부풀려진다.`;
      else if (h.A < H_TRUE.A - EPS) why += ` 이번 h(A) = ${은(h.A)} 직선거리 ${fmt(H_TRUE.A)}보다 작다. 실제 남은 거리를 넘지 않으므로 최단 경로 보장은 그대로이다.`;
      else if (h.A > H_TRUE.A + EPS) why += ` 이번 h(A) = ${은(h.A)} 직선거리보다 크지만 실제 남은 거리 ${fmt(REMAIN.A)} 이하라서 과대평가가 아니다.`;
    } else if (over && u === 'A') {
      why = `부풀려졌던 A가 이제야 꺼내진다. A의 f는 ${fmt(pri('A'))}이지만 A를 지나는 실제 경로는 ${fmt(g.A + REMAIN.A)}이다. 늦어진 만큼 불필요한 탐색이 생겼지만, 아직 목표를 꺼내기 전이라 정답은 지킬 수 있다.${tie === t ? ` 단, ${t}와 f가 같았는데 알파벳순 규칙 덕분에 A가 먼저 나온 아슬아슬한 경우이다.` : ''}`;
    } else if (over && g.A !== undefined && !closed.has('A')) {
      why = deadEnd
        ? `${u}는 막다른 정점이다. 정상 h였다면 꺼낼 필요가 없었지만, 최단 경로 쪽 A가 f = ${로(pri('A'))} 밀려 있어서 대신 꺼내게 된다.`
        : `A가 f = ${로(pri('A'))} 밀려 있어서 ${u} 쪽 우회로를 따라간다. h가 틀리면 A*가 엉뚱한 방향을 먼저 펼친다.`;
    } else if (deadEnd) {
      why = `${u}는 막다른 정점이다. 지금 큐에서 f가 가장 작아서 꺼냈지만 갱신할 것이 없다.`;
    } else {
      why = `${u}는 목표 쪽에 있어 h가 작고, 그래서 f가 가장 작다. A*는 이렇게 목표 방향 정점을 먼저 펼친다.`;
    }
    snap({ current: u, lines: [3, 5, 6, 7, 8], title: `${u} 꺼냄`, what, why });
  }
  return { steps, path: null };
}

// ---------------- 그리기 ----------------
const THEME = {
  dijkstra: { main: '#0f766e', light: '#ccfbf1' },
  astar: { main: '#7c3aed', light: '#ede9fe' },
  path: '#16a34a', wrong: '#dc2626', edge: '#cbd5e1', current: '#facc15',
};
const px = ([x, y]) => [80 + (x + 2) * 62, 70 + (3 - y) * 58];
// 정점 옆 숫자 라벨 위치 (간선과 겹치지 않는 쪽)
const LABEL_POS = { S: [24, 34, 'start'], A: [0, 38, 'middle'], G: [0, 38, 'middle'], B: [0, 38, 'middle'], D: [0, 38, 'middle'], C: [0, -30, 'middle'], F: [0, -30, 'middle'] };

function graphSVG(step) {
  const R = 18, th = THEME[step.kind], isA = step.kind === 'astar';
  const pathEdges = new Set();
  if (step.path) for (let i = 1; i < step.path.length; i++) pathEdges.add([step.path[i - 1], step.path[i]].sort().join());
  const treeEdges = new Set(Object.entries(step.parent).map(([v, u]) => [u, v].sort().join()));
  const closed = new Set(step.closed), inPQ = new Set(step.pq.map(e => e.v));
  let out = `<svg viewBox="0 0 620 430" xmlns="http://www.w3.org/2000/svg" class="graph" role="img" aria-label="${isA ? 'A*' : 'Dijkstra'} 단계 그래프">`;

  for (const [a, b, w] of GRAPH.edges) {
    const [x1, y1] = px(GRAPH.coord[a]), [x2, y2] = px(GRAPH.coord[b]);
    const key = [a, b].sort().join();
    let color = THEME.edge, sw = 3;
    if (treeEdges.has(key)) { color = th.main; sw = 4.5; }
    if (pathEdges.has(key)) { color = step.wrong ? THEME.wrong : THEME.path; sw = 7; }
    out += `<line x1="${x1}" y1="${y1}" x2="${x2}" y2="${y2}" stroke="${color}" stroke-width="${sw}" stroke-linecap="round"/>`;
    const mx = (x1 + x2) / 2, my = (y1 + y2) / 2;
    out += `<rect x="${mx - 11}" y="${my - 11}" width="22" height="22" rx="5" fill="#fff" stroke="#e2e8f0"/>`;
    out += `<text x="${mx}" y="${my + 5}" text-anchor="middle" font-size="13" font-weight="700" fill="#334155">${w}</text>`;
  }

  // A*: 현재 정점에서 목표까지 직선(h)을 점선으로 표시
  if (isA && step.current && step.current !== GRAPH.goal) {
    const [x1, y1] = px(GRAPH.coord[step.current]), [x2, y2] = px(GRAPH.coord[GRAPH.goal]);
    out += `<line x1="${x1}" y1="${y1}" x2="${x2}" y2="${y2}" stroke="${th.main}" stroke-width="2" stroke-dasharray="6 5" opacity="0.7"/>`;
    // 간선 가중치 라벨(중점)과 겹치지 않게 현재 정점 쪽 1/3 지점에 표시
    const mx = x1 + (x2 - x1) / 3, my = y1 + (y2 - y1) / 3 - 10;
    out += `<text x="${mx}" y="${my}" text-anchor="middle" font-size="12.5" font-weight="700" fill="${th.main}">h = ${fmt(step.h[step.current])}</text>`;
  }

  for (const v of GRAPH.nodes) {
    const [x, y] = px(GRAPH.coord[v]);
    let fill = '#fff', stroke = '#94a3b8', text = '#0f172a', sw = 2;
    if (closed.has(v)) { fill = th.main; stroke = th.main; text = '#fff'; }
    else if (inPQ.has(v)) { fill = th.light; stroke = th.main; sw = 3.5; }
    if (step.path && step.path.includes(v)) { stroke = step.wrong ? THEME.wrong : THEME.path; sw = 4; }
    if (step.current === v) out += `<circle cx="${x}" cy="${y}" r="${R + 7}" fill="none" stroke="${THEME.current}" stroke-width="5"/>`;
    out += `<circle cx="${x}" cy="${y}" r="${R}" fill="${fill}" stroke="${stroke}" stroke-width="${sw}"/>`;
    out += `<text x="${x}" y="${y + 6}" text-anchor="middle" font-size="17" font-weight="800" fill="${text}">${v}</text>`;

    const [dx, dy, anchor] = LABEL_POS[v], gv = step.g[v];
    const lines = isA
      ? [`h=${fmt(step.h[v])}`, gv === undefined ? 'g=∞' : `g=${fmt(gv)} f=${fmt(gv + step.h[v])}`]
      : [`g=${fmt(gv)}`];
    const base = dy < 0 ? y + dy - (lines.length - 1) * 14 : y + dy;
    lines.forEach((l, i) => {
      const strong = i === lines.length - 1 && gv !== undefined;
      out += `<text x="${x + dx}" y="${base + i * 14}" text-anchor="${anchor}" font-size="12" font-weight="${strong ? 700 : 500}" fill="${strong ? '#0f172a' : '#64748b'}">${l}</text>`;
    });
  }
  const [sx, sy] = px(GRAPH.coord.S), [gx, gy] = px(GRAPH.coord.G);
  out += `<text x="${sx - 28}" y="${sy - 22}" text-anchor="middle" font-size="12" font-weight="700" fill="#0f172a">출발</text>`;
  out += `<text x="${gx + 28}" y="${gy - 22}" text-anchor="middle" font-size="12" font-weight="700" fill="#0f172a">목표</text>`;
  return out + '</svg>';
}

function pqHTML(pq, kind) {
  if (!pq.length) return '<span class="empty">(비어 있음)</span>';
  return pq.map(({ v, p }) => `<span class="box ${kind}"><b>${v}</b><small>${fmt(p)}</small></span>`).join('');
}

const CODE = kind => [
  'SEARCH(G, s, t)',
  '    g[s] = 0;  PQ.insert(s, 우선순위(s))',
  '    while PQ not empty',
  '        u = PQ.extractMin()          // 우선순위가 가장 작은 정점',
  '        if u == t: return path(parent, t)',
  '        for (v, w) in adj[u]',
  '            if g[u] + w < g[v]       // 더 짧은 길을 찾았으면',
  '                g[v] = g[u] + w;  parent[v] = u',
  '                PQ.insertOrDecrease(v, 우선순위(v))',
  '',
  kind === 'astar'
    ? '우선순위(v) = g[v] + h[v]   // 실제 비용 + 목표까지 직선거리'
    : '우선순위(v) = g[v]          // 출발점부터 실제 비용',
];
