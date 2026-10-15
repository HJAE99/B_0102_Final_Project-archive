// SCV server page — 3-3차 (login · 모니터(순찰 시작 · 중단) · 이력 · 정보)
import { html, createRoot, useState, useEffect, useRef, useCallback, useMemo } from '/static/vendor/react-htm.mjs';
import { signalColor, createHeatLayer } from '/static/server/heatmap.mjs';

// ---------------------------------------------------------------- API
async function api(path, opts = {}) {
  const o = { credentials: 'same-origin', ...opts };
  if (o.json !== undefined) {
    o.method = o.method || 'POST';
    o.headers = { 'Content-Type': 'application/json' };
    o.body = JSON.stringify(o.json);
    delete o.json;
  }
  const r = await fetch(path, o);
  let data = null;
  try { data = await r.json(); } catch { /* 본문 없음 */ }
  if (!r.ok) {
    const d = data && data.detail;
    const err = new Error(typeof d === 'string' ? d : (d ? JSON.stringify(d) : `오류 (${r.status})`));
    err.status = r.status;
    throw err;
  }
  return data;
}

const TABS = [
  { id: 'monitor', label: '실시간 관제' },
  { id: 'history', label: '점검 결과' },
  { id: 'master', label: '설정' },
];
const pageFromPath = () => (location.pathname.match(/^\/server\/(\w+)/) || [])[1] || 'login';

// ---------------------------------------------------------------- 표시 도우미
const fmtTime = (v) => {
  if (!v) return '—';
  const d = new Date(v);
  const p = (n) => String(n).padStart(2, '0');
  return `${d.getFullYear()}-${p(d.getMonth() + 1)}-${p(d.getDate())} ${p(d.getHours())}:${p(d.getMinutes())}`;
};
const fmtNum = (v) => (v === null || v === undefined ? '—' : (Number.isInteger(v) ? String(v) : Number(v).toFixed(3).replace(/\.?0+$/, '')));

function showValue(col, v) {
  if (v === null || v === undefined || v === '') return html`<span class="no">—</span>`;
  switch (col.type) {
    case 'bool': return v ? html`<span class="yes">예</span>` : html`<span class="no">아니요</span>`;
    case 'datetime': return html`<span class="nowrap">${fmtTime(v)}</span>`;
    case 'real': case 'int': return html`<span class="num">${fmtNum(v)}</span>`;
    case 'pose': return html`<span class="num">${fmtNum(v.x)}, ${fmtNum(v.y)}, ${fmtNum(v.yaw)}</span>`;
    case 'docks': {
      const ks = Object.keys(v);
      return ks.length ? html`<span class="num">${ks.map((k) => `${k} (${fmtNum(v[k].x)}, ${fmtNum(v[k].y)})`).join(' · ')}</span>`
        : html`<span class="no">—</span>`;
    }
    case 'select': {
      const o = (col.options || []).find((x) => String(x.value) === String(v));
      return o ? o.label : String(v);
    }
    default: return String(v);
  }
}

// 자세 입력칸 안내 글자 — 지도 좌표(m)와 방향(rad) 단위를 함께 보여 줌 (도 °로 잘못 넣지 않게)
const POSE_PH = { x: 'x (m)', y: 'y (m)', yaw: 'yaw (rad)' };

function Input({ col, value, onChange }) {
  const set = (v) => onChange(col.name, v);
  switch (col.type) {
    case 'bool':
      return html`<input type="checkbox" checked=${!!value} onChange=${(e) => set(e.target.checked)} />`;
    case 'int': case 'real':
      return html`<input type="number" step="any" value=${value ?? ''} onInput=${(e) => set(e.target.value)} />`;
    case 'password':
      return html`<input type="password" value=${value ?? ''} onInput=${(e) => set(e.target.value)} />`;
    case 'select':
      return html`<select value=${value ?? ''} onChange=${(e) => set(e.target.value)}>
        <option value="">선택</option>
        ${(col.options || []).map((o) => html`<option key=${o.value} value=${o.value}>${o.label}</option>`)}
      </select>`;
    case 'pose': {                                      // 관측 자세 — x · y는 지도 좌표 m, yaw는 rad (안내 글자에 단위)
      const p = value || { x: '', y: '', yaw: '' };
      const part = (k, ph) => html`<input type="number" step="any" placeholder=${ph} value=${p[k] ?? ''}
        onInput=${(e) => set({ ...p, [k]: e.target.value })} />`;
      return html`<div class="pose">${part('x', POSE_PH.x)}${part('y', POSE_PH.y)}${part('yaw', POSE_PH.yaw)}</div>`;
    }
    case 'docks': {                                     // 로봇마다 x · y(m) · yaw(rad) (비우면 그 로봇 도크 없음)
      const v = value || {};
      const part = (rid, k) => html`<input type="number" step="any" placeholder=${POSE_PH[k]} value=${(v[rid] || {})[k] ?? ''}
        onInput=${(e) => set({ ...v, [rid]: { ...(v[rid] || {}), [k]: e.target.value } })} />`;
      return html`<div class="docks">${(col.options || []).map((rid) => html`<div class="pose" key=${rid}>
        <span class="rid">${rid}</span>${part(rid, 'x')}${part(rid, 'y')}${part(rid, 'yaw')}</div>`)}
        <span class="help">도크 바로 앞에서 도크를 바라보는 자세 · 비우면 도크 없음</span></div>`;
    }
    default:
      return html`<input type="text" value=${value ?? ''} onInput=${(e) => set(e.target.value)} />`;
  }
}

// 쪽 번호 — 최신 순 10행씩. 쪽이 많으면 처음 · 끝 · 현재 앞뒤 2쪽만 보이고 나머지는 …
function Pager({ page, pages, onPage, disabled }) {
  if (!pages || pages <= 1) return null;
  const nums = [];
  for (let i = 1; i <= pages; i++) {
    if (i === 1 || i === pages || Math.abs(i - page) <= 2) nums.push(i);
    else if (nums[nums.length - 1] !== '…') nums.push('…');
  }
  return html`<div class="pager">
    <button class="pg" disabled=${disabled || page <= 1} onClick=${() => onPage(page - 1)} title="이전 쪽">‹</button>
    ${nums.map((n, i) => n === '…' ? html`<span key=${'e' + i} class="pg gap">…</span>`
      : html`<button key=${n} class=${'pg' + (n === page ? ' on' : '')} disabled=${disabled}
          onClick=${() => n !== page && onPage(n)}>${n}</button>`)}
    <button class="pg" disabled=${disabled || page >= pages} onClick=${() => onPage(page + 1)} title="다음 쪽">›</button>
  </div>`;
}

// ---------------------------------------------------------------- 로그인
function LoginPage({ onLogin }) {
  const [username, setU] = useState('');
  const [password, setP] = useState('');
  const [msg, setMsg] = useState('');
  const [busy, setBusy] = useState(false);
  const submit = async (e) => {
    e.preventDefault();
    setBusy(true); setMsg('');
    try {
      onLogin(await api('/api/login', { json: { username, password } }));
    } catch (err) { setMsg(err.message); }
    setBusy(false);
  };
  return html`<form class="login" onSubmit=${submit}>
    <h2>관리자 로그인</h2>
    <p class="sub">로그인하면 모니터 · 이력 · 정보 탭을 쓸 수 있습니다.</p>
    <div class="field"><label>아이디</label>
      <input autofocus value=${username} onInput=${(e) => setU(e.target.value)} autocomplete="username" /></div>
    <div class="field"><label>비밀번호</label>
      <input type="password" value=${password} onInput=${(e) => setP(e.target.value)} autocomplete="current-password" /></div>
    <button class="btn primary" disabled=${busy}>로그인</button>
    ${msg && html`<div class="msg">${msg}</div>`}
  </form>`;
}

// ---------------------------------------------------------------- monitor
const ROBOT_COLORS = { robot3: '#c2410c', robot4: '#6d28d9' };
const robotColor = (rid) => ROBOT_COLORS[rid] || '#2f5d7c';

// 로봇 상태 분류 — 정상 / 연결 끊김(E3 대기) / 응답 없음 / 비활성
function robotPhase(r) {
  if (r.state && r.connected) return ['ok', '정상'];
  if (r.state) return ['wait', `연결 끊김 — ${r.state.ttl_s}초 뒤 응답 없음`];
  if (r.is_active || r.connected) return ['bad', '응답 없음'];
  return ['off', '비활성'];
}

// 섹터 색 (3-2차) — 모니터 격자 · 범례가 같이 씀
const SECTOR_COLOR = { 1: '#2563eb', 2: '#d97706' };

// 지도 범위 [[y0, x0], [y1, x1]] — 이미지 크기를 알면 이미지 전체, 모르면(파일 없음) 격자 · 도크 · AP · 설비가 들어가는 범위
function mapBounds(m) {
  if (m.width && m.height) {
    return [[m.origin_y, m.origin_x], [m.origin_y + m.height * m.resolution_m, m.origin_x + m.width * m.resolution_m]];
  }
  const g = m.grid_size_m || 1, xs = [], ys = [];
  const add = (x, y) => { if (Number.isFinite(x) && Number.isFinite(y)) { xs.push(x); ys.push(y); } };
  (m.cells || []).forEach(([, ix, iy]) => { add(m.origin_x + ix * g, m.origin_y + iy * g); add(m.origin_x + (ix + 1) * g, m.origin_y + (iy + 1) * g); });
  Object.values(m.dock_poses || {}).forEach((p) => add(p.x, p.y));
  (m.aps || []).forEach((a) => add(a.x, a.y));
  (m.facilities || []).forEach((f) => { add(f.expected_x, f.expected_y); if (f.view_pose) add(f.view_pose.x, f.view_pose.y); });
  if (!xs.length) return [[m.origin_y, m.origin_x], [m.origin_y + 10, m.origin_x + 10]];
  const pad = 0.5;
  return [[Math.min(...ys) - pad, Math.min(...xs) - pad], [Math.max(...ys) + pad, Math.max(...xs) + pad]];
}

function MapView({ m, robots, overlay, mini, layers = {}, onSelect, linked = true, selection, paths }) {
  const ref = useRef(null), mapRef = useRef(null), groups = useRef({}), markers = useRef({});
  const select = useRef(onSelect); select.current = onSelect;
  const options = { heat: true, smooth: true, opacity: 0.5, path: false, ...layers };
  const [heatError, setHeatError] = useState('');
  useEffect(() => {
    const L = window.L, container = ref.current;
    const map = L.map(container, { crs: L.CRS.Simple, minZoom: -3, maxZoom: 9, zoomSnap: 0.25, attributionControl: false });
    const bounds = mapBounds(m);
    map.createPane('basePane'); map.getPane('basePane').style.zIndex = 250;
    if (m.width && m.height) L.imageOverlay(m.image_url, bounds, { className: 'base-map', pane: 'basePane', interactive: false }).addTo(map);
    map.createPane('heatPane'); map.getPane('heatPane').style.zIndex = 350;
    let moved = false;
    const markMoved = () => { moved = true; };
    container.addEventListener('pointerdown', markMoved);
    container.addEventListener('wheel', markMoved, { passive: true });
    const fit = () => { map.invalidateSize(); map.fitBounds(bounds, { padding: [24, 24], animate: false }); };
    const FitAll = L.Control.extend({ onAdd: () => {
      const b = L.DomUtil.create('a', 'leaflet-bar map-fit');
      b.href = '#'; b.title = '지도 전체 보기'; b.textContent = '⤢'; b.setAttribute('aria-label', b.title);
      L.DomEvent.on(b, 'click', (e) => { L.DomEvent.stop(e); fit(); moved = false; });
      L.DomEvent.disableClickPropagation(b); return b;
    } });
    new FitAll({ position: 'topleft' }).addTo(map);
    groups.current = Object.fromEntries(['grid', 'info', 'reference', 'heat', 'path', 'result', 'robot', 'selection'].map((k) => [k, L.layerGroup().addTo(map)]));
    markers.current = {}; mapRef.current = map; fit();
    const ro = new ResizeObserver(() => { map.invalidateSize(); if (!moved) fit(); }); ro.observe(container);
    return () => { ro.disconnect(); container.removeEventListener('pointerdown', markMoved); container.removeEventListener('wheel', markMoved); map.remove(); mapRef.current = null; };
  }, [m.map_id, m.image_url, m.width, m.height, m.origin_x, m.origin_y, m.resolution_m]);

  useEffect(() => {
    const L = window.L, layer = groups.current.grid;
    if (!layer) return; layer.clearLayers();
    const g = m.grid_size_m;
    if (options.grid) (m.cells || []).forEach(([id, ix, iy]) => {
      const x = m.origin_x + ix * g, y = m.origin_y + iy * g;
      L.rectangle([[y, x], [y + g, x + g]], { weight: 1, color: '#869ba7', fill: false, interactive: false }).addTo(layer);
      L.marker([y + g / 2, x + g / 2], { interactive: false, icon: L.divIcon({ className: 'cell-label', iconSize: null, html: `<b>${id}</b>` }) }).addTo(layer);
    });
    if (options.zones) (m.configuration?.zones || []).forEach((z, i) => {
      const poly = L.polygon(z.polygon.map(([x, y]) => [y, x]), { color: robotColor(z.robot), weight: 2, fillOpacity: 0.035, dashArray: '6 6' });
      poly.bindTooltip(`${z.id} · ${z.robot}`, { direction: 'center' }).addTo(layer);
    });
  }, [m, options.grid, options.zones]);

  useEffect(() => {
    const L = window.L, info = groups.current.info, reference = groups.current.reference;
    if (!info) return; info.clearLayers(); reference.clearLayers();
    const click = (marker, data) => marker.on('click', () => select.current?.(data));
    (m.aps || []).forEach((a) => {
      if (!Number.isFinite(a.x) || !Number.isFinite(a.y)) return;
      click(L.marker([a.y, a.x], { icon: L.divIcon({ className: 'asset ap-asset', iconSize: [26, 26], html: '<b>AP</b>' }) })
        .bindTooltip(a.ssid).addTo(info), { kind: 'ap', data: a });
    });
    const FAC = { extinguisher: '소', shutter: '셔', exit_light: '유', rack: '랙' };
    (m.facilities || []).forEach((f) => {
      if (!Number.isFinite(f.expected_x) || !Number.isFinite(f.expected_y)) return;
      click(L.marker([f.expected_y, f.expected_x], { icon: L.divIcon({ className: 'asset facility-asset', iconSize: [26, 26], html: `<b>${FAC[f.facility_type] || '?'}</b>` }) })
        .bindTooltip(f.facility_id).addTo(info), { kind: 'facility', data: f });
    });
    // Physical Dock and calibrated undock reference have different meanings.
    (m.configuration?.zones || []).forEach((z) => {
      if (!z.dock) return;
      L.circleMarker([z.dock.y, z.dock.x], { radius: 5, color: '#75828b', fillColor: '#fff', fillOpacity: 1, weight: 2 })
        .bindTooltip(`${z.robot} 실제 Dock`).addTo(info);
    });
    if (options.references) (m.configuration?.references || []).forEach((p) => {
      click(L.circleMarker([p.y, p.x], { radius: 7, color: robotColor(p.robot_id), weight: 2, fill: false, dashArray: '2 2' })
        .bindTooltip(`${p.robot_id} 시작 참고점 · 실측 위치 아님`).addTo(reference), { kind: 'reference', data: p });
    });
  }, [m, options.references]);

  useEffect(() => {
    const L = window.L, layer = groups.current.robot, mk = markers.current;
    if (!layer) return;
    const seen = new Set();
    (robots || []).forEach((r) => {
      const st = r.state;
      const live = linked && r.connected && st?.pose_ok !== false;
      const p = live && Number.isFinite(st?.x) && Number.isFinite(st?.y) ? st : st?.last_valid_pose;
      if (!p || !Number.isFinite(p.x) || !Number.isFinite(p.y)) return;
      seen.add(r.robot_id);
      const color = live ? robotColor(r.robot_id) : '#9ba9b5', yaw = p.yaw || 0;
      const head = [[p.y, p.x], [p.y + Math.sin(yaw) * 0.36, p.x + Math.cos(yaw) * 0.36]];
      const tip = `${r.robot_id}${live ? '' : ' · 마지막 유효 위치'}`;
      if (!mk[r.robot_id]) {
        mk[r.robot_id] = {
          line: L.polyline(head, { color, weight: 3, interactive: false }).addTo(layer),
          dot: L.circleMarker([p.y, p.x], { radius: 10, color: '#fff', weight: 3, fillColor: color, fillOpacity: 1 })
            .bindTooltip(tip, { permanent: true, direction: 'top', offset: [0, -10], className: 'robot-tip' })
            .on('click', () => select.current?.({ kind: 'robot', data: { robot_id: r.robot_id } })).addTo(layer),
        };
      }
      mk[r.robot_id].line.setLatLngs(head).setStyle({ color, dashArray: live ? null : '3 3' });
      mk[r.robot_id].dot.setLatLng([p.y, p.x]).setStyle({ fillColor: color, fillOpacity: live ? 1 : 0.55 }).setTooltipContent(tip);
    });
    Object.keys(mk).forEach((rid) => { if (!seen.has(rid)) { layer.removeLayer(mk[rid].line); layer.removeLayer(mk[rid].dot); delete mk[rid]; } });
  }, [robots, linked, m.map_id]);

  useEffect(() => {
    const group = groups.current, map = mapRef.current;
    if (!group.heat || !map) return;
    setHeatError('');
    if (!overlay || !options.heat) return;
    if (!overlay.readings) { drawOverlay(window.L, group.heat, m, { ...overlay, tasks: [], candidates: [], inspections: [], path: null, track: null }); return () => group.heat.clearLayers(); }
    const heat = createHeatLayer(window.L, m, overlay, options, (d) => select.current?.(d), setHeatError);
    heat.group.addTo(group.heat);
    return () => { heat.dispose(); group.heat.removeLayer(heat.group); };
  }, [overlay, m, options.heat, options.smooth, options.opacity]);
  useEffect(() => {
    const group = groups.current;
    if (!group.path) return;
    group.path.clearLayers();
    if (options.path) (paths || []).forEach((p) => {
      window.L.polyline(p.points.map((v) => [v.y,v.x]), {color:robotColor(p.robot_id),weight:2.5,opacity:.8,dashArray:'6 4'})
        .bindTooltip(`${p.robot_id} · Nav2 계획 경로`).addTo(group.path);
    });
    if (overlay && (options.path || mini || overlay.path || overlay.track)) drawOverlay(window.L, group.path, m, { ...overlay, heat: null, readings: null, candidates: [], inspections: [] });
  }, [overlay, m, options.path, mini, paths]);
  useEffect(() => {
    const group = groups.current;
    if (!group.result) return; group.result.clearLayers();
    if (overlay) drawOverlay(window.L, group.result, m, { candidates: overlay.candidates, inspections: overlay.inspections });
  }, [overlay, m]);
  useEffect(() => {
    const layer = groups.current.selection;
    if (!layer) return; layer.clearLayers();
    if (selection?.kind !== 'cell') return;
    const cell = (m.cells || []).find(([id]) => id === selection.data.cell_id);
    if (!cell) return;
    const x = m.origin_x + cell[1] * m.grid_size_m, y = m.origin_y + cell[2] * m.grid_size_m;
    window.L.rectangle([[y,x],[y+m.grid_size_m,x+m.grid_size_m]], {color:'#234f66',weight:2,fill:false,interactive:false}).addTo(layer);
  }, [selection, m]);
  return html`<div class="map-surface">${heatError && html`<div class="msg">${heatError}</div>`}${mini ? html`<div class="map-mini" ref=${ref}></div>` : html`<div id="map" ref=${ref}></div>`}</div>`;
}

// ---------------------------------------------------------------- 작업 탭 지도 표시 (3-3차)
// RSSI 색 — 기준(rssi_suspect_dbm, 기본 −72) 미만 빨강, 그 위 10 dB까지 주황, 더 세면 초록
const rssiColor = (v) => signalColor(v);
const CAND_COLOR = { OPEN: '#ea580c', REINSPECTING: '#7c3aed', CONFIRMED: '#b91c1c', CLEARED: '#15803d', UNCONFIRMED: '#64748b' };
const FINAL_COLOR = { normal: '#16a34a', abnormal: '#dc2626', unconfirmed: '#64748b' };
function drawOverlay(L, layer, m, ov) {
  const g = m.grid_size_m, byId = {};
  (m.cells || []).forEach(([id, ix, iy]) => { byId[id] = [ix, iy]; });
  if (ov.heat) {                                     // 칸별 평균 RSSI (선택한 AP, 또는 가장 센 AP)
    Object.entries(ov.heat).forEach(([cid, byAp]) => {
      const c = byId[cid];
      if (!c) return;
      const vals = Object.entries(byAp).filter(([ap]) => !ov.ap || String(ap) === String(ov.ap));
      if (!vals.length) return;
      const [ap, [mean, n]] = vals.reduce((a, b) => (b[1][0] > a[1][0] ? b : a));
      const x0 = m.origin_x + c[0] * g, y0 = m.origin_y + c[1] * g, col = rssiColor(mean, ov.thr);
      L.rectangle([[y0, x0], [y0 + g, x0 + g]], { weight: 0, fillColor: col, fillOpacity: 0.42 })
        .bindTooltip(`칸 ${cid} · ${ov.apName[ap] || ap} ${ov.readings ? '중앙값' : '평균'} ${mean} dBm · 표본 ${n}`, { direction: 'top' }).addTo(layer);
    });
  }
  (ov.tasks || []).forEach((t) => {                 // 경로 — 수행 중 실선, 대기 점선
    if (!['ACTIVE', 'PENDING'].includes(t.status) || !(t.waypoints || []).length) return;
    const col = robotColor(t.robot_id), pts = t.waypoints.map((p) => [p.y, p.x]);
    L.polyline(pts, { color: col, weight: t.status === 'ACTIVE' ? 3 : 1.5, opacity: t.status === 'ACTIVE' ? 0.9 : 0.5,
      dashArray: t.status === 'ACTIVE' ? null : '4 4', interactive: false }).addTo(layer);
    if (t.status === 'ACTIVE') pts.forEach((p) => L.circleMarker(p, { radius: 2, color: col, weight: 1, interactive: false }).addTo(layer));
  });
  if (ov.path && ov.path.pts.length) {              // task 「자세히」 — 계획 경로점(점선 · 번호) · 실제로 지나간 길(실선)
    const pts = ov.path.pts.map((p) => [p.y, p.x]), col = ov.path.color;
    L.polyline(pts, { color: col, weight: 1.5, opacity: 0.6, dashArray: '4 4', interactive: false }).addTo(layer);
    pts.forEach((p, i) => L.marker(p, { keyboard: false, icon: L.divIcon({ className: 'wp-num', iconSize: [16, 16],
      html: `<b style="border-color:${col};color:${col}">${i + 1}</b>` }) })
      .bindTooltip(`경로점 ${i + 1}`, { direction: 'top' }).addTo(layer));
  }
  if (ov.track && ov.track.pts.length) {
    const pts = ov.track.pts.map((p) => [p.y, p.x]), col = ov.track.color;
    L.polyline(pts, { color: col, weight: 3.5, opacity: 0.9, interactive: false }).addTo(layer);
    L.circleMarker(pts[0], { radius: 5, color: col, weight: 2, fillColor: '#fff', fillOpacity: 1 })
      .bindTooltip('시작', { direction: 'top' }).addTo(layer);
    L.circleMarker(pts[pts.length - 1], { radius: 5, color: '#fff', weight: 2, fillColor: col, fillOpacity: 1 })
      .bindTooltip('끝', { direction: 'top' }).addTo(layer);
  }
  (ov.candidates || []).forEach((c) => {           // 후보 — 상태 · 결론 색
    if (c.target_x === null) return;
    const col = CAND_COLOR[c.outcome || c.status] || '#ea580c';
    L.circleMarker([c.target_y, c.target_x], { radius: 7, color: '#fff', weight: 2, fillColor: col, fillOpacity: 0.95 })
      .bindTooltip(`${c.candidate_id} · 칸 ${c.ref} · ${c.outcome || c.status} · ${fmtNum(c.score)} dBm`, { direction: 'top' }).addTo(layer);
  });
  (ov.inspections || []).forEach((i) => {          // 설비 판정 — 설비 위치에 결과 색 테두리
    L.circleMarker([i.expected_y, i.expected_x], { radius: 13, color: FINAL_COLOR[i.final] || '#64748b', weight: 3, fill: false })
      .bindTooltip(`${i.facility_id} · ${i.detail} · ${i.final}`, { direction: 'bottom' }).addTo(layer);
  });
}

// /ws/ui — 로봇 상태 1초마다. 끊기면 2초 뒤 다시 접속, 로그인이 풀렸으면(4401) 로그인 화면으로
function useUiSocket(onAuthLost) {
  const [data, setData] = useState(null);
  const [linked, setLinked] = useState(false);
  const [now, setNow] = useState(Date.now());
  const lastRx = useRef(0);
  useEffect(() => { const t = setInterval(() => setNow(Date.now()), 1000); return () => clearInterval(t); }, []);
  useEffect(() => {
    let ws, timer, alive = true;
    const open = () => {
      ws = new WebSocket(`${location.protocol === 'https:' ? 'wss' : 'ws'}://${location.host}/ws/ui`);
      ws.onopen = () => setLinked(true);
      ws.onmessage = (e) => { try { const d = JSON.parse(e.data); if (d.type === 'robots') { lastRx.current = Date.now(); setData(d); } } catch { /* 무시 */ } };
      ws.onclose = (e) => {
        setLinked(false);
        if (!alive) return;
        if (e.code === 4401) { onAuthLost(); return; }
        timer = setTimeout(open, 2000);
      };
    };
    open();
    return () => { alive = false; clearTimeout(timer); ws && ws.close(); };
  }, []);
  return [data, linked && now - lastRx.current < 5000];
}

const ago = (iso) => {
  if (!iso) return '—';
  const s = Math.max(0, (Date.now() - new Date(iso).getTime()) / 1000);
  return s < 60 ? `${s.toFixed(0)}초 전` : `${(s / 60).toFixed(0)}분 전`;
};

function RobotPanel({ data, linked, onStart, onAbort }) {
  const robots = data ? data.robots : [];
  const st = data && data.start;
  const runId = data && data.run_id;
  const ready = st ? st.robots.filter((r) => r.ready).map((r) => r.robot_id) : [];
  const why = !st ? '' : runId ? '' : st.problems.length ? st.problems.join(' · ')
    : !ready.length ? st.robots.map((r) => `${r.robot_id}: ${r.why.join(' · ')}`).join(' / ') : '';
  return html`<div class="rpanel">
    <h3>로봇 상태
      ${onStart && (runId ? html`<button class="btn small danger" onClick=${() => onAbort(runId)}>순찰 중단 (run ${runId})</button>`
        : html`<button class="btn small primary" disabled=${!st || !st.can_start} title=${why} onClick=${onStart}>순찰 시작</button>`)}
      <span class=${'live ' + (linked ? 'on' : '')}>${linked ? '실시간' : '연결 중…'}</span></h3>
    ${onStart && !runId && why && html`<p class="startwhy">순찰 시작 불가 — ${why}</p>`}
    ${robots.length === 0 && html`<p>등록된 로봇이 없습니다.</p>`}
    ${robots.map((r) => {
      const [k, label] = robotPhase(r), st = r.state;
      return html`<div key=${r.robot_id} class="rrow">
        <div class="rh"><i style=${{ background: robotColor(r.robot_id) }}></i><b>${r.robot_id}</b>
          <span class=${'chip ' + k}>${label}</span></div>
        ${st ? html`<div class="rkv">
            <span>위치</span><span class="num">${st.pose_ok === false ? '위치 불확실·갱신 대기' : `${fmtNum(st.x)}, ${fmtNum(st.y)}`} · ${st.yaw == null ? '—' : (st.yaw * 180 / Math.PI).toFixed(0) + '°'}</span>
            <span>배터리</span><span class="num">${st.battery_pct == null ? '미수신' : st.battery_pct.toFixed(1) + '%'}</span>
            <span>상태</span><span>${st.nav_state || '—'}${st.is_docked ? ' · 도킹' : ''}${st.estop ? ' · 정지' : ''}</span>
            <span>작업</span><span>${st.run_id ? `run ${st.run_id}` : '—'}${st.task_id ? ` · ${st.task_id}` : ''}</span>
            <span>마지막 보고</span><span>${ago(st.ts)}</span>
          </div>
          ${r.is_active && st.raw && html`<details class="raw">
            <summary>관제 PC 메모리 상태 <code>${r.robot_id}</code> 전체 · E3까지 ${st.ttl_s}초</summary>
            <div class="rkv mono">${Object.entries(st.raw).map(([k, v]) => html`<span key=${k}>${k}</span><span>${v === '' ? '—' : v}</span>`)}</div>
          </details>`}`
          : html`<p>${r.last_e3 ? `${fmtTime(r.last_e3)} 응답 없음(E3) 기록` : '받은 상태 없음'}</p>`}
      </div>`;
    })}
    ${data && html`<p class="foot">${data.timeout_s}초 동안 상태가 오지 않으면 응답 없음(E3)으로 기록</p>`}
  </div>`;
}

function useMonitorData(onAuthLost) {
  const [state, setState] = useState({ loading: true });
  const [detail, setDetail] = useState(null);
  const [error, setError] = useState('');
  useEffect(() => {
    let alive = true, timer;
    const tick = async () => {
      try {
        const d = await api('/api/maps/active');
        const r = await api('/api/runs');
        if (alive) setState({ ...d, runId: r.current ?? r.runs[0]?.run_id ?? null, updated: new Date().toISOString() });
      } catch (e) { if (alive) { if (e.status === 401) onAuthLost(); else setState((s) => ({ ...s, loading: false, error: e.message })); } }
      if (alive) timer = setTimeout(tick, 5000);
    };
    tick(); return () => { alive = false; clearTimeout(timer); };
  }, []);
  const runId = state.runId;
  useEffect(() => {
    let alive = true, timer;
    setDetail(null); setError('');
    if (!runId) return;
    const tick = async () => {
      let done = false;
      try {
        const d = await api(`/api/runs/${runId}`);
        if (alive) { setDetail(d); setError(''); }
        done = d.run.status !== 'running';
      } catch (e) { if (alive) { if (e.status === 401) onAuthLost(); else setError(e.message); } }
      if (alive && !done) timer = setTimeout(tick, 2000);
    };
    tick(); return () => { alive = false; clearTimeout(timer); };
  }, [runId]);
  return [state, detail, error];
}

function preferredAP(aps, readings) {
  const measured = new Set((readings || []).map((r) => String(r.ap_id)));
  const used = aps.filter((a) => measured.has(String(a.ap_id)));
  return (used.find((a) => a.is_target) || used[0] || aps.find((a) => a.is_target) || aps[0])?.ap_id;
}

function readingOverlay(d, ap) {
  if (!d || !ap) return null;
  const heat = {};
  (d.readings || []).filter((r) => String(r.ap_id) === String(ap) && r.median != null && r.n > 0 && !r.interpolated).forEach((r) => {
    heat[r.cell_id] = { [r.ap_id]: [r.median, r.n] };
  });
  return { heat, readings: (d.readings || []).filter((r) => String(r.ap_id) === String(ap)), ap,
    apName: Object.fromEntries(d.aps.map((a) => [a.ap_id, a.ssid])), thr: d.thr,
    tasks: d.tasks, candidates: d.candidates, inspections: d.inspections };
}

function RobotCards({ robots, linked, onSelect }) {
  return html`<section class="panel robot-cards"><div class="section-title"><h3>로봇 현황</h3><span class=${'live-dot ' + (linked ? 'on' : '')}>${linked ? '수신 중' : '연결 대기'}</span></div>
    ${!robots.length && html`<p class="no">등록된 로봇 없음</p>`}
    ${robots.map((r) => {
      const s = r.state, fresh = linked && r.connected;
      const label = !s ? '상태 미수신' : !fresh ? '연결 끊김' : s.nav_state || '상태 수신';
      const progress = s && Number.isFinite(s.progress) ? Math.max(0, Math.min(1, s.progress)) : null;
      return html`<button class="robot-card" onClick=${() => onSelect({ kind: 'robot', data: { robot_id: r.robot_id } })}>
        <div class="robot-card-head"><span class="robot-symbol" style=${{ background: robotColor(r.robot_id) }}>R${r.robot_id.replace('robot', '')}</span><div><b>${r.robot_id}</b><small>${fresh && s?.pose_ok !== false ? '위치 수신' : s?.last_valid_pose ? '마지막 위치 보관' : '현재 위치 미수신'}</small></div><span class=${'chip ' + (!fresh ? 'off' : s?.warning || s?.estop ? 'bad' : 'ok')}>${label}</span></div>
        <div class="robot-metrics"><span>배터리 <b>${fresh && s?.battery_pct != null ? `${Math.round(s.battery_pct)}%` : '미수신'}</b></span><span>${s?.task_id ? TYPE_LABEL[s.task_type] || ({0:'측정',1:'설비 점검',2:'정밀측정',3:'복귀'})[s.task_type] || '작업 중' : '배정 작업 없음'}</span></div>
        <div class="progress-track"><i style=${{ width: `${fresh && progress != null ? Math.round(progress * 100) : 0}%`, background: robotColor(r.robot_id) }}></i></div>
        <div class="robot-bottom"><span>${s?.task_id && progress != null ? `작업 진행 ${Math.round(progress * 100)}%` : '작업 대기'}</span><span>${s?.ts ? ago(s.ts) : '보고 대기'}</span></div>
        ${s?.warning && html`<p class="robot-warning">${s.warning}</p>`}
      </button>`;
    })}</section>`;
}

function SelectionDetail({ selection, robots, d, m, ap, liveInspections }) {
  if (!selection) return html`<section class="panel selection"><h3>선택 상세</h3><div class="selection-empty"><span>⌖</span><p>지도의 로봇·설비·측정 영역을<br/>선택하면 상세를 볼 수 있습니다.</p></div></section>`;
  const { kind, data } = selection;
  let title, body;
  if (kind === 'robot') {
    const r = robots.find((r) => r.robot_id === data.robot_id), s = r?.state;
    title = data.robot_id;
    body = html`<dl class="kv"><dt>현재 상태</dt><dd>${r?.connected ? s?.nav_state || '미수신' : '연결 끊김·미수신'}</dd><dt>위치</dt><dd>${r?.connected && s?.pose_ok && s.x != null ? `${fmtNum(s.x)}, ${fmtNum(s.y)} m` : '위치 불확실·미수신'}</dd><dt>작업</dt><dd>${s?.task_id || '없음'}</dd><dt>회차</dt><dd>${s?.run_id || '미배정'}</dd><dt>최근 수신</dt><dd>${s?.received_at ? ago(s.received_at) : '없음'}</dd></dl>`;
  } else if (kind === 'cell') {
    title = '측정 영역';
    const rs = (d?.readings || []).filter((r) => r.cell_id === data.cell_id && String(r.ap_id) === String(ap));
    const r = rs[0];
    body = r ? html`<div class="signal-value">${r.median == null ? '—' : fmtNum(r.median)}<small>dBm</small></div><dl class="kv"><dt>통계</dt><dd>${r.statistic === 'committed_median' ? '저장된 중앙값' : '실시간 중앙값'}</dd><dt>측정 수</dt><dd>${r.n}개 · 미검출 ${r.misses}회</dd><dt>판정</dt><dd>${({normal:'정상',coverage_suspect:'커버리지 저하 의심',unknown:'판단 불가',unclassified:'판정 대기'})[r.state] || r.state}</dd><dt>표본 신뢰도</dt><dd>${r.confidence === 'ok' ? '충분' : '부족'}</dd></dl><p class="foot">색상 보간은 표시용입니다. 위 값과 판정은 원본 칸 데이터입니다.</p>` : html`<p class="no">이 AP의 유효 측정값이 없습니다.</p>`;
  } else if (kind === 'facility') {
    title = data.facility_id;
    const ins = [...(d?.inspections || [])].reverse().find((i) => i.facility_id === data.facility_id);
    const live = [...(liveInspections || [])].reverse().find((i) => i.facility_id === data.facility_id);
    body = html`<dl class="kv"><dt>설비</dt><dd>${FAC_LABEL[data.facility_type] || data.facility_type}</dd><dt>저장 판정</dt><dd>${ins ? ({normal:'정상',abnormal:'이상',unconfirmed:'미확인'})[ins.final] : '저장된 판정 없음'}</dd><dt>실시간 판정</dt><dd>${live ? (['정상','미검출','가림','위치 이상','이상','미확인'][live.verdict] || live.detail) : '미수신'}</dd></dl>
      ${(live?.photo_url || live?.photo) && html`<img class="detail-photo" src=${live.photo_url || live.photo} alt="ROS 실시간 근거 사진"/><p class="foot">실시간 수신 사진 · 이력 저장 여부는 별도</p>`}
      ${(ins?.photos || []).map((p) => html`<a href=${'/media/' + p} target="_blank" rel="noreferrer"><img class="detail-photo" src=${'/media/' + p} alt="저장된 설비 점검 사진"/></a>`)}`;
  } else if (kind === 'reference') {
    title = `${data.robot_id} 시작 참고점`;
    body = html`<p>기본 설정에서 읽은 undock 직후 참고 자세입니다.</p><dl class="kv"><dt>참고 좌표</dt><dd>${fmtNum(data.x)}, ${fmtNum(data.y)} m</dd><dt>방향</dt><dd>${(data.yaw * 180 / Math.PI).toFixed(1)}°</dd><dt>출처</dt><dd>공통 시작 설정</dd></dl><p class="foot">실측 현재 위치가 아닙니다. 실행 시 지정값은 별도로 적용될 수 있습니다.</p>`;
  } else {
    title = data.ssid;
    body = html`<dl class="kv"><dt>측정 대상</dt><dd>${data.is_target ? '선택됨' : '대상 아님'}</dd><dt>설치 위치</dt><dd>${fmtNum(data.x)}, ${fmtNum(data.y)} m</dd></dl><p class="foot">등록 정보입니다. 수신 중인 신호 유무는 측정 결과로 확인합니다.</p>`;
  }
  return html`<section class="panel selection"><div class="section-title"><h3>${title}</h3></div>${body}</section>`;
}

function MonitorPage({ onAuthLost }) {
  const [state, d, heatError] = useMonitorData(onAuthLost), [ui, linked] = useUiSocket(onAuthLost);
  const [ap, setAp] = useState(''), [selection, setSelection] = useState(null);
  const manualAP = useRef(false);
  const [layers, setLayers] = useState({ heat: true, smooth: true, opacity: 0.5, path: false, grid: false, zones: false, references: false });
  const [starting, setStarting] = useState(false), [message, setMessage] = useState('');
  const m = state.map, sameMap = m && d?.run?.map_id === m.map_id;
  const owner = state.run_owner || ui?.run_owner, ros = owner === 'ros';
  const aps = d?.aps || m?.aps || [];
  const apSignature = aps.map((a) => `${a.ap_id}:${a.is_target}`).join(',') + ':' + [...new Set((d?.readings || []).map((r) => r.ap_id))].join(',');
  useEffect(() => { manualAP.current = false; }, [d?.run?.run_id]);
  useEffect(() => { setAp((old) => manualAP.current && aps.some((a) => String(a.ap_id) === old) ? old : String(preferredAP(aps, d?.readings) || '')); }, [apSignature, d?.run?.run_id]);
  useEffect(() => { setSelection(null); }, [m?.map_id, d?.run?.run_id]);
  const robots = ui?.robots || [];
  const safeRobots = robots.map((r) => ({ ...r, connected: linked && r.connected }));
  const navPaths = linked && sameMap && m?.configuration?.status === 'ok' ? (ui?.telemetry?.paths || []).filter((p) => p.run_id === d.run.run_id) : [];
  const liveInspections = sameMap ? (ui?.telemetry?.inspections || []).filter((i) => i.run_id === d.run.run_id) : [];
  const alerts = ui?.telemetry?.alerts || [];
  const overlay = useMemo(() => sameMap ? readingOverlay(d, ap) : null, [d, ap, sameMap]);
  const toggle = (k) => setLayers((l) => ({ ...l, [k]: !l[k] }));
  const abort = async () => {
    if (!confirm('진행 중인 순찰을 중단하고 로봇을 복귀시킬까요?')) return;
    try { await api(`/api/runs/${d.run.run_id}/abort`, { method:'POST' }); } catch(e) { setMessage(e.message); }
  };
  return html`<main class="dashboard">
    ${ui?.preview && html`<div class="preview-banner">샘플 데이터 미리보기 · 실제 로봇 및 운영 DB 미연결</div>`}
    <div class="dashboard-heading"><div><p class="eyebrow">SCV CONTROL CENTER</p><h2>통신·설비 점검 현황</h2></div><div class="connection-summary"><span class=${'live-dot ' + (linked ? 'on' : '')}>${linked ? '웹 상태 수신 중' : '서버 연결 대기'}</span><small>${ui?.ts ? `최근 갱신 ${hm(ui.ts)}` : '수신 대기'}</small></div></div>
    ${(state.error || heatError || message) && html`<div class="msg">${state.error || heatError || message} · 마지막 수신값을 유지합니다.</div>`}
    ${ros && ui?.telemetry?.bridge?.error && html`<div class="msg">ROS 상태 연결 실패: ${ui.telemetry.bridge.error}</div>`}
    <div class="overview-strip"><div><small>표시 지도</small><b>${m?.name || '등록 대기'}</b></div><div><small>점검 회차</small><b>${d ? `#${d.run.run_id} · ${d.run.status === 'running' ? '점검 진행 중' : `최근 회차 · ${RUN_LABEL[d.run.status] || d.run.status}`}` : '회차 없음'}</b></div><div><small>수신 로봇</small><b>${safeRobots.filter((r) => r.connected).length} / ${robots.length}</b></div><div><small>측정 표본</small><b>${d?.samples?.toLocaleString() || '—'}</b></div><div><small>운용 방식</small><b>${ros ? 'ROS 관제 연결' : owner === 'web' ? '웹 관제' : '확인 중'}</b></div></div>
    <div class="dashboard-grid"><section class="mapbox monitor-map"><div class="head"><b>점검 지도</b><span>${m?.name || ''}</span><span class="map-hint">대상을 눌러 상세 확인</span></div>
      <div class="map-toolbar">${[['heat','히트맵'],['path','경로'],['zones','구역'],['references','시작 참고점'],['grid','격자·번호']].map(([k, label]) => html`<button class=${'layer-btn ' + (layers[k] ? 'on' : '')} aria-pressed=${layers[k]} onClick=${() => toggle(k)}>${label}</button>`)}
        <select aria-label="측정 AP" value=${ap} onChange=${(e) => { manualAP.current = true; setAp(e.target.value); }}>${!aps.length && html`<option value="">측정 AP 없음</option>`}${aps.map((a) => html`<option value=${a.ap_id}>${a.ssid}${a.is_target ? ' · 등록 대상' : ''}</option>`)}</select></div>
      ${state.loading ? html`<div class="empty">지도를 불러오고 있습니다.</div>` : !m ? html`<div class="empty">현재 지도가 없습니다. 설정에서 기준 지도를 등록해 주세요.</div>` : !m.width ? html`<div class="empty">지도 이미지 파일을 찾을 수 없습니다.</div>` : html`<${MapView} m=${m} robots=${ros && m.configuration?.status !== 'ok' ? [] : safeRobots} linked=${linked} overlay=${overlay} layers=${layers} onSelect=${setSelection} selection=${selection} paths=${navPaths}/>`}
      <div class="heat-legend"><div><b>신호 세기</b><div class="signal-gradient"></div><div class="gradient-labels"><span>약함 −90</span><span>−65</span><span>−40 dBm 강함</span></div></div><label><input type="checkbox" checked=${layers.smooth} onChange=${() => toggle('smooth')}/> 부드럽게</label><label>농도 <input aria-label="히트맵 농도" type="range" min="0.15" max="0.8" step="0.05" value=${layers.opacity} onInput=${(e) => setLayers((l) => ({...l, opacity: Number(e.target.value)}))}/></label><small>점무늬: 미검출 · 빈 영역: 미측정<br/>중앙값 · 보간은 표시 전용${d ? ` · 의심 기준 ${d.thr} dBm` : ''}</small></div>
      ${d && !sameMap && html`<div class="msg">회차와 지도가 달라 히트맵을 생략했습니다.</div>`}
      ${m?.configuration?.status !== 'ok' && m?.configuration && html`<p class="map-status">${m.configuration.message}</p>`}
      <details class="map-diagnostics"><summary>지도·데이터 정보</summary><p>해상도 ${m?.resolution_m} m/px · 격자 ${m?.grid_size_m} m · 최근 DB 조회 ${d?.fetched_at ? fmtTime(d.fetched_at) : '대기'}</p><p>${m?.configuration?.message}</p><p>경로는 DB 작업 경로점 또는 수신된 Nav2 계획 경로입니다. 실제 주행 궤적은 아닙니다. 미수신 경로는 표시하지 않습니다. 설비·AP는 DB 등록값이며 ROS 설정을 자동 덮어쓰지 않습니다.</p></details>
    </section><aside class="dashboard-side"><${RobotCards} robots=${safeRobots} linked=${linked} onSelect=${setSelection}/><${SelectionDetail} selection=${selection} robots=${safeRobots} d=${sameMap ? d : null} m=${m} ap=${ap} liveInspections=${liveInspections}/>
      ${owner === 'web' && html`<section class="panel"><button class="btn primary" onClick=${() => d?.run.status === 'running' ? abort() : setStarting(true)}>${d?.run.status === 'running' ? '순찰 중단' : '순찰 시작'}</button></section>`}
    </aside></div>
    <section class="panel alert-panel"><div class="section-title"><h3>경보·최근 기록</h3><span class="no">최근 수신 기준</span></div>${alerts.length ? html`<div class="alert-items">${alerts.slice(-5).reverse().map((a) => html`<div class=${'alert-item ' + a.level}><span class="alert-level">${a.level === 'error' ? '이상' : a.level === 'warn' ? '주의' : '안내'}</span><b>${a.robot_id || '관제'}</b><span>${a.message}</span><small>${ago(a.received_at)}${a.delayed ? ' · 지연 수신' : ''}</small></div>`)}</div>` : html`<p class="no">수신된 ROS 경보 없음${!linked ? ' · 연결 상태를 확인해 주세요.' : ''}</p>`}<details><summary>회차 이벤트 전체 보기</summary><${EventList} events=${ui?.events || []} max=${100}/></details></section>
    ${starting && html`<${StartModal} onClose=${() => setStarting(false)} onStarted=${() => setStarting(false)}/>`}
  </main>`;
}

// ---------------------------------------------------------------- 순찰 시작 창 · 이력 (3-3차)
const TYPE_LABEL = { SURVEY_SEGMENT: '측정', REINSPECTION: '정밀측정', FACILITY_CHECK: '탐지', DOCK: '복귀' };
const TASK_FILTERS = [['', '전체'], ['SURVEY_SEGMENT', '측정'], ['REINSPECTION', '정밀측정'], ['FACILITY_CHECK', '탐지'], ['DOCK', '복귀']];
const STATUS_CHIP = { running: 'ok', done: 'ok', partial: 'wait', aborted: 'bad',
  ACTIVE: 'ok', PENDING: 'off', DONE: 'ok', FAILED: 'bad', CANCELED: 'off' };
const RUN_TYPE_LABEL = { patrol: '순찰' };          // 순찰 종류는 순찰 하나 (DB의 이전 demo · remeasure 회차는 값 그대로 표시)
const RUN_LABEL = { running: '진행 중', done: '완료', partial: '일부 섹터 실패', aborted: '중단 · 실패' };
const pct = (v) => (v === null || v === undefined ? '—' : `${Math.round(v * 100)}%`);
const hm = (v) => {
  if (!v) return '—';
  const d = new Date(v), p = (n) => String(n).padStart(2, '0');
  return `${p(d.getHours())}:${p(d.getMinutes())}:${p(d.getSeconds())}`;
};

function EventList({ events, max = 12 }) {
  const ev = (events || []).slice(-max).reverse();
  if (!ev.length) return html`<p class="no">이벤트 없음</p>`;
  return html`<ul class="evlist">${ev.map((e) => html`<li key=${e.event_id} class=${'lv-' + e.level}>
    <span class="t">${hm(e.ts)}</span><b>${e.code}</b><span>${e.robot_id}${e.task_id ? ' · ' + e.task_id : ''}</span>
    <small>${e.detail ? Object.entries(e.detail).filter(([k]) => k !== 'medians').map(([k, v]) => `${k} ${typeof v === 'object' ? JSON.stringify(v) : v}`).join(' · ') : ''}</small>
  </li>`)}</ul>`;
}

function StartModal({ onClose, onStarted }) {
  const [form, setForm] = useState(null);
  const [sel, setSel] = useState({});
  const [msg, setMsg] = useState('');
  const [busy, setBusy] = useState(false);
  useEffect(() => {
    api('/api/runs/form').then((f) => { setForm(f); setSel(f.default || {}); }).catch((e) => setMsg(e.message));
  }, []);
  const submit = async () => {
    setBusy(true); setMsg('');
    try { const r = await api('/api/runs', { json: { sector_robots: sel } }); onStarted(r.run_id); }
    catch (e) { setMsg(e.message); }
    setBusy(false);
  };
  return html`<div class="backdrop"><div class="modal narrow">
    <div class="mh"><b>순찰 시작</b><code>${form && form.map ? form.map.name : ''}</code>
      <button class="btn small x" onClick=${onClose}>닫기</button></div>
    <div class="mdesc">섹터마다 순찰할 로봇을 고릅니다. 정상 · 도킹 중인 로봇만 고를 수 있고, 로봇 1대 = 섹터 1개입니다.
      현재 지도 · 격자와 활성 측정 AP가 있어야 합니다.
      로봇은 측정 → 후보 칸 정밀측정 → 설비 탐지(1회 판정) → 복귀를 차례로 합니다.</div>
    <div class="body">${!form ? html`<p>불러오는 중…</p>` : html`
      <table class="grid"><thead><tr><th>섹터</th><th>칸</th><th>로봇</th></tr></thead><tbody>
        ${form.sectors.map((x) => html`<tr key=${x.sector}>
          <td><i class="sec" style=${{ background: SECTOR_COLOR[x.sector] }}></i> 섹터 ${x.sector}</td><td>${x.cells}칸</td>
          <td><select value=${sel[x.sector] || ''} onChange=${(e) => setSel({ ...sel, [x.sector]: e.target.value || null })}>
            <option value="">순찰 안 함</option>
            ${form.robots.map((r) => html`<option key=${r.robot_id} value=${r.robot_id} disabled=${!r.ready}>
              ${r.robot_id}${r.ready ? '' : ` (${r.why.join(' · ') || '불가'})`}</option>`)}
          </select></td></tr>`)}
      </tbody></table>
      ${form.target_ap && html`<p class="no">측정 AP ${form.target_ap.ssid} — ${form.target_ap.active ? '활성' : '비활성'}</p>`}
      ${form.problems.length > 0 && html`<p class="lockmsg">순찰 시작 불가 — ${form.problems.join(' · ')}</p>`}
      ${form.running && html`<p class="lockmsg">순찰 run ${form.running}이 진행 중입니다.</p>`}`}
    </div>
    ${msg && html`<div class="msg">${msg}</div>`}
    <div class="tools"><span class="sp"></span>
      <button class="btn primary" disabled=${busy || !form || !form.can_start} onClick=${submit}>순찰 시작</button></div>
  </div></div>`;
}

function HistoryPage({ onAuthLost }) {
  const [m, setM] = useState(null);
  const [events, setEvents] = useState([]);
  const [runs, setRuns] = useState({ runs: [], current: null });
  const [runId, setRunId] = useState(null);
  const [d, setD] = useState(null);
  const [filter, setFilter] = useState('');
  const [ap, setAp] = useState('');
  const [msg, setMsg] = useState('');
  const [detail, setDetail] = useState(null);       // 「자세히」 task_id
  const [advanced, setAdvanced] = useState(false);
  const fail = (e) => (e.status === 401 ? onAuthLost() : setMsg(e.message));
  const loadRuns = () => api('/api/runs').then((r) => { setRuns(r); return r; }).catch(fail);
  useEffect(() => {                                  // 현재 지도의 순찰 목록 — 진행 중 순찰이 생기거나 끝나면 5초 안에 반영
    api('/api/maps/active').then((x) => setM(x.map)).catch(fail);
    const f = () => loadRuns().then((r) => r && setRunId((v) => v ?? r.current ?? (r.runs[0] && r.runs[0].run_id) ?? null));
    f();
    const t = setInterval(f, 5000);
    return () => clearInterval(t);
  }, []);
  useEffect(() => {
    let alive = true, timer;
    setD(null); setEvents([]);
    if (!runId) return;
    const tick = async () => {
      let completed = false;
      try {
        const x = await api(`/api/runs/${runId}`);
        const events = await api(`/api/runs/${runId}/events`);
        const map = await api(`/api/maps/${x.run.map_id}`);
        if (alive) { setD(x); setM(map.map); setEvents(events.events); setMsg(''); }
        completed = x.run.status !== 'running';
      } catch (e) { if (alive) fail(e); }
      if (alive && !completed) timer = setTimeout(tick, 2000);
    };
    tick(); return () => { alive = false; clearTimeout(timer); };
  }, [runId]);
  const historyAPs = d?.aps || [];
  useEffect(() => { setAp((a) => historyAPs.some((p) => String(p.ap_id) === a) ? a : String(preferredAP(historyAPs, d?.readings) || '')); }, [d?.run?.run_id]);
  const run = d && d.run;
  const apName = {};
  ((d && d.aps) || []).forEach((a) => { apName[a.ap_id] = a.ssid; });
  const sameMap = m && run && m.map_id === run.map_id;
  const overlay = useMemo(() => d && sameMap ? readingOverlay(d, ap) : null, [d, sameMap, ap]);
  const tasks = ((d && d.tasks) || []).filter((t) => !filter || t.type === filter);
  const byRobot = {};
  ((d && d.tasks) || []).forEach((t) => {
    const b = byRobot[t.robot_id] = byRobot[t.robot_id] || { n: 0, done: 0, active: null };
    b.n += 1; if (['DONE', 'FAILED', 'CANCELED'].includes(t.status)) b.done += 1; if (t.status === 'ACTIVE') b.active = t;
  });
  return html`<main>
    <h2>점검 결과</h2>
    <p class="sub">회차를 선택해 통신 상태와 설비 점검 결과를 확인합니다. 색상은 중앙값이며, 작업·판정의 원본 표는 필요할 때 펼칠 수 있습니다.</p>
    <div class="runbar panel">
      <select value=${runId || ''} onChange=${(e) => setRunId(Number(e.target.value) || null)}>
        ${!runs.runs.length && html`<option value="">이 지도의 순찰 기록 없음</option>`}
        ${runs.runs.map((r) => html`<option key=${r.run_id} value=${r.run_id}>run ${r.run_id} · ${RUN_LABEL[r.status] || r.status} · ${fmtTime(r.started_at)}</option>`)}
      </select>
      ${run && html`<span class=${'chip ' + (STATUS_CHIP[run.status] || 'off')}>${RUN_LABEL[run.status] || run.status}</span>
        <span>${run.map_name} · ${RUN_TYPE_LABEL[run.run_type] || run.run_type} · 배정 ${Object.entries(run.sector_robots).map(([k, v]) => `섹터 ${k} = ${v}`).join(', ')}</span>
        <span class="no">시작 ${fmtTime(run.started_at)}${run.started_by_name ? ' · ' + run.started_by_name : ''}${run.ended_at ? ' · 종료 ' + fmtTime(run.ended_at) : ''}</span>
        ${run.is_baseline_ok && html`<span class="chip ok">기준선 재료</span>`}`}
    </div>
    ${msg && html`<div class="msg">${msg}</div>`}
    <div class="workgrid">
      <div class="mapbox">
        <div class="head">
          ${run ? html`<b>run ${run.run_id}</b>` : html`<b>${m ? m.name : '지도 없음'}</b>`}
          ${Object.entries(byRobot).map(([rid, b]) => html`<span key=${rid}><i class="dot" style=${{ background: robotColor(rid) }}></i>
            ${rid} ${b.done}/${b.n}${b.active ? ` · ${TYPE_LABEL[b.active.type]} ${b.active.task_id} ${pct(b.active.progress)}` : ''}</span>`)}
          ${d && html`<span>표본 ${d.samples}</span>`}
          <span class="legend">
            <span class="lg">히트맵 <select class="mini" value=${ap} onChange=${(e) => setAp(e.target.value)}>
              <option value="" disabled>측정 AP 선택</option>${((d && d.aps) || []).map((a) => html`<option key=${a.ap_id} value=${a.ap_id}>${a.ssid}</option>`)}
            </select></span>
            <span class="lg"><span class="signal-gradient" style=${{width:120,height:8,display:'inline-block'}}></span>중앙값 −90…−40 dBm · DB 의심 기준 ${d?.thr ?? '—'} dBm</span>
            <span class="lg">● 후보 (주황 열림 · 보라 정밀측정 중 · 빨강 확정 · 초록 해제)</span>
            <span class="lg">◯ 설비 판정 (초록 정상 · 빨강 이상 · 회색 미확인)</span>
          </span>
        </div>
        ${!m ? html`<div class="empty">현재 지도가 없습니다.</div>`
          : html`<${MapView} m=${m} robots=${[]} overlay=${overlay} />`}
        ${run && !sameMap && m && html`<div class="empty">이 run은 다른 지도(${run.map_name})라 지도 위 표시는 생략합니다.</div>`}
      </div>
      <div class="side">
        <div class="panel"><h3>이 순찰의 이벤트 <span class="no">${events.length}건</span></h3><${EventList} events=${events} max=${200} /></div>
      </div>
    </div>
    <button class="btn" aria-expanded=${advanced} onClick=${() => setAdvanced(!advanced)}>${advanced ? '상세 표 접기' : '작업·판정 상세 표 보기'}</button>
    ${d && advanced && html`<div class="panel">
      <div class="ttl"><b>작업</b><code>tasks</code>
        <span class="filters">${TASK_FILTERS.map(([k, label]) => html`<button key=${k} class=${'pg' + (filter === k ? ' on' : '')}
          onClick=${() => setFilter(k)}>${label} ${(d.tasks || []).filter((t) => !k || t.type === k).length}</button>`)}</span></div>
      <table class="grid"><thead><tr><th>task</th><th>로봇</th><th>종류</th><th>대상</th><th>점</th><th>상태</th><th>진행률</th><th>사유</th><th>보낸 시각</th><th>끝난 시각</th><th></th></tr></thead>
      <tbody>${tasks.map((t) => html`<tr key=${t.task_id}>
        <td class="mono">${t.task_id}</td><td>${t.robot_id}</td><td>${TYPE_LABEL[t.type] || t.type}</td><td class="mono">${t.target_ref || '—'}</td>
        <td>${(t.waypoints || []).length}</td><td><span class=${'chip ' + (STATUS_CHIP[t.status] || 'off')}>${t.status}</span></td>
        <td>${pct(t.progress)}</td><td class="mono">${t.reason || ''}</td><td>${hm(t.assigned_at)}</td><td>${hm(t.ended_at)}</td>
        <td><button class="btn small" onClick=${() => setDetail(t.task_id)}>자세히</button></td></tr>`)}
      </tbody></table></div>
    <div class="cols2">
      <div class="panel"><div class="ttl"><b>후보 → 정밀측정 결론</b><code>candidates</code><span class="n">${d.candidates.length}</span></div>
        ${d.candidates.length ? html`<table class="grid"><thead><tr><th>후보</th><th>칸</th><th>측정 중앙값</th><th>정밀측정 task</th><th>정밀측정 중앙값</th><th>결론</th></tr></thead>
        <tbody>${d.candidates.map((c) => html`<tr key=${c.candidate_id}>
          <td class="mono">${c.candidate_id}</td><td>${c.ref} (${c.ix},${c.iy})</td><td>${fmtNum(c.score)} dBm</td>
          <td class="mono">${c.reinspect_task_id || '—'}</td><td>${c.precise_best === null ? '—' : fmtNum(c.precise_best) + ' dBm'}</td>
          <td><span class="cand" style=${{ background: CAND_COLOR[c.outcome || c.status] }}>${c.outcome || c.status}</span>${c.finding_id ? ' · 확정 이상 ' + c.finding_id : ''}</td></tr>`)}
        </tbody></table>` : html`<p class="no">후보 없음 — 칸 중앙값(가장 센 AP)이 ${d.thr} dBm 미만인 칸이 나오면 생깁니다.</p>`}</div>
      <div class="panel"><div class="ttl"><b>설비 탐지 (1회 판정)</b><code>inspections</code><span class="n">${d.inspections.length}</span></div>
        ${d.inspections.length ? html`<table class="grid"><thead><tr><th>설비</th><th>판정</th><th>최종</th><th>정면</th><th>보정</th><th>압력계</th><th>사진</th></tr></thead>
        <tbody>${d.inspections.map((i) => html`<tr key=${i.inspection_id}>
          <td class="mono">${i.facility_id}</td><td>${i.detail}</td>
          <td><span class="cand" style=${{ background: FINAL_COLOR[i.final] }}>${i.final}</span></td>
          <td>${i.frontal ? '예' : '아니요'}</td><td>${i.pose_retry}</td><td>${i.gauge_state || '—'}</td>
          <td>${(i.photos || []).map((k) => html`<a key=${k} href=${'/media/' + k} target="_blank"><img class="thumb" src=${'/media/' + k} /></a>`)}</td></tr>`)}
        </tbody></table>` : html`<p class="no">판정 없음</p>`}</div>
    </div>`}
    ${detail && html`<${TaskDetail} taskId=${detail} m=${sameMap ? m : null} onClose=${() => setDetail(null)} onAuthLost=${onAuthLost} />`}
  </main>`;
}

// ---------------------------------------------------------------- 이력 › task 「자세히」 (3-3차)
//   측정: 거친 칸 · 칸별 AP 측정값 · 이 구간 후보 / 정밀측정: 후보 · 정밀측정 · 결론 / 탐지: 설비 · 판정 · 사진 / 공통: 경로 · 이벤트
const FAC_LABEL = { extinguisher: '소화기', shutter: '셔터', exit_light: '유도등', rack: '랙' };   // 「자세히」 설비 종류 이름
const dur = (a, b) => {
  if (!a) return '—';
  const s = Math.max(0, Math.round(((b ? new Date(b) : new Date()) - new Date(a)) / 1000));
  return s >= 60 ? `${Math.floor(s / 60)}분 ${s % 60}초` : `${s}초`;
};
const trackLen = (tr) => tr.slice(1).reduce((s, p, i) => s + Math.hypot(p.x - tr[i].x, p.y - tr[i].y), 0).toFixed(1);
const dbm = (v, thr) => (v === null || v === undefined ? '—'
  : html`<span class="dbm" style=${{ color: thr === undefined ? 'inherit' : rssiColor(v, thr) }}>${fmtNum(Math.round(v * 10) / 10)}</span>`);

function TaskDetail({ taskId, m, onClose, onAuthLost }) {
  const [d, setD] = useState(null);
  const [msg, setMsg] = useState('');
  const [big, setBig] = useState(null);              // 크게 보는 사진 object_key
  useEffect(() => {
    let alive = true, timer = null;
    const load = () => api(`/api/tasks/${encodeURIComponent(taskId)}`)
      .then((x) => {
        if (!alive) return;
        setD(x);
        if (['ACTIVE', 'PENDING'].includes(x.task.status)) timer = setTimeout(load, 2000);   // 진행 중이면 계속 갱신
      })
      .catch((e) => (e.status === 401 ? onAuthLost() : setMsg(e.message)));
    load();
    return () => { alive = false; clearTimeout(timer); };
  }, [taskId]);
  useEffect(() => {                                  // Esc — 큰 사진이 열려 있으면 사진만, 아니면 창을 닫음
    const esc = (e) => { if (e.key === 'Escape') (big ? setBig(null) : onClose()); };
    addEventListener('keydown', esc);
    return () => removeEventListener('keydown', esc);
  }, [big]);
  const t = d && d.task;
  const apName = {};
  ((d && d.aps) || []).forEach((a) => { apName[a.ap_id] = a.ssid; });
  const thr = d ? d.thr : -72;
  // 지도 표시 — 이 task의 경로 + 측정 칸(칸별 가장 센 AP 중앙값) + 후보 + 판정
  let overlay = null;
  if (d && m) {
    const heat = {};
    ((d.survey && d.survey.cells) || []).forEach((c) => {
      heat[c.cell_id] = Object.fromEntries(Object.entries(c.by_ap).map(([ap, v]) => [ap, [v.median, v.n]]));
    });
    const cands = d.candidates || (d.candidate ? [d.candidate] : []);
    const ins = (d.inspections || []).map((i) => ({ ...i, facility_id: d.facility.facility_id,
      expected_x: d.facility.expected_x, expected_y: d.facility.expected_y }));
    overlay = { heat, ap: '', apName, thr, path: { pts: t.waypoints || [], color: robotColor(t.robot_id) },
      track: { pts: d.track || [], color: robotColor(t.robot_id) }, candidates: cands, inspections: ins };
  }
  const events = ((d && d.events) || []).map((e) => ({ ...e, robot_id: t.robot_id, task_id: null }));
  // 칸 순서 — 계획(경로점마다 칸) · 실제(지난 칸, 다시 지나면 따로). 계획 밖 칸 · 다시 지난 칸 · 못 지난 계획 칸
  const plan = (d && d.plan_cells) || [], visits = (d && d.visits) || [];
  const planIdx = {};
  plan.forEach((p) => { if (p.cell_id !== null) (planIdx[p.cell_id] = planIdx[p.cell_id] || []).push(p.i); });
  const seen = new Set(), visitTags = visits.map((v) => {
    const tag = !planIdx[v.cell_id] ? 'extra' : seen.has(v.cell_id) ? 'revisit' : '';
    seen.add(v.cell_id);
    return tag;
  });
  const missed = plan.filter((p) => p.cell_id !== null && !seen.has(p.cell_id));
  const pathRange = plan.length && plan[0].path_i !== null ? ` (섹터 경로 ${plan[0].path_i}~${plan[plan.length - 1].path_i}번)` : '';
  const seenAp = new Set(((d && d.survey && d.survey.cells) || []).flatMap((c) => Object.keys(c.by_ap).map(Number)));
  const apIds = ((d && d.aps) || []).map((a) => a.ap_id).filter((a) => seenAp.has(a));   // 이 task에서 잰 AP만 열로
  return html`<div class="backdrop" onClick=${(e) => e.target === e.currentTarget && onClose()}><div class="modal taskd">
    <div class="mh"><b>task ${taskId}</b>
      ${t && html`<span>${TYPE_LABEL[t.type] || t.type} · ${t.robot_id}</span>
        <span class=${'chip ' + (STATUS_CHIP[t.status] || 'off')}>${t.status}</span>`}
      <button class="btn small x" onClick=${onClose}>닫기</button></div>
    ${msg && html`<div class="msg">${msg}</div>`}
    <div class="body">${!d ? html`<p>불러오는 중…</p>` : html`
      <div class="tdtop">
        <div class="tdmap">${m ? html`<${MapView} m=${m} robots=${[]} overlay=${overlay} mini=${true} />`
          : html`<div class="empty">이 run은 현재 지도가 아니라 지도 표시는 생략합니다.</div>`}</div>
        <div class="tdsum">
          <dl class="kv">
            <dt>대상</dt><dd class="mono">${t.target_ref || '—'}</dd>
            <dt>경로점</dt><dd>${(t.waypoints || []).length}점</dd>
            <dt>진행률</dt><dd>${pct(t.progress)}</dd>
            <dt>보낸 시각</dt><dd>${fmtTime(t.assigned_at)}</dd>
            <dt>끝난 시각</dt><dd>${fmtTime(t.ended_at)}</dd>
            <dt>걸린 시간</dt><dd>${dur(t.assigned_at, t.ended_at)}${t.ended_at ? '' : (t.assigned_at ? ' (진행 중)' : '')}</dd>
            ${(d.track || []).length > 0 && html`<dt>지나간 길</dt><dd>${trackLen(d.track)} m · 위치 ${d.track.length}개 (제어 PC status, 지도 실선)</dd>
              <dt>배터리</dt><dd>${Math.round(d.track[0].battery_pct)}% → ${Math.round(d.track[d.track.length - 1].battery_pct)}%</dd>`}
            ${t.reason && html`<dt>사유</dt><dd class="mono">${t.reason}</dd>`}
            ${d.survey && html`<dt>표본</dt><dd>${d.survey.totals.n}개 · 칸 ${d.survey.cells.length}개
              ${d.survey.totals.pose_bad ? ` · 위치 불확실 ${d.survey.totals.pose_bad}` : ''}${d.survey.totals.off_grid ? ` · 격자 밖 ${d.survey.totals.off_grid}` : ''}</dd>
              <dt>후보</dt><dd>${(d.candidates || []).length}개 (칸 중앙값 · 가장 센 AP ${'<'} ${thr} dBm)</dd>`}
          </dl>
          ${d.survey && d.survey.basis === 'none' && html`<p class="no">이 task의 표본이 없습니다.</p>`}
          ${d.survey && d.survey.basis === 'time' && html`<p class="no">이 task의 표본에는 task 기록이 없어(3-3차 이전) 같은 로봇 · run에서 이 task 수행 시간 안의 표본으로 보여 줍니다.</p>`}
          <h4>이 task의 이벤트 <span class="no">${events.length}건</span></h4>
          <${EventList} events=${events} max=${100} />
        </div>
      </div>

      ${(plan.length > 0 || visits.length > 0) && html`<h4>칸 순서 <span class="no">칸 번호(ix,iy)</span></h4>
        <div class="seq"><span class="sl">계획</span><span class="sv">${plan.length ? plan.map((p) => html`<span key=${p.i} class="cc plan"
          title=${`경로점 ${p.i}${p.path_i !== null ? ' · 섹터 경로 ' + p.path_i + '번' : ''}`}><i>${p.i}</i>${p.cell_id === null ? '격자 밖' : `${p.cell_id}(${p.ix},${p.iy})`}</span>`)
          : html`<span class="no">경로점 없음</span>`}</span></div>
        <div class="seq"><span class="sl">실제</span><span class="sv">${visits.length ? visits.map((v, i) => html`<span key=${i} class=${'cc ' + visitTags[i]}
          title=${`${hm(v.enter)} ~ ${hm(v.leave)} · ${d.visits_source === 'samples' ? '표본' : '위치'} ${v.n}개${visitTags[i] === 'extra' ? ' · 계획 밖' : visitTags[i] === 'revisit' ? ' · 다시 지남' : ''}`}>
          <i>${i + 1}</i>${v.cell_id}(${v.ix},${v.iy})</span>`) : html`<span class="no">기록 없음${['ACTIVE', 'PENDING'].includes(t.status) ? ' (아직 시작 전 · 진행 중)' : ''}</span>`}</span></div>
        <p class="no">계획 칸 ${Object.keys(planIdx).length}개${pathRange} 중 지난 칸 ${Object.keys(planIdx).length - new Set(missed.map((p) => p.cell_id)).size}개
          ${missed.length ? html` · <span class=${'cc ' + (d.visits_source === 'samples' ? 'missed' : 'nolog')}>${d.visits_source === 'samples' ? '못 지난 계획 칸' : '위치 기록에 없는 계획 칸(1초 간격이라 짧게 머문 칸은 빠짐)'} ${[...new Set(missed.map((p) => `${p.cell_id}(${p.ix},${p.iy})`))].join(', ')}</span>` : ''}
          ${visitTags.includes('extra') ? html` · <span class="cc extra">계획 밖 ${visitTags.filter((x) => x === 'extra').length}칸</span>` : ''}
          ${visitTags.includes('revisit') ? html` · <span class="cc revisit">다시 지남 ${visitTags.filter((x) => x === 'revisit').length}번</span>` : ''}
          ${' · '}실제 순서는 ${d.visits_source === 'samples' ? '측정 표본(rssi 주기) 위치' : d.visits_source === 'status' ? '제어 PC status 위치(1초마다)' : '—'} 기준 — 짧게 스친 칸은 빠지고, 칸 경계에서는 앞뒤 칸이 섞일 수 있음</p>`}

      ${d.survey && html`<h4>지난 칸 · 칸별 측정값 <span class="no">지난 순서 · 중앙값 dBm (표본 수, 최소 ~ 최대)</span></h4>
        ${d.survey.cells.length ? html`<table class="grid"><thead><tr><th>#</th><th>계획 #</th><th>칸</th><th>섹터</th>
          ${apIds.map((a) => html`<th key=${a}>${apName[a]}</th>`)}${apIds.length > 1 && html`<th>가장 센 AP</th>`}<th>후보</th></tr></thead>
          <tbody>${d.survey.cells.map((c, i) => {
            const vals = Object.values(c.by_ap).map((v) => v.median);
            const best = vals.length ? Math.max(...vals) : null;
            const cand = (d.candidates || []).find((x) => String(x.ref) === String(c.cell_id));
            return html`<tr key=${c.cell_id}><td>${i + 1}</td>
              <td>${planIdx[c.cell_id] ? planIdx[c.cell_id].join(', ') : html`<span class="cc extra">계획 밖</span>`}</td>
              <td class="mono">${c.cell_id} (${c.ix},${c.iy})</td><td>${c.sector}</td>
              ${apIds.map((a) => { const v = c.by_ap[a]; return html`<td key=${a}>${v ? html`${dbm(v.median, thr)} <small class="no">(${v.n}, ${v.min} ~ ${v.max})</small>` : '—'}</td>`; })}
              ${apIds.length > 1 && html`<td>${dbm(best, thr)}</td>`}
              <td>${cand ? html`<span class="cand" style=${{ background: CAND_COLOR[cand.outcome || cand.status] }}>${cand.candidate_id} · ${cand.outcome || cand.status}</span>`
                : best !== null && best < thr ? html`<small class="no">기준 미만 (표본 부족)</small>` : ''}</td></tr>`;
          })}</tbody></table>` : html`<p class="no">위치가 확실한 표본이 없습니다.</p>`}`}

      ${t.type === 'REINSPECTION' && html`<h4>후보 → 정밀측정 → 결론</h4>
        ${d.candidate ? html`<dl class="kv wide">
          <dt>후보</dt><dd class="mono">${d.candidate.candidate_id} · ${d.candidate.source} · ${d.candidate.kind}</dd>
          <dt>칸 · 지점</dt><dd>${d.candidate.ref}${d.candidate.ix !== null && d.candidate.ix !== undefined ? ` (${d.candidate.ix},${d.candidate.iy})` : ''} · (${fmtNum(d.candidate.target_x)}, ${fmtNum(d.candidate.target_y)})</dd>
          <dt>발견</dt><dd class="mono">${d.candidate.detect_task_id} · ${fmtTime(d.candidate.detected_at)} · 점수 ${fmtNum(d.candidate.score)}${d.candidate.source === 'NETWORK' ? ' dBm' : ''}</dd>
          <dt>측정 때 칸 중앙값</dt><dd>${(d.survey_cell || []).length ? d.survey_cell.map((v) => html`<span key=${v.ap_id} class="apv">${apName[v.ap_id]} ${dbm(v.median, thr)} <small class="no">(${v.n})</small></span>`) : '—'}</dd>
          <dt>결론</dt><dd><span class="cand" style=${{ background: CAND_COLOR[d.candidate.outcome || d.candidate.status] }}>${d.candidate.outcome || d.candidate.status}</span>
            ${d.candidate.finding_id ? ` 확정 이상 ${d.candidate.finding_id} (${d.candidate.finding_status})` : ''}${d.candidate.decided_at ? ' · ' + fmtTime(d.candidate.decided_at) : ''}</dd>
        </dl>` : html`<p class="no">후보 정보가 없습니다.</p>`}
        ${(d.precise || []).length ? html`<table class="grid"><thead><tr><th>AP</th><th>측정 시간</th><th>표본</th><th>중앙값</th><th>IQR</th><th>잰 시각</th></tr></thead>
          <tbody>${d.precise.map((p) => html`<tr key=${p.ap_id}><td>${p.ssid}</td><td>${p.duration_s}초</td><td>${p.n_samples}</td>
            <td>${dbm(p.rssi_median, thr)} dBm</td><td>${fmtNum(p.rssi_iqr)} dB</td><td>${fmtTime(p.measured_at)}</td></tr>`)}</tbody></table>`
          : html`<p class="no">정밀측정 기록 없음</p>`}`}

      ${t.type === 'FACILITY_CHECK' && html`<h4>설비 · 판정 · 사진</h4>
        ${d.facility && html`<dl class="kv wide">
          <dt>설비</dt><dd class="mono">${d.facility.facility_id} · ${FAC_LABEL[d.facility.facility_type] || d.facility.facility_type}${d.facility.has_gauge ? ' · 압력계' : ''}${d.facility.is_active ? '' : ' (비활성)'}</dd>
          <dt>등록 위치</dt><dd>(${fmtNum(d.facility.expected_x)} m, ${fmtNum(d.facility.expected_y)} m) · 관측 자세 (${fmtNum(d.facility.view_pose.x)} m, ${fmtNum(d.facility.view_pose.y)} m, ${fmtNum(d.facility.view_pose.yaw)} rad)</dd>
        </dl>`}
        ${(d.inspections || []).length ? d.inspections.map((i) => html`<div class="insp" key=${i.inspection_id}>
          <dl class="kv wide">
            <dt>판정</dt><dd>${i.detail} · <span class="cand" style=${{ background: FINAL_COLOR[i.final] }}>${i.final}</span></dd>
            <dt>정면 · 보정</dt><dd>${i.frontal ? '정면' : '정면 아님'} · 자세 보정 ${i.pose_retry}회</dd>
            <dt>압력계 · 신뢰도</dt><dd>${i.gauge_state || '—'} · ${i.confidence === null ? '—' : Math.round(i.confidence * 100) + '%'}</dd>
            <dt>bbox · 모델</dt><dd class="mono">${i.bbox ? JSON.stringify(i.bbox) : '—'} · ${i.model_version}</dd>
            <dt>판정 시각</dt><dd>${fmtTime(i.inspected_at)}</dd>
          </dl>
          <div class="photos">${(i.media || []).length ? i.media.map((md) => html`<figure key=${md.object_key}>
            <img src=${'/media/' + md.object_key} onClick=${() => setBig(md.object_key)} title="크게 보기" />
            <figcaption>${md.kind} · ${Math.round(md.size_bytes / 1024)} KB · ${hm(md.captured_at)}</figcaption></figure>`)
            : html`<p class="no">사진 없음</p>`}</div>
        </div>`) : html`<p class="no">판정 기록 없음${['ACTIVE', 'PENDING'].includes(t.status) ? ' (아직 판정 전)' : ''}</p>`}`}
    `}</div>
    ${big && html`<div class="photobig" onClick=${() => setBig(null)}><img src=${'/media/' + big} /><span>${big} — 누르면 닫힘</span></div>`}
  </div></div>`;
}

// ---------------------------------------------------------------- 정보 (master)
//   오른쪽 세로 탭에서 표를 누를 때마다 선택·해제. 선택한 표는 선택한 순서대로 왼쪽에 세로로 쌓임
function MasterPage({ onAuthLost, selected, setSelected }) {
  const [ui] = useUiSocket(onAuthLost);              // AP 활성 · 측정 AP 실시간 (3-3차)
  const apSig = ui ? JSON.stringify(ui.aps || []) : '';
  const [cards, setCards] = useState(null);
  const [err, setErr] = useState('');
  const [advanced, setAdvanced] = useState(false);
  const [open, setOpen] = useState(null);
  const load = useCallback(() => {
    api('/api/master/summary')
      .then((d) => setCards(d.cards))
      .catch((e) => (e.status === 401 ? onAuthLost() : setErr(e.message)));
  }, []);
  useEffect(load, []);
  const turnPage = (t, page) => api(`/api/master/summary/${t}?page=${page}`)
    .then((c) => setCards((cs) => cs.map((x) => (x.table === t ? c : x))))
    .catch((e) => (e.status === 401 ? onAuthLost() : setErr(e.message)));
  const toggle = (t) => setSelected((s) => (s.includes(t) ? s.filter((x) => x !== t) : [...s, t]));
  useEffect(() => {                                  // AP 활성이 바뀌면 AP 패널을 다시 읽음 (지금 쪽 그대로)
    if (!apSig || !cards) return;
    const c = cards.find((x) => x.table === 'aps');
    if (c) turnPage('aps', c.page || 1);
  }, [apSig]);
  const byTable = Object.fromEntries((cards || []).map((c) => [c.table, c]));
  const shown = selected.filter((t) => byTable[t] && (advanced || ['maps','aps','facilities','robots'].includes(t)));

  return html`<main>
    <h2>설정</h2>
    <p class="sub">지도·측정 AP·설비 등록 정보를 관리합니다. 공통 시작 참고값과 ROS 구역은 지도에서 자동으로 읽습니다.</p>
    <label class="settings-advanced"><input type="checkbox" checked=${advanced} onChange=${() => setAdvanced(!advanced)}/> 고급 관리 항목 표시</label>
    ${err && html`<div class="msg">${err}</div>`}
    <div class="info">
      <div class="stack">
        ${cards && shown.length === 0 && html`<div class="panel empty">오른쪽 탭에서 볼 표를 선택하세요.</div>`}
        ${shown.map((t) => {
          const c = byTable[t];
          return html`<section key=${t} class="panel">
            <div class="ttl"><b>${c.title}</b>${advanced && html`<code>${c.table}</code>`}<span class="n">${c.count}개 등록</span>
              <button class="btn" onClick=${() => setOpen(c.table)}>목록</button></div>
            <div class="desc">${c.desc}</div>
            ${advanced && c.items.length
              ? html`<div class=${'kvgrid' + (c.items.some((x) => x[2]) ? ' sub' : '')}>
                  ${c.items.map(([k, v, sub], i) => html`<div key=${i} class="kvi">
                    <span>${k}${sub ? html`<em>${sub}</em>` : ''}</span><span>${v}</span></div>`)}
                </div>`
              : html`<div class="desc">${c.count ? '등록 내용은 목록에서 확인·수정할 수 있습니다.' : '등록된 항목이 없습니다.'}</div>`}
            <${Pager} page=${c.page} pages=${c.pages} onPage=${(p) => turnPage(t, p)} />
          </section>`;
        })}
      </div>
      <nav class="vtabs">
        ${(cards || []).filter((c) => advanced || ['maps','aps','facilities','robots'].includes(c.table)).map((c) => {
          const on = selected.includes(c.table);
          return html`<button key=${c.table} class=${'vtab' + (on ? ' on' : '')} onClick=${() => toggle(c.table)}>
            <span class="chk">${on ? selected.indexOf(c.table) + 1 : ''}</span>
            <span class="vt"><b>${c.title}</b>${advanced && html`<code>${c.table}</code>`}</span>
            <span class="n">${c.count}</span>
          </button>`;
        })}
      </nav>
    </div>
    ${open && html`<${ListModal} table=${open} onAuthLost=${onAuthLost} live=${open === 'aps' ? apSig : ''}
      onClose=${() => { setOpen(null); load(); }} />`}
  </main>`;
}

function ListModal({ table, onClose, onAuthLost, live }) {
  const [data, setData] = useState(null);
  const [msg, setMsg] = useState(null);           // { text, ok }
  const [adding, setAdding] = useState(false);
  const [newRow, setNewRow] = useState({});
  const [editKey, setEditKey] = useState(null);
  const [editRow, setEditRow] = useState({});
  const [mapId, setMapId] = useState(null);       // 지도에 속한 표(격자 칸 · AP · 설비)의 조회 지도
  const [page, setPage] = useState(1);
  const [busy, setBusy] = useState(false);

  const fail = (e) => (e.status === 401 ? onAuthLost() : setMsg({ text: e.message }));
  const load = useCallback((mid, pg = 1) => {
    const q = new URLSearchParams({ page: pg });
    if (mid) q.set('map_id', mid);
    return api(`/api/master/${table}?${q}`).then((d) => {
      setData(d);
      setPage(d.page);
      if (d.maps) setMapId(d.map_id);
    }).catch(fail);
  }, [table]);
  useEffect(() => { load(); }, [load]);
  useEffect(() => {                                  // AP 목록 — 활성 · 측정 AP가 바뀌면 다시 읽음 (입력 중이면 기다림)
    if (live && data && !adding && editKey === null) load(mapId, page);
  }, [live]);
  useEffect(() => {
    const k = (e) => e.key === 'Escape' && onClose();
    addEventListener('keydown', k);
    return () => removeEventListener('keydown', k);
  }, []);

  const run = async (fn, okText, toPage = page) => {
    setBusy(true); setMsg(null);
    try {
      const r = await fn();
      setMsg({ text: typeof okText === 'function' ? okText(r) : okText, ok: true });
      await load(mapId, toPage);
      return true;
    } catch (e) { fail(e); return false; } finally { setBusy(false); }
  };

  if (!data) {
    return html`<div class="backdrop"><div class="modal"><div class="mh"><b>불러오는 중…</b>
      <button class="btn x" onClick=${onClose}>닫기</button></div>
      ${msg && html`<div class="msg">${msg.text}</div>`}</div></div>`;
  }
  const { meta, rows } = data;
  const ops = meta.ops;
  const shown = meta.columns.filter((c) => (c.type !== 'password' || c.edit) && !c.hide);
  const addCols = meta.columns.filter((c) => c.add);
  const pk = meta.pk;
  const byMap = !!data.maps;                       // 지도에 속한 표 — 목록 위 지도 선택
  const locked = byMap && data.running_run;        // 그 지도로 순찰 중 → 추가·수정·삭제 잠금
  const mq = meta.map_scoped && mapId ? `?map_id=${mapId}` : '';

  const startAdd = () => {
    const init = {};
    addCols.forEach((c) => { init[c.name] = c.default ?? (c.type === 'pose' ? { x: '', y: '', yaw: '' } : ''); });
    setNewRow(init); setAdding(true); setEditKey(null);
  };
  const saveAdd = async () => {
    const body = meta.map_scoped ? { ...newRow, map_id: mapId } : newRow;    // 고른 지도에 추가
    if (await run(() => api(`/api/master/${table}`, { json: body }), '추가했습니다', 1)) setAdding(false);   // 최신 순이라 1쪽
  };
  const startEdit = (r) => {
    const init = {};
    meta.columns.filter((c) => c.edit && canEdit(r, c)).forEach((c) => { init[c.name] = c.type === 'password' ? '' : r[c.name]; });
    setEditRow(init); setEditKey(r[pk]); setAdding(false);
  };
  const saveEdit = async () => {
    if (await run(() => api(`/api/master/${table}/${encodeURIComponent(editKey)}${mq}`, { method: 'PATCH', json: editRow }),
      '값을 바꿨습니다')) setEditKey(null);
  };
  const del = (r) => {
    if (!confirm(`${meta.title} 「${r[pk]}」 행을 삭제할까요?`)) return;
    if (table === 'maps' && !confirm('이 지도의 격자 칸 · AP · 설비도 함께 지워집니다. 계속할까요?')) return;
    const path = table === 'maps' ? `/api/master/maps/${r.map_id}` : `/api/master/${table}/${encodeURIComponent(r[pk])}${mq}`;
    run(() => api(path, { method: 'DELETE' }), '삭제했습니다');
  };
  const target = (r) => run(() => api(`/api/aps/${r.ap_id}/target`, { method: 'POST' }),
    `「${r.ssid}」을(를) 측정 AP로 지정했습니다 — 순찰은 이 SSID만 측정합니다`);
  const activate = (r) => run(() => api(`/api/maps/${r.map_id}/activate`, { method: 'POST' }),
    (d) => `「${r.name}」을(를) 현재 지도로 바꿨습니다` + (d.created ? ` — 격자 ${d.created}칸을 만들었습니다` : ` (격자 ${d.cell_count}칸)`));

  const hasAct = ops.some((o) => ['edit', 'delete', 'activate', 'target'].includes(o));
  const canEdit = (r, c) => !r._edit_only || r._edit_only.includes(c.name);   // 수퍼 계정은 비밀번호만

  return html`<div class="backdrop" onMouseDown=${(e) => e.target === e.currentTarget && onClose()}>
    <div class="modal">
      <div class="mh"><b>${meta.title}</b><code>${table}</code>
        <span class="sub" style=${{ margin: 0 }}>${data.total}${table === 'grid_cells' ? '칸' : '행'} · 최신 순</span>
        <button class="btn x" onClick=${onClose}>닫기</button></div>
      <div class="mdesc">${meta.desc}</div>

      <div class="tools">
        ${ops.includes('add') && html`<button class="btn primary" disabled=${adding || busy || locked || (byMap && !mapId)}
          title=${locked ? '순찰 중인 지도는 바꿀 수 없습니다' : ''} onClick=${startAdd}>추가</button>`}
        ${ops.includes('upload') && html`<button class="btn primary" disabled=${adding || busy} onClick=${() => setAdding(true)}>지도 등록</button>`}
        ${byMap && html`
          <label class="sub" style=${{ margin: 0 }}>지도</label>
          <select value=${mapId ?? ''} disabled=${adding || editKey !== null} onChange=${(e) => { setMsg(null); load(e.target.value, 1); }}>
            ${data.maps.length === 0 && html`<option value="">등록된 지도 없음</option>`}
            ${data.maps.map((o) => html`<option key=${o.value} value=${o.value}>${o.label}</option>`)}
          </select>
          ${locked && html`<span class="lockmsg">순찰 run ${data.running_run} 진행 중 — 이 지도는 잠김</span>`}`}
        ${table === 'grid_cells' && html`
          <span class="sp"></span>
          <button class="btn" disabled=${!mapId || busy} onClick=${() => confirm('이 지도의 격자를 지우고 다시 만들까요?') &&
            run(() => api(`/api/maps/${mapId}/grid`, { method: 'POST' }), (d) => `격자 ${d.cell_count}칸을 다시 만들었습니다`)}>격자 다시 생성</button>
          <button class="btn danger" disabled=${!mapId || busy} onClick=${() => confirm('이 지도의 격자를 모두 삭제할까요?') &&
            run(() => api(`/api/maps/${mapId}/grid`, { method: 'DELETE' }), (d) => `격자 ${d.deleted}칸을 삭제했습니다`)}>격자 삭제</button>`}
      </div>

      ${table === 'maps' && adding && html`<${MapUpload} busy=${busy}
        onCancel=${() => setAdding(false)}
        onSubmit=${async (fd) => {
          if (await run(() => api('/api/maps/upload', { method: 'POST', body: fd }),
            (d) => `지도를 등록했습니다 (${d.width} × ${d.height} px). 「현재 지도로」를 누르면 격자가 생성됩니다`)) setAdding(false);
        }} />`}

      ${msg && html`<div class=${'msg' + (msg.ok ? ' ok' : '')}>${msg.text}</div>`}

      <div class="body">
        <table class="grid">
          <thead><tr>
            ${(adding && table !== 'maps' ? meta.columns : shown).map((c) => html`<th key=${c.name}>${c.label}</th>`)}
            ${hasAct && html`<th></th>`}
          </tr></thead>
          <tbody>
            ${adding && table !== 'maps' && html`<tr class="new">
              ${meta.columns.map((c) => html`<td key=${c.name}>
                ${c.add ? html`<${Input} col=${c} value=${newRow[c.name]}
                  onChange=${(k, v) => setNewRow((p) => ({ ...p, [k]: v }))} />` : html`<span class="no">자동</span>`}
                ${c.add && c.help && html`<span class="help">${c.help}</span>`}
              </td>`)}
              <td class="act">
                <button class="btn small primary" disabled=${busy} onClick=${saveAdd}>저장</button>
                <button class="btn small" onClick=${() => setAdding(false)}>취소</button>
              </td>
            </tr>`}
            ${rows.map((r) => {
              const editing = editKey !== null && String(editKey) === String(r[pk]);
              const cols = adding && table !== 'maps' ? meta.columns : shown;
              return html`<tr key=${r[pk]} class=${editing ? 'edit' : ''}>
                ${cols.map((c) => html`<td key=${c.name}>
                  ${editing && c.edit && canEdit(r, c)
                    ? html`<${Input} col=${c} value=${editRow[c.name]} onChange=${(k, v) => setEditRow((p) => ({ ...p, [k]: v }))} />
                      ${c.type === 'password' && html`<span class="help">바꿀 때만 입력</span>`}`
                    : c.type === 'password' ? html`<span class="no">••••</span>`
                    : c.name === 'is_target' ? (r.is_target ? html`<span class="chip ok">측정 AP</span>` : html`<span class="no">—</span>`)
                    : showValue(c, r[c.name])}
                </td>`)}
                ${hasAct && html`<td class="act">
                  ${editing ? html`
                    <button class="btn small primary" disabled=${busy} onClick=${saveEdit}>저장</button>
                    <button class="btn small" onClick=${() => setEditKey(null)}>취소</button>` : html`
                    ${ops.includes('activate') && !r.is_active && html`<button class="btn small" disabled=${busy} onClick=${() => activate(r)}>현재 지도로</button>`}
                    ${ops.includes('target') && !r.is_target && html`<button class="btn small" disabled=${busy || locked || !r.is_active}
                      title=${r.is_active ? '' : '비활성 AP는 고를 수 없습니다 (client가 이 SSID 값을 받을 때 활성)'} onClick=${() => target(r)}>측정 AP로</button>`}
                    ${ops.includes('edit') && html`<button class="btn small" disabled=${busy || adding || locked} onClick=${() => startEdit(r)}>수정</button>`}
                    ${ops.includes('delete') && !r._no_delete && html`<button class="btn small danger" disabled=${busy || locked} onClick=${() => del(r)}>삭제</button>`}`}
                </td>`}
              </tr>`;
            })}
            ${rows.length === 0 && !adding && html`<tr><td colspan=${shown.length + 1} class="no">등록된 행이 없습니다.</td></tr>`}
          </tbody>
        </table>
        <${Pager} page=${page} pages=${data.pages} disabled=${busy || adding || editKey !== null}
          onPage=${(p) => { setMsg(null); load(mapId, p); }} />
      </div>
    </div>
  </div>`;
}

function MapUpload({ busy, onSubmit, onCancel }) {
  const ref = useRef(null);
  const [source, setSource] = useState(null), [message, setMessage] = useState('');
  const [importing, setImporting] = useState(false);
  useEffect(() => { api('/api/reference-map').then(setSource).catch((e) => setMessage(e.message)); }, []);
  const submit = (e) => { e.preventDefault(); onSubmit(new FormData(ref.current)); };
  const importReference = async () => {
    const robotPath = ref.current.elements.robot_map_path.value;
    if (!robotPath.trim()) { setMessage('로봇 PC에서 사용하는 지도 YAML 경로를 입력해 주세요.'); return; }
    setImporting(true);
    try {
      await api('/api/maps/import-reference', { json: { robot_map_path: robotPath } });
      // Close/reload the owning list without a second upload.
      onCancel(); location.reload();
    } catch (e) { setMessage(e.message); } finally { setImporting(false); }
  };
  return html`<div><form class="upload" ref=${ref} onSubmit=${submit}>
    <div class="field"><label>이름</label><input name="name" required placeholder="지도 이름" /></div>
    <div class="field"><label>로봇 PC 지도 경로 (yaml)</label><input name="robot_map_path" required defaultValue=${source?.robot_map_path || ''} placeholder="로봇이 실제 사용하는 YAML 경로" /></div>
    <div class="field"><label>격자 크기 (m)</label><input key=${source?.grid_size_m || 'pending'} name="grid_size_m" type="number" step="any" min="0.1" defaultValue=${source?.grid_size_m || ''} placeholder="측정 간격" required /></div>
    <div class="field"><label>YAML 파일</label><input name="yaml_file" type="file" accept=".yaml,.yml" required /></div>
    <div class="field"><label>PGM 파일</label><input name="pgm_file" type="file" accept=".pgm,.png" required /></div>
    <div style=${{display:'flex',gap:6}}><button class="btn" disabled=${busy || importing}>파일로 등록</button><button class="btn" type="button" onClick=${onCancel}>취소</button></div>
  </form><div class="reference-import">${source?.available && html`<b>공통 기준 지도 ${source.name}</b><span>측정 간격 ${source.grid_size_m} m를 설정에서 읽었습니다. 파일 선택 없이 등록할 수 있습니다.</span><button class="btn primary" disabled=${busy || importing} onClick=${importReference}>기준 지도 읽어 등록</button>`}${message && html`<p class="msg">${message}</p>`}</div></div>`;
}

// ---------------------------------------------------------------- 앱
function App() {
  const [me, setMe] = useState(undefined);        // undefined = 확인 중, null = 로그인 안 됨
  const [page, setPage] = useState(pageFromPath());
  const [infoSel, setInfoSel] = useState(['maps', 'aps', 'facilities']);    // 정보 탭에서 선택한 표 (다른 탭에 다녀와도 유지)
  const go = useCallback((p) => {
    if (location.pathname !== `/server/${p}`) history.pushState({}, '', `/server/${p}`);
    setPage(p);
  }, []);
  useEffect(() => {
    const f = () => setPage(pageFromPath());
    addEventListener('popstate', f);
    return () => removeEventListener('popstate', f);
  }, []);
  useEffect(() => { api('/api/me').then((d) => setMe(d.user)).catch(() => setMe(null)); }, []);
  useEffect(() => {
    if (me === null && page !== 'login') go('login');
    if (me && (page === 'login' || !TABS.some((t) => t.id === page))) go('monitor');
  }, [me, page]);
  const logout = async () => { await api('/api/logout', { method: 'POST' }).catch(() => {}); setMe(null); };
  const onAuthLost = () => setMe(null);

  return html`
    <header class="top">
      <div class="brand">SCV 관제<small>CONTROL</small></div>
      <nav class="tabs">
        ${TABS.map((t) => html`<button key=${t.id} class=${'tab' + (page === t.id ? ' on' : '')}
          disabled=${!me} title=${me ? '' : '로그인 후 사용할 수 있습니다'} onClick=${() => go(t.id)}>${t.label}</button>`)}
      </nav>
      <div class="who">${me
        ? html`<span>${me.username}</span><button class="btn small ghost" onClick=${logout}>로그아웃</button>`
        : html`<span>로그인 필요</span>`}</div>
    </header>
    ${me === undefined ? html`<main><p class="sub">확인 중…</p></main>`
      : !me ? html`<${LoginPage} onLogin=${(u) => setMe(u)} />`
      : page === 'master' ? html`<${MasterPage} onAuthLost=${onAuthLost} selected=${infoSel} setSelected=${setInfoSel} />`
      : page === 'history' ? html`<${HistoryPage} onAuthLost=${onAuthLost} />`
      : html`<${MonitorPage} onAuthLost=${onAuthLost} />`}
  `;
}

createRoot(document.getElementById('root')).render(html`<${App} />`);
