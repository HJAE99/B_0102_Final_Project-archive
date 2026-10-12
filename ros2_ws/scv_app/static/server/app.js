// SCV server page — 3-2차 (login · monitor · 정보)
import { html, createRoot, useState, useEffect, useRef, useCallback } from '/static/vendor/react-htm.mjs';

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
  { id: 'monitor', label: '모니터' },
  { id: 'master', label: '정보' },
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
    case 'pose': {
      const p = value || { x: '', y: '', yaw: '' };
      const part = (k, ph) => html`<input type="number" step="any" placeholder=${ph} value=${p[k] ?? ''}
        onInput=${(e) => set({ ...p, [k]: e.target.value })} />`;
      return html`<div class="pose">${part('x', 'x')}${part('y', 'y')}${part('yaw', 'yaw')}</div>`;
    }
    case 'docks': {                                     // 로봇마다 x · y · yaw (비우면 그 로봇 도크 없음)
      const v = value || {};
      const part = (rid, k) => html`<input type="number" step="any" placeholder=${k} value=${(v[rid] || {})[k] ?? ''}
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
    <p class="sub">로그인하면 모니터 · 정보 탭을 쓸 수 있습니다.</p>
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

function MapView({ m, robots }) {
  const ref = useRef(null);
  const mapRef = useRef(null);
  const robotLayerRef = useRef(null);
  useEffect(() => {
    const L = window.L;
    const map = L.map(ref.current, {
      crs: L.CRS.Simple, minZoom: -3, maxZoom: 9, zoomSnap: 0.25, attributionControl: false,
    });
    // map 좌표(m): 가로 x = lng, 세로 y = lat. 이미지 왼쪽 아래 = origin
    const w = m.width * m.resolution_m;
    const h = m.height * m.resolution_m;
    const bounds = [[m.origin_y, m.origin_x], [m.origin_y + h, m.origin_x + w]];
    L.imageOverlay(m.image_url, bounds).addTo(map);

    // 격자 칸 (grid_cells) — 칸 범위 = origin + ix·g ~ origin + (ix+1)·g
    const g = m.grid_size_m;
    const renderer = L.canvas({ padding: 0.5 });
    L.layerGroup((m.cells || []).map(([id, ix, iy, sector]) => {
      const x0 = m.origin_x + ix * g, y0 = m.origin_y + iy * g;
      const col = SECTOR_COLOR[sector] || '#0b7a84';
      return L.rectangle([[y0, x0], [y0 + g, x0 + g]], {
        renderer, weight: 1, color: col, opacity: 0.6, fillColor: col, fillOpacity: 0.12,
        interactive: false,
      });
    })).addTo(map);
    // 격자 번호 — 칸이 화면에서 작으면 숨김, 크면 ix,iy도 함께
    const labelLayer = L.layerGroup((m.cells || []).map(([id, ix, iy]) => {
      const cx = m.origin_x + (ix + 0.5) * g, cy = m.origin_y + (iy + 0.5) * g;
      return L.marker([cy, cx], {
        interactive: false, keyboard: false,
        icon: L.divIcon({ className: 'cell-label', iconSize: null,
          html: `<b>${id}</b><small>${ix},${iy}</small>` }),
      });
    }));
    const container = ref.current;
    const updateLabels = () => {
      const px = g * Math.pow(2, map.getZoom());       // CRS.Simple: 1 m = 2^zoom px
      const show = px >= 26;
      if (show && !map.hasLayer(labelLayer)) labelLayer.addTo(map);
      if (!show && map.hasLayer(labelLayer)) map.removeLayer(labelLayer);
      container.classList.toggle('cell-xy', px >= 46);
    };
    map.on('zoomend', updateLabels);
    map.fitBounds(bounds);
    updateLabels();
    // 지도 위 등록 정보 (3차) — 도크(maps.dock_poses) · AP 설치 위치(aps.x, y) · 점검 설비(facilities)
    const info = L.layerGroup().addTo(map);
    const label = (txt, cls) => ({ permanent: true, direction: 'right', offset: [8, 0], className: 'map-tip ' + cls });
    Object.entries(m.dock_poses || {}).forEach(([rid, p]) => {
      const c = robotColor(rid), d = 0.22;
      L.rectangle([[p.y - d, p.x - d], [p.y + d, p.x + d]], { color: c, weight: 2, fillColor: c, fillOpacity: 0.25 })
        .bindTooltip(`${rid} 도크`, label('', 'dock')).addTo(info);
      L.polyline([[p.y, p.x], [p.y + Math.sin(p.yaw || 0) * 0.4, p.x + Math.cos(p.yaw || 0) * 0.4]],
        { color: c, weight: 2, dashArray: '3 3', interactive: false }).addTo(info);
    });
    (m.aps || []).forEach((a) => {
      L.marker([a.y, a.x], { icon: L.divIcon({ className: 'ap-icon', iconSize: [18, 18], html: '<b>📶</b>' }) })
        .bindTooltip(`${a.ssid}${a.is_active ? '' : ' (비활성)'}`, label('', 'ap')).addTo(info);
    });
    const FAC = { extinguisher: ['소', '#dc2626'], shutter: ['셔', '#7c3aed'], exit_light: ['유', '#16a34a'] };
    (m.facilities || []).forEach((f) => {
      const [ch, col] = FAC[f.facility_type] || ['?', '#555'];
      const c = f.is_active ? col : '#9aa3aa', v = f.view_pose || {};
      if (v.x !== undefined) {                           // 관측 자세 → 설비 (점선)
        L.polyline([[v.y, v.x], [f.expected_y, f.expected_x]], { color: c, weight: 1.5, dashArray: '2 3', interactive: false }).addTo(info);
        L.circleMarker([v.y, v.x], { radius: 3, color: c, weight: 1, fillColor: c, fillOpacity: 0.6, interactive: false }).addTo(info);
      }
      L.marker([f.expected_y, f.expected_x], { icon: L.divIcon({ className: 'fac-icon', iconSize: [20, 20],
        html: `<b style="background:${c}">${ch}</b>` }) })
        .bindTooltip(`${f.facility_id}${f.has_gauge ? ' · 압력계' : ''}${f.is_active ? '' : ' (비활성)'}`, label('', 'fac')).addTo(info);
    });
    robotLayerRef.current = L.layerGroup().addTo(map);  // 로봇 표시는 격자·번호 위
    markersRef.current = {};
    mapRef.current = map;
    const onResize = () => map.invalidateSize();
    addEventListener('resize', onResize);
    return () => { removeEventListener('resize', onResize); map.remove(); mapRef.current = null; };
  }, [m.map_id, m.image_url]);

  // 로봇 위치 — /ws/ui로 1초마다 받은 값으로 마커를 옮김 (지도·마커는 다시 만들지 않음)
  const markersRef = useRef({});
  useEffect(() => {
    const L = window.L, layer = robotLayerRef.current, mk = markersRef.current;
    if (!layer) return;
    const seen = new Set();
    (robots || []).forEach((r) => {
      const st = r.state;
      if (!st || st.x === null || st.y === null) return;
      seen.add(r.robot_id);
      const live = r.connected;
      const color = live ? robotColor(r.robot_id) : '#8a969e';
      const yaw = st.yaw || 0, len = 0.45;
      const head = [[st.y, st.x], [st.y + Math.sin(yaw) * len, st.x + Math.cos(yaw) * len]];
      const tip = `${r.robot_id}${live ? '' : ' (연결 끊김)'}`;
      let m = mk[r.robot_id];
      if (!m || !layer.hasLayer(m.dot)) {
        m = mk[r.robot_id] = {
          line: L.polyline(head, { color, weight: 3, interactive: false }).addTo(layer),
          dot: L.circleMarker([st.y, st.x], { radius: 9, color: '#fff', weight: 2, fillColor: color, fillOpacity: 0.95 })
            .bindTooltip(tip, { permanent: true, direction: 'top', offset: [0, -8], className: 'robot-tip' }).addTo(layer),
        };
      } else {
        m.line.setLatLngs(head).setStyle({ color });
        m.dot.setLatLng([st.y, st.x]).setStyle({ fillColor: color, fillOpacity: live ? 0.95 : 0.6 });
        m.dot.setTooltipContent(tip);
      }
    });
    Object.keys(mk).forEach((rid) => {                 // 상태가 없어진 로봇은 지움
      if (!seen.has(rid)) { layer.removeLayer(mk[rid].line); layer.removeLayer(mk[rid].dot); delete mk[rid]; }
    });
  }, [robots]);
  return html`<div id="map" ref=${ref}></div>`;
}

// /ws/ui — 로봇 상태 1초마다. 끊기면 2초 뒤 다시 접속, 로그인이 풀렸으면(4401) 로그인 화면으로
function useUiSocket(onAuthLost) {
  const [data, setData] = useState(null);
  const [linked, setLinked] = useState(false);
  useEffect(() => {
    let ws, timer, alive = true;
    const open = () => {
      ws = new WebSocket(`${location.protocol === 'https:' ? 'wss' : 'ws'}://${location.host}/ws/ui`);
      ws.onopen = () => setLinked(true);
      ws.onmessage = (e) => { try { const d = JSON.parse(e.data); if (d.type === 'robots') setData(d); } catch { /* 무시 */ } };
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
  return [data, linked];
}

const ago = (iso) => {
  if (!iso) return '—';
  const s = Math.max(0, (Date.now() - new Date(iso).getTime()) / 1000);
  return s < 60 ? `${s.toFixed(0)}초 전` : `${(s / 60).toFixed(0)}분 전`;
};

function RobotPanel({ data, linked }) {
  const robots = data ? data.robots : [];
  return html`<div class="rpanel">
    <h3>로봇 상태 <span class=${'live ' + (linked ? 'on' : '')}>${linked ? '실시간' : '연결 중…'}</span></h3>
    ${robots.length === 0 && html`<p>등록된 로봇이 없습니다.</p>`}
    ${robots.map((r) => {
      const [k, label] = robotPhase(r), st = r.state;
      return html`<div key=${r.robot_id} class="rrow">
        <div class="rh"><i style=${{ background: robotColor(r.robot_id) }}></i><b>${r.robot_id}</b>
          <span class=${'chip ' + k}>${label}</span></div>
        ${st ? html`<div class="rkv">
            <span>위치</span><span class="num">${fmtNum(st.x)}, ${fmtNum(st.y)} · ${st.yaw === null ? '—' : (st.yaw * 180 / Math.PI).toFixed(0) + '°'}</span>
            <span>배터리</span><span class="num">${st.battery_pct === null ? '—' : st.battery_pct.toFixed(1) + '%'}</span>
            <span>상태</span><span>${st.nav_state || '—'}${st.is_docked ? ' · 도킹' : ''}${st.estop ? ' · 정지' : ''}</span>
            <span>작업</span><span>${st.run_id ? `run ${st.run_id}` : '—'}${st.task_id ? ` · ${st.task_id}` : ''}</span>
            <span>마지막 보고</span><span>${ago(st.ts)}</span>
          </div>
          ${r.is_active && st.raw && html`<details class="raw" open>
            <summary>관제 PC 메모리 상태 <code>${r.robot_id}</code> 전체 · E3까지 ${st.ttl_s}초</summary>
            <div class="rkv mono">${Object.entries(st.raw).map(([k, v]) => html`<span key=${k}>${k}</span><span>${v === '' ? '—' : v}</span>`)}</div>
          </details>`}`
          : html`<p>${r.last_e3 ? `${fmtTime(r.last_e3)} 응답 없음(E3) 기록` : '받은 상태 없음'}</p>`}
      </div>`;
    })}
    ${data && html`<p class="foot">${data.timeout_s}초 동안 상태가 오지 않으면 응답 없음(E3)으로 기록</p>`}
  </div>`;
}

function MonitorPage({ onAuthLost }) {
  const [state, setState] = useState({ loading: true });
  const [ui, linked] = useUiSocket(onAuthLost);
  useEffect(() => {
    api('/api/maps/active')
      .then((d) => setState({ map: d.map }))
      .catch((e) => (e.status === 401 ? onAuthLost() : setState({ error: e.message })));
  }, []);
  const m = state.map;
  return html`<main>
    <h2>모니터</h2>
    <p class="sub">현재 지도(정보 › 지도에서 「현재 지도로」 선택한 지도)와 로봇 위치를 표시합니다.</p>
    <div class="mapbox">
      ${m && html`<div class="head">
        <b>${m.name}</b>
        <span>해상도 ${fmtNum(m.resolution_m)} m/px</span>
        <span>크기 ${fmtNum(m.width * m.resolution_m)} × ${fmtNum(m.height * m.resolution_m)} m</span>
        <span>원점 (${fmtNum(m.origin_x)}, ${fmtNum(m.origin_y)})</span>
        <span>격자 ${fmtNum(m.grid_size_m)} m · ${m.cell_count}칸</span>
        <span class="legend">
          ${(m.sectors || []).filter((x) => x.cells).map((x) => html`<span key=${x.sector} class="lg">
            <i style=${{ borderColor: SECTOR_COLOR[x.sector], background: SECTOR_COLOR[x.sector] + '26' }}></i>
            섹터 ${x.sector} · ${x.cells}칸${x.robot ? ` · 기본 ${x.robot}` : ''}</span>`)}
          <span class="lg"><i class="sq"></i>도크 ${Object.keys(m.dock_poses || {}).length}</span>
          <span class="lg">📶 AP ${(m.aps || []).length}</span>
          <span class="lg"><b class="fac-dot" style=${{ background: '#dc2626' }}>소</b><b class="fac-dot" style=${{ background: '#7c3aed' }}>셔</b><b class="fac-dot" style=${{ background: '#16a34a' }}>유</b>
            설비 ${(m.facilities || []).length} (점선 끝 = 관측 자세)</span>
          <span class="lg">큰 숫자 = 칸 번호(cell_id) · 작은 숫자 = ix,iy (칸이 작으면 숨김)</span>
          ${m.running_run && html`<span class="lg run">순찰 run ${m.running_run} 진행 중 — 격자 변경 · 현재 지도 바꾸기 잠김</span>`}
        </span>
      </div>`}
      ${state.loading ? html`<div class="empty">불러오는 중…</div>`
        : state.error ? html`<div class="empty">${state.error}</div>`
        : !m ? html`<div class="empty">현재 지도가 없습니다. 정보 › 지도에서 지도를 등록하고 「현재 지도로」를 누르세요.</div>`
        : !m.width ? html`<div class="empty">지도 이미지 파일이 없습니다 (${m.image_key})</div>`
        : html`<${MapView} m=${m} robots=${ui ? ui.robots : []} />`}
    </div>
    <div class="later">
      <${RobotPanel} data=${ui} linked=${linked} />
      <div><h3>순찰 진행</h3><p>추후 구현 — run · task 진행률</p></div>
      <div><h3>이벤트</h3><p>추후 구현 — 로봇 사건 · 이상 후보 · 확정</p></div>
    </div>
  </main>`;
}

// ---------------------------------------------------------------- 정보 (master)
//   오른쪽 세로 탭에서 표를 누를 때마다 선택·해제. 선택한 표는 선택한 순서대로 왼쪽에 세로로 쌓임
function MasterPage({ onAuthLost, selected, setSelected }) {
  const [cards, setCards] = useState(null);
  const [err, setErr] = useState('');
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
  const byTable = Object.fromEntries((cards || []).map((c) => [c.table, c]));
  const shown = selected.filter((t) => byTable[t]);

  return html`<main>
    <h2>정보</h2>
    <p class="sub">오른쪽 탭에서 표를 선택하면 아래로 쌓여 보입니다(다시 누르면 해제). 「목록」에서 행을 추가·삭제하고 값을 바꿉니다.</p>
    ${err && html`<div class="msg">${err}</div>`}
    <div class="info">
      <div class="stack">
        ${cards && shown.length === 0 && html`<div class="panel empty">오른쪽 탭에서 볼 표를 선택하세요.</div>`}
        ${shown.map((t) => {
          const c = byTable[t];
          return html`<section key=${t} class="panel">
            <div class="ttl"><b>${c.title}</b><code>${c.table}</code><span class="n">${c.count}행</span>
              <button class="btn" onClick=${() => setOpen(c.table)}>목록</button></div>
            <div class="desc">${c.desc}</div>
            ${c.items.length
              ? html`<div class=${'kvgrid' + (c.items.some((x) => x[2]) ? ' sub' : '')}>
                  ${c.items.map(([k, v, sub], i) => html`<div key=${i} class="kvi">
                    <span>${k}${sub ? html`<em>${sub}</em>` : ''}</span><span>${v}</span></div>`)}
                </div>`
              : html`<div class="desc">등록된 행이 없습니다.</div>`}
            <${Pager} page=${c.page} pages=${c.pages} onPage=${(p) => turnPage(t, p)} />
          </section>`;
        })}
      </div>
      <nav class="vtabs">
        ${(cards || []).map((c) => {
          const on = selected.includes(c.table);
          return html`<button key=${c.table} class=${'vtab' + (on ? ' on' : '')} onClick=${() => toggle(c.table)}>
            <span class="chk">${on ? selected.indexOf(c.table) + 1 : ''}</span>
            <span class="vt"><b>${c.title}</b><code>${c.table}</code></span>
            <span class="n">${c.count}</span>
          </button>`;
        })}
      </nav>
    </div>
    ${open && html`<${ListModal} table=${open} onAuthLost=${onAuthLost}
      onClose=${() => { setOpen(null); load(); }} />`}
  </main>`;
}

function ListModal({ table, onClose, onAuthLost }) {
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
  const activate = (r) => run(() => api(`/api/maps/${r.map_id}/activate`, { method: 'POST' }),
    (d) => `「${r.name}」을(를) 현재 지도로 바꿨습니다` + (d.created ? ` — 격자 ${d.created}칸을 만들었습니다` : ` (격자 ${d.cell_count}칸)`));

  const hasAct = ops.some((o) => ['edit', 'delete', 'activate'].includes(o));
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
                    : c.type === 'password' ? html`<span class="no">••••</span>` : showValue(c, r[c.name])}
                </td>`)}
                ${hasAct && html`<td class="act">
                  ${editing ? html`
                    <button class="btn small primary" disabled=${busy} onClick=${saveEdit}>저장</button>
                    <button class="btn small" onClick=${() => setEditKey(null)}>취소</button>` : html`
                    ${ops.includes('activate') && !r.is_active && html`<button class="btn small" disabled=${busy} onClick=${() => activate(r)}>현재 지도로</button>`}
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
  const submit = (e) => { e.preventDefault(); onSubmit(new FormData(ref.current)); };
  return html`<form class="upload" ref=${ref} onSubmit=${submit}>
    <div class="field"><label>이름</label><input name="name" required placeholder="testbed_20261010" /></div>
    <div class="field"><label>제어 PC 지도 경로 (yaml)</label>
      <input name="robot_map_path" required placeholder="/opt/scv/maps/testbed_20261010.yaml" /></div>
    <div class="field"><label>격자 크기 (m)</label><input name="grid_size_m" type="number" step="any" min="0.1" defaultValue="1.0" /></div>
    <div class="field"><label>yaml 파일</label><input name="yaml_file" type="file" accept=".yaml,.yml" required /></div>
    <div class="field"><label>pgm 파일</label><input name="pgm_file" type="file" accept=".pgm,.png" required /></div>
    <div style=${{ display: 'flex', gap: '6px' }}>
      <button class="btn primary" disabled=${busy}>업로드</button>
      <button class="btn" type="button" onClick=${onCancel}>취소</button>
    </div>
  </form>`;
}

// ---------------------------------------------------------------- 앱
function App() {
  const [me, setMe] = useState(undefined);        // undefined = 확인 중, null = 로그인 안 됨
  const [page, setPage] = useState(pageFromPath());
  const [infoSel, setInfoSel] = useState([]);    // 정보 탭에서 선택한 표 (다른 탭에 다녀와도 유지)
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
      <div class="brand">SCV 관제<small>server page</small></div>
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
      : html`<${MonitorPage} onAuthLost=${onAuthLost} />`}
  `;
}

createRoot(document.getElementById('root')).render(html`<${App} />`);
