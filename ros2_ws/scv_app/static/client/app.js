// SCV client page — 3-2차 (login · 로봇 정보 · 카메라)
import { html, createRoot, useState, useEffect, useCallback } from '/static/vendor/react-htm.mjs';

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
    const err = new Error(typeof d === 'string' ? d : `오류 (${r.status})`);
    err.status = r.status;
    throw err;
  }
  return data;
}

const TABS = [
  { id: 'robot', label: '로봇 정보' },
  { id: 'camera', label: '카메라' },
];
const pageFromPath = () => (location.pathname.match(/^\/client\/(\w+)/) || [])[1] || 'login';

// 주기 조회 — 401이면 로그인 화면으로
function usePoll(path, ms, onAuthLost) {
  const [data, setData] = useState(null);
  const [err, setErr] = useState('');
  useEffect(() => {
    let alive = true;
    const f = () => api(path).then((d) => { if (alive) { setData(d); setErr(''); } })
      .catch((e) => { if (!alive) return; e.status === 401 ? onAuthLost() : setErr(e.message); });
    f();
    const id = setInterval(f, ms);
    return () => { alive = false; clearInterval(id); };
  }, [path]);
  return [data, err];
}

const fmtTime = (sec) => {
  if (!sec) return '—';
  const d = new Date(sec * 1000);
  const p = (n) => String(n).padStart(2, '0');
  return `${p(d.getHours())}:${p(d.getMinutes())}:${p(d.getSeconds())}`;
};
const num = (v, unit = '') => (v === null || v === undefined ? '—' : `${v}${unit}`);

// ---------------------------------------------------------------- 로그인
function LoginPage({ robotId, onLogin }) {
  const [username, setU] = useState('');
  const [password, setP] = useState('');
  const [msg, setMsg] = useState('');
  const [busy, setBusy] = useState(false);
  const submit = async (e) => {
    e.preventDefault();
    setBusy(true); setMsg('');
    try { onLogin(await api('/api/login', { json: { username, password } })); } catch (err) { setMsg(err.message); }
    setBusy(false);
  };
  return html`<form class="login" onSubmit=${submit}>
    <h2>client page 로그인</h2>
    <p class="sub">${robotId}의 제어 PC입니다. 관제 PC에 등록된 사용자 계정으로 로그인하세요.</p>
    <div class="field"><label>아이디</label>
      <input autofocus value=${username} onInput=${(e) => setU(e.target.value)} autocomplete="username" /></div>
    <div class="field"><label>비밀번호</label>
      <input type="password" value=${password} onInput=${(e) => setP(e.target.value)} autocomplete="current-password" /></div>
    <button class="btn primary" disabled=${busy}>로그인</button>
    ${msg && html`<div class="msg">${msg}</div>`}
  </form>`;
}

// ---------------------------------------------------------------- 상태 표시줄 (로봇 활성 · 관제 PC 연결)
function StatusBar({ d }) {
  if (!d) return null;
  const r = d.ros, l = d.link;
  const rosChip = r.active ? ['ok', '로봇 활성'] : r.checking ? ['wait', '로봇 확인 중'] : ['off', '로봇 비활성'];
  const linkChip = { connected: ['ok', '관제 PC 연결됨'], connecting: ['wait', '관제 PC 접속 중'],
    retry: ['bad', '관제 PC 재접속 대기'], waiting: ['off', '관제 PC 보고 대기'] }[l.state] || ['off', l.state];
  return html`<div class="statusbar">
    <div class="st">
      <span class=${'chip ' + rosChip[0]}>${rosChip[1]}</span>
      <span class="stx">${r.active
        ? `${fmtTime(r.active_at)}에 확인 — /${d.robot_id}/… 발행 토픽 ${r.found_topics.length}개`
        : r.checking && r.off_at ? `${fmtTime(r.off_at)}에 로봇 값이 ${r.stale_s}초 넘게 끊겨 비활성 — 토픽 목록을 ${r.check_s}초마다 다시 확인 중`
        : r.checking ? `토픽 목록을 ${r.check_s}초마다 확인 · 마지막 ${fmtTime(r.checked_at)} — 아직 /${d.robot_id}/… 발행자 없음`
        : '로그인하면 확인을 시작합니다'}</span>
    </div>
    <div class="st">
      <span class=${'chip ' + linkChip[0]}>${linkChip[1]}</span>
      <span class="stx">${l.server} · ${l.detail}${l.hello_at ? ` · hello ${fmtTime(l.hello_at)}` : ''}</span>
    </div>
  </div>`;
}

// ---------------------------------------------------------------- 로봇 정보
function InfoCard({ title, item, children, note }) {
  const state = !item ? 'none' : item.stale ? 'stale' : 'live';
  return html`<section class=${'icard ' + state}>
    <div class="ih"><b>${title}</b>
      <span class=${'dot ' + state}></span>
      <span class="age">${!item ? '수신 없음' : item.stale ? `${item.age_s}초 전 (끊김)` : `${item.age_s}초 전`}</span></div>
    <code class="tp">${item ? item.topic : note}</code>
    <div class="ib">${item ? children(item.data) : html`<p class="sub" style=${{ margin: 0 }}>토픽 값을 기다리는 중</p>`}</div>
  </section>`;
}

const Row = ({ k, v }) => html`<div class="ir"><span>${k}</span><span>${v}</span></div>`;

function RobotPage({ onAuthLost }) {
  const [d, err] = usePoll('/api/robot', 1000, onAuthLost);
  const ns = d ? `/${d.robot_id}` : '';
  const info = d ? d.info : {};
  return html`<main>
    <h2>로봇 정보 ${d && html`<small class="rid">${d.robot_id}</small>`}</h2>
    <p class="sub">로봇의 기본 정보 토픽을 구독해 1초마다 보여 줍니다.</p>
    ${err && html`<div class="msg">${err}</div>`}
    <${StatusBar} d=${d} />
    <div class="icards">
      <${InfoCard} title="배터리" item=${info.battery} note=${`${ns}/battery_state`}>${(v) => html`
        <div class="big">${num(v.percentage, '%')}</div>
        <div class="bar"><i style=${{ width: `${Math.max(0, Math.min(100, v.percentage || 0))}%` }}></i></div>
        <${Row} k="상태" v=${v.status} />
        <${Row} k="전압" v=${num(v.voltage, ' V')} />
        <${Row} k="전류" v=${num(v.current, ' A')} />
        <${Row} k="온도" v=${num(v.temperature, ' ℃')} />`}<//>
      <${InfoCard} title="도킹" item=${info.dock}
        note=${d && !d.dock_supported ? 'irobot_create_msgs 없음 — 도킹 정보 생략' : `${ns}/dock_status`}>${(v) => html`
        <div class="big">${v.is_docked ? '도킹됨' : '도킹 아님'}</div>
        <${Row} k="도크 보임" v=${v.dock_visible ? '예' : '아니요'} />`}<//>
      <${InfoCard} title="지도 위치 (AMCL)" item=${info.pose} note=${`${ns}/amcl_pose`}>${(v) => html`
        <div class="big">${v.x}, ${v.y}<small> m</small></div>
        <${Row} k="방향" v=${num(v.yaw_deg, '°')} />
        <${Row} k="좌표계" v=${v.frame || '—'} />
        <${Row} k="위치 불확실성 σ" v=${num(v.sigma_xy, ' m')} />
        <p class="hint">관제 PC로 보내는 status의 위치</p>`}<//>
      <${DockCard} d=${d} />
      <${InfoCard} title="위치 (오도메트리)" item=${info.odom} note=${`${ns}/odom`}>${(v) => html`
        <div class="big">${v.x}, ${v.y}<small> m</small></div>
        <${Row} k="방향" v=${num(v.yaw_deg, '°')} />
        <${Row} k="선속도" v=${num(v.linear, ' m/s')} />
        <${Row} k="각속도" v=${num(v.angular, ' rad/s')} />
        <p class="hint">odom 좌표 — 지도(map) 좌표가 아님</p>`}<//>
      <${InfoCard} title="네트워크" item=${info.ip} note=${`${ns}/ip`}>${(v) => html`
        <div class="big mono">${v.ip}</div>`}<//>
    </div>
  </main>`;
}

// 관제 PC config — 현재 지도 · 이 로봇의 도크 접근 자세 → initialpose (3-2차)
function DockCard({ d }) {
  if (!d) return null;
  const c = d.link.config, ip = d.initialpose;
  const dock = c && c.dock_pose;
  return html`<section class=${'icard ' + (c ? 'live' : 'none')}>
    <div class="ih"><b>도크 접근 자세 (관제 PC)</b><span class=${'dot ' + (c ? 'live' : 'none')}></span>
      <span class="age">${c ? `${fmtTime(d.link.config_at)} 받음` : '받은 config 없음'}</span></div>
    <code class="tp">WS /ws/agent config → /${d.robot_id}/initialpose</code>
    <div class="ib">${c ? html`
      <div class="big">${dock ? html`${dock.x}, ${dock.y}<small> m</small>` : '등록 안 됨'}</div>
      <${Row} k="방향" v=${dock ? `${(dock.yaw * 180 / Math.PI).toFixed(1)}°` : '—'} />
      <${Row} k="현재 지도" v=${c.map ? c.map.name : '없음'} />
      <${Row} k="initialpose" v=${ip ? `${fmtTime(ip.at)} 보냄` : '보내지 않음'} />
      ${ip && html`<p class="hint">${ip.reason}</p>`}
      ${!dock && html`<p class="hint">server page › 정보 › 지도 › 수정에서 이 로봇의 도크 접근 자세를 넣으세요</p>`}`
      : html`<p class="sub" style=${{ margin: 0 }}>관제 PC 연결 뒤 받음</p>`}</div>
  </section>`;
}

// ---------------------------------------------------------------- 카메라
function CameraPage({ onAuthLost }) {
  const [c, err] = usePoll('/api/camera', 1000, onAuthLost);
  const [key] = useState(() => Date.now());           // 탭을 열 때마다 새 스트림
  return html`<main>
    <h2>카메라</h2>
    <p class="sub">이 탭을 보는 동안만 카메라 토픽을 구독합니다(다른 탭으로 가면 구독 해제).</p>
    ${err && html`<div class="msg">${err}</div>`}
    <div class="cambox">
      <div class="head">
        <code>${c ? c.topic : '…'}</code>
        ${c && html`<span>${c.fps ? `${c.fps} fps` : '영상 없음'}</span>
          <span>${c.format || '—'}</span>
          <span>${c.size ? `${Math.round(c.size / 1024)} KB/장` : ''}</span>
          <span>받은 장 수 ${c.frames}</span>`}
      </div>
      <div class="screen">
        <img src=${`/api/camera/stream?k=${key}`} alt="" />
        ${c && !c.fps && html`<div class="waiting">${c.frames ? '영상이 끊겼습니다' : '영상을 기다리는 중'} — ${c.topic}</div>`}
      </div>
    </div>
  </main>`;
}

// ---------------------------------------------------------------- 앱
function App() {
  const [me, setMe] = useState(undefined);
  const [robotId, setRobotId] = useState('');
  const [page, setPage] = useState(pageFromPath());
  const go = useCallback((p) => {
    if (location.pathname !== `/client/${p}`) history.pushState({}, '', `/client/${p}`);
    setPage(p);
  }, []);
  useEffect(() => {
    const f = () => setPage(pageFromPath());
    addEventListener('popstate', f);
    return () => removeEventListener('popstate', f);
  }, []);
  useEffect(() => { api('/api/me').then((d) => { setRobotId(d.robot_id); setMe(d.user); }).catch(() => setMe(null)); }, []);
  useEffect(() => {
    if (me === null && page !== 'login') go('login');
    if (me && (page === 'login' || !TABS.some((t) => t.id === page))) go('robot');
  }, [me, page]);
  const logout = async () => { await api('/api/logout', { method: 'POST' }).catch(() => {}); setMe(null); };
  const onAuthLost = () => setMe(null);

  return html`
    <header class="top client">
      <div class="brand">SCV 제어 PC<small>client page${robotId ? ` · ${robotId}` : ''}</small></div>
      <nav class="tabs">
        ${TABS.map((t) => html`<button key=${t.id} class=${'tab' + (page === t.id ? ' on' : '')}
          disabled=${!me} title=${me ? '' : '로그인 후 사용할 수 있습니다'} onClick=${() => go(t.id)}>${t.label}</button>`)}
      </nav>
      <div class="who">${me
        ? html`<span>${me.username}${me.role === 'admin' ? ' (admin)' : ''}</span><button class="btn small ghost" onClick=${logout}>로그아웃</button>`
        : html`<span>로그인 필요</span>`}</div>
    </header>
    ${me === undefined ? html`<main><p class="sub">확인 중…</p></main>`
      : !me ? html`<${LoginPage} robotId=${robotId} onLogin=${(u) => setMe(u)} />`
      : page === 'camera' ? html`<${CameraPage} onAuthLost=${onAuthLost} />`
      : html`<${RobotPage} onAuthLost=${onAuthLost} />`}
  `;
}

createRoot(document.getElementById('root')).render(html`<${App} />`);
