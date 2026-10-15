// Run against preview_web.py only; all mutation/control endpoints are absent there.
const { chromium } = require('playwright');
const assert = require('node:assert/strict');
const path = require('node:path');
(async () => {
  const browser = await chromium.launch({executablePath:process.env.CHROME_PATH || '/usr/bin/google-chrome',headless:true,args:['--no-sandbox']});
  const errors=[], failures=[], checks=[];
  const out = process.env.SCV_PREVIEW_OUTPUT || '/tmp';
  const page = await browser.newPage({viewport:{width:1440,height:1000}});
  page.on('pageerror',e=>errors.push(e.message));
  const check=(name,condition)=>{assert.ok(condition,name);checks.push(name);};
  try {
    await page.goto('http://127.0.0.1:8765/server/monitor');
    await page.locator('.signal-canvas').waitFor();
    await page.getByText('2 / 2',{exact:true}).waitFor();
    check('sample-data notice visible',await page.locator('.preview-banner').isVisible());
    check('one stable map / grid hidden',await page.locator('.leaflet-container').count()===1 && await page.locator('.cell-label').count()===0);
    check('ROS mode has no web patrol button',await page.getByRole('button',{name:'순찰 시작',exact:true}).count()===0);
    const pane=page.locator('.leaflet-map-pane');
    await page.getByRole('button',{name:'Zoom in'}).click();
    await page.waitForTimeout(400);
    const before=await pane.getAttribute('style');
    await page.getByRole('button',{name:'격자·번호'}).click();
    check('grid toggle displays the measured grid',await page.locator('.cell-label').count()===98);
    await page.getByRole('button',{name:'격자·번호'}).click();
    check('layer toggle preserves zoom/pan',await pane.getAttribute('style')===before);
    await page.getByRole('link',{name:'지도 전체 보기'}).click();
    await page.getByRole('button',{name:'시작 참고점',exact:true}).click();
    check('reference toggle active',await page.getByRole('button',{name:'시작 참고점',exact:true}).getAttribute('aria-pressed')==='true');
    await page.getByRole('button',{name:'시작 참고점',exact:true}).click();
    const first=await page.locator('.signal-canvas').getAttribute('src');
    await page.getByLabel('측정 AP',{exact:true}).selectOption('2');
    await page.waitForFunction(src=>document.querySelector('.signal-canvas')?.src!==src,first);
    check('AP changes rendered values',await page.locator('.signal-canvas').getAttribute('src')!==first);
    await page.getByLabel('측정 AP',{exact:true}).selectOption('1');
    await page.waitForTimeout(250);
    const box=await page.locator('.signal-canvas').boundingBox();
    await page.mouse.click(box.x+(1.5+.15)/3.45*box.width,box.y+box.height-(4.0+.25)/6.9*box.height);
    await page.locator('.selection .signal-value').waitFor();
    check('measurement click shows original statistic',await page.locator('.selection').innerText().then(t=>t.includes('저장된 중앙값')&&t.includes('표시용')));
    await page.locator('.robot-card').first().click();
    check('robot selection updates detail',await page.locator('.selection').innerText().then(t=>t.includes('robot3')&&t.includes('현재 상태')));
    await page.getByLabel('히트맵 농도').fill('0.65');
    await page.waitForTimeout(250);
    check('opacity control changes layer',await page.locator('.signal-canvas').evaluate(e=>e.style.opacity==='0.65'));
    await page.getByLabel('히트맵 농도').fill('0.5');
    await page.getByRole('button',{name:'히트맵',exact:true}).click();
    check('heat toggle hides only heat',await page.locator('.signal-canvas').count()===0&&await page.locator('.base-map').count()===1);
    await page.getByRole('button',{name:'히트맵',exact:true}).click();
    await page.locator('.signal-canvas').waitFor();
    for(const [width,height] of [[1440,1000],[1366,768],[390,844]]) {
      await page.setViewportSize({width,height});await page.waitForTimeout(300);
      check(`no horizontal overflow ${width}`,await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth));
      await page.screenshot({path:path.join(out,`SCV_웹_미리보기_${width}.png`),fullPage:true});
    }
    await page.setViewportSize({width:1440,height:1000});
    await page.route('**/api/runs/21',r=>r.fulfill({status:503,json:{detail:'시험용 연결 중단'}}));
    await page.getByText(/시험용 연결 중단/).waitFor({timeout:7000});
    check('API failure retains last heat and reports stale data',await page.locator('.signal-canvas').count()===1);
    await page.unroute('**/api/runs/21');
    const fixture=await page.request.get('http://127.0.0.1:8765/api/runs/21').then(r=>r.json());
    await page.route('**/api/runs',r=>r.fulfill({json:{runs:[{...fixture.run,run_id:22}],current:22}}));
    await page.route('**/api/runs/22',r=>r.fulfill({json:{...fixture,run:{...fixture.run,run_id:22,map_id:2}}}));
    await page.getByText('회차와 지도가 달라 히트맵을 생략했습니다.').waitFor({timeout:10000});
    check('different run map cannot overlay heat',await page.locator('.signal-canvas').count()===0);
    await page.unroute('**/api/runs');await page.unroute('**/api/runs/22');
    await page.getByRole('button',{name:'점검 결과',exact:true}).click();
    await page.locator('.signal-canvas').waitFor();
    check('history uses same heat renderer',await page.getByRole('button',{name:'작업·판정 상세 표 보기'}).isVisible());
    await page.getByRole('button',{name:'작업·판정 상세 표 보기'}).click();
    check('history technical tables are opt-in',await page.getByRole('button',{name:'상세 표 접기'}).isVisible());
    await page.getByRole('button',{name:'설정',exact:true}).click();
    check('settings advanced controls opt-in',!(await page.getByLabel('고급 관리 항목 표시').isChecked()));
    // Dedicated socket fixture: open-but-silent and disconnected states cannot show live robot data.
    const offline=await browser.newPage({viewport:{width:1366,height:768}});
    offline.on('pageerror',e=>errors.push(e.message));
    await offline.routeWebSocket('**/ws/ui',ws=>{
      ws.send(JSON.stringify({type:'robots',preview:true,ts:new Date().toISOString(),robots:[{robot_id:'robot3',connected:true,state:{x:null,y:null,yaw:null,pose_ok:false,battery_pct:null,nav_state:'대기'}}],telemetry:{alerts:[],inspections:[]}}));
    });
    await offline.goto('http://127.0.0.1:8765/server/monitor');await offline.locator('.signal-canvas').waitFor();
    check('missing pose and battery are not fabricated',await offline.locator('.robot-card').innerText().then(t=>t.includes('현재 위치 미수신')&&t.includes('미수신')));
    await offline.locator('.robot-card').filter({hasText:'연결 끊김'}).waitFor({timeout:9000});
    check('silent websocket expires',await offline.locator('.robot-card').innerText().then(t=>t.includes('연결 끊김')));
    await offline.close();
    check('no browser runtime errors',errors.length===0);
    console.log(JSON.stringify({passed:checks.length,checks,errors},null,2));
  } finally {await browser.close();}
})().catch(e=>{console.error(e);process.exit(1);});
