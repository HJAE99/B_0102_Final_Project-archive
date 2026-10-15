import {test} from 'node:test';
import assert from 'node:assert/strict';
import {makeHeatField,bilinear,signalRgb,freeMask} from '../static/server/heatmap.mjs';
const m={width:20,height:20,resolution_m:.1,origin_x:0,origin_y:0,grid_size_m:1,cells:[[1,0,0],[2,1,0],[3,0,1],[4,1,1]]};
const readings=m.cells.map(([cell_id],i)=>({cell_id,n:4,median:-80+i*10,confidence:'ok',state:'normal'}));
const free=()=>new Uint8Array(400).fill(1);
test('bilinear stays within measured bounds and colors clamp',()=>{
  assert.equal(bilinear([-80,-60,-60,-40],.5,.5),-60);
  assert.deepEqual(signalRgb(-999),signalRgb(-90));assert.deepEqual(signalRgb(0),signalRgb(-40));
});
test('all four valid neighbors are necessary',()=>{
  assert.equal(makeHeatField(m,readings,free()).sample(1,1).estimated,true);
  assert.equal(makeHeatField(m,readings.slice(0,3),free()).sample(1,1),null);
});
test('walls exclude blending, even if the output pixel itself is free',()=>{
  const mask=free();mask[10*20+10]=0;
  const s=makeHeatField(m,readings,mask).sample(.95,.95);
  assert.equal(s.estimated,false);assert.equal(s.value,-80);
});
test('low confidence and unknown neighbors cannot be interpolated',()=>{
  for(const bad of [{confidence:'low'},{state:'unknown'},{interpolated:true}]) {
    const rs=readings.map((r,i)=>i===3?{...r,...bad}:r);
    assert.equal(makeHeatField(m,rs,free()).sample(.95,.95).estimated,false);
  }
});
test('misses stay separate, unmeasured stays empty, raw mode returns original',()=>{
  const rs=readings.map(r=>({...r,n:0,median:0,detection:'not_detected'}));
  assert.equal(makeHeatField(m,rs,free()).sample(.5,.5).miss,true);
  assert.equal(makeHeatField(m,[],free()).sample(.5,.5),null);
  assert.equal(makeHeatField(m,readings,free()).sample(.95,.95,false).value,-80);
});
test('map y is flipped correctly and unknown/occupied pixels are masked',()=>{
  const mask=free();mask[19*20]=0;
  assert.equal(makeHeatField(m,readings,mask).sample(.05,.05),null);
  assert.equal(makeHeatField(m,readings,mask).sample(.05,1.95).r.cell_id,3);
  assert.deepEqual([...freeMask(new Uint8Array([0,0,0,255,205,205,205,255,255,255,255,255]),3,1)],[0,0,1]);
});
