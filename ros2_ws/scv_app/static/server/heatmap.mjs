// Pure display math. Never modifies measured values or inspection decisions.
const STOPS = [[231,111,104],[244,174,113],[241,217,138],[140,203,180],[39,156,153]];
export function signalRgb(value) {
  const t = Math.max(0, Math.min(1, (value + 90) / 50)) * 4;
  const i = Math.min(3, Math.floor(t)), f = t - i;
  return STOPS[i].map((v, k) => Math.round(v + (STOPS[i+1][k] - v) * f));
}
export const signalColor = (v) => `rgb(${signalRgb(v).join(',')})`;
export function bilinear(values, tx, ty) {
  return values[0]*(1-tx)*(1-ty) + values[1]*tx*(1-ty) + values[2]*(1-tx)*ty + values[3]*tx*ty;
}
export function freeMask(rgba, width, height) {
  const mask = new Uint8Array(width * height);
  // Upload accepts negate=0 only. Unknown 205 and occupied pixels remain transparent.
  for (let i=0; i<mask.length; i++) mask[i] = rgba[i*4] > 205 && rgba[i*4+3] > 0 ? 1 : 0;
  return mask;
}
export function makeHeatField(m, readings, mask) {
  const w=m.width, h=m.height, g=m.grid_size_m, res=m.resolution_m;
  const cells=new Map((m.cells || []).map(([id,ix,iy]) => [id,[ix,iy]]));
  const byXY=new Map(), key=(ix,iy)=>`${ix},${iy}`;
  for (const r of readings || []) {
    const xy=cells.get(r.cell_id);
    if (xy) byXY.set(key(...xy),r);
  }
  // Integral obstacle mask: a bilinear block is allowed only if its whole rectangle is free.
  // This is deliberately stricter than hiding walls after blending values across them.
  const prefix=new Uint32Array((w+1)*(h+1));
  for(let y=0;y<h;y++) for(let x=0;x<w;x++) {
    const i=(y+1)*(w+1)+x+1;
    prefix[i]=(mask[y*w+x]?0:1)+prefix[i-1]+prefix[i-w-1]-prefix[i-w-2];
  }
  const point=(x,y)=>[Math.floor((x-m.origin_x)/res),Math.floor(h-(y-m.origin_y)/res)];
  function blockFree(ix,iy) {
    let [x0,y1]=point(m.origin_x+(ix+.5)*g,m.origin_y+(iy+.5)*g);
    let [x1,y0]=point(m.origin_x+(ix+1.5)*g,m.origin_y+(iy+1.5)*g);
    if(x0<0 || y0<0 || x1>=w || y1>=h) return false;
    x1++; y1++;
    return prefix[y1*(w+1)+x1]-prefix[y0*(w+1)+x1]-prefix[y1*(w+1)+x0]+prefix[y0*(w+1)+x0]===0;
  }
  const blocks=new Map();
  const eligible=(r)=>r && r.n>0 && Number.isFinite(r.median) && r.confidence==='ok' && !r.interpolated && r.state!=='unknown';
  function sample(x,y,smooth=true) {
    const [px,py]=point(x,y);
    if(px<0 || py<0 || px>=w || py>=h || !mask[py*w+px]) return null;
    const ix=Math.floor((x-m.origin_x)/g),iy=Math.floor((y-m.origin_y)/g),r=byXY.get(key(ix,iy));
    if(!r) return null;
    if(!r.n || !Number.isFinite(r.median) || r.interpolated) return r.detection==='not_detected' ? {r,miss:true} : null;
    let value=r.median,estimated=false;
    if(smooth && eligible(r)) {
      const gx=(x-m.origin_x)/g-.5, gy=(y-m.origin_y)/g-.5, bx=Math.floor(gx),by=Math.floor(gy), k=key(bx,by);
      if(!blocks.has(k)) {
        const rs=[[bx,by],[bx+1,by],[bx,by+1],[bx+1,by+1]].map(([i,j])=>byXY.get(key(i,j)));
        blocks.set(k,rs.every(eligible) && blockFree(bx,by) ? rs.map((v)=>v.median) : null);
      }
      const values=blocks.get(k);
      if(values) { value=bilinear(values,gx-bx,gy-by); estimated=true; }
    }
    return {value,estimated,r};
  }
  return {sample};
}

export function createHeatLayer(L, m, overlay, options, onSelect, onError) {
  // ImageOverlay uses a dedicated pane below vectors so heat never tints robot/asset markers.
  let disposed=false, layer=null;
  const group=L.layerGroup();
  const image=new Image(); image.crossOrigin='anonymous';
  image.onerror=()=>{ if(!disposed) onError?.('히트맵 마스크용 지도 이미지를 읽지 못했습니다.'); };
  image.onload=()=>{
    if(disposed) return;
    try {
      const src=document.createElement('canvas');src.width=m.width;src.height=m.height;
      const ctx=src.getContext('2d',{willReadFrequently:true});ctx.drawImage(image,0,0,m.width,m.height);
      const mask=freeMask(ctx.getImageData(0,0,m.width,m.height).data,m.width,m.height);
      const field=makeHeatField(m,overlay.readings,mask), scale=Math.min(4,Math.max(1,Math.floor(1800/Math.max(m.width,m.height))));
      const canvas=document.createElement('canvas');canvas.width=m.width*scale;canvas.height=m.height*scale;
      const c=canvas.getContext('2d'), pixels=c.createImageData(canvas.width,canvas.height);
      for(let y=0;y<canvas.height;y++) for(let x=0;x<canvas.width;x++) {
        const wx=m.origin_x+(x+.5)/scale*m.resolution_m,wy=m.origin_y+(m.height-(y+.5)/scale)*m.resolution_m;
        const s=field.sample(wx,wy,options.smooth);
        if(!s) continue;
        const i=(y*canvas.width+x)*4;
        if(s.miss) {
          if((Math.floor(x/scale)+Math.floor(y/scale))%4===0) {pixels.data.set([102,116,134,150],i);}
        } else pixels.data.set([...signalRgb(s.value),Math.round(255*(s.r.confidence==='ok'?1:.45))],i);
      }
      c.putImageData(pixels,0,0);
      layer=L.imageOverlay(canvas.toDataURL(),[[m.origin_y,m.origin_x],[m.origin_y+m.height*m.resolution_m,m.origin_x+m.width*m.resolution_m]],
        {opacity:options.opacity,pane:'heatPane',interactive:true,className:'signal-canvas'}).addTo(group);
      layer.on('click',(e)=>{
        const s=field.sample(e.latlng.lng,e.latlng.lat,options.smooth);
        if(s) onSelect?.({kind:'cell',data:{cell_id:s.r.cell_id}});
      });
    } catch(e) { if(!disposed) onError?.('히트맵 표시 실패: '+e.message); }
  };
  image.src=m.image_url;
  return {group,dispose:()=>{disposed=true;group.clearLayers();image.onload=null;image.onerror=null;}};
}
