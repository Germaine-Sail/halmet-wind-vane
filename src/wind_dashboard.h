#pragma once

// Phone wind dashboard served by the board: http://halmet-wind.local/wind
//
//   Numbers  just the angle and the speed, as large as the screen allows
//   Dial  boat-up instrument face, port/starboard close-hauled sectors,
//         apparent wind angle and speed in large type
//   Rose  where the apparent wind has come from over the last 10 minutes,
//         one petal per 10 degrees, shaded by average speed
//
// Night mode (all red), knots or m/s, and keep-screen-on are remembered by the
// phone. Add it to the iPhone home screen for a full-screen instrument.
//
// Data: GET /api/wind         {"awa":deg|null,"aws":mps|null}
//       GET /api/wind/history {"awa":[deg|-1,...],"aws":[cm/s,...]} 1 Hz,
//                              oldest first, up to 10 minutes

#include <Arduino.h>
#include <esp_http_server.h>
#include <math.h>

#include <functional>

#include "firmware_update.h"  // HttpServerAccess

namespace winddash {

static std::function<float()> get_awa_rad;
static std::function<float()> get_aws_mps;

static constexpr int kHistory = 600;  // 10 min at 1 Hz
static int16_t hist_awa[kHistory];    // whole degrees -179..180, or -999
static uint16_t hist_aws[kHistory];   // cm/s
static int hist_head = 0, hist_count = 0;

inline void sample() {
  const float a = get_awa_rad ? get_awa_rad() : NAN;
  const float s = get_aws_mps ? get_aws_mps() : NAN;
  hist_awa[hist_head] = isfinite(a) ? (int16_t)lroundf(a * 180.0f / M_PI) : -999;
  hist_aws[hist_head] =
      isfinite(s) && s >= 0 ? (uint16_t)fminf(s * 100.0f, 65000.0f) : 0;
  hist_head = (hist_head + 1) % kHistory;
  if (hist_count < kHistory) hist_count++;
}

static const char kWindPage[] = R"HTML(<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover">
<meta name="apple-mobile-web-app-capable" content="yes">
<meta name="mobile-web-app-capable" content="yes">
<meta name="apple-mobile-web-app-status-bar-style" content="black-translucent">
<meta name="apple-mobile-web-app-title" content="Wind">
<title>Wind</title>
<style>
:root{--bg:#dfe5ea;--face:#fff;--ink:#0f1f2c;--soft:#7d8b96;--rule:#c4ccd3;
--port:#d4252b;--stbd:#13945a;--fly:#f08a00;--boat:#9aa7b1}
body.night{--bg:#000;--face:#070000;--ink:#d0141c;--soft:#6e0a0f;--rule:#3a0507;
--port:#8c0d12;--stbd:#d0141c;--fly:#ff2a2a;--boat:#4a0609}
*{box-sizing:border-box;-webkit-tap-highlight-color:transparent}
html,body{margin:0;height:100%;background:var(--bg);color:var(--ink)}
body{font-family:ui-rounded,"SF Pro Rounded",system-ui,-apple-system,sans-serif;
padding:env(safe-area-inset-top) 16px env(safe-area-inset-bottom);
display:flex;flex-direction:column;max-width:560px;margin:0 auto;
font-variant-numeric:tabular-nums}
svg{display:block;width:100%;height:auto}
#face{margin-top:10px}
.read{display:grid;grid-template-columns:1fr 1fr;gap:8px;margin:6px 0 14px}
.read div{background:var(--face);border-radius:18px;padding:10px 14px 12px}
.read b{display:block;font-size:clamp(44px,15vw,76px);font-weight:700;
line-height:1;letter-spacing:-.02em}
.read span{font-size:15px;color:var(--soft)}
#side.p{color:var(--port)}#side.s{color:var(--stbd)}
nav{display:flex;gap:8px;margin-top:auto;padding-bottom:12px}
nav button{flex:1;font:inherit;font-size:15px;font-weight:600;color:var(--ink);
background:var(--face);border:0;border-radius:14px;padding:12px 0}
nav button[aria-pressed=true]{background:var(--ink);color:var(--face)}
nav button:focus-visible{outline:3px solid var(--fly);outline-offset:2px}
#note{font-size:14px;color:var(--soft);text-align:center;min-height:20px;margin:0}
.stale .read b,.stale #fly{opacity:.25}
body.big #face,body.big #note{display:none}
body.big .read{grid-template-columns:1fr;flex:1;margin:12px 0}
body.big .read div{display:flex;flex-direction:column;justify-content:center;
align-items:center;padding:4px 8px}
body.big .read b{font-size:min(30vh,34vw)}
body.big .read span{font-size:20px}
@media (orientation:landscape){body.big .read{grid-template-columns:1fr 1fr}
body.big .read b{font-size:min(50vh,22vw)}}
</style></head><body>
<svg id="face" viewBox="0 0 400 400" aria-label="Apparent wind"></svg>
<p id="note"></p>
<div class="read">
 <div><b id="awa">--</b><span id="side">Angle</span></div>
 <div><b id="aws">--</b><span id="unit">knots</span></div>
</div>
<nav>
 <button id="vDial" aria-pressed="true">Dial</button>
 <button id="vRose" aria-pressed="false">Rose</button>
 <button id="vBig" aria-pressed="false">Numbers</button>
 <button id="bUnit">m/s</button>
 <button id="bNight">Night</button>
</nav>
<script>
const $=id=>document.getElementById(id),NS='http://www.w3.org/2000/svg';
const st={view:localStorage.view||'dial',kn:localStorage.unit!=='ms',
night:localStorage.night==='1'};
let awa=null,aws=null,last=0,hist=null;
const C=200,R=176;
function el(t,a,p){const e=document.createElementNS(NS,t);
 for(const k in a)e.setAttribute(k,a[k]);(p||$('face')).appendChild(e);return e;}
function pt(deg,r){const t=(deg-90)*Math.PI/180;return[C+r*Math.cos(t),C+r*Math.sin(t)];}
function arc(a0,a1,r){const[x0,y0]=pt(a0,r),[x1,y1]=pt(a1,r);
 return`M${x0} ${y0}A${r} ${r} 0 ${a1-a0>180?1:0} 1 ${x1} ${y1}`;}
function boat(){el('path',{d:'M200 128C222 158 226 210 220 262L214 284H186L180 262C174 210 178 158 200 128Z',
 fill:'none',stroke:'var(--boat)','stroke-width':3});}
function drawDial(){
 const f=$('face');f.innerHTML='';
 el('circle',{cx:C,cy:C,r:R+16,fill:'var(--face)'});
 el('path',{d:arc(-55,-25,R-8),stroke:'var(--port)','stroke-width':14,fill:'none'});
 el('path',{d:arc(25,55,R-8),stroke:'var(--stbd)','stroke-width':14,fill:'none'});
 for(let d=0;d<360;d+=10){const big=d%30==0,[x0,y0]=pt(d,R),[x1,y1]=pt(d,R-(big?22:11));
  el('line',{x1:x0,y1:y0,x2:x1,y2:y1,stroke:'var(--ink)','stroke-width':big?3:1.5,
  'stroke-linecap':'round'});
  if(big&&d){const n=d>180?360-d:d,[tx,ty]=pt(d,R-40);
   const t=el('text',{x:tx,y:ty+6,'text-anchor':'middle','font-size':17,
   'font-weight':600,fill:'var(--soft)'});t.textContent=n;}}
 boat();
 const g=el('g',{id:'fly'});
 el('path',{d:'M200 34L186 70H214Z',fill:'var(--fly)'},g);
 el('line',{x1:200,y1:70,x2:200,y2:150,stroke:'var(--fly)','stroke-width':5,
  'stroke-linecap':'round'},g);
 place();}
function place(){const g=$('fly');if(!g)return;
 g.setAttribute('transform',`rotate(${awa==null?0:awa} ${C} ${C})`);
 g.style.display=awa==null?'none':'';}
function drawRose(){
 const f=$('face');f.innerHTML='';
 el('circle',{cx:C,cy:C,r:R+16,fill:'var(--face)'});
 [0.33,0.66,1].forEach(k=>el('circle',{cx:C,cy:C,r:R*k,fill:'none',
  stroke:'var(--rule)','stroke-width':1}));
 for(let d=0;d<360;d+=30){const[x,y]=pt(d,R);el('line',{x1:C,y1:C,x2:x,y2:y,
  stroke:'var(--rule)','stroke-width':1});}
 if(!hist||!hist.awa.length){$('note').textContent='Collecting wind history';boat();return;}
 const n=new Array(36).fill(0),sp=new Array(36).fill(0);let tot=0;
 hist.awa.forEach((a,i)=>{if(a<-200)return;const b=Math.floor(((a+360)%360)/10)%36;
  n[b]++;sp[b]+=hist.aws[i]/100;tot++;});
 const mx=Math.max(1,...n),top=Math.max(0.1,...sp.map((s,i)=>n[i]?s/n[i]:0));
 n.forEach((c,b)=>{if(!c)return;const r=R*Math.sqrt(c/mx),a0=b*10+1,a1=b*10+9;
  const[x0,y0]=pt(a0,r),[x1,y1]=pt(a1,r),k=(sp[b]/c)/top;
  el('path',{d:`M${C} ${C}L${x0} ${y0}A${r} ${r} 0 0 1 ${x1} ${y1}Z`,
  fill:'var(--fly)','fill-opacity':(0.25+0.75*k).toFixed(2)});});
 boat();
 if(awa!=null){const[x,y]=pt(awa,R+6),[x2,y2]=pt(awa,R-26);
  el('line',{x1:x,y1:y,x2:x2,y2:y2,stroke:'var(--ink)','stroke-width':5,
  'stroke-linecap':'round'});}
 const mins=Math.round(tot/60);
 $('note').textContent=tot?`Last ${mins<1?'minute':mins+' minutes'}, darker petals mean stronger wind`:
 'No wind readings yet';}
function draw(){document.body.classList.toggle('big',st.view==='big');
 if(st.view==='rose')drawRose();else if(st.view==='dial'){drawDial();$('note').textContent='';}}
function show(){
 const stale=Date.now()-last>4000;document.body.classList.toggle('stale',stale);
 if(awa==null){$('awa').textContent='--';$('side').textContent='Angle';$('side').className='';}
 else{const a=Math.round(Math.abs(awa));$('awa').textContent=a+'\u00b0';
  $('side').textContent=a==0?'Ahead':a==180?'Astern':awa<0?'Port':'Starboard';
  $('side').className=a==0||a==180?'':awa<0?'p':'s';}
 $('aws').textContent=aws==null?'--':(aws*(st.kn?1.943844:1)).toFixed(1);
 $('unit').textContent=st.kn?'knots':'m/s';$('bUnit').textContent=st.kn?'m/s':'knots';
 if(stale&&last)$('note').textContent='No data from the board for a few seconds';
 if(st.view==='dial')place();}
async function poll(){
 try{const j=await(await fetch('/api/wind',{cache:'no-store'})).json();
  awa=j.awa;aws=j.aws;last=Date.now();}catch(e){}
 show();setTimeout(poll,500);}
async function pollHist(){
 if(st.view==='rose'){try{hist=await(await fetch('/api/wind/history',{cache:'no-store'})).json();drawRose();}catch(e){}}
 setTimeout(pollHist,5000);}
function setView(v){st.view=localStorage.view=v;
 $('vDial').setAttribute('aria-pressed',v==='dial');$('vRose').setAttribute('aria-pressed',v==='rose');
 $('vBig').setAttribute('aria-pressed',v==='big');
 draw();if(v==='rose'){hist=null;fetch('/api/wind/history',{cache:'no-store'}).then(r=>r.json())
 .then(j=>{hist=j;drawRose();}).catch(()=>{});}}
$('vDial').onclick=()=>setView('dial');$('vRose').onclick=()=>setView('rose');
$('vBig').onclick=()=>setView('big');
$('bUnit').onclick=()=>{st.kn=!st.kn;localStorage.unit=st.kn?'kn':'ms';show();};
$('bNight').onclick=()=>{st.night=!st.night;localStorage.night=st.night?'1':'0';
 document.body.classList.toggle('night',st.night);$('bNight').textContent=st.night?'Day':'Night';};
document.body.classList.toggle('night',st.night);$('bNight').textContent=st.night?'Day':'Night';
let lock=null;async function wake(){try{lock=await navigator.wakeLock.request('screen');}catch(e){}}
document.addEventListener('visibilitychange',()=>{if(!document.hidden)wake();});wake();
setView(st.view);show();poll();pollHist();
</script></body></html>)HTML";

static esp_err_t handle_page(httpd_req_t* req) {
  httpd_resp_set_type(req, "text/html; charset=utf-8");
  httpd_resp_send(req, kWindPage, HTTPD_RESP_USE_STRLEN);
  return ESP_OK;
}

static esp_err_t handle_now(httpd_req_t* req) {
  const float a = get_awa_rad ? get_awa_rad() : NAN;
  const float s = get_aws_mps ? get_aws_mps() : NAN;
  char awa[16] = "null", aws[16] = "null";
  if (isfinite(a)) snprintf(awa, sizeof(awa), "%.1f", a * 180.0f / M_PI);
  if (isfinite(s)) snprintf(aws, sizeof(aws), "%.2f", s);
  char json[64];
  snprintf(json, sizeof(json), "{\"awa\":%s,\"aws\":%s}", awa, aws);
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  httpd_resp_sendstr(req, json);
  return ESP_OK;
}

// Streamed in pieces so the 10-minute history never needs one big buffer.
static esp_err_t handle_history(httpd_req_t* req) {
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  const int n = hist_count, start = (hist_head - n + kHistory) % kHistory;
  char buf[512];
  for (int pass = 0; pass < 2; pass++) {
    int len = snprintf(buf, sizeof(buf), pass ? "],\"aws\":[" : "{\"awa\":[");
    for (int i = 0; i < n; i++) {
      const int k = (start + i) % kHistory;
      const int v = pass ? hist_aws[k] : (hist_awa[k] == -999 ? -999 : hist_awa[k]);
      len += snprintf(buf + len, sizeof(buf) - len, i ? ",%d" : "%d", v);
      if (len > (int)sizeof(buf) - 16) {
        httpd_resp_send_chunk(req, buf, len);
        len = 0;
      }
    }
    if (len) httpd_resp_send_chunk(req, buf, len);
  }
  httpd_resp_sendstr_chunk(req, "]}");
  httpd_resp_send_chunk(req, nullptr, 0);
  return ESP_OK;
}

inline void add_wind_dashboard(std::function<float()> awa_rad,
                               std::function<float()> aws_mps) {
  get_awa_rad = awa_rad;
  get_aws_mps = aws_mps;
  using sensesp::HTTPRequestHandler;
  auto& server = halmet::HttpServerAccess::of(*sensesp::sensesp_app);
  auto page = std::make_shared<HTTPRequestHandler>(1 << HTTP_GET, "/wind",
                                                   handle_page);
  auto now = std::make_shared<HTTPRequestHandler>(1 << HTTP_GET, "/api/wind",
                                                  handle_now);
  auto hist = std::make_shared<HTTPRequestHandler>(
      1 << HTTP_GET, "/api/wind/history", handle_history);
  server->add_handler(page);
  server->add_handler(now);
  server->add_handler(hist);
  sensesp::event_loop()->onRepeat(1000, []() { sample(); });
}

}  // namespace winddash
