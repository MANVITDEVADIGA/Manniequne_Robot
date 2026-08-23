// ============================================================
//  R4 ROBOT — WITH ULTRASONIC SENSOR
//  Arduino UNO R4 WiFi
//  Servo1: pin 9 | Servo2: pin 10
//  Motors: IN1-4 on 2,3,4,5 | ENA pin 6 | ENB pin 7
//  Ultrasonic: TRIG pin 11 | ECHO pin 12   <-- NEW
// ============================================================

#include <WiFiS3.h>
#include <Servo.h>

const char* SSID     = "R4_Robot";
const char* PASSWORD = "12345678";
WiFiServer  server(80);

#define IN1 2
#define IN2 3
#define IN3 4
#define IN4 5
#define ENA 6
#define ENB 7

int motorSpeed = 150;

#define SERVO1_PIN 9
#define SERVO2_PIN 10
Servo servo1;
Servo servo2;

// ============================================================
//  ULTRASONIC SENSOR PINS & STATE  <-- NEW
// ============================================================
#define TRIG_PIN 11
#define ECHO_PIN 12

// How often to take a distance reading (non-blocking interval)
const unsigned long ULTRA_INTERVAL_MS = 150;

// Detection threshold in centimetres
const int DETECT_CM = 30;

// How long servo1 stays at 90 before returning to 0
const unsigned long SERVO_HOLD_MS = 10000UL;

unsigned long lastUltraCheck  = 0;     // timestamp of last reading
bool          ultraTriggered  = false; // true while 10-s hold is active
unsigned long servoReturnTime = 0;     // millis() when servo1 must return to 0
int           lastDistanceCm  = 0;     // most recent measured distance

// ============================================================
//  HTML PAGE in PROGMEM (Flash) - saves RAM
// ============================================================
const char PAGE[] PROGMEM = R"===(<!DOCTYPE html>
<html lang='en'><head>
<meta charset='UTF-8'>
<meta name='viewport' content='width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no'>
<title>R4 Robot</title>
<style>
:root{--bg:#0a0a0f;--card:#12121a;--border:#1e1e30;--cyan:#00f5d4;--pink:#ff2d78;--text:#e8e8f0;--muted:#555570}
*{box-sizing:border-box;margin:0;padding:0;-webkit-tap-highlight-color:transparent}
body{background:var(--bg);color:var(--text);font-family:'Courier New',monospace;padding:14px}
h1{text-align:center;font-size:1.3rem;letter-spacing:.3em;color:var(--cyan);margin-bottom:2px}
.sub{text-align:center;color:var(--muted);font-size:.65rem;letter-spacing:.2em;margin-bottom:16px}
.card{background:var(--card);border:1px solid var(--border);border-radius:12px;padding:14px;margin-bottom:12px}
.ct{font-size:.6rem;letter-spacing:.25em;color:var(--muted);text-transform:uppercase;margin-bottom:10px}
.dpad{display:grid;grid-template-columns:repeat(3,72px);grid-template-rows:repeat(3,72px);gap:6px;justify-content:center}
.btn{background:var(--bg);border:1px solid var(--border);border-radius:10px;font-size:1.6rem;cursor:pointer;display:flex;align-items:center;justify-content:center;user-select:none;touch-action:none}
.btn:active{background:var(--cyan);color:#000}
.sbtn{background:#1a0a12;border-color:var(--pink);color:var(--pink)}
.sbtn:active{background:var(--pink);color:#fff}
.emp{visibility:hidden}
.row{display:flex;align-items:center;gap:10px;margin-bottom:12px}
.row:last-child{margin-bottom:0}
.lbl{color:var(--muted);font-size:.65rem;white-space:nowrap;width:46px}
input[type=range]{flex:1;-webkit-appearance:none;height:6px;border-radius:3px;outline:none;background:linear-gradient(to right,var(--cyan) var(--p,50%),var(--border) var(--p,50%))}
input[type=range]::-webkit-slider-thumb{-webkit-appearance:none;width:26px;height:26px;border-radius:50%;background:var(--cyan);border:2px solid var(--bg);cursor:pointer}
.val{font-size:.9rem;width:40px;text-align:right}
.vc{color:var(--cyan)}.vp{color:var(--pink)}
#st{text-align:center;font-size:.6rem;color:var(--muted);letter-spacing:.1em;margin-top:8px;padding:6px;border-radius:6px;min-height:20px}
#st.ok{color:var(--cyan)}#st.er{color:var(--pink)}
.uval{font-size:2.2rem;font-weight:bold;color:var(--cyan);text-align:center;line-height:1}
.uunit{font-size:.75rem;color:var(--muted)}
.ubar-wrap{background:var(--bg);border-radius:6px;height:8px;margin:10px 0 6px;overflow:hidden}
.ubar{height:100%;border-radius:6px;transition:width .3s,background .3s;background:var(--cyan);width:0%}
.utrig{text-align:center;font-size:.65rem;color:var(--pink);min-height:14px;letter-spacing:.1em}
</style></head><body>
<h1>&#x1F916; R4 ROBOT</h1>
<p class='sub'>WIFI CONTROLLER</p>

<div class='card'>
<div class='ct'>&#9654; Drive</div>
<div class='dpad'>
<div class='btn emp'></div>
<div class='btn' ontouchstart="mo('/forward')" ontouchend="mo('/stop')" onmousedown="mo('/forward')" onmouseup="mo('/stop')">&#8679;</div>
<div class='btn emp'></div>
<div class='btn' ontouchstart="mo('/left')" ontouchend="mo('/stop')" onmousedown="mo('/left')" onmouseup="mo('/stop')">&#8678;</div>
<div class='btn sbtn' ontouchstart="mo('/stop')" onmousedown="mo('/stop')">&#9632;</div>
<div class='btn' ontouchstart="mo('/right')" ontouchend="mo('/stop')" onmousedown="mo('/right')" onmouseup="mo('/stop')">&#8680;</div>
<div class='btn emp'></div>
<div class='btn' ontouchstart="mo('/back')" ontouchend="mo('/stop')" onmousedown="mo('/back')" onmouseup="mo('/stop')">&#8681;</div>
<div class='btn emp'></div>
</div></div>

<div class='card'>
<div class='ct'>&#9889; Speed</div>
<div class='row'>
<span class='lbl'>SLOW</span>
<input type='range' min='0' max='255' value='150' oninput='spd(this)'>
<span class='lbl'>FAST</span>
<span class='val vc' id='sv'>150</span>
</div></div>

<div class='card'>
<div class='ct'>&#x1F9BE; Servo 1 &nbsp;<span id='v1' style='color:var(--pink)'>0&deg;</span></div>
<div class='row'>
<span class='lbl'>0&deg;</span>
<input type='range' id='r1' min='0' max='180' value='0' oninput='srv(1,this)'>
<span class='lbl' style='text-align:right'>180&deg;</span>
</div>
<div style='display:flex;gap:8px;justify-content:center;margin-top:4px'>
<button onclick="setS(1,0)"   style='flex:1;padding:10px;background:var(--bg);border:1px solid var(--border);color:var(--text);border-radius:8px;font-family:monospace;cursor:pointer'>0&deg;</button>
<button onclick="setS(1,90)"  style='flex:1;padding:10px;background:var(--bg);border:1px solid var(--border);color:var(--text);border-radius:8px;font-family:monospace;cursor:pointer'>90&deg;</button>
<button onclick="setS(1,180)" style='flex:1;padding:10px;background:var(--bg);border:1px solid var(--border);color:var(--text);border-radius:8px;font-family:monospace;cursor:pointer'>180&deg;</button>
</div></div>

<div class='card'>
<div class='ct'>&#x1F9BE; Servo 2 &nbsp;<span id='v2' style='color:var(--pink)'>0&deg;</span></div>
<div class='row'>
<span class='lbl'>0&deg;</span>
<input type='range' id='r2' min='0' max='180' value='0' oninput='srv(2,this)'>
<span class='lbl' style='text-align:right'>180&deg;</span>
</div>
<div style='display:flex;gap:8px;justify-content:center;margin-top:4px'>
<button onclick="setS(2,0)"   style='flex:1;padding:10px;background:var(--bg);border:1px solid var(--border);color:var(--text);border-radius:8px;font-family:monospace;cursor:pointer'>0&deg;</button>
<button onclick="setS(2,90)"  style='flex:1;padding:10px;background:var(--bg);border:1px solid var(--border);color:var(--text);border-radius:8px;font-family:monospace;cursor:pointer'>90&deg;</button>
<button onclick="setS(2,180)" style='flex:1;padding:10px;background:var(--bg);border:1px solid var(--border);color:var(--text);border-radius:8px;font-family:monospace;cursor:pointer'>180&deg;</button>
</div></div>

<div class='card'>
<div class='ct'>&#x26A1; Both Servos</div>
<div style='display:flex;gap:8px'>
<button onclick="liftBoth(180)" style='flex:1;padding:14px 0;background:var(--cyan);border:none;color:#000;border-radius:10px;font-family:monospace;font-size:.95rem;font-weight:bold;cursor:pointer'>&#8679; LIFT (180&deg;)</button>
<button onclick="liftBoth(0)"   style='flex:1;padding:14px 0;background:var(--bg);border:1px solid var(--border);color:var(--text);border-radius:10px;font-family:monospace;font-size:.95rem;cursor:pointer'>&#8681; DOWN (0&deg;)</button>
</div></div>

<!-- NEW: Ultrasonic sensor card -->
<div class='card'>
<div class='ct'>&#x1F4E1; Ultrasonic Sensor</div>
<div class='uval'><span id='ucm'>--</span> <span class='uunit'>cm</span></div>
<div class='ubar-wrap'><div class='ubar' id='ubar'></div></div>
<div class='utrig' id='utrig'></div>
</div>

<div id='st'>READY</div>

<script>
var st=document.getElementById('st');

function tf(el){el.style.setProperty('--p',(el.value-el.min)/(el.max-el.min)*100+'%');}
tf(document.getElementById('r1'));
tf(document.getElementById('r2'));

function send(url){
  st.className='';st.textContent='>> '+url;
  var x=new XMLHttpRequest();
  x.open('GET',url,true);
  x.timeout=2000;
  x.onload=function(){st.className='ok';st.textContent='OK: '+x.responseText.trim();};
  x.onerror=function(){st.className='er';st.textContent='ERROR';};
  x.ontimeout=function(){st.className='er';st.textContent='TIMEOUT';};
  x.send();
}

function mo(cmd){send(cmd);}

var ts;
function spd(el){
  tf(el);
  document.getElementById('sv').textContent=el.value;
  clearTimeout(ts);
  ts=setTimeout(function(){send('/spd?v='+el.value);},250);
}

function setS(n,a){
  document.getElementById('r'+n).value=a;
  tf(document.getElementById('r'+n));
  document.getElementById('v'+n).textContent=a+'\u00b0';
  send('/s'+n+'?a='+a);
}

function liftBoth(a){
  document.getElementById('r1').value=a; tf(document.getElementById('r1'));
  document.getElementById('v1').textContent=a+'\u00b0';
  document.getElementById('r2').value=a; tf(document.getElementById('r2'));
  document.getElementById('v2').textContent=a+'\u00b0';
  s1a=a; s2a=a;
  st.className='';st.textContent='>> BOTH -> '+a+'deg';
  var x1=new XMLHttpRequest();
  x1.open('GET','/s1?a='+a,true);
  x1.timeout=2000;
  x1.onload=function(){
    var x2=new XMLHttpRequest();
    x2.open('GET','/s2?a='+a,true);
    x2.timeout=2000;
    x2.onload=function(){st.className='ok';st.textContent='OK: BOTH='+a+'deg';};
    x2.onerror=function(){st.className='er';st.textContent='ERR S2';};
    x2.ontimeout=function(){st.className='er';st.textContent='TIMEOUT S2';};
    x2.send();
  };
  x1.onerror=function(){st.className='er';st.textContent='ERR S1';};
  x1.ontimeout=function(){st.className='er';st.textContent='TIMEOUT S1';};
  x1.send();
}

var s1a=0, s2a=0, t1, t2;
function srv(n,el){
  tf(el);
  var a=parseInt(el.value);
  if(n===1){
    s1a=a;
    document.getElementById('v1').textContent=a+'\u00b0';
    clearTimeout(t1);
    t1=setTimeout(function(){send('/s1?a='+s1a);},250);
  } else {
    s2a=a;
    document.getElementById('v2').textContent=a+'\u00b0';
    clearTimeout(t2);
    t2=setTimeout(function(){send('/s2?a='+s2a);},250);
  }
}

// NEW: Poll /ultra every 500 ms
// Response format: "distanceCm,countdown"  (countdown=-1 when idle)
setInterval(function(){
  var x=new XMLHttpRequest();
  x.open('GET','/ultra',true);
  x.timeout=1000;
  x.onload=function(){
    var p=x.responseText.split(',');
    var d=parseInt(p[0]);
    var c=parseInt(p[1]);
    var cm=document.getElementById('ucm');
    var bar=document.getElementById('ubar');
    var trig=document.getElementById('utrig');
    if(d>0){
      cm.textContent=d;
      var pct=Math.min(d,200)/200*100;
      bar.style.width=pct+'%';
      bar.style.background=(d<=30)?'var(--pink)':'var(--cyan)';
    } else {
      cm.textContent='--';
      bar.style.width='0%';
    }
    if(c>=0){
      trig.textContent='OBJECT DETECTED \u2014 S1 returns in '+c+'s';
      document.getElementById('r1').value=90;
      tf(document.getElementById('r1'));
      document.getElementById('v1').textContent='90\u00b0';
    } else {
      trig.textContent='';
    }
  };
  x.onerror=function(){};x.ontimeout=function(){};
  x.send();
},500);
</script>
</body></html>
)===";

// ============================================================
//  SETUP
// ============================================================
void setup() {
  Serial.begin(9600);
  delay(1000);
  Serial.println(F("=== R4 ROBOT BOOT ==="));

  // Motors
  pinMode(IN1,OUTPUT); pinMode(IN2,OUTPUT);
  pinMode(IN3,OUTPUT); pinMode(IN4,OUTPUT);
  pinMode(ENA,OUTPUT); pinMode(ENB,OUTPUT);
  digitalWrite(ENB, HIGH);
  digitalWrite(IN1,LOW); digitalWrite(IN2,LOW);
  digitalWrite(IN3,LOW); digitalWrite(IN4,LOW);
  analogWrite(ENA, 0);
  Serial.println(F("Motors OK"));

  // Servos - attach BEFORE WiFi, start at 0 on power-on
  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);
  delay(100);
  servo1.write(0);    // S1: 0 user = 0 physical
  servo2.write(180);  // S2: 0 user = 180 physical (reversed mount)
  delay(700);
  Serial.println(F("Servo1 OK (pin 9)  -> 0"));
  Serial.println(F("Servo2 OK (pin 10) -> 0"));

  // NEW: Ultrasonic sensor
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);
  Serial.println(F("Ultrasonic OK (TRIG=11, ECHO=12)"));

  // WiFi AP
  WiFi.beginAP(SSID, PASSWORD);
  delay(2000);
  Serial.print(F("AP IP: "));
  Serial.println(WiFi.localIP());
  server.begin();
  Serial.println(F("Ready. Connect to R4_Robot WiFi."));
}

// ============================================================
//  NEW: Measure distance - blocks max ~25 ms (25000 us timeout)
// ============================================================
int getDistanceCm() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long dur = pulseIn(ECHO_PIN, HIGH, 25000); // 25 ms timeout ~ 430 cm max
  if (dur == 0) return 0;                    // no echo = out of range
  return (int)(dur * 0.0343f / 2.0f);
}

// ============================================================
//  HTTP SEND
// ============================================================
void sendPage(WiFiClient& client) {
  client.println(F("HTTP/1.1 200 OK"));
  client.println(F("Content-Type: text/html"));
  client.println(F("Connection: close"));
  client.println();
  int len=strlen_P(PAGE), sent=0;
  char buf[65];
  while(sent<len){
    int chunk=min(64,len-sent);
    memcpy_P(buf,PAGE+sent,chunk);
    buf[chunk]='\0';
    client.write(buf,chunk);
    sent+=chunk;
  }
}

void sendOK(WiFiClient& client, const String& msg) {
  client.println(F("HTTP/1.1 200 OK"));
  client.println(F("Content-Type: text/plain"));
  client.println(F("Connection: close"));
  client.println();
  client.print(msg);
}

// ============================================================
//  LOOP
// ============================================================
void loop() {

  // ── NEW: Non-blocking ultrasonic logic ───────────────────
  unsigned long now = millis();

  // Take a reading every ULTRA_INTERVAL_MS milliseconds
  if (now - lastUltraCheck >= ULTRA_INTERVAL_MS) {
    lastUltraCheck = now;
    lastDistanceCm = getDistanceCm();

    // Only trigger when idle — prevents re-arming during the 10-s hold
    if (!ultraTriggered && lastDistanceCm > 0 && lastDistanceCm <= DETECT_CM) {
      servo1.write(90);
      ultraTriggered  = true;
      servoReturnTime = now + SERVO_HOLD_MS;
      Serial.print(F("ULTRA: Object at "));
      Serial.print(lastDistanceCm);
      Serial.println(F(" cm -> S1=90"));
    }
  }

  // Return servo1 to 0 exactly after 10 seconds
  if (ultraTriggered && now >= servoReturnTime) {
    servo1.write(0);
    ultraTriggered = false;
    Serial.println(F("ULTRA: 10s elapsed -> S1=0"));
  }
  // ─────────────────────────────────────────────────────────

  WiFiClient client = server.available();
  if (!client) return;

  // Read first HTTP line
  unsigned long t = millis();
  String req = "";
  while (client.connected() && millis()-t < 2000) {
    if (client.available()) {
      char c = client.read();
      if (c=='\r') continue;
      if (c=='\n') break;
      if (req.length() < 120) req += c;
    }
  }
  // Drain headers
  t = millis();
  while (client.connected() && millis()-t < 400) {
    if (client.available()) { client.read(); t=millis(); }
  }

  Serial.println(req);

  // Root page
  if (req.indexOf("GET / ") >= 0 || req.length() < 8) {
    sendPage(client);

  // NEW: Ultrasonic status endpoint
  // Returns: "distanceCm,countdown"  countdown=-1 when idle
  } else if (req.indexOf("/ultra") >= 0) {
    int countdown = -1;
    if (ultraTriggered) {
      unsigned long rem = (millis() < servoReturnTime)
                          ? (servoReturnTime - millis()) / 1000UL
                          : 0;
      countdown = (int)rem;
    }
    sendOK(client, String(lastDistanceCm) + "," + String(countdown));

  // Both servos together (single request fallback)
  } else if (req.indexOf("/both?") >= 0) {
    int i=req.indexOf("a="); i+=2;
    String n="";
    while(i<(int)req.length()&&isDigit(req[i])) n+=req[i++];
    int a=n.toInt();
    if(a>=0&&a<=180){
      servo1.write(a);
      servo2.write(180-a);
      Serial.print(F("BOTH=")); Serial.println(a);
    }
    sendOK(client, "BOTH="+String(a));

  // Servo 1
  } else if (req.indexOf("/s1?") >= 0) {
    int i=req.indexOf("a="); i+=2;
    String n="";
    while(i<(int)req.length()&&isDigit(req[i])) n+=req[i++];
    int a=n.toInt();
    if(a>=0&&a<=180){ servo1.write(a); Serial.print(F("S1=")); Serial.println(a); }
    sendOK(client, "S1="+String(a));

  // Servo 2
  } else if (req.indexOf("/s2?") >= 0) {
    int i=req.indexOf("a="); i+=2;
    String n="";
    while(i<(int)req.length()&&isDigit(req[i])) n+=req[i++];
    int a=n.toInt();
    if(a>=0&&a<=180){ servo2.write(180-a); Serial.print(F("S2 requested=")); Serial.print(a); Serial.print(F(" actual=")); Serial.println(180-a); }
    sendOK(client, "S2="+String(a));

  // Speed
  } else if (req.indexOf("/spd?") >= 0) {
    int i=req.indexOf("v="); i+=2;
    String n="";
    while(i<(int)req.length()&&isDigit(req[i])) n+=req[i++];
    motorSpeed=constrain(n.toInt(),0,255);
    Serial.print(F("SPD=")); Serial.println(motorSpeed);
    sendOK(client, "SPD="+String(motorSpeed));

  // Motors
  } else if (req.indexOf("/forward") >= 0) {
    analogWrite(ENA,motorSpeed);
    digitalWrite(IN1,HIGH);digitalWrite(IN2,LOW);
    digitalWrite(IN3,HIGH);digitalWrite(IN4,LOW);
    Serial.println(F("FWD"));
    sendOK(client,"FWD");

  } else if (req.indexOf("/back") >= 0) {
    analogWrite(ENA,motorSpeed);
    digitalWrite(IN1,LOW);digitalWrite(IN2,HIGH);
    digitalWrite(IN3,LOW);digitalWrite(IN4,HIGH);
    Serial.println(F("BWD"));
    sendOK(client,"BWD");

  } else if (req.indexOf("/left") >= 0) {
    analogWrite(ENA,motorSpeed);
    digitalWrite(IN1,LOW);digitalWrite(IN2,HIGH);
    digitalWrite(IN3,HIGH);digitalWrite(IN4,LOW);
    Serial.println(F("LEFT"));
    sendOK(client,"LEFT");

  } else if (req.indexOf("/right") >= 0) {
    analogWrite(ENA,motorSpeed);
    digitalWrite(IN1,HIGH);digitalWrite(IN2,LOW);
    digitalWrite(IN3,LOW);digitalWrite(IN4,HIGH);
    Serial.println(F("RIGHT"));
    sendOK(client,"RIGHT");

  } else if (req.indexOf("/stop") >= 0) {
    digitalWrite(IN1,LOW);digitalWrite(IN2,LOW);
    digitalWrite(IN3,LOW);digitalWrite(IN4,LOW);
    analogWrite(ENA,0);
    Serial.println(F("STOP"));
    sendOK(client,"STOP");

  } else {
    sendOK(client,"?");
  }

  delay(10);
  client.stop();
}
