#include <Arduino.h>
#include <WiFi.h>
#include "esp_camera.h"
#include "esp_http_server.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/soc.h"

// AI-Thinker ESP32-CAM pinout, matching esp32_cam_gpio_config.h in this package.
#define PWDN_GPIO_NUM  32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM   0
#define SIOD_GPIO_NUM  26
#define SIOC_GPIO_NUM  27
#define Y9_GPIO_NUM    35
#define Y8_GPIO_NUM    34
#define Y7_GPIO_NUM    39
#define Y6_GPIO_NUM    36
#define Y5_GPIO_NUM    21
#define Y4_GPIO_NUM    19
#define Y3_GPIO_NUM    18
#define Y2_GPIO_NUM     5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM  23
#define PCLK_GPIO_NUM  22

// ESP32-CAM creates this 2.4 GHz Wi-Fi hotspot for the phone.
static const char *AP_SSID = "ESP32-CAM";
static const char *AP_PASSWORD = "12345678";

static httpd_handle_t server = nullptr;

static const char INDEX_HTML[] PROGMEM = R"HTML(
<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover">
  <title>ESP32-CAM 图传</title>
  <style>
    :root{color-scheme:dark;--blue:#3b82f6;--red:#ef4444}
    *{box-sizing:border-box}
    body{margin:0;min-height:100svh;display:grid;place-items:center;background:#07111f;
      color:#eef6ff;font-family:system-ui,-apple-system,"Microsoft YaHei",sans-serif}
    main{width:min(94vw,900px);text-align:center}
    h1{font-size:clamp(24px,6vw,38px);margin:0 0 8px}
    #status{height:28px;color:#a9bbd0;margin-bottom:16px}
    #stage{display:none;position:relative;overflow:hidden;border-radius:18px;background:#000;
      box-shadow:0 18px 60px #0008;aspect-ratio:4/3}
    #camera,#recordCanvas{position:absolute;inset:0;width:100%;height:100%;object-fit:contain}
    #recordCanvas{visibility:hidden;pointer-events:none}
    #rec{display:none;position:absolute;left:14px;top:14px;padding:7px 11px;border-radius:999px;
      background:#b91c1ce6;font-weight:700;font-size:13px}
    #rec:before{content:"";display:inline-block;width:9px;height:9px;margin-right:7px;
      border-radius:50%;background:white;animation:pulse 1s infinite}
    @keyframes pulse{50%{opacity:.25}}
    .buttons{display:flex;justify-content:center;gap:18px;margin-top:22px}
    button{width:132px;min-height:52px;border:0;border-radius:14px;color:white;font-size:18px;
      font-weight:700;box-shadow:0 8px 22px #0005;touch-action:manipulation}
    #start{background:var(--blue)} #stop{background:var(--red)}
    button:disabled{filter:grayscale(.8);opacity:.4}
    #tip{font-size:13px;color:#7f93aa;margin-top:18px;line-height:1.6}
  </style>
</head>
<body>
<main>
  <h1>ESP32-CAM 图传</h1>
  <div id="status">摄像头已就绪</div>
  <div id="stage">
    <img id="camera" alt="摄像头实时画面">
    <canvas id="recordCanvas"></canvas>
    <span id="rec">正在录像 <b id="clock">00:00</b></span>
  </div>
  <div class="buttons">
    <button id="start">开始</button>
    <button id="stop" disabled>结束</button>
  </div>
  <div id="tip">结束后将自动生成并下载 MP4；录像期间请勿锁屏或切换应用。</div>
</main>
<script>
(() => {
  const $ = id => document.getElementById(id);
  const img=$('camera'), canvas=$('recordCanvas'), ctx=canvas.getContext('2d');
  const start=$('start'), stop=$('stop'), stage=$('stage'), rec=$('rec');
  let recorder, chunks=[], drawing=false, timer, startedAt=0, mime='';

  function chooseMp4Mime(){
    if(!window.MediaRecorder) return '';
    const choices=['video/mp4;codecs=avc1.42E01E','video/mp4;codecs=h264','video/mp4'];
    return choices.find(x => MediaRecorder.isTypeSupported(x)) || '';
  }
  function status(text){ $('status').textContent=text; }
  function tick(){
    const sec=Math.floor((Date.now()-startedAt)/1000);
    $('clock').textContent=String(sec/60|0).padStart(2,'0')+':'+String(sec%60).padStart(2,'0');
  }
  function draw(){
    if(!drawing) return;
    if(img.complete && img.naturalWidth){
      try{ ctx.drawImage(img,0,0,canvas.width,canvas.height); }catch(e){}
    }
    requestAnimationFrame(draw);
  }
  function download(blob){
    const stamp=new Date().toISOString().replace(/[:.]/g,'-');
    const url=URL.createObjectURL(blob);
    const a=document.createElement('a');
    a.href=url; a.download='ESP32-CAM-'+stamp+'.mp4';
    document.body.appendChild(a); a.click(); a.remove();
    setTimeout(()=>URL.revokeObjectURL(url),30000);
  }

  start.onclick=async () => {
    mime=chooseMp4Mime();
    if(!mime){
      status('当前浏览器不能录制 MP4，请使用最新版 Chrome 或 Safari');
      alert('此浏览器不支持网页直接生成 MP4。请升级系统浏览器，Android 使用最新版 Chrome，iPhone 使用 Safari。');
      return;
    }
    start.disabled=true; status('正在连接摄像头…');
    img.src='/stream?t='+Date.now();
    try{
      await new Promise((resolve,reject)=>{
        const timeout=setTimeout(()=>reject(new Error('连接超时')),10000);
        img.onload=()=>{clearTimeout(timeout);resolve()};
        img.onerror=()=>{clearTimeout(timeout);reject(new Error('画面连接失败'))};
      });
      canvas.width=img.naturalWidth || 1280;
      canvas.height=img.naturalHeight || 720;
      chunks=[]; drawing=true; draw();
      const stream=canvas.captureStream(15);
      recorder=new MediaRecorder(stream,{mimeType:mime,videoBitsPerSecond:2500000});
      recorder.ondataavailable=e=>{if(e.data.size)chunks.push(e.data)};
      recorder.onerror=e=>{status('录像错误：'+(e.error?.message||'未知错误'))};
      recorder.onstop=()=>{
        const blob=new Blob(chunks,{type:mime});
        if(blob.size){download(blob);status('MP4 已生成（'+(blob.size/1048576).toFixed(1)+' MB）')}
        else status('录像为空，请重新尝试');
        chunks=[]; start.disabled=false;
      };
      recorder.start(1000);
      stage.style.display='block'; rec.style.display='block';
      stop.disabled=false; startedAt=Date.now(); tick(); timer=setInterval(tick,500);
      status('实时画面传输中');
    }catch(e){
      img.src=''; drawing=false; start.disabled=false;
      status(e.message+'，请检查 ESP32-CAM');
    }
  };

  stop.onclick=()=>{
    stop.disabled=true; drawing=false; clearInterval(timer);
    rec.style.display='none'; img.src='';
    status('正在封装 MP4，请稍候…');
    if(recorder && recorder.state!=='inactive') recorder.stop();
  };
  addEventListener('beforeunload',()=>{img.src=''});
})();
</script>
</body>
</html>
)HTML";

static esp_err_t indexHandler(httpd_req_t *req) {
  httpd_resp_set_type(req, "text/html; charset=utf-8");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  return httpd_resp_send(req, INDEX_HTML, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t streamHandler(httpd_req_t *req) {
  static const char *CONTENT_TYPE = "multipart/x-mixed-replace;boundary=frame";
  static const char *BOUNDARY = "\r\n--frame\r\n";
  static const char *PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";
  char header[64];
  esp_err_t result = httpd_resp_set_type(req, CONTENT_TYPE);
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

  while (result == ESP_OK) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Camera capture failed");
      result = ESP_FAIL;
      break;
    }
    result = httpd_resp_send_chunk(req, BOUNDARY, strlen(BOUNDARY));
    if (result == ESP_OK) {
      const size_t len = snprintf(header, sizeof(header), PART, fb->len);
      result = httpd_resp_send_chunk(req, header, len);
    }
    if (result == ESP_OK) {
      result = httpd_resp_send_chunk(req, reinterpret_cast<const char *>(fb->buf), fb->len);
    }
    esp_camera_fb_return(fb);
    vTaskDelay(1);
  }
  return result;
}

static bool initCamera() {
  camera_config_t c = {};
  c.ledc_channel = LEDC_CHANNEL_0;
  c.ledc_timer = LEDC_TIMER_0;
  c.pin_d0 = Y2_GPIO_NUM; c.pin_d1 = Y3_GPIO_NUM;
  c.pin_d2 = Y4_GPIO_NUM; c.pin_d3 = Y5_GPIO_NUM;
  c.pin_d4 = Y6_GPIO_NUM; c.pin_d5 = Y7_GPIO_NUM;
  c.pin_d6 = Y8_GPIO_NUM; c.pin_d7 = Y9_GPIO_NUM;
  c.pin_xclk = XCLK_GPIO_NUM; c.pin_pclk = PCLK_GPIO_NUM;
  c.pin_vsync = VSYNC_GPIO_NUM; c.pin_href = HREF_GPIO_NUM;
  c.pin_sscb_sda = SIOD_GPIO_NUM; c.pin_sscb_scl = SIOC_GPIO_NUM;
  c.pin_pwdn = PWDN_GPIO_NUM; c.pin_reset = RESET_GPIO_NUM;
  c.xclk_freq_hz = 20000000;
  c.pixel_format = PIXFORMAT_JPEG;

  if (psramFound()) {
    c.frame_size = FRAMESIZE_HD;       // 1280 x 720
    c.jpeg_quality = 12;               // lower is better quality
    c.fb_count = 2;
    c.grab_mode = CAMERA_GRAB_LATEST;
    c.fb_location = CAMERA_FB_IN_PSRAM;
  } else {
    c.frame_size = FRAMESIZE_VGA;
    c.jpeg_quality = 14;
    c.fb_count = 1;
  }
  const esp_err_t error = esp_camera_init(&c);
  if (error != ESP_OK) {
    Serial.printf("Camera init failed: 0x%x\n", error);
    return false;
  }
  sensor_t *sensor = esp_camera_sensor_get();
  sensor->set_vflip(sensor, 1);
  sensor->set_hmirror(sensor, 1);
  return true;
}

static bool startWebServer() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.max_open_sockets = 5;
  config.lru_purge_enable = true;
  config.stack_size = 8192;
  if (httpd_start(&server, &config) != ESP_OK) return false;

  httpd_uri_t indexUri = { .uri="/", .method=HTTP_GET, .handler=indexHandler, .user_ctx=nullptr };
  httpd_uri_t streamUri = { .uri="/stream", .method=HTTP_GET, .handler=streamHandler, .user_ctx=nullptr };
  return httpd_register_uri_handler(server, &indexUri) == ESP_OK &&
         httpd_register_uri_handler(server, &streamUri) == ESP_OK;
}

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  Serial.begin(115200);
  Serial.setDebugOutput(false);
  delay(300);
  Serial.println("\nESP32-CAM video transmitter starting...");

  if (!initCamera()) return;

  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);
  const IPAddress localIp(192, 168, 4, 1);
  const IPAddress gateway(192, 168, 4, 1);
  const IPAddress subnet(255, 255, 255, 0);
  if (!WiFi.softAPConfig(localIp, gateway, subnet)) {
    Serial.println("Wi-Fi AP address configuration failed");
    return;
  }
  if (!WiFi.softAP(AP_SSID, AP_PASSWORD, 6, false, 2)) {
    Serial.println("Wi-Fi AP start failed");
    return;
  }
  if (!startWebServer()) {
    Serial.println("Web server start failed");
    return;
  }
  Serial.printf("Wi-Fi hotspot: %s\n", AP_SSID);
  Serial.printf("Wi-Fi password: %s\n", AP_PASSWORD);
  Serial.print("Open in browser: http://");
  Serial.println(WiFi.softAPIP());
}

void loop() {
  delay(1000);
}

