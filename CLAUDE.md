# ArgusAR（原 AlvaAR 升级项目）— 构建与协作指南

仓库：https://github.com/WangZhewei1027/ArgusAR （独立仓库，upstream=alanross/AlvaAR 仅作参照）。Demo 部署在 https://wangzhewei1027.github.io/ArgusAR/ 。

浏览器端 wasm 视觉 SLAM（OV²SLAM/ORB-SLAM2 血统，GPLv3）。改造总计划见 `ROADMAP.md`（6 阶段：SIMD 地基 → 平面系统 → SLAM 稳健性 → 米制尺度 → GPU 前端 → 多线程发布）。

## 构建环境（必读，有坑）

1. **Emscripten 固定 3.1.40**（新版本与老依赖不兼容，勿升级）：
   ```
   git clone --depth 1 https://github.com/emscripten-core/emsdk.git ~/Development/emsdk
   cd ~/Development/emsdk && ./emsdk install 3.1.40 && ./emsdk activate 3.1.40
   source ~/Development/emsdk/emsdk_env.sh
   ```
   路径必须是 `~/Development/emsdk` —— `src/libs/build.sh` 和 `src/slam/CMakeLists.txt` 硬编码了它。
2. **cmake ≥ 4 必须设**：`export CMAKE_POLICY_VERSION_MINIMUM=3.5`（老项目 cmake_minimum_required < 3.5 会被拒）。
3. 需要 `python3`（OpenCV js 构建脚本）；`build.sh` 已改为 python3 / 无 -march=native / -j6。

## 构建顺序

```
cd src/libs && ./build.sh          # 全部 7 个依赖 → build/*.a，约 40–60 分钟
cd ../slam && mkdir -p build && cd build
emcmake cmake .. && emmake make install -j6   # → dist/argus_ar.{js,wasm} + examples/public/assets/argus_ar.{js,wasm}
```

- 依赖库产物 `src/libs/build/` 被 gitignore（`build/*`），每个新环境需重编一次，之后增量很快。
- `src/libs/eigen/scripts/buildtests.in` 是我们补的 stub（上游裁剪丢失），被 eigen 自身 gitignore 误伤，改动它需 `git add -f`。
- SIMD 重编：`BUILD_TYPE=SIMD ./build.sh`（环境变量注入；产物在 build_simd/）；多线程同理 `THREADS` → build_threads/。
- **重要发现（2026-08）**：OpenCV 4.5.5 `--simd` 的手写 wasm 内核（intrin_wasm.hpp）在 emsdk 3.1.40 下**结果错误**（SLAM 跟踪率 0%）。`intrin_wasm.hpp` 已打补丁（补 `#include <emscripten/version.h>`，修版本宏失效），可编译通过，但 SIMD 路径运行时仍不正确。**生产构建用混合方案**：OpenCV 取 build/（标量），其余 6 库取 build_simd/——`src/libs/build_mix/` 放符号链接（gitignored，按此重建），主项目 `emcmake cmake .. -DLIBS_BUILD_FOLDER=../libs/build_mix`。根治靠升级 vendored OpenCV ≥4.7 或 Phase 5 GPU 前端替掉这些内核。
- 主项目现输出**拆分产物**：`argus_ar.js`（~130KB）+ `argus_ar.wasm`（~4MB），两者必须同目录部署。JS API 类名为 `ArgusAR`，wasm 工厂 `ArgusARWasm`，worker 为 `assets/argus_worker.js`。
- **js/wasm 版本锁定**：system.js 会把自己 URL 的 `?查询串` 通过 locateFile 传给 wasm 请求，两者永远同版本；所有 demo 以 `?v=<tag>` 引入 argus_ar.js，默认 tag 统一写死在各 demo（当前 `b3`）——**改动 wasm/system.js 后必须全局把 tag 升一号**（防 CDN/浏览器缓存拿到新旧混搭，症状=静默无渲染）。
- **CMake 已把 `src/system.js` 声明为 LINK_DEPENDS**：只改 system.js 也会触发重链。此前只改 system.js 不重链、install 却刷新 mtime，产物看似新实为旧，极难察觉。
- demo 验收标准：`video_planes.html?drive=seek&v=<新tag>`（seek 驱动确定性回放，不依赖页面可见性；隐藏面板中 rAF/video.play 均被浏览器挂起，绝不能用普通模式做自动化验收）。基准页 `examples/public/bench.html?label=X&v=Y`（v 是防缓存版本号，改了 wasm 必须换新值）。

## 验证

- 本地：`python3 -m http.server 8123 --directory examples` → `/public/video.html`（桌面视频回放，验证 SLAM 跟踪+特征点+轨迹）。
- 手机：GitHub Pages 部署后访问 `/examples/public/camera.html`（需 HTTPS 才能开摄像头）。
- 正常指标：video demo 每帧 slam 2–3ms，内存 ~25MB，无 console 报错。

## 关键代码位置

- SLAM 入口/绑定：`src/slam/src/system.cpp`、`embind.cpp`；JS 包装：`src/system.js`（编译时经 --extern-post-js 注入）
- 平面检测（Phase 2 已完成）：`plane_detector.hpp/cpp` 的 PlaneManager——顺序 RANSAC 多平面、ID 持久化+EMA 平滑+淘汰、凸包边界；System::getPlanes/hitTest 序列化到共享内存，JS 端 `getPlanes()/hitTest()/createAnchor()`。旧的 `processPlane`/`findPlane` 保留兼容。管线回归测试页：`examples/public/sandbox/planes_test.html?v=N`（seek 驱动，不依赖页面可见性；注意隐藏页面里 video.play() 会被浏览器节能暂停，rAF 也会挂起——验证一律用 seek 驱动页）。平面分类（horizontal/vertical）相对初始相机系，Phase 4 接入重力后才是真实方向。
- IMU（现状为空壳，数据被丢弃）：`system.cpp` 的 `findCameraPoseWithIMU`
- 回环检测库已编译未接入：`src/libs/ibow_lcd`、`obindex2`

## 测试素材与验证陷阱（2026-08-11 补）

- 测试视频：`assets/video.mp4`（桌面横扫，基线 tracked 0.993）+ `assets/tour.mp4`（看房前进式漫游，Pexels id 7578552 by Kindel Media，已重编码全内帧，tracked 0.779）。
- **隐藏浏览器面板会冻结/节流一切媒体解码**：video seek 返回旧帧（静默!）、Image.decode() 永不返回、img.onload 被限到 ~1/s、后台顺序 fetch 也被限速。唯一可靠通路：**并发预取 blob + createImageBitmap**。回归一律用 `planes_test.html?frames=<dir>&count=N`（帧序列模式）；帧目录用 ffmpeg 从 mp4 现场生成（gitignored）：`ffmpeg -i assets/xxx.mp4 -vf fps=30 -q:v 5 assets/xxx_frames/f_%04d.jpg`。
- 回环遥测：`getLoopStats()` → {keyframes, candidates, verifyRejects, loops, lastStatus}；camera HUD 实时显示。`setDebug(true)` 开 C++ 逐帧日志。
- OpenGV RANSAC 已固定种子（state.multiViewRandomEnabled_=false），全管线确定性。
