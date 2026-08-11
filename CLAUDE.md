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
emcmake cmake .. && emmake make install -j6   # → dist/alva_ar.js + examples/public/assets/alva_ar.js
```

- 依赖库产物 `src/libs/build/` 被 gitignore（`build/*`），每个新环境需重编一次，之后增量很快。
- `src/libs/eigen/scripts/buildtests.in` 是我们补的 stub（上游裁剪丢失），被 eigen 自身 gitignore 误伤，改动它需 `git add -f`。
- SIMD 重编：`build.sh` 顶部 `BUILD_TYPE="SIMD"`；多线程：`BUILD_TYPE="THREADS"`（产物分别在 build_simd/ build_threads/）。

## 验证

- 本地：`python3 -m http.server 8123 --directory examples` → `/public/video.html`（桌面视频回放，验证 SLAM 跟踪+特征点+轨迹）。
- 手机：GitHub Pages 部署后访问 `/examples/public/camera.html`（需 HTTPS 才能开摄像头）。
- 正常指标：video demo 每帧 slam 2–3ms，内存 ~25MB，无 console 报错。

## 关键代码位置

- SLAM 入口/绑定：`src/slam/src/system.cpp`、`embind.cpp`；JS 包装：`src/system.js`（编译时经 --extern-post-js 注入）
- 平面检测（待重写）：`system.cpp` 的 `processPlane`（现状：单平面 RANSAC、仅水平面、无持久化）
- IMU（现状为空壳，数据被丢弃）：`system.cpp` 的 `findCameraPoseWithIMU`
- 回环检测库已编译未接入：`src/libs/ibow_lcd`、`obindex2`
