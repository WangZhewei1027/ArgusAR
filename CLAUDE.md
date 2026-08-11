# AlvaAR 升级项目 — 构建与协作指南

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
- SIMD 重编：`BUILD_TYPE=SIMD ./build.sh`（环境变量注入；产物在 build_simd/）；多线程同理 `THREADS` → build_threads/。
- **重要发现（2026-08）**：OpenCV 4.5.5 `--simd` 的手写 wasm 内核（intrin_wasm.hpp）在 emsdk 3.1.40 下**结果错误**（SLAM 跟踪率 0%）。`intrin_wasm.hpp` 已打补丁（补 `#include <emscripten/version.h>`，修版本宏失效），可编译通过，但 SIMD 路径运行时仍不正确。**生产构建用混合方案**：OpenCV 取 build/（标量），其余 6 库取 build_simd/——`src/libs/build_mix/` 放符号链接（gitignored，按此重建），主项目 `emcmake cmake .. -DLIBS_BUILD_FOLDER=../libs/build_mix`。根治靠升级 vendored OpenCV ≥4.7 或 Phase 5 GPU 前端替掉这些内核。
- 主项目现输出**拆分产物**：`alva_ar.js`（~130KB）+ `alva_ar.wasm`（~4MB），两者必须同目录部署。基准页 `examples/public/bench.html?label=X&v=Y`（v 是防缓存版本号，改了 wasm 必须换新值）。

## 验证

- 本地：`python3 -m http.server 8123 --directory examples` → `/public/video.html`（桌面视频回放，验证 SLAM 跟踪+特征点+轨迹）。
- 手机：GitHub Pages 部署后访问 `/examples/public/camera.html`（需 HTTPS 才能开摄像头）。
- 正常指标：video demo 每帧 slam 2–3ms，内存 ~25MB，无 console 报错。

## 关键代码位置

- SLAM 入口/绑定：`src/slam/src/system.cpp`、`embind.cpp`；JS 包装：`src/system.js`（编译时经 --extern-post-js 注入）
- 平面检测（待重写）：`system.cpp` 的 `processPlane`（现状：单平面 RANSAC、仅水平面、无持久化）
- IMU（现状为空壳，数据被丢弃）：`system.cpp` 的 `findCameraPoseWithIMU`
- 回环检测库已编译未接入：`src/libs/ibow_lcd`、`obindex2`
