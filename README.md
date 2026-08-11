# ArgusAR

ArgusAR is a realtime **visual SLAM engine for the open web** — WebAssembly-powered world tracking, multi-plane detection, and (in progress) metric scale estimation. It targets the places native AR can't reach: **iOS Safari, in-app webviews, and any browser a QR code can open** — no app install, no ARKit/ARCore/WebXR required.

![image](examples/public/assets/image.gif)

## Features

- **World tracking** — monocular visual SLAM (OV²SLAM / ORB-SLAM2 lineage), compiled to WASM, ~5ms per frame on a laptop
- **Multi-plane detection** — sequential RANSAC over the live map with persistent plane identities, EMA-smoothed poses, convex-hull boundaries, and horizontal / vertical / arbitrary classification
- **Hit testing** — cast a ray from any screen pixel onto tracked planes to place content
- **Web Worker pipeline** — SLAM runs off the main thread; the UI stays at full framerate
- **Split streaming artifacts** — ~130KB JS loader + streaming-compiled `.wasm`, with a version-locking scheme so cached JS/wasm pairs can never mismatch
- **Adaptive quality** — steps processing resolution under load to keep older phones interactive

See [ROADMAP.md](ROADMAP.md) for what's done and what's next (SLAM robustness, loop closure, metric scale via IMU + depth networks, WebGPU frontend, multithreading).

## Try it

| Demo | What it shows |
|---|---|
| [Camera](https://wangzhewei1027.github.io/ArgusAR/examples/public/camera.html) (open on a phone) | Live camera tracking + plane detection; tap to place objects on detected planes |
| [Planes](https://wangzhewei1027.github.io/ArgusAR/examples/public/video_planes.html) | Video playback with detected plane polygons + hit-test placement |
| [Video](https://wangzhewei1027.github.io/ArgusAR/examples/public/video.html) | Classic tracking demo on a prerecorded video |
| [Worker](https://wangzhewei1027.github.io/ArgusAR/examples/public/video_worker.html) | SLAM in a Web Worker with adaptive resolution |
| [Benchmark](https://wangzhewei1027.github.io/ArgusAR/examples/public/bench.html) | Deterministic per-frame timing stats (seek-driven, frame-exact) |

Scan to open the camera demo:

<img width="75" src="examples/public/assets/qr.png">

## Usage

```javascript
import { ArgusAR } from 'argus_ar.js';

const argus = await ArgusAR.Initialize( width, height );

function loop()
{
    const frame = ctx.getImageData( 0, 0, width, height );

    // camera pose (column-major 4x4, null while lost/initializing)
    const cameraPose = argus.findCameraPose( frame );

    // all persistently tracked planes:
    // { id, type: 'horizontal'|'vertical'|'arbitrary', inliers,
    //   pose, extent: {u, v}, hull: [{u, v}, ...] }
    const planes = argus.getPlanes();

    // ray-cast a screen pixel onto the planes -> { planeId, pose } | null
    const hit = argus.hitTest( x, y );

    // 2D feature points of the current frame (for debug overlays)
    const points = argus.getFramePoints();
}
```

Serve `argus_ar.js` and `argus_ar.wasm` from the same directory. Import the JS with a version tag (`argus_ar.js?v=123`) — it is automatically propagated to the `.wasm` request, so the pair always comes from the same build even through caches and CDNs.

To run the examples locally:

```
python3 -m http.server 8123 --directory examples
```

Then open [http://localhost:8123/public/video_planes.html](http://localhost:8123/public/video_planes.html). The camera demo needs HTTPS (browser requirement for camera access) — use the GitHub Pages deployment or any TLS-terminating host.

## Build

Prerequisites:

- **Emscripten 3.1.40** (pinned — newer versions break the vendored dependencies):
  ```
  git clone https://github.com/emscripten-core/emsdk.git ~/Development/emsdk
  cd ~/Development/emsdk && ./emsdk install 3.1.40 && ./emsdk activate 3.1.40
  source ~/Development/emsdk/emsdk_env.sh
  ```
- CMake (with CMake ≥ 4: `export CMAKE_POLICY_VERSION_MINIMUM=3.5`), Python 3

Build the vendored dependencies (Eigen, OpenCV, Ceres, Sophus, OpenGV, OBIndex2, iBoW-LCD — one-time, ~40-60 min):

```
cd src/libs
./build.sh                     # scalar baseline -> build/
BUILD_TYPE=SIMD ./build.sh     # SIMD variants   -> build_simd/
```

Then link the engine against the mixed set (scalar OpenCV + SIMD math libs — OpenCV 4.5.5's wasm SIMD kernels miscompile under modern LLVM; see CLAUDE.md):

```
cd src/libs && mkdir build_mix && ln -s ../build/opencv build_mix/opencv
for lib in Sophus ceres-solver eigen ibow_lcd obindex2 opengv; do ln -s ../build_simd/$lib build_mix/$lib; done

cd ../slam && mkdir build && cd build
emcmake cmake .. -DLIBS_BUILD_FOLDER=../libs/build_mix
emmake make install
```

Artifacts land in `dist/` and `examples/public/assets/`.

## License

ArgusAR is released under the [GPLv3 license](https://www.gnu.org/licenses/gpl-3.0.txt), inherited from the projects it builds on. Third-party dependency licenses live in `src/libs/`.

## Credits

ArgusAR began as a fork of [AlvaAR](https://github.com/alanross/AlvaAR) by Alan Ross ([@alan_ross](https://twitter.com/alan_ross)), which is itself a heavily modified version of [OV²SLAM](https://github.com/ov2slam/ov2slam) and [ORB-SLAM2](https://github.com/raulmur/ORB_SLAM2). The demo video and GIF above are from the original AlvaAR project.
