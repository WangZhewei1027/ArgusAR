# ArgusAR

ArgusAR is a realtime visual SLAM engine for the open web — WebAssembly-powered world tracking, plane detection, and (in progress) metric scale estimation, targeting the browsers where native AR (ARKit/ARCore/WebXR) isn't available, such as iOS Safari and in-app webviews.

It began as a fork of [AlvaAR](https://github.com/alanross/AlvaAR) by Alan Ross, which is itself a heavily modified version of the [OV²SLAM](https://github.com/ov2slam/ov2slam) and [ORB-SLAM2](https://github.com/raulmur/ORB_SLAM2) projects. All of them are GPLv3, and so is this project. See [ROADMAP.md](ROADMAP.md) for the upgrade plan and current status.

![image](examples/public/assets/image.gif)


## Examples
The examples use [ThreeJS](https://threejs.org/) to apply and render the estimated camera pose to a 3d environment.  

[Video Demo](https://wangzhewei1027.github.io/ArgusAR/examples/public/video.html): A desktop browser version using a video file as input.  
[Camera Demo](https://wangzhewei1027.github.io/ArgusAR/examples/public/camera.html): The mobile version will access the device camera as input.

<img width="75" src="examples/public/assets/qr.png">

### Run with http server
To run the examples on your local machine, start a simple http server in the examples/ folder:

`$: python 2: python -m SimpleHTTPServer 8080` or   
`$: python 3: python -m http.server 8080` or  
`$: emrun --browser chrome ./`

Then open [http://localhost:8080/public/video.html](http://localhost:8080/public/video.html]) in your browser.

### Run with https server
To run the examples on another device in your local network, they must be served via https. For convenience, a simple https server was added to this project – do not use for production.

#### 1) Install server dependencies
```
    $: cd ./ArgusAR/examples/
    $: npm install
```

#### 2) Generate self-signed certificate
```
    $: cd ./ArgusAR/examples
    $: mkdir ssl/
    $: cd ssl/
    $: openssl req -nodes -new -x509 -keyout key.pem -out cert.pem
```

#### 3) Run
```
    $: cd ./ArgusAR/examples/
    $: nvm use 13.2
    $: npm start
``` 
Then open [https://YOUR_IP:443/video.html](https://YOUR_IP:443/video.html]) in your browser.
If met with a <b>ERR_CERT_INVALID</b> error in Chrome,
try typing <i>badidea</i> or <i>thisisunsafe</i> directly in Chrome on the same page.
Don’t do this unless the site is one you trust or develop.


## Usage

This code shows how to send image data to ArgusAR to compute the camera pose.

```javascript
import { ArgusAR } from 'argus_ar.js';

const videoOrWebcam = /*...*/;

const width = videoOrWebcam.width;
const height = videoOrWebcam.height;

const canvas = document.getElementById( 'canvas' );
const ctx = canvas.getContext( '2d' );

canvas.width = width;
canvas.height = height;

const argus = await ArgusAR.Initialize( width, height );

function loop()
{
    ctx.clearRect( 0, 0, width, height );
    ctx.drawImage( videoOrWebcam, 0, 0, width, height );
    
    const frame = ctx.getImageData( 0, 0, width, height );
    
    // cameraPose holds the rotation/translation information where the camera is estimated to be
    const cameraPose = argus.findCameraPose( frame );
    
    // planePose holds the rotation/translation information of a detected plane
    const planePose = argus.findPlane();
    
    // The tracked points in the frame
    const points = argus.getFramePoints();

    for( const p of points )
    {
        ctx.fillRect( p.x, p.y, 2, 2 );
    }
};
```


## Build

### Prerequisites

#### Emscripten
Ensure [Emscripten](https://emscripten.org/docs/getting_started/Tutorial.html) is installed and activated in your session.

```
    $: source [PATH]/emsdk/emsdk_env.sh 
    $: emcc -v
```

#### C++11 or Higher
Alva makes use of C++11 features and should thus be compiled with a C++11 or higher flag.

### Dependencies

| Dependency             | Description                                                                                                                                                                                                                                                                                                                                                                                                                         |
|------------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| Eigen3                 | Download Eigen 3.4. Find all releases [here](https://eigen.tuxfamily.org/index.php?title=Main_Page).This project has been tested with 3.4.0                                                                                                                                                                                                                                                                                         |
| OpenCV                 | Download OpenCV 4.5. Find all releases [here](https://opencv.org/releases/).This project has been tested with [4.5.5](https://github.com/opencv/opencv/archive/4.5.5.zip).                                                                                                                                                                                                                                                          |
| iBoW-LCD               | A modified version of [iBoW-LCD](https://github.com/emiliofidalgo/ibow-lcd) is included in the libs folder. It has been turned into a static shared lib. Same goes for [OBIndex2](https://github.com/emiliofidalgo/obindex2), the required dependency for iBoW-LCD. Check the lcdetector.h and lcdetector.cc files to see the modifications w.r.t. to the original code. Both CMakeList have been adjusted to work with Emscripten. |
| Sophus                 | [Sophus](https://github.com/strasdat/Sophus) is used for _*SE(3), SO(3)*_ elements representation.                                                                                                                                                                                                                                                                                                                                  |
| Ceres Solver           | [Ceres](https://github.com/ceres-solver/ceres-solver) is used for optimization related operations such as PnP, Bundle Adjustment or PoseGraph Optimization. Note that [Ceres dependencies](http://ceres-solver.org/installation.html) are still required.                                                                                                                                                                           |
| OpenGV                 | [OpenGV](https://github.com/laurentkneip/opengv) is used for Multi-View-Geometry (MVG) operations.                                                                                                                                                                                                                                                                                                                                  |

#### Build Dependencies
For convenience, a copy of all required libraries has been included in the libs/ folder. Run the following script to compile all libraries to wasm modules which can be linked into the main project.

```
    $: cd ./ArgusAR/src/libs/
    $: ./build.sh
```

#### Build Project

Run the following in your shell before invoking emcmake or emmake:

```
    $: [PATH]/emsdk/emsdk_env.sh
```

Then, run the following:

```
    $: cd ./ArgusAR/src/slam
    $: mkdir build/
    $: cd build/
    $: emcmake cmake .. 
    $: emmake make install
```


## Roadmap
- [ ] Improve the initialisation phase to be more stable and predictable.
- [ ] Move feature extraction and tracking to GPU.
- [ ] Blend visual SLAM with IMU data to increase robustness. 


## License

ArgusAR is released under the [GPLv3 license](https://www.gnu.org/licenses/gpl-3.0.txt).  

OV²SLAM and ORB-SLAM2 are both released under the [GPLv3 license](https://www.gnu.org/licenses/gpl-3.0.txt). Please see 3rd party dependency licenses in libs/.


## Credits

Original author of AlvaAR, which this project is based on: Alan Ross ([@alan_ross](https://twitter.com/alan_ross), [alanross/AlvaAR](https://github.com/alanross/AlvaAR))  
Built on [OV²SLAM](https://github.com/ov2slam/ov2slam) and [ORB-SLAM2](https://github.com/raulmur/ORB_SLAM2).