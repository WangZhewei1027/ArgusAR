class SharedMemory
{
    constructor( wasm, type, size )
    {
        const numBytes = size * type.BYTES_PER_ELEMENT;

        this.wasm = wasm;
        this.type = type;
        this.ptr = this.wasm._malloc( numBytes );

        this.TypedArray = null;

        if( wasm.HEAP8 === type ) this.TypedArray = Int8Array;
        if( wasm.HEAP16 === type ) this.TypedArray = Int16Array;
        if( wasm.HEAP32 === type ) this.TypedArray = Int32Array;
        if( wasm.HEAPU8 === type ) this.TypedArray = Uint8Array;
        if( wasm.HEAPU16 === type ) this.TypedArray = Uint16Array;
        if( wasm.HEAPU32 === type ) this.TypedArray = Uint32Array;
        if( wasm.HEAPF32 === type ) this.TypedArray = Float32Array;
        if( wasm.HEAPF64 === type ) this.TypedArray = Float64Array;

        if( this.TypedArray === null ) throw new Error( 'Unknown Data Type. Try "HEAP8"' );

        this.heap = new this.TypedArray( this.type.buffer, this.ptr, numBytes );
    }

    write( data )
    {
        this.heap.set( data );
    }

    read( length )
    {
        return new this.TypedArray( this.type.buffer, this.ptr, length );
    }

    dispose()
    {
        this.wasm._free( this.ptr );
        this.ptr = null;
        this.heap = null;
    }
}

class ArgusAR
{
    static async Initialize( width, height, fov = 45 )
    {
        const wasm = {};

        wasm.ready = ArgusARWasm().then( module => wasm.module = module );

        await wasm.ready;

        return new ArgusAR( wasm, width, height, fov );
    }

    constructor( wasm, width, height, fov )
    {
        this.wasm = wasm;
        this.intrinsics = this.getCameraIntrinsics( width, height, fov );

        this.memCam = new SharedMemory( wasm.module, wasm.module.HEAPF32, 16 );
        this.memObj = new SharedMemory( wasm.module, wasm.module.HEAPF32, 16 );
        this.memHit = new SharedMemory( wasm.module, wasm.module.HEAPF32, 16 );
        this.memPlanes = new SharedMemory( wasm.module, wasm.module.HEAPF32, 4096 );
        this.anchors = new Map();
        this.nextAnchorId = 1;
        this.memPts = new SharedMemory( wasm.module, wasm.module.HEAPU32, 4096 );
        this.memIMU = new SharedMemory( wasm.module, wasm.module.HEAPF64, 256 );
        this.memImg = new SharedMemory( wasm.module, wasm.module.HEAPU8, width * height * 4 );

        this.system = new wasm.module.System();
        this.system.configure(
            this.intrinsics.width,
            this.intrinsics.height,
            this.intrinsics.fx,
            this.intrinsics.fy,
            this.intrinsics.cx,
            this.intrinsics.cy,
            this.intrinsics.k1,
            this.intrinsics.k2,
            this.intrinsics.p1,
            this.intrinsics.p2
        );
    }

    getCameraIntrinsics( width, height, fov = 45, near = 0.001, far = 1000 )
    {
        // Camera Intrinsic Matrix
        // | fx s cx |
        // | 0 fy cy |
        // | 0  0  1 |

        // fx/fy : X/Y-axis focal length of the camera in pixels.
        // cx/cy : Coordinates of principal point. X/Y-axis optical center of the camera in pixels.

        // angleOfView = 90;
        // fx = width / ( Math.tan( deg2rad( angleOfView / 2 ) ) * 2 );
        // fy = fx;

        // fx = width / 2 / Math.tan( angleOfView / 2 * Math.PI / 360 );
        // fy = height / 2 / Math.tan( angleOfView / 2 * Math.PI / 360 );

        let fovH;
        let fovV;

        const aspect = (width / height);

        if( width > height )
        {
            fovH = fov * aspect;
            fovV = fov;
        }
        else
        {
            fovH = fov;
            fovV = fov * aspect;
        }

        const deg2rad = ( deg ) => deg * 0.01745329251994329576;

        const cx = width * 0.5;
        const cy = height * 0.5;
        const fx = (width * 0.5) / Math.tan( deg2rad( fovH * 0.5 ) );
        const fy = (height * 0.5) / Math.tan( deg2rad( fovV * 0.5 ) );
        const f = Math.min( fx, fy );

        return {
            width: width,       // image width
            height: height,     // image height
            fov: fov,           // field of view
            near: near,         // z-near
            far: far,           // z-far
            fx: f,              // X-axis focal length of the camera in pixels.
            fy: f,              // Y-axis focal length of the camera in pixels.
            cx: cx,             // Coordinates of principal point. X-axis optical center of the camera in pixels.
            cy: cy,             // Coordinates of principal point. Y-axis optical center of the camera in pixels.
            k1: 0,              // distortion coefficients k1 - p2
            k2: 0,
            k3: 0,
            p1: 0,
            p2: 0
        };
    }

    findCameraPoseWithIMU( frame, orientation, motion )
    {
        const imuData = [orientation.w, orientation.x, orientation.y, orientation.z, motion.length];

        while( motion.length )
        {
            const sample = motion.pop();
            imuData.push( sample.timestamp, sample.gx, sample.gy, sample.gz, sample.ax, sample.ay, sample.az );
        }

        this.memImg.write( frame.data );
        this.memIMU.write( imuData );

        const status = this.system.findCameraPoseWithIMU( this.memImg.heap.byteOffset, this.memIMU.heap.byteOffset, this.memCam.ptr );

        if( status === 1 )
        {
            // memCam data format = [
            //   r(0,0) r(0,1) r(0,2) 0
            //   r(1,0) r(1,1) r(1,2) 0
            //   r(2,0) r(2,1) r(2,2) 0
            //   tx     ty     tz     1
            // ]

            return this.memCam.read( 16 );
        }

        return null;
    }

    findCameraPose( frame )
    {
        this.memImg.write( frame.data );

        const status = this.system.findCameraPose( this.memImg.heap.byteOffset, this.memCam.ptr );

        if( status === 1 )
        {
            // memCam data format = [
            //   r(0,0) r(0,1) r(0,2) 0
            //   r(1,0) r(1,1) r(1,2) 0
            //   r(2,0) r(2,1) r(2,2) 0
            //   tx     ty     tz     1
            // ]

            return this.memCam.read( 16 );
        }
        else if( status === 2 )
        {
            // Tracking reset
        }
        else
        {
            // Tracking initializing
        }

        return null;
    }

    findPlane( numIterations = 250 )
    {
        const status = this.system.findPlane( this.memObj.ptr, numIterations );

        if( status === 1 )
        {
            return this.memObj.read( 16 );
        }

        return null;
    }

    /**
     * All persistently tracked planes.
     * Returns [{ id, type: 'horizontal'|'vertical'|'arbitrary', inliers,
     *            pose: Float32Array(16),        // column-major, Y = plane normal
     *            extent: { u, v },              // size in world units
     *            hull: [{x, y, z}, ...] }]      // world-space boundary polygon
     */
    getPlanes()
    {
        const numPlanes = this.system.getPlanes( this.memPlanes.ptr );

        const TYPE = [ 'horizontal', 'vertical', 'arbitrary' ];
        const data = this.memPlanes.read( 4096 );
        const planes = [];

        let offset = 1;

        for( let i = 0; i < numPlanes; i++ )
        {
            const id = data[offset++];
            const type = TYPE[data[offset++]] || 'arbitrary';
            const inliers = data[offset++];

            const pose = new Float32Array( data.subarray( offset, offset + 16 ) );
            offset += 16;

            const extent = { u: data[offset++], v: data[offset++] };

            const hullCount = data[offset++];
            const hull = new Array( hullCount );

            for( let h = 0; h < hullCount; h++ )
            {
                hull[h] = { x: data[offset++], y: data[offset++], z: data[offset++] };
            }

            planes.push( { id, type, inliers, pose, extent, hull } );
        }

        return planes;
    }

    /**
     * Ray-cast a screen/image pixel against the tracked planes.
     * Returns { planeId, pose: Float32Array(16) } or null.
     * x, y are in image pixel coordinates (same space as the input frames).
     */
    hitTest( x, y )
    {
        const planeId = this.system.hitTest( x, y, this.memHit.ptr );

        if( planeId === 0 )
        {
            return null;
        }

        return { planeId: planeId, pose: new Float32Array( this.memHit.read( 16 ) ) };
    }

    /**
     * Store a world-space pose as an anchor. Returns the anchor id.
     * Note: until relocalization/loop-closure lands, anchors are static
     * world poses; they do not self-correct after tracking drift.
     */
    createAnchor( pose )
    {
        const id = this.nextAnchorId++;
        this.anchors.set( id, new Float32Array( pose ) );
        return id;
    }

    getAnchor( id )
    {
        return this.anchors.get( id ) || null;
    }

    removeAnchor( id )
    {
        this.anchors.delete( id );
    }

    getFramePoints()
    {
        const numPoints = this.system.getFramePoints( this.memPts.ptr );

        const points = new Array( numPoints );

        if( numPoints > 0 )
        {
            const data = this.memPts.read( numPoints * 2 );

            for( let i = 0, j = 0; i < numPoints; i++ )
            {
                points[i] = { x: data[j++], y: data[j++] };
            }
        }

        return points;
    }

    reset()
    {
        this.system.reset();
    }
}

export { ArgusAR };