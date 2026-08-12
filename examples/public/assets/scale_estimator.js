/**
 * IMUScaleEstimator — loosely-coupled metric scale + gravity estimation.
 *
 * Monocular SLAM positions are unit-free. The accelerometer measures real
 * m/s². Rotate IMU samples into the SLAM world frame using the visual
 * rotations; the near-constant component there is gravity. What remains is
 * linear acceleration in m/s², which must equal the second derivative of the
 * SLAM trajectory times the metric scale. A least-squares fit of the two
 * accelerations yields metersPerUnit; the constant component yields the
 * world-frame gravity direction.
 *
 * All math is dependency-free (no three.js in the engine layer).
 */

// --- minimal quaternion / vector helpers -----------------------------------

function quatFromPose( m )
{
    // column-major pose: R columns at [0..2], [4..6], [8..10]
    const m00 = m[0], m01 = m[4], m02 = m[8];
    const m10 = m[1], m11 = m[5], m12 = m[9];
    const m20 = m[2], m21 = m[6], m22 = m[10];

    const tr = m00 + m11 + m22;
    let x, y, z, w;

    if( tr > 0 )
    {
        const s = Math.sqrt( tr + 1.0 ) * 2;
        w = 0.25 * s;
        x = ( m21 - m12 ) / s;
        y = ( m02 - m20 ) / s;
        z = ( m10 - m01 ) / s;
    }
    else if( m00 > m11 && m00 > m22 )
    {
        const s = Math.sqrt( 1.0 + m00 - m11 - m22 ) * 2;
        w = ( m21 - m12 ) / s;
        x = 0.25 * s;
        y = ( m01 + m10 ) / s;
        z = ( m02 + m20 ) / s;
    }
    else if( m11 > m22 )
    {
        const s = Math.sqrt( 1.0 + m11 - m00 - m22 ) * 2;
        w = ( m02 - m20 ) / s;
        x = ( m01 + m10 ) / s;
        y = 0.25 * s;
        z = ( m12 + m21 ) / s;
    }
    else
    {
        const s = Math.sqrt( 1.0 + m22 - m00 - m11 ) * 2;
        w = ( m10 - m01 ) / s;
        x = ( m02 + m20 ) / s;
        y = ( m12 + m21 ) / s;
        z = 0.25 * s;
    }

    return [ x, y, z, w ];
}

function quatSlerp( a, b, t )
{
    let [ ax, ay, az, aw ] = a;
    let [ bx, by, bz, bw ] = b;

    let dot = ax * bx + ay * by + az * bz + aw * bw;

    if( dot < 0 )
    {
        bx = -bx; by = -by; bz = -bz; bw = -bw;
        dot = -dot;
    }

    if( dot > 0.9995 )
    {
        const x = ax + t * ( bx - ax ), y = ay + t * ( by - ay ), z = az + t * ( bz - az ), w = aw + t * ( bw - aw );
        const n = Math.hypot( x, y, z, w );
        return [ x / n, y / n, z / n, w / n ];
    }

    const th = Math.acos( dot );
    const s = Math.sin( th );
    const wa = Math.sin( ( 1 - t ) * th ) / s;
    const wb = Math.sin( t * th ) / s;

    return [ ax * wa + bx * wb, ay * wa + by * wb, az * wa + bz * wb, aw * wa + bw * wb ];
}

function quatRotate( q, v )
{
    const [ qx, qy, qz, qw ] = q;
    const [ vx, vy, vz ] = v;

    // t = 2 q_vec x v
    const tx = 2 * ( qy * vz - qz * vy );
    const ty = 2 * ( qz * vx - qx * vz );
    const tz = 2 * ( qx * vy - qy * vx );

    return [
        vx + qw * tx + qy * tz - qz * ty,
        vy + qw * ty + qz * tx - qx * tz,
        vz + qw * tz + qx * ty - qy * tx
    ];
}

// rear camera in portrait vs DeviceMotion device axes (nominal):
// cam_x = dev_x, cam_y = -dev_y, cam_z = -dev_z
function deviceToCamera( a )
{
    return [ a[0], -a[1], -a[2] ];
}

// --- estimator --------------------------------------------------------------

class IMUScaleEstimator
{
    constructor( { windowMs = 5000, minExcitation = 0.35 } = {} )
    {
        this.windowMs = windowMs;
        this.minExcitation = minExcitation;   // m/s² below which samples are noise

        this.imu = [];     // { t, a: [x,y,z] device frame, m/s² incl. gravity }
        this.frames = [];  // { t, p: [x,y,z] slam units, q: [x,y,z,w] world-from-camera }

        this.lastEstimate = null;
    }

    reset()
    {
        this.imu.length = 0;
        this.frames.length = 0;
        this.lastEstimate = null;
    }

    feedIMU( t, ax, ay, az )
    {
        this.imu.push( { t, a: [ ax, ay, az ] } );
        this._prune( this.imu, t );
    }

    feedPose( t, pose )
    {
        this.frames.push( { t, p: [ pose[12], pose[13], pose[14] ], q: quatFromPose( pose ) } );
        this._prune( this.frames, t );
    }

    _prune( arr, now )
    {
        const cutoff = now - this.windowMs;
        while( arr.length && arr[0].t < cutoff ) arr.shift();
    }

    // rotation (world-from-device) interpolated at time t, or null outside range
    _worldFromDeviceAt( t )
    {
        const f = this.frames;

        if( f.length < 2 || t < f[0].t || t > f[f.length - 1].t )
        {
            return null;
        }

        let lo = 0;
        while( lo + 1 < f.length && f[lo + 1].t < t ) lo++;

        const a = f[lo], b = f[lo + 1];
        const u = ( b.t === a.t ) ? 0 : ( t - a.t ) / ( b.t - a.t );

        return quatSlerp( a.q, b.q, u );
    }

    /**
     * Returns { metersPerUnit, gravity: [x,y,z] unit world vector,
     *           confidence: 0..1, pairs } or null while not observable.
     */
    estimate()
    {
        const f = this.frames;

        if( f.length < 30 || this.imu.length < 60 )
        {
            return null;
        }

        // 1) rotate IMU into the world frame via interpolated visual rotations
        const rotated = [];

        for( const s of this.imu )
        {
            const q = this._worldFromDeviceAt( s.t );

            if( q )
            {
                rotated.push( { t: s.t, a: quatRotate( q, deviceToCamera( s.a ) ) } );
            }
        }

        if( rotated.length < 60 )
        {
            return null;
        }

        // 2) gravity = mean world-frame measurement (linear accel averages out)
        const g = [ 0, 0, 0 ];
        for( const r of rotated ) { g[0] += r.a[0]; g[1] += r.a[1]; g[2] += r.a[2]; }
        g[0] /= rotated.length; g[1] /= rotated.length; g[2] /= rotated.length;

        const gNorm = Math.hypot( g[0], g[1], g[2] );

        // 3) visual acceleration by local quadratic least-squares fits over a
        //    7-frame window (Savitzky-Golay style): raw second differences
        //    amplify position noise ~1/dt² and bias the scale fit downwards
        const P = f.map( fr => fr.p );
        const T = f.map( fr => fr.t / 1000 ); // seconds

        const visT = [];
        const visA = [];

        const HALF = 3;

        for( let i = HALF; i < f.length - HALF; i++ )
        {
            const t0 = T[i];

            if( T[i + HALF] - T[i - HALF] > 0.5 ) continue; // gap in tracking

            // normal equations for p(τ) = a + bτ + cτ², τ = t - t0
            let s0 = 0, s1 = 0, s2 = 0, s3 = 0, s4 = 0;
            const b0 = [ 0, 0, 0 ], b1 = [ 0, 0, 0 ], b2 = [ 0, 0, 0 ];

            for( let j = i - HALF; j <= i + HALF; j++ )
            {
                const tau = T[j] - t0;
                const tau2 = tau * tau;
                s0 += 1; s1 += tau; s2 += tau2; s3 += tau2 * tau; s4 += tau2 * tau2;

                for( let k = 0; k < 3; k++ )
                {
                    b0[k] += P[j][k];
                    b1[k] += P[j][k] * tau;
                    b2[k] += P[j][k] * tau2;
                }
            }

            // solve the symmetric 3x3 [s0 s1 s2; s1 s2 s3; s2 s3 s4] x = [b0 b1 b2]
            const det = s0 * ( s2 * s4 - s3 * s3 ) - s1 * ( s1 * s4 - s3 * s2 ) + s2 * ( s1 * s3 - s2 * s2 );

            if( Math.abs( det ) < 1e-12 ) continue;

            const acc = [ 0, 1, 2 ].map( k =>
            {
                // Cramer for the τ² coefficient c
                const c = ( s0 * ( s2 * b2[k] - s3 * b1[k] ) - s1 * ( s1 * b2[k] - s3 * b0[k] ) + s2 * ( s1 * b1[k] - s2 * b0[k] ) ) / det;
                return 2 * c;
            } );

            visT.push( f[i].t );
            visA.push( acc );
        }

        // 4) pair visual accel with (world IMU accel - gravity) at same times
        let num = 0, den = 0, corrNum = 0, corrA = 0, corrB = 0, used = 0;

        for( let i = 0; i < visT.length; i++ )
        {
            // nearest rotated sample
            let best = null, bestDt = 40; // ms tolerance

            for( const r of rotated )
            {
                const dt = Math.abs( r.t - visT[i] );
                if( dt < bestDt ) { bestDt = dt; best = r; }
            }

            if( !best ) continue;

            const lin = [ best.a[0] - g[0], best.a[1] - g[1], best.a[2] - g[2] ];
            const linN = Math.hypot( ...lin );
            const visN = Math.hypot( ...visA[i] );

            if( linN < this.minExcitation || visN < 1e-6 ) continue;

            // s * a_vis ≈ a_lin  →  least squares over vector components
            num += visA[i][0] * lin[0] + visA[i][1] * lin[1] + visA[i][2] * lin[2];
            den += visN * visN;
            corrNum += visA[i][0] * lin[0] + visA[i][1] * lin[1] + visA[i][2] * lin[2];
            corrA += visN * visN;
            corrB += linN * linN;
            used++;
        }

        if( used < 15 || den < 1e-9 )
        {
            return null;
        }

        const k = num / den;                                   // meters per slam-unit (sign = axis convention fit)
        const corr = corrNum / Math.sqrt( corrA * corrB );     // -1..1

        const estimate = {
            metersPerUnit: Math.abs( k ),
            gravity: gNorm > 1e-6 ? [ g[0] / gNorm, g[1] / gNorm, g[2] / gNorm ] : [ 0, 1, 0 ],
            gravityMagnitude: gNorm,
            confidence: Math.min( 1, Math.abs( corr ) ) * Math.min( 1, used / 60 ),
            pairs: used
        };

        this.lastEstimate = estimate;

        return estimate;
    }
}

export { IMUScaleEstimator };
