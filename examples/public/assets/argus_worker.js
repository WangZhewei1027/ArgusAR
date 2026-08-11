/**
 * SLAM worker: owns the ArgusAR/wasm instance so the main thread never blocks on tracking.
 *
 * Protocol (main -> worker):
 *   { type: 'init',  width, height }
 *   { type: 'frame', buffer, width, height }   // RGBA pixels, buffer is transferred
 *   { type: 'reconfigure', width, height }     // resolution change (adaptive quality)
 *   { type: 'reset' }
 *
 * Protocol (worker -> main):
 *   { type: 'ready' }
 *   { type: 'result', pose: Float32Array|null, dots: [{x,y}]|null, planePose: Float32Array|null, ms }
 */

let ArgusAR = null;
let argus = null;
let findPlaneRequested = false;

self.onmessage = async ( event ) =>
{
    const msg = event.data;

    switch( msg.type )
    {
        case 'init':
        case 'reconfigure':
        {
            if( !ArgusAR )
            {
                // dynamic import with version tag so a rebuilt wasm is never
                // masked by the browser's module cache
                ( { ArgusAR } = await import( `./argus_ar.js?v=${ msg.v || '0' }` ) );
            }

            // Re-initialize on resolution change: System.configure() and the shared
            // image memory are sized to one resolution, so a fresh instance is the
            // safe path. Rare event (adaptive steps), ~100-300ms.
            argus = await ArgusAR.Initialize( msg.width, msg.height );
            self.postMessage( { type: 'ready', width: msg.width, height: msg.height } );
            break;
        }

        case 'findPlane':
        {
            findPlaneRequested = true;
            break;
        }

        case 'frame':
        {
            if( !argus )
            {
                break;
            }

            const t0 = performance.now();

            const frame = {
                data: new Uint8ClampedArray( msg.buffer ),
                width: msg.width,
                height: msg.height
            };

            const pose = argus.findCameraPose( frame );

            let planePose = null;
            let dots = null;

            if( pose )
            {
                if( findPlaneRequested )
                {
                    const p = argus.findPlane();

                    if( p )
                    {
                        planePose = new Float32Array( p );
                        findPlaneRequested = false;
                    }
                }
            }
            else
            {
                dots = argus.getFramePoints();
            }

            const ms = performance.now() - t0;

            self.postMessage( {
                type: 'result',
                pose: pose ? new Float32Array( pose ) : null,
                planePose: planePose,
                dots: dots,
                ms: ms
            } );
            break;
        }

        case 'reset':
        {
            if( argus )
            {
                argus.reset();
            }
            break;
        }
    }
};
