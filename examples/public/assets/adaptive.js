/**
 * AdaptiveQuality v1 — steps processing resolution down/up based on measured
 * frame cost, with hysteresis so it doesn't oscillate.
 *
 * The controller only decides a scale factor; the host owns applying it
 * (resize capture canvas + reconfigure SLAM intrinsics).
 *
 *   const aq = new AdaptiveQuality( {
 *       budgetMs: 33,                      // target per-frame cost
 *       scales: [ 1.0, 0.75, 0.5 ],        // allowed resolution factors
 *       onChange: ( scale ) => { ... }     // apply the new factor
 *   } );
 *   aq.sample( frameMs );                  // call once per processed frame
 */
class AdaptiveQuality
{
    constructor( { budgetMs = 33, scales = [ 1.0, 0.75, 0.5 ], onChange = null, windowSize = 30, stepDownAfter = 20, stepUpAfter = 120 } = {} )
    {
        this.budgetMs = budgetMs;
        this.scales = scales;
        this.onChange = onChange;
        this.windowSize = windowSize;

        // hysteresis: consecutive frames over/under budget before stepping
        this.stepDownAfter = stepDownAfter;
        this.stepUpAfter = stepUpAfter;

        this.level = 0;
        this.samples = [];
        this.overCount = 0;
        this.underCount = 0;
        this.cooldown = 0;
    }

    get scale()
    {
        return this.scales[this.level];
    }

    get meanMs()
    {
        if( this.samples.length === 0 )
        {
            return 0;
        }

        return this.samples.reduce( ( a, b ) => a + b, 0 ) / this.samples.length;
    }

    sample( frameMs )
    {
        this.samples.push( frameMs );

        if( this.samples.length > this.windowSize )
        {
            this.samples.shift();
        }

        // let the pipeline settle after a resolution change
        if( this.cooldown > 0 )
        {
            this.cooldown--;
            return;
        }

        const mean = this.meanMs;

        if( mean > this.budgetMs )
        {
            this.overCount++;
            this.underCount = 0;
        }
        else if( mean < this.budgetMs * 0.6 )
        {
            // only step up when comfortably under budget
            this.underCount++;
            this.overCount = 0;
        }
        else
        {
            this.overCount = 0;
            this.underCount = 0;
        }

        if( this.overCount >= this.stepDownAfter && this.level < this.scales.length - 1 )
        {
            this._setLevel( this.level + 1 );
        }
        else if( this.underCount >= this.stepUpAfter && this.level > 0 )
        {
            this._setLevel( this.level - 1 );
        }
    }

    _setLevel( level )
    {
        this.level = level;
        this.overCount = 0;
        this.underCount = 0;
        this.samples = [];
        this.cooldown = this.windowSize;

        if( this.onChange )
        {
            this.onChange( this.scales[level] );
        }
    }
}

export { AdaptiveQuality };
