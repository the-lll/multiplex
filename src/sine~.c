/*
 * MULTIPLEX: sine~
 * Signal-rate oscillator with multiple operating perspectives.
 * * Inlets:
 * 1: Frequency (Signal/Float)
 * 2: Sync/Clock (Signal) - Trigger threshold 0.5
 * 3: Phase Offset (Signal)
 * 4: Drift Intensity (Signal)
 *
 * Perspectives:
 * Simplex:  9th-order polynomial approximation for high precision.
 * Complex:  Hard/Soft sync and automated frequency clocking.
 * Multiplex: Analog-style shaper, DC blocker, and PRNG phase drift.
 */

#include <m_pd.h>
#include <math.h>
#include <stdlib.h>
#include <stdint.h>

static t_class *sine_class;

typedef struct _sine {
    t_object  x_obj;
    double    x_phase;       /* Current phase position [0..1] */
    t_float   x_freq;        /* Internal frequency value */
    int       x_soft;        /* Sync mode: 1 = Soft, 0 = Hard */
    t_float   x_dir;         /* Direction: 1.0 forward, -1.0 backward */
    uint32_t  x_rng;         /* PRNG state for phase drift */
    t_float   x_sr_rec;      /* Reciprocal of sample rate */
    t_float   x_last_sync;   /* Edge detection for sync inlet */
    t_float   x_dc_state;    /* One-pole filter state for DC blocker */
    int       x_filter;      /* DC filter toggle: 1 = ON, 0 = OFF */
    int       x_clock_mode;  /* Mode toggle: 1 = Clock, 0 = Sync */
    uint32_t  x_count;       /* Sample counter for clock timing */
    t_float   x_sr;          /* Current system sample rate */
} t_sine;

/* --- DSP HELPERS --- */

/* 9th-order polynomial approximation of sin(2*pi*x) */
static inline t_sample multiplex_sine_poly(double x) {
    double t = 2.0 * x - 1.0;
    double t2 = t * t;
    return (t_sample)(t * (t2 - 1.0) * (3.141521 + t2 * (-2.024773 + t2 * (0.517492 + t2 * -0.063691))));
}

/* Analog shaper providing subtle 3rd-harmonic saturation */
static inline t_sample multiplex_analog_shaper(t_sample x) {
    return x * (1.005f - 0.05f * x * x);
}

/* Linear Congruential Generator for high-performance pseudo-random noise */
static inline uint32_t multiplex_rand(uint32_t *state) {
    *state = *state * 196314165 + 907633515;
    return *state;
}

/* --- PERFORM ROUTINE --- */

static t_int *sine_perform(t_int *w) {
    t_sine *x = (t_sine *)(w[1]);
    t_sample *in_f     = (t_sample *)(w[2]); 
    t_sample *in_s     = (t_sample *)(w[3]); 
    t_sample *in_o     = (t_sample *)(w[4]); 
    t_sample *in_d     = (t_sample *)(w[5]); 
    t_sample *out      = (t_sample *)(w[6]);
    int n = (int)(w[7]);

    double ph = x->x_phase;
    t_float dir = x->x_dir;
    t_float last_s = x->x_last_sync;
    t_float dc = x->x_dc_state;
    uint32_t count = x->x_count;

    while (n--) {
        t_sample f_sig = *in_f++;
        t_sample s_sig = *in_s++;
        t_sample o_sig = *in_o++;
        t_sample d_sig = *in_d++;

        count++; 

        /* Sync / Clock Logic */
        if (s_sig >= 0.5f && last_s < 0.5f) {
            if (x->x_clock_mode) {
                if (count > 0) x->x_freq = x->x_sr / (t_float)count;
                count = 0; 
                ph = 0;    
            } else {
                if (x->x_soft) dir *= -1.0f;
                else ph = 0;
            }
        }
        last_s = s_sig;

        /* Frequency + Drift calculation */
        t_float freq_in = (f_sig != 0) ? f_sig : x->x_freq;
        uint32_t r = multiplex_rand(&x->x_rng);
        t_float noise = ((float)r * (1.0f / 4294967296.0f) - 0.5f) * (float)d_sig;
        double step = (double)(freq_in + (freq_in * noise)) * (double)x->x_sr_rec * (double)dir;

        /* Output phase accumulation with offset */
        double wrapped_ph = ph + (double)o_sig;
        wrapped_ph -= floor(wrapped_ph);
        if (wrapped_ph < 0) wrapped_ph += 1.0;

        t_sample sig = multiplex_analog_shaper(multiplex_sine_poly(wrapped_ph));

        /* DC Blocker (One-pole highpass) */
        if (x->x_filter) {
            dc += 0.0005f * (sig - dc);
            *out++ = sig - dc;
        } else {
            *out++ = sig;
            dc = 0;
        }

        ph += step;
        ph -= floor(ph);
    }

    x->x_phase = ph;
    x->x_dir = dir;
    x->x_last_sync = last_s;
    x->x_dc_state = dc;
    x->x_count = count;
    return (w + 8);
}

static void sine_dsp(t_sine *x, t_signal **sp) {
    x->x_sr = (t_float)sp[0]->s_sr;
    x->x_sr_rec = (x->x_sr > 0) ? (1.0f / x->x_sr) : (1.0f / 44100.0f);
    dsp_add(sine_perform, 7, x, sp[0]->s_vec, sp[1]->s_vec, sp[2]->s_vec, sp[3]->s_vec, sp[4]->s_vec, sp[0]->s_n);
}

/* --- MESSAGE HANDLERS --- */

static void sine_soft(t_sine *x, t_floatarg f) {
    x->x_soft = (int)(f != 0);
}

static void sine_seed(t_sine *x, t_floatarg f) {
    x->x_rng = (uint32_t)f;

    /* Warm up the PRNG */
    for(int i = 0; i < 8; i++) {
        x->x_rng = x->x_rng * 196314165 + 907633515;
    }

    x->x_phase = 0;      /* Reset wave */
    x->x_dc_state = 0;   /* Reset filter */
    x->x_last_sync = 0;  /* Reset sync edge detector */

    /* CRITICAL FOR SOFT SYNC: */
    x->x_dir = 1.0f;     /* Force wave to move forward */

}

static void sine_filter(t_sine *x, t_floatarg f) {
    x->x_filter = (int)(f != 0);
}

static void sine_clock(t_sine *x, t_floatarg f) {
    x->x_clock_mode = (int)(f != 0);
    x->x_count = 0;
}

static void sine_float(t_sine *x, t_floatarg f) {
    x->x_freq = f;
}

/* --- CONSTRUCTOR / DESTRUCTOR --- */

static void *sine_new(t_symbol *s, int argc, t_atom *argv) {
    t_sine *x = (t_sine *)pd_new(sine_class);

    /* Initialize defaults */
    x->x_freq = 0;
    x->x_soft = 0;
    x->x_dir = 1.0f;
    x->x_phase = 0;
    x->x_last_sync = 0;
    x->x_dc_state = 0;
    x->x_filter = 1;
    x->x_clock_mode = 0;
    x->x_count = 0;
    x->x_sr = 44100.0f;

    /*
     * UNIQUE SEED GENERATION:
     * We use the memory address of 'x' to ensure every
     * instance of sine~ starts with a unique drift character.
     */
    x->x_rng = (uint32_t)((uintptr_t)x);

    /* Parse arguments like -soft, -lfo, or frequency */
    while (argc > 0) {
        if (argv->a_type == A_SYMBOL) {
            t_symbol *sym = atom_getsymbol(argv);
            if (sym == gensym("-soft")) x->x_soft = 1;
            if (sym == gensym("-lfo")) {
                x->x_filter = 0;
                x->x_clock_mode = 1;
            }
            argc--; argv++;
        }
        else if (argv->a_type == A_FLOAT) {
            x->x_freq = atom_getfloat(argv);
            argc--; argv++;
        }
        else { argc--; argv++; }
    }

    inlet_new(&x->x_obj, &x->x_obj.ob_pd, &s_signal, &s_signal);
    inlet_new(&x->x_obj, &x->x_obj.ob_pd, &s_signal, &s_signal);
    inlet_new(&x->x_obj, &x->x_obj.ob_pd, &s_signal, &s_signal);
    outlet_new(&x->x_obj, &s_signal);

    return (void *)x;
}

void sine_tilde_setup(void) {
    sine_class = class_new(gensym("sine~"), (t_newmethod)sine_new, 
        0, sizeof(t_sine), CLASS_DEFAULT, A_GIMME, 0);

    class_addmethod(sine_class, (t_method)sine_dsp, gensym("dsp"), A_CANT, 0);
    class_addmethod(sine_class, (t_method)sine_soft, gensym("soft"), A_DEFFLOAT, 0);
    class_addmethod(sine_class, (t_method)sine_seed, gensym("seed"), A_DEFFLOAT, 0);
    class_addmethod(sine_class, (t_method)sine_filter, gensym("filter"), A_DEFFLOAT, 0);
    class_addmethod(sine_class, (t_method)sine_clock, gensym("clock"), A_DEFFLOAT, 0);
    
    CLASS_MAINSIGNALIN(sine_class, t_sine, x_freq);
}
