#include <m_pd.h>
#include <math.h>
#include <stdlib.h>
#include <stdint.h>

static t_class *sine_class;

typedef struct _sine {
    t_object  x_obj;
    double    x_phase;     
    t_float   x_freq;      
    int       x_soft;      
    t_float   x_dir;       
    uint32_t  x_rng;       
    t_float   x_sr_rec;    
    t_float   x_last_sync; 
    
    /* Analog Flavor / LFO states */
    t_float   x_dc_state;
    int       x_filter;      /* 1 = ON (Audio), 0 = OFF (LFO/Drone) */
    
    /* Clock/Timer states */
    int       x_clock_mode;  /* 1 = Clock, 0 = Sync */
    uint32_t  x_count;       
    t_float   x_sr;          
} t_sine;

/* 9th-order polynomial sine approximation */
static inline t_sample multiplex_sine_poly(double x) {
    double t = 2.0 * x - 1.0;
    double t2 = t * t;
    return (t_sample)(t * (t2 - 1.0) * (3.141521 + t2 * (-2.024773 + t2 * (0.517492 + t2 * -0.063691))));
}

/* Analog shaper for 3rd-harmonic warmth */
static inline t_sample multiplex_analog_shaper(t_sample x) {
    return x * (1.005f - 0.05f * x * x);
}

static inline uint32_t multiplex_rand(uint32_t *state) {
    *state = *state * 196314165 + 907633515;
    return *state;
}

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
        t_sample sync_in = *in_s++;
        t_sample offset  = *in_o++;
        t_sample d_sig   = *in_d++;

        count++; 

        /* Sync / Clock Logic (Threshold 0.5) */
        if (sync_in >= 0.5f && last_s < 0.5f) {
            if (x->x_clock_mode) {
                if (count > 0) x->x_freq = x->x_sr / (t_float)count;
                count = 0; 
                ph = 0;    
            } else {
                if (x->x_soft) dir *= -1.0f;
                else ph = 0;
            }
        }
        last_s = sync_in;

        /* Frequency Logic */
        t_float freq_in = (f_sig != 0) ? f_sig : x->x_freq;
        uint32_t r = multiplex_rand(&x->x_rng);
        t_float noise = ((float)r * (1.0f / 4294967296.0f) - 0.5f) * d_sig;
        double step = (double)(freq_in + (freq_in * noise)) * (double)x->x_sr_rec * (double)dir;

        /* Output calculation */
        double wrapped_ph = ph + (double)offset;
        wrapped_ph -= floor(wrapped_ph);
        if (wrapped_ph < 0) wrapped_ph += 1.0;

        t_sample sig = multiplex_sine_poly(wrapped_ph);
        sig = multiplex_analog_shaper(sig);

        /* DC Blocker */
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

static void sine_soft(t_sine *x, t_floatarg f) {
    x->x_soft = (int)(f != 0);
}

static void sine_seed(t_sine *x, t_floatarg f) {
    x->x_rng = (uint32_t)f;
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

static void *sine_new(t_symbol *s, int argc, t_atom *argv) {
    t_sine *x = (t_sine *)pd_new(sine_class);
    x->x_freq = 0;
    x->x_soft = 0;
    x->x_dir = 1.0f;
    x->x_phase = 0;
    x->x_rng = 12345;
    x->x_last_sync = 0;
    x->x_dc_state = 0;
    x->x_filter = 1; 
    x->x_clock_mode = 0;
    x->x_count = 0;
    x->x_sr = 44100.0f;

    while (argc > 0) {
        if (argv->a_type == A_SYMBOL) {
            t_symbol *sym = atom_getsymbol(argv);
            if (sym == gensym("-soft")) x->x_soft = 1;
            /* -lfo flag now switches both Filter OFF and Clock ON */
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
