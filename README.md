# Multiplex

**Multiplex** is a collection of standalone DSP objects for Pure Data. 

Rather than sticking to a single sonic philosophy, this library explores sound through multiple perspectives—ranging from clinical digital precision to organic, circuit-inspired modeling.

## Philosophy

Inspired by Samuel R. Delany's *Empire Star*, the library is built on the transition of perspectives:
1. **Simplex:** The fundamental mathematical representation of a signal.
2. **Complex:** The signal as a functional tool within a synthesis system (Modulation, Sync, Logic).
3. **Multiplex:** The signal as a living entity, accounting for the "imperfections" of time and physics—such as phase drift, saturation, and circuit-like stability.

Multiplex objects are designed to be versatile; they can be used as high-precision digital modulators or as "heavy" hardware-modeled sound sources.

## Current Objects

### `sine~`
A high-resolution oscillator based on a 9th-order polynomial approximation.
- **Versatile Modes:** Includes an **Audio mode** with analog-style shaping and DC-blocking, and an **LFO/Drone mode** for high-precision DC-stable modulation.
- **Clock Sync:** Can function as a standard oscillator or a clocked modulator that measures frequency from incoming pulses.
- **Organic Character:** Features internal phase-drift logic with a seedable PRNG for reproducible "living" movement.

## Installation & Building

Multiplex is built as a set of standalone modular objects, following the model of the `else` library.

### Requirements
- Pure Data headers
- CMake (version 3.14 or higher)

### Build Instructions
```bash
mkdir build
cd build
cmake ..
make
