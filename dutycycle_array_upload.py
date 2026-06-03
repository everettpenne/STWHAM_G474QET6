"""
dc_ramp_gen.py
==============
Generates DMA arrays for the STM32 3-phase H-bridge controller in a mode where:
  - All three phases run at the SAME constant frequency and are IN-PHASE.
  - Each phase pair has an INDEPENDENT duty cycle that ramps according to
    user-supplied (time, duty-cycle) setpoints, linearly interpolated.
  - The complement of each phase is generated automatically with deadtime.

Replaces the frequency-ramp logic in hvpscontrollervalgen.py / array_upload.py.

Timer topology (unchanged from hardware):
  TIM1  — Phase 1,  master,  PWM mode 1 (starts HIGH, falls at CCR1)
  TIM2  — Phase 1N, slave,   PWM mode 2, triggered by TIM1 update (ITR0)
  TIM3  — Phase 2,  slave,   PWM mode 2, triggered by TIM1 update (ITR0)
  TIM4  — Phase 2N, slave,   PWM mode 2, triggered by TIM3 OC1REF  (ITR2)
  TIM5  — Phase 3,  slave,   PWM mode 2, triggered by TIM3 OC1REF  (ITR2)
  TIM8  — Phase 3N, slave,   PWM mode 2, triggered by TIM5 OC1REF  (ITR3)

All phases are now in-phase, so every slave timer fires at t=0 of the master
period, exactly as TIM2 does.  There is no inter-phase offset in the CCR values.

Derived formulas (T = baseARR = 80e6/freq, dt = deadTime_us * 80):
  TIM1:  ARR1 = T          CCR1 = T*dc1 - dt        (PWM mode 1: HIGH 0→CCR1)
  TIM2:  ARR2 = T          CCR2 = T*dc1 + dt        (PWM mode 2: HIGH CCR2→ARR2)
  TIM3:  ARR3 = T*dc2 - dt CCR3 = dt               (PWM mode 2: HIGH CCR3→ARR3)
  TIM4:  ARR4 = T - dt     CCR4 = T*dc2             (local to TIM3 trigger)
  TIM5:  ARR5 = T*dc3 - dt CCR5 = dt               (local to TIM3 trigger, in-phase)
  TIM8:  ARR8 = T - 2*dt   CCR8 = T*dc3             (local to TIM5 trigger)

The "Freq" arrays (ARR1-ARR8) are constant for all entries because frequency is
fixed.  Only the "Duty" arrays (CCR1-CCR8) vary entry-by-entry.

Array layout returned (matches array_upload.py upload order):
  [0]  ARR1   [1]  ARR2   [2]  ARR3   [3]  ARR4   [4]  ARR5   [5]  ARR8
  [6]  CCR1   [7]  CCR2   [8]  CCR3   [9]  CCR4   [10] CCR5   [11] CCR8
"""

import struct

import matplotlib.pyplot as plt
import numpy as np
import serial
import time
import sys

np.set_printoptions(threshold=np.inf)

# ── Constants ──────────────────────────────────────────────────────────────────
CLCK_SPEED  = 80e6   # STM32 timer clock (Hz)
DEAD_TIME   = 0.25    # deadtime (µs) — edit here if hardware changes
ARRAY_SIZE  = 1000   # number of entries in each DMA array
IDLE_ARR    = 9000   # ARR value used during idle / shutdown frames
MIN_DC      = 5.0    # minimum allowed duty cycle (%)
MAX_DC      = 95.0   # maximum allowed duty cycle (%)

# ── Serial config ──────────────────────────────────────────────────────────────
SERIAL_PORT = "/dev/tty.usbserial-1430"
BAUD        = 115200
TIMEOUT     = 1.0


# ── Core generation ────────────────────────────────────────────────────────────

def generateDMAArrays(freq, pulseLength,
                      tSPs1, dcSPs1,
                      tSPs2, dcSPs2,
                      tSPs3, dcSPs3):
    """
    Generate 12 DMA arrays for a constant-frequency, variable-duty-cycle pulse.

    Parameters
    ----------
    freq        : float   — PWM frequency in Hz (same for all three phases)
    pulseLength : float   — total pulse duration in seconds
    tSPs1       : array-like of float — time setpoints for Phase 1 duty cycle (s)
    dcSPs1      : array-like of float — duty cycle setpoints for Phase 1 (%)
    tSPs2       : array-like of float — time setpoints for Phase 2 duty cycle (s)
    dcSPs2      : array-like of float — duty cycle setpoints for Phase 2 (%)
    tSPs3       : array-like of float — time setpoints for Phase 3 duty cycle (s)
    dcSPs3      : array-like of float — duty cycle setpoints for Phase 3 (%)

    Returns
    -------
    np.ndarray of shape (12, ARRAY_SIZE), dtype int
    """

    tSPs1, dcSPs1 = np.asarray(tSPs1, float), np.asarray(dcSPs1, float)
    tSPs2, dcSPs2 = np.asarray(tSPs2, float), np.asarray(dcSPs2, float)
    tSPs3, dcSPs3 = np.asarray(tSPs3, float), np.asarray(dcSPs3, float)

    _validate_setpoints(tSPs1, dcSPs1, "Phase 1")
    _validate_setpoints(tSPs2, dcSPs2, "Phase 2")
    _validate_setpoints(tSPs3, dcSPs3, "Phase 3")

    # ── Fixed timing values (constant frequency) ──────────────────────────────
    T  = round(CLCK_SPEED / freq)   # master period in ticks
    dt = DEAD_TIME * 80             # deadtime in ticks

    # ── Simulate the master timer to find each update event time ──────────────
    # We step through every clock tick and record the time whenever the master
    # counter rolls over.  This gives the exact sequence of times at which the
    # DMA will fire a new array entry — identical logic to the original code,
    # but now T is constant so the loop is much faster.
    t_vals = np.linspace(0, pulseLength, int(CLCK_SPEED * pulseLength))
    tUp    = []   # absolute time of each master timer update event
    cnt    = 0

    for i in range(len(t_vals)):
        if cnt == T:
            cnt = 0
            tUp.append(t_vals[i])
        cnt += 1

    n_updates = len(tUp)

    # ── Interpolate duty cycles at each update event ──────────────────────────
    dc1 = _interp_dc(tUp, tSPs1, dcSPs1) / 100.0   # fractional
    dc2 = _interp_dc(tUp, tSPs2, dcSPs2) / 100.0
    dc3 = _interp_dc(tUp, tSPs3, dcSPs3) / 100.0

    # ── Pre-fill arrays with idle values ─────────────────────────────────────
    # Idle state: ARR=IDLE_ARR, CCR=IDLE_ARR (CCR=ARR in PWM mode 2 → 0% DC)
    ARR1 = [IDLE_ARR] * ARRAY_SIZE;  CCR1 = [0]        * ARRAY_SIZE
    ARR2 = [IDLE_ARR] * ARRAY_SIZE;  CCR2 = [IDLE_ARR] * ARRAY_SIZE
    ARR3 = [IDLE_ARR] * ARRAY_SIZE;  CCR3 = [IDLE_ARR] * ARRAY_SIZE
    ARR4 = [IDLE_ARR] * ARRAY_SIZE;  CCR4 = [IDLE_ARR] * ARRAY_SIZE
    ARR5 = [IDLE_ARR] * ARRAY_SIZE;  CCR5 = [IDLE_ARR] * ARRAY_SIZE
    ARR8 = [IDLE_ARR] * ARRAY_SIZE;  CCR8 = [IDLE_ARR] * ARRAY_SIZE

    # ── Fill active entries ───────────────────────────────────────────────────
    for i in range(min(n_updates, ARRAY_SIZE)):
        d1, d2, d3 = dc1[i], dc2[i], dc3[i]

        # TIM1 — master, PWM mode 1: HIGH from 0 → CCR1, LOW from CCR1 → ARR1
        ARR1[i] = round(T)
        CCR1[i] = round(T * d1 - dt)

        # TIM2 — Phase 1N, PWM mode 2, triggered at t=0 (TIM1 update)
        # Stays LOW until CCR2, then HIGH until ARR2 (one-pulse stops here)
        ARR2[i] = round(T) - 1
        CCR2[i] = round(T * d1 + dt)

        # TIM3 — Phase 2, PWM mode 2, triggered at t=0 (TIM1 update, in-phase)
        # Stays LOW for deadtime, then HIGH until ARR3
        ARR3[i] = round(T * d2 - dt)
        CCR3[i] = round(dt)

        # TIM4 — Phase 2N, PWM mode 2, triggered by TIM3's OC1REF
        # TIM3 OC1REF goes HIGH at local time dt from the TIM1 trigger.
        # TIM4 counts from that trigger.  HIGH from local T*d2 → local T-dt.
        ARR4[i] = round(T - dt) - 1
        CCR4[i] = round(T * d2)

        # TIM5 — Phase 3, PWM mode 2, triggered by TIM3's OC1REF (in-phase)
        # Same trigger as TIM4.  HIGH from local dt → local T*d3-dt.
        ARR5[i] = round(T * d3 - dt)
        CCR5[i] = round(dt)

        # TIM8 — Phase 3N, PWM mode 2, triggered by TIM5's OC1REF
        # TIM5 OC1REF goes HIGH at local (from TIM3 trigger) time dt.
        # TIM8 counts from that trigger.  HIGH from local T*d3 → local T-2dt.
        ARR8[i] = round(T - 2 * dt) - 1
        CCR8[i] = round(T * d3)

    # ── Shutdown taper: zero DC for last 10 + 100 entries ────────────────────
    tail_start = max(0, n_updates - 10)
    for i in range(tail_start, min(n_updates + 100, ARRAY_SIZE)):
        CCR1[i] = 0
        ARR2[i] = IDLE_ARR;  CCR2[i] = IDLE_ARR
        ARR3[i] = IDLE_ARR;  CCR3[i] = IDLE_ARR
        ARR4[i] = IDLE_ARR;  CCR4[i] = IDLE_ARR
        ARR5[i] = IDLE_ARR;  CCR5[i] = IDLE_ARR
        ARR8[i] = IDLE_ARR;  CCR8[i] = IDLE_ARR

    # ── Startup taper: zero DC for first 10 entries ───────────────────────────
    for i in range(10):
        CCR1[i] = 0
        CCR2[i] = ARR2[i]
        CCR3[i] = ARR3[i]
        CCR4[i] = ARR4[i]
        CCR5[i] = ARR5[i]
        CCR8[i] = ARR8[i]

    data = np.array([ARR1, ARR2, ARR3, ARR4, ARR5, ARR8,
                     CCR1, CCR2, CCR3, CCR4, CCR5, CCR8])
    return data.astype(int)


# ── Helpers ────────────────────────────────────────────────────────────────────

def _validate_setpoints(tSPs, dcSPs, name):
    """Sanity-check a pair of setpoint arrays."""
    if len(tSPs) != len(dcSPs):
        raise ValueError(f"{name}: tSPs and dcSPs must have the same length.")
    if len(tSPs) < 2:
        raise ValueError(f"{name}: at least 2 setpoints required.")
    for i in range(len(tSPs) - 1):
        if tSPs[i] >= tSPs[i + 1]:
            raise ValueError(f"{name}: time setpoints must be strictly increasing.")
    if np.any(dcSPs < MIN_DC) or np.any(dcSPs > MAX_DC):
        raise ValueError(
            f"{name}: duty cycle setpoints must be between {MIN_DC}% and {MAX_DC}%."
        )


def _interp_dc(tUp, tSPs, dcSPs):
    """
    Linearly interpolate duty cycle at each time in tUp using the piecewise
    linear function defined by (tSPs, dcSPs).  Values outside the setpoint
    range are clamped to the nearest endpoint.
    """
    return np.interp(tUp, tSPs, dcSPs)


# ── Serial helpers ─────────────────────────────────────────────────────────────

def open_port():
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD, timeout=TIMEOUT,
                            write_timeout=TIMEOUT)
        print(f"Opened {SERIAL_PORT}")
        time.sleep(0.2)
        return ser
    except Exception as e:
        print(f"Could not open UART: {e}")
        sys.exit(1)


def send_scpi(ser, cmd):
    ser.write((cmd + "\r\n").encode("ascii"))
    ser.flush()
    time.sleep(0.01)


def upload_array(ser, index, values):
    """Upload one 1000-entry array to the STM32 via SCPI SOUR:ARRA."""
    payload = b"".join(struct.pack("<I", int(v)) for v in values)
    assert len(payload) == 4000, f"Payload {len(payload)} bytes, expected 4000"
    send_scpi(ser, f"SOUR:ARRA {index}")
    time.sleep(0.2)
    ser.write(payload)
    time.sleep(0.5)
    ser.flush()
    print(f"  Uploaded array index {index}")


# ── Plotting ───────────────────────────────────────────────────────────────────

def plot_arrays(dma_arrays, freq, pulseLength):
    """Plot ARR and CCR arrays, showing only active (non-idle) entries."""
    # Find the last non-idle index for a sensible x-axis
    active = np.where(dma_arrays[0] != IDLE_ARR)[0]
    x_max  = int(active[-1]) + 20 if len(active) else 100

    idx = np.arange(ARRAY_SIZE)

    fig, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=True)

    labels_arr = ["ARR1 (TIM1)", "ARR2 (TIM2)", "ARR3 (TIM3)",
                  "ARR4 (TIM4)", "ARR5 (TIM5)", "ARR8 (TIM8)"]
    labels_ccr = ["CCR1 (TIM1)", "CCR2 (TIM2)", "CCR3 (TIM3)",
                  "CCR4 (TIM4)", "CCR5 (TIM5)", "CCR8 (TIM8)"]

    for i, lbl in enumerate(labels_arr):
        axes[0].plot(idx, dma_arrays[i],     label=lbl, linewidth=1.2)
    for i, lbl in enumerate(labels_ccr):
        axes[1].plot(idx, dma_arrays[6 + i], label=lbl, linewidth=1.2)

    axes[0].set_ylabel("ARR value (ticks)")
    axes[0].set_title(f"ARR arrays — freq={freq:.0f} Hz, pulse={pulseLength*1e3:.1f} ms")
    axes[0].legend(loc="best", fontsize=9)
    axes[0].grid(True, alpha=0.3)

    axes[1].set_ylabel("CCR value (ticks)")
    axes[1].set_title("CCR arrays (duty cycle)")
    axes[1].legend(loc="best", fontsize=9)
    axes[1].grid(True, alpha=0.3)
    axes[1].set_xlabel("Array index")

    plt.xlim(0, x_max)
    plt.tight_layout()
    plt.savefig("dc_ramp_arrays.png", dpi=300, bbox_inches="tight")
    print("Saved plot to dc_ramp_arrays.png")
    plt.close()


# ── Main ───────────────────────────────────────────────────────────────────────

def main():
    # ── Pulse parameters ───────────────────────────────────────────────────────
    freq        = 95000.0    # Hz — fixed for all three phases
    pulseLength = 3e-3      # seconds

    # Phase 1 duty cycle ramp: (time_s, duty_cycle_%)
    #tSPs1  = [0.0,   5e-3,  10e-3, 50e-3]
    #dcSPs1 = [50.0,  70.0,  70.0,  50.0 ]
    tSPs1 = [0.0, 3e-3]
    dcSPs1 = [6.0, 6.0]

    # Phase 2 duty cycle ramp
    #tSPs2  = [0.0,   5e-3,  10e-3, 50e-3]
    #dcSPs2 = [50.0,  60.0,  60.0,  50.0 ]
    #tSPs1 = [0.0,   0.1e-3, 0.4e-3, 0.5e-3, 0.55e-3, 0.6e-3, 1.4e-3, 1.5e-3, 1.55e-3, 1.6e-3, 2.3e-3, 2.4e-3, 2.45e-3, 2.5e-3,  3e-3]
    #dcSPs1 = [10.0, 10.0,   10.0,   90.0,   90.0,    10.0,   10.0,   90.0,   90.0,    10.0,   10.0,   90.0,   90.0,   10.0,    10.0]
    tSPs2 = [0.0, 3e-3]
    dcSPs2 = [75.0, 75.0]


    # Phase 3 duty cycle ramp
    #tSPs3  = [0.0,   5e-3,  10e-3, 50e-3]
    #dcSPs3 = [50.0,  55.0,  55.0,  50.0 ]
    #tSPs3 = [0.0,   0.1e-3, 0.4e-3, 0.5e-3, 0.55e-3, 0.6e-3, 1.4e-3, 1.5e-3, 1.55e-3, 1.6e-3, 2.3e-3, 2.4e-3, 2.45e-3, 2.5e-3,  3e-3]
    #dcSPs3 = [10.0, 10.0,   10.0,   90.0,   90.0,    10.0,   10.0,   90.0,   90.0,    10.0,   10.0,   90.0,   90.0,   10.0,    10.0]
    #tSPs3 = [0, 0.05e-3, 0.2e-3, 3e-3]
    #dcSPs3 = [90, 10, 10, 50]
    tSPs3 = [0.0, 1e-3, 2e-3, 3e-3]
    dcSPs3 = [6.0, 6.0, 75.0, 75.0]

    # ── Generate arrays ────────────────────────────────────────────────────────
    dma_arrays = generateDMAArrays(
        freq, pulseLength,
        tSPs1, dcSPs1,
        tSPs2, dcSPs2,
        tSPs3, dcSPs3,
    )
    print(f"Generated DMA arrays: shape {dma_arrays.shape}")

    # ── Upload ─────────────────────────────────────────────────────────────────
    ser = open_port()
    time.sleep(0.2)

    upload_array(ser, 1,  dma_arrays[0])   # FreqRampArray  (ARR1)
    upload_array(ser, 2,  dma_arrays[1])   # FreqRampArray2 (ARR2)
    upload_array(ser, 3,  dma_arrays[2])   # FreqRampArray3 (ARR3)
    upload_array(ser, 4,  dma_arrays[3])   # FreqRampArray4 (ARR4)
    upload_array(ser, 5,  dma_arrays[4])   # FreqRampArray5 (ARR5)
    upload_array(ser, 8,  dma_arrays[5])   # FreqRampArray8 (ARR8)
    upload_array(ser, 11, dma_arrays[6])   # DutyRampArray  (CCR1)
    upload_array(ser, 12, dma_arrays[7])   # DutyRampArray2 (CCR2)
    upload_array(ser, 13, dma_arrays[8])   # DutyRampArray3 (CCR3)
    upload_array(ser, 14, dma_arrays[9])   # DutyRampArray4 (CCR4)
    upload_array(ser, 15, dma_arrays[10])  # DutyRampArray5 (CCR5)
    upload_array(ser, 18, dma_arrays[11])  # DutyRampArray8 (CCR8)

    time.sleep(1)
    ser.close()

    # ── Plot ───────────────────────────────────────────────────────────────────
    plot_arrays(dma_arrays, freq, pulseLength)


if __name__ == "__main__":
    main()
