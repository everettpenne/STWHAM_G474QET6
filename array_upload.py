import os
import struct
import sys
import time

import matplotlib.pyplot as plt
import numpy as np
import serial

# ------------------------------------------------------------
# Configuration
# ------------------------------------------------------------
# SERIAL_PORT = "/dev/tty.usbserial-1140"  # Change for your Mac
SERIAL_PORT = "/dev/tty.usbserial-11430"
BAUD = 115200  # From STM32 USART3 config
TIMEOUT = 1.0

# ------------------------------------------------------------
# Low-level serial helpers
# ------------------------------------------------------------


def open_port():
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD, timeout=TIMEOUT, write_timeout=TIMEOUT)
        print(f"Opened {SERIAL_PORT}")
        time.sleep(0.2)
        return ser
    except Exception as e:
        print(f"Could not open UART: {e}")
        sys.exit(1)


def send_scpi(ser, cmd):
    """
    Sends an ASCII SCPI command terminated by newline.
    """
    line = (cmd + "\r\n").encode("ascii")
    ser.write(line)
    ser.flush()
    time.sleep(0.01)


def query_scpi(ser, cmd):
    """
    Sends a SCPI command and reads back a line.
    """
    send_scpi(ser, cmd)
    return ser.readline().decode("ascii", errors="ignore")


def fix_length(values, length=1000, pad_value=0):
    """
    Returns a list that is exactly `length` long.
    - If `values` is shorter, it pads with pad_value.
    - If `values` is longer, it truncates.
    """
    if len(values) > length:
        return values[:length]

    if len(values) < length:
        return values + [pad_value] * (length - len(values))

    return values


# ------------------------------------------------------------
# Array generation
# ------------------------------------------------------------
def generateDMAArrays(tSPs, freqSPs, dutyCycle, pulseLength):
    # print('running')
    ## Controller constants
    clck_speed = 80e6  # (Hz)
    deadTime = 0.04  # (us)

    ## Check if inputs are valid ##
    # if (len(freqSPs)!=len(tSPs)):
    #    raise ValueError("There must be one time for each frequency setpoint.")
    # if ((max(tSPs)>pulseLength)|(min(tSPs)<0)|(tSPs[0]!=0)|(tSPs[-1]!=pulseLength)):
    #    raise ValueError("Setpoint times are invalid.")

    for i in range(len(tSPs) - 1):
        if tSPs[i] >= tSPs[i + 1]:
            raise ValueError("Setpoint times are out of order.")

    ## Generate Auto Reload Register (counter period) (ARR) and Capture/Compare Register ##
    ## (output change state value) (CCR) values to match desired frequency ramp. ##
    ## This works by simulating the master timer and generating an ARR and CCR value at each update based on the ##
    ## interpolated input frequency at that time. ##

    # initializing arrays
    t_vals = np.linspace(
        0, pulseLength, int(clck_speed * pulseLength)
    )  # Time value array for each clock tick during a pulse
    freqOutUp = []  # Frequency at each master timer (TIM1) update
    tUp = []  # Time at each master timer (TIM1) update
    baseARRs = [
        0
    ]  # Raw ARR value based on desired frequency without accounting for deadtime or differing PWM modes
    baseCCRs = [
        0
    ]  # Raw CCR value based on desired frequency and duty cycle without accounting for deadtime or differing PWM modes

    cnt = 0  # simulated counter value

    # Simulating the master timer by counting up on each clock tick
    for i in range(len(t_vals)):
        # Check if cnt is at auto reload value. If it is, we reset count to 0 and calculate the ARR and CCR for the next timer cycle
        if cnt == baseARRs[-1]:
            cnt = 0
            # Checking which frequency setpoints to interpolate between by checking when in the shot we are
            for j in range(len(tSPs) - 1):
                if (t_vals[i] >= tSPs[j]) & (t_vals[i] < tSPs[j + 1]):
                    # linear interpolation between previous frequency setpoint and next frequency setpoint
                    curr_freq = freqSPs[j] + (freqSPs[j + 1] - freqSPs[j]) * (
                        t_vals[i] - tSPs[j]
                    ) / (tSPs[j + 1] - tSPs[j])

                    freqOutUp.append(
                        curr_freq
                    )  # Add calculated frequency to the array of frequency values at each update event
                    tUp.append(
                        t_vals[i]
                    )  # Add time of this update event to update event time array

                    baseARRs.append(
                        np.round(80e6 / curr_freq)
                    )  # calculating and storing base ARR value
                    baseCCRs.append(
                        np.round(baseARRs[-1] * dutyCycle / 100)
                    )  # calculating and storing base CCR value
        cnt += 1  # incrementing the counter

    print(baseARRs)

    # removing initial 0 from beginning of arrays to prevent glitches on startup of PWM
    baseARRs.pop(0)
    baseCCRs.pop(0)
    ## Calculate ARRs for each timer ##
    ARR1 = []
    ARR2 = []
    ARR3 = []
    ARR4 = []
    ARR5 = []
    ARR8 = []
    CCR1 = []
    CCR2 = []
    CCR3 = []
    CCR4 = []
    CCR5 = []
    CCR8 = []

    for i in range(1000):
        ARR1.append(9000)
        ARR2.append(9000)
        ARR3.append(9000)
        ARR4.append(9000)
        ARR5.append(9000)
        ARR8.append(9000)
        CCR1.append(0)
        CCR2.append(9000)
        CCR3.append(9000)
        CCR4.append(9000)
        CCR5.append(9000)
        CCR8.append(9000)

    for i in range(len(baseARRs)):
        # TIM1 values (PHASE 1): TIM1 is the master timer. It is in PWM mode 1, so it starts high and transitions to low when the counter reaches the CCR value.
        ARR1[i] = np.round(baseARRs[i])
        CCR1[i] = np.round(baseCCRs[i] - deadTime * 80)

        # TIM2 values (PHASE 1 complimentary AKA PHASE 1N): TIM2 is a slave timer in one pulse mode and PWM mode 2, and is triggered off the leading edge of TIM1, so it stays low until exactly 1/2 of the
        # timer period. Then it goes high until it reaches it ARR. This means that the ARR must be adjusted for the desired duty cycle and dead time.
        # If duty cycle is 50% it is important that the deadtime is not 0, otherwise it will not be for ready the trigger from TIM1.
        ARR2[i] = np.round(
            baseARRs[i] - baseARRs[i] * (50 - dutyCycle) / 100 - deadTime * 80
        )
        CCR2[i] = np.round(baseARRs[i] / 2)

        # TIM3 values (PHASE 2): TIM3 is a slave timer in one pulse mode and PWM mode 2, and is triggered off the leading edge of TIM1, so it stays low until exactly 1/3 of the
        # timer period. Then it goes high until it reaches it ARR. This means that the ARR must be adjusted for the desired duty cycle and dead time.
        ARR3[i] = np.round(
            baseARRs[i] * 5 / 6 - baseARRs[i] * (50 - dutyCycle) / 100 - deadTime * 80
        )
        CCR3[i] = np.round(baseARRs[i] / 3)

        # TIM4 values (PHASE 2 complimentary AKA PHASE 2N): TIM4 is a slave timer in one pulse mode and PWM mode 2, and it is triggered off the leading edge of TIM3.
        # Just like TIM2, it stays low for 1/2 of the counter period, and then goes high until it reaches the ARR value, meaning that it's ARR value controls
        # deadtime and duty cycle.
        ARR4[i] = np.round(
            baseARRs[i] - baseARRs[i] * (50 - dutyCycle) / 100 - deadTime * 80
        )
        CCR4[i] = np.round(baseARRs[i] / 2)

        # TIM5 values (PHASE 3): TIM5 is a slave timer in one pulse mode and PWM mode 2, and is triggered off the leading edge of TIM3, so it stays low until exactly 1/3 of the
        # timer period. Then it goes high until it reaches it ARR. This means that the ARR must be adjusted for the desired duty cycle and dead time.
        ARR5[i] = np.round(
            baseARRs[i] * 5 / 6 - baseARRs[i] * (50 - dutyCycle) / 100 - deadTime * 80
        )
        CCR5[i] = np.round(baseARRs[i] / 3)

        # TIM8 values (PHASE # complimentary AKA PHASE 3N): TIM8 is a slave timer in one pulse mode and PWM mode 2, and it is triggered off the leading edge of TIM5.
        # Just like TIM2, it stays low for 1/2 of the counter period, and then goes high until it reaches the ARR value, meaning that it's ARR value controls
        # deadtime and duty cycle.
        ARR8[i] = np.round(
            baseARRs[i] - baseARRs[i] * (50 - dutyCycle) / 100 - deadTime * 80
        )
        CCR8[i] = np.round(baseARRs[i] / 2)

    for i in range(len(baseARRs) - 10, len(baseARRs) + 100):
        CCR1[i] = 0
        ARR2[i] = 9000
        ARR3[i] = 9000
        ARR4[i] = 9000
        ARR5[i] = 9000
        ARR8[i] = 9000
        CCR2[i] = ARR2[i]
        CCR3[i] = ARR3[i]
        CCR4[i] = ARR4[i]
        CCR5[i] = ARR5[i]
        CCR8[i] = ARR8[i]
    for i in range(0, 10):
        CCR1[i] = 0
        CCR2[i] = ARR2[i]
        CCR3[i] = ARR3[i]
        CCR4[i] = ARR4[i]
        CCR5[i] = ARR5[i]
        CCR8[i] = ARR8[i]
    data = np.array(
        [ARR1, ARR2, ARR3, ARR4, ARR5, ARR8, CCR1, CCR2, CCR3, CCR4, CCR5, CCR8]
    )
    data = data.astype(int)
    return data


# ------------------------------------------------------------
# Array upload functionality
# ------------------------------------------------------------


def upload_array(ser, index, values):
    """
    Uploads a numeric array to the STM32.
    Ensures correct length (1000 entries) and correct binary format (4-byte LE).
    """

    # Fix length to exactly 1000 values
    # values_fixed = fix_length(values, 1000, pad_value=0)

    # Convert each entry to 32-bit little-endian
    payload = b"".join(struct.pack("<I", v) for v in values)

    # Sanity check (should be exactly 4000 bytes)
    assert len(payload) == 4000, f"Payload is {len(payload)} bytes, expected 4000"

    # Trigger the STM32 SCPI receive mode
    # cmd = f"TEST:RECEIVE\n"
    cmd = f"SOUR:ARRA {index}"
    send_scpi(ser, cmd)

    # Short delay (STM32 enters receiveArray() handler)
    time.sleep(0.2)

    # Send raw bytes
    ser.write(payload)
    time.sleep(0.5)
    ser.flush()

    print(f"Sent array index {index}: {len(values)} values (4000 bytes)")


# ------------------------------------------------------------
# Plotting functionality
# ------------------------------------------------------------


def plot_arrays(dma_arrays):
    """
    Creates two plots:
    1. Frequency arrays (dma_arrays 0-5)
    2. Duty cycle arrays (dma_arrays 6-11)
    Saves both plots as PNG images.
    """
    # Create index array (0 to 999)
    indices = np.arange(len(dma_arrays[0]))

    # Plot 1: Frequency Arrays (ARR values)
    plt.figure(figsize=(12, 6))

    labels_freq = [
        "ARR1 (TIM1)",
        "ARR2 (TIM2)",
        "ARR3 (TIM3)",
        "ARR4 (TIM4)",
        "ARR5 (TIM5)",
        "ARR8 (TIM8)",
    ]

    for i in range(6):
        plt.scatter(
            indices, dma_arrays[i], label=labels_freq[i], alpha=0.7, linewidth=1.5
        )

    plt.xlabel("Index", fontsize=12)
    plt.ylabel("ARR Value", fontsize=12)
    plt.title("Frequency Control Arrays (ARR Values)", fontsize=14, fontweight="bold")
    plt.xlim(0, 100)
    plt.legend(loc="best", fontsize=10)
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig("frequency_arrays.png", dpi=300, bbox_inches="tight")
    print("Saved frequency arrays plot to frequency_arrays.png")
    plt.close()

    # Plot 2: Duty Cycle Arrays (CCR values)
    plt.figure(figsize=(12, 6))

    labels_duty = [
        "CCR1 (TIM1)",
        "CCR2 (TIM2)",
        "CCR3 (TIM3)",
        "CCR4 (TIM4)",
        "CCR5 (TIM5)",
        "CCR8 (TIM8)",
    ]

    for i in range(6, 12):
        plt.scatter(
            indices, dma_arrays[i], label=labels_duty[i - 6], alpha=0.7, linewidth=1.5
        )

    plt.xlabel("Index", fontsize=12)
    plt.ylabel("CCR Value", fontsize=12)
    plt.title("Duty Cycle Control Arrays (CCR Values)", fontsize=14, fontweight="bold")
    plt.xlim(0, 100)
    plt.legend(loc="best", fontsize=10)
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig("duty_cycle_arrays.png", dpi=300, bbox_inches="tight")
    print("Saved duty cycle arrays plot to duty_cycle_arrays.png")
    plt.close()


# ------------------------------------------------------------
# Main function
# ------------------------------------------------------------
#
def main():
    ser = open_port()

    time.sleep(0.2)

    # ------------------------------------------------------------
    # Define pulse parameters
    # ------------------------------------------------------------
    # Time setpoints (in seconds)
    tSPs = [0, 8e-3]

    # Frequency setpoints (in Hz) corresponding to each time setpoint
    freqSPs = [19000, 20000]  # Example: ramp from 5kHz to 10kHz

    # Duty cycle (percentage, 0-100)
    dutyCycle = 50  # 50% duty cycle

    # Total pulse length (in seconds)
    pulseLength = 8e-3  # 8ms pulse

    # ------------------------------------------------------------
    # Generate DMA arrays
    # ------------------------------------------------------------
    dma_arrays = generateDMAArrays(tSPs, freqSPs, dutyCycle, pulseLength)
    print(f"DMA Arrays shape: {dma_arrays.shape}")

    # In array_upload.py, after generating arrays:
    print("First 20 values of dma_arrays[0]:", dma_arrays[0][:20])
    print("Last 10 values of dma_arrays[0]:", dma_arrays[0][-10:])
    print("First 20 values of dma_arrays[1]:", dma_arrays[1][:20])
    print("Last 10 values of dma_arrays[1]:", dma_arrays[1][-10:])
    print("First 20 values of dma_arrays[2]:", dma_arrays[2][:20])
    print("Last 10 values of dma_arrays[2]:", dma_arrays[2][-10:])
    print("First 20 values of dma_arrays[3]:", dma_arrays[3][:20])
    print("Last 10 values of dma_arrays[3]:", dma_arrays[3][-10:])
    print("First 20 values of dma_arrays[4]:", dma_arrays[4][:20])
    print("Last 10 values of dma_arrays[4]:", dma_arrays[4][-10:])
    print("First 20 values of dma_arrays[5]:", dma_arrays[5][:20])
    print("Last 10 values of dma_arrays[5]:", dma_arrays[5][-10:])
    print("First 20 values of dma_arrays[6]:", dma_arrays[6][:20])
    print("Last 10 values of dma_arrays[6]:", dma_arrays[6][-10:])
    print("First 20 values of dma_arrays[7]:", dma_arrays[7][:20])
    print("Last 10 values of dma_arrays[7]:", dma_arrays[7][-10:])
    print("First 20 values of dma_arrays[8]:", dma_arrays[8][:20])
    print("Last 10 values of dma_arrays[8]:", dma_arrays[8][-10:])
    print("First 20 values of dma_arrays[9]:", dma_arrays[9][:20])
    print("Last 10 values of dma_arrays[9]:", dma_arrays[9][-10:])
    print("First 20 values of dma_arrays[10]:", dma_arrays[10][:20])
    print("Last 10 values of dma_arrays[10]:", dma_arrays[10][-10:])
    print("First 20 values of dma_arrays[11]:", dma_arrays[11][:20])
    print("Last 10 values of dma_arrays[11]:", dma_arrays[11][-10:])

    # Upload arrays to STM32
    upload_array(ser, 1, dma_arrays[0])  # FreqRampArray
    upload_array(ser, 2, dma_arrays[1])  # FreqRampArray2
    upload_array(ser, 3, dma_arrays[2])  # FreqRampArray3
    upload_array(ser, 4, dma_arrays[3])  # FreqRampArray4
    upload_array(ser, 5, dma_arrays[4])  # FreqRampArray5
    upload_array(ser, 8, dma_arrays[5])  # FreqRampArray6
    upload_array(ser, 11, dma_arrays[6])  # DutyRampArray
    upload_array(ser, 12, dma_arrays[7])  # DutyRampArray2
    upload_array(ser, 13, dma_arrays[8])  # DutyRampArray3
    upload_array(ser, 14, dma_arrays[9])  # DutyRampArray4
    upload_array(ser, 15, dma_arrays[10])  # DutyRampArray5
    upload_array(ser, 18, dma_arrays[11])  # DutyRampArray8

    time.sleep(1)

    # Plot the arrays after transmission
    print("\nGenerating plots...")
    plot_arrays(dma_arrays)

    # Create arrays directory if it doesn't exist
    os.makedirs("./arrays", exist_ok=True)

    # Array names for the filenames
    array_names = [
        "ARR1_TIM1",
        "ARR2_TIM2",
        "ARR3_TIM3",
        "ARR4_TIM4",
        "ARR5_TIM5",
        "ARR8_TIM8",
        "CCR1_TIM1",
        "CCR2_TIM2",
        "CCR3_TIM3",
        "CCR4_TIM4",
        "CCR5_TIM5",
        "CCR8_TIM8",
    ]

    # Save each array to its own text file
    for i, (array, name) in enumerate(zip(dma_arrays, array_names)):
        filename = f"./arrays/{name}.txt"
        with open(filename, "w") as f:
            # Write each value on its own line
            for value in array:
                f.write(f"{value}\n")
        print(f"Saved {name} to {filename} ({len(array)} values)")

    print(f"\nAll arrays saved to ./arrays/ directory\n")

    ser.close()


if __name__ == "__main__":
    main()
