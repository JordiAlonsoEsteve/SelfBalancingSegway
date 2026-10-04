import os
import struct
import serial
import multiprocessing as mp
import collections
import queue

import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np

# ==========================================================
# CONFIGURATION
# ==========================================================
SERIAL_PORT = "/dev/rfcomm0"
BAUD_RATE = 500000

PACKET_SIZE = 46  # 2 bytes header + 44 bytes data
BATCH_COUNT = 40
BATCH_TOTAL_BYTES = PACKET_SIZE * BATCH_COUNT

# Format: H (uint16 header), 10 floats, I (uint32 time)
UNPACK_FORMAT = "<HffffffffffI"

# Plot queue sizing: keep it small so the GUI never lags far behind
# reality; if it fills up we drop old points rather than block the reader.
PLOT_QUEUE_MAXSIZE = 4000
MAX_LEN = 400  # points shown on screen


# ==========================================================
# READER PROCESS
# ==========================================================
def reader_process(plot_queue: mp.Queue, stop_event: mp.Event):
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
        print(f"✅ Connected successfully to {SERIAL_PORT}")
    except Exception as e:
        print(f"❌ Failed to open serial port {SERIAL_PORT}: {e}")
        return

    buffer = bytearray()

    print("⏳ Reading live serial stream. Press Ctrl+C to stop.\n")

    try:
        while not stop_event.is_set():
            if ser.in_waiting > 0:
                buffer.extend(ser.read(ser.in_waiting))

            # Resynchronize stream by looking for the 0xABCD header
            while len(buffer) >= PACKET_SIZE:
                if buffer[0:2] != b'\xcd\xab':  # little-endian 0xABCD
                    buffer.pop(0)
                    continue

                if len(buffer) < BATCH_TOTAL_BYTES:
                    break  # wait for a full batch

                batch_bytes = buffer[:BATCH_TOTAL_BYTES]
                del buffer[:BATCH_TOTAL_BYTES]

                for i in range(BATCH_COUNT):
                    pkt_bytes = batch_bytes[i * PACKET_SIZE: (i + 1) * PACKET_SIZE]
                    try:
                        unpacked_data = struct.unpack(UNPACK_FORMAT, pkt_bytes)
                    except struct.error:
                        continue

                    (header, angle, target, rate, pos1, speed1, pos2, speed2,
                     u_left, u_right, time_taken, cur_time) = unpacked_data

                    # Push to the plotting process. Never block the reader:
                    # if the plot queue is full, drop the oldest point.
                    sample = (angle, target, rate, pos1, pos2, speed1, speed2,
                              u_left, u_right, time_taken)
                    try:
                        plot_queue.put_nowait(sample)
                    except queue.Full:
                        try:
                            plot_queue.get_nowait()
                        except queue.Empty:
                            pass
                        try:
                            plot_queue.put_nowait(sample)
                        except queue.Full:
                            pass

    except KeyboardInterrupt:
        pass
    finally:
        print("\n🛑 Reader stopped.")
        ser.close()


# ==========================================================
# PLOTTING PROCESS
# ==========================================================
def plotting_process(plot_queue: mp.Queue, stop_event: mp.Event):
    angles     = collections.deque([0.0] * MAX_LEN, maxlen=MAX_LEN)
    targets    = collections.deque([0.0] * MAX_LEN, maxlen=MAX_LEN)
    rates      = collections.deque([0.0] * MAX_LEN, maxlen=MAX_LEN)
    positions1 = collections.deque([0.0] * MAX_LEN, maxlen=MAX_LEN)
    positions2 = collections.deque([0.0] * MAX_LEN, maxlen=MAX_LEN)
    pos_diff   = collections.deque([0.0] * MAX_LEN, maxlen=MAX_LEN)  # Left - Right
    speeds1    = collections.deque([0.0] * MAX_LEN, maxlen=MAX_LEN)
    speeds2    = collections.deque([0.0] * MAX_LEN, maxlen=MAX_LEN)
    u_lefts    = collections.deque([0.0] * MAX_LEN, maxlen=MAX_LEN)
    u_rights   = collections.deque([0.0] * MAX_LEN, maxlen=MAX_LEN)
    loop_times = collections.deque([0.0] * MAX_LEN, maxlen=MAX_LEN)

    fig, axes = plt.subplots(3, 2, figsize=(12, 9))
    ax1, ax2 = axes[0, 0], axes[0, 1]
    ax3, ax4 = axes[1, 0], axes[1, 1]
    ax5, ax6 = axes[2, 0], axes[2, 1]

    (line_angle,)  = ax1.plot(angles, label="Filtered Angle", color="royalblue")
    (line_target,) = ax1.plot(targets, label="Target Angle", color="crimson", linestyle="--")
    ax1.set_ylabel("Angle (rad)")
    ax1.set_title("Angles")
    ax1.grid(True)
    ax1.legend(loc="upper left")
    ax1.set_ylim(-np.pi / 8, np.pi / 8)

    # --- Wheel Positions + Separation (twin y-axis on ax2) ---
    (line_pos1,) = ax2.plot(positions1, label="Left Pos", color="darkgreen")
    (line_pos2,) = ax2.plot(positions2, label="Right Pos", color="mediumseagreen", linestyle="--")
    ax2.set_ylabel("Position (m)")
    ax2.set_title("Wheel Positions & Separation")
    ax2.grid(True)

    ax2b = ax2.twinx()
    (line_diff,) = ax2b.plot(pos_diff, label="Left − Right", color="black", linestyle=":")
    ax2b.set_ylabel("Difference (m)")

    # Combine legends from both axes into a single legend box
    lines_2, labels_2 = ax2.get_legend_handles_labels()
    lines_2b, labels_2b = ax2b.get_legend_handles_labels()
    ax2.legend(lines_2 + lines_2b, labels_2 + labels_2b, loc="upper left")

    (line_speed1,) = ax3.plot(speeds1, label="Left Speed", color="darkorange")
    (line_speed2,) = ax3.plot(speeds2, label="Right Speed", color="dodgerblue", linestyle="--")
    ax3.set_ylabel("Speed (m/s)")
    ax3.set_title("Wheel Speeds")
    ax3.grid(True)
    ax3.legend(loc="upper left")
    ax3.set_ylim(-1.5, 1.5)

    (line_u_left,)  = ax4.plot(u_lefts, label="Left PWM Norm", color="forestgreen")
    (line_u_right,) = ax4.plot(u_rights, label="Right PWM Norm", color="purple", linestyle="--")
    ax4.set_ylabel("Normalized Effort")
    ax4.set_title("Control Output (u)")
    ax4.grid(True)
    ax4.legend(loc="upper left")
    ax4.set_ylim(-1.2, 1.2)

    (line_time,) = ax5.plot(loop_times, label="Loop Execution Time", color="purple")
    ax5.set_ylabel("Time (ms)")
    ax5.set_xlabel("Sample")
    ax5.set_title("Loop Execution Time")
    ax5.grid(True)
    ax5.legend(loc="upper left")
    ax5.set_ylim(0, 5)

    (line_rate,) = ax6.plot(rates, label="Angular Rate", color="darkcyan")
    ax6.set_ylabel("Rate (rad/s)")
    ax6.set_xlabel("Sample")
    ax6.set_title("Angle Derivative (Rate)")
    ax6.grid(True)
    ax6.legend(loc="upper left")
    ax6.set_ylim(-np.pi / 3, np.pi / 3)

    def update(frame):
        updated = False

        # Drain everything currently available without blocking.
        while True:
            try:
                (angle, target, rate, p1, p2, s1, s2, ul, ur, time_taken) = plot_queue.get_nowait()
            except queue.Empty:
                break

            angles.append(angle)
            targets.append(target)
            rates.append(rate)
            positions1.append(p1)
            positions2.append(p2)
            pos_diff.append(p1 - p2)
            speeds1.append(s1)
            speeds2.append(s2)
            u_lefts.append(ul)
            u_rights.append(ur)
            loop_times.append(time_taken)
            updated = True

        if updated:
            line_angle.set_ydata(angles)
            line_target.set_ydata(targets)
            line_rate.set_ydata(rates)
            line_pos1.set_ydata(positions1)
            line_pos2.set_ydata(positions2)
            line_diff.set_ydata(pos_diff)
            line_speed1.set_ydata(speeds1)
            line_speed2.set_ydata(speeds2)
            line_u_left.set_ydata(u_lefts)
            line_u_right.set_ydata(u_rights)
            line_time.set_ydata(loop_times)

            ax2.relim()
            ax2.autoscale_view()
            ax2b.relim()
            ax2b.autoscale_view()

        return (line_angle, line_target, line_rate, line_pos1, line_pos2, line_diff,
                line_speed1, line_speed2, line_u_left, line_u_right, line_time)

    ani = animation.FuncAnimation(
        fig, update, interval=25, blit=False, cache_frame_data=False
    )

    plt.suptitle("Self Balancing Robot Telemetry", fontsize=14, fontweight="bold")
    plt.tight_layout()
    plt.show()

    stop_event.set()  # closing the plot window signals the reader to stop too


# ==========================================================
# ENTRY POINT
# ==========================================================
def main():
    plot_queue = mp.Queue(maxsize=PLOT_QUEUE_MAXSIZE)
    stop_event = mp.Event()

    reader = mp.Process(target=reader_process, args=(plot_queue, stop_event), daemon=True)
    reader.start()

    try:
        plotting_process(plot_queue, stop_event)
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        reader.join(timeout=2.0)
        if reader.is_alive():
            reader.terminate()


if __name__ == "__main__":
    main()