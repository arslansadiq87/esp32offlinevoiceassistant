import serial
import wave
import time
import os
import math
import sys
from array import array

PORT = "COM15"          # Change this to your ESP32-S3 port
BAUD = 921600
SAMPLE_RATE = 16000
MARVIN_DURATION_SECONDS = 2.0
ACTION_DURATION_SECONDS = 2.5
NOISE_DURATION_SECONDS = 2.5
COMMAND_RETRY_SECONDS = 0.5
START_TIMEOUT_SECONDS = 15
BYTES_PER_SAMPLE = 2
QUALITY_FRAME_MS = 20
COMMAND_HEAD_CHECK_MS = 120
COMMAND_TAIL_CHECK_MS = 300
MIN_COMMAND_PEAK = 1200
MIN_COMMAND_RMS = 120
CLIPPING_SAMPLE_LEVEL = 32760
CLIPPING_WARN_PERCENT = 0.05

LABELS = ["marvin", "turn_on", "turn_off", "noise"]
LABEL_CONFIG = {
    "marvin": (MARVIN_DURATION_SECONDS, b"M\n"),
    "turn_on": (ACTION_DURATION_SECONDS, b"O\n"),
    "turn_off": (ACTION_DURATION_SECONDS, b"F\n"),
    "noise": (NOISE_DURATION_SECONDS, b"N\n"),
}
MIN_COMMAND_VOICE_MS = {
    "marvin": 250,
    "turn_on": 350,
    "turn_off": 350,
}

def save_wav(filename, pcm_data):
    with wave.open(filename, "wb") as wav:
        wav.setnchannels(1)
        wav.setsampwidth(2)
        wav.setframerate(SAMPLE_RATE)
        wav.writeframes(pcm_data)

def pcm16_samples(pcm_data):
    samples = array("h")
    samples.frombytes(pcm_data)

    if sys.byteorder != "little":
        samples.byteswap()

    return samples

def calculate_rms(samples):
    if not samples:
        return 0.0

    return math.sqrt(sum(sample * sample for sample in samples) / len(samples))

def calculate_frame_rms(samples):
    frame_size = max(1, int(SAMPLE_RATE * QUALITY_FRAME_MS / 1000))
    frame_rms = []

    for start in range(0, len(samples), frame_size):
        frame = samples[start:start + frame_size]
        if len(frame) == frame_size:
            frame_rms.append(calculate_rms(frame))

    return frame_rms

def estimate_voice_activity(frame_rms, overall_rms):
    if not frame_rms:
        return [], 0.0

    sorted_rms = sorted(frame_rms)
    noise_floor_index = max(0, int(len(sorted_rms) * 0.2) - 1)
    noise_floor = sorted_rms[noise_floor_index]
    threshold = max(250.0, noise_floor * 3.5, overall_rms * 0.25)

    return [value >= threshold for value in frame_rms], threshold

def count_active_frames(active_frames, frame_count):
    if frame_count <= 0:
        return 0

    return sum(1 for active in active_frames[-frame_count:] if active)

def analyze_audio_quality(label, pcm_data):
    samples = pcm16_samples(pcm_data)
    sample_count = len(samples)
    warnings = []

    if sample_count == 0:
        return {
            "warnings": ["no audio samples received"],
            "rms": 0.0,
            "peak": 0,
            "clipping_percent": 0.0,
        }

    peak = max(abs(sample) for sample in samples)
    rms = calculate_rms(samples)
    clipped_count = sum(1 for sample in samples if abs(sample) >= CLIPPING_SAMPLE_LEVEL)
    clipping_percent = (clipped_count / sample_count) * 100

    if clipping_percent >= CLIPPING_WARN_PERCENT:
        warnings.append(
            f"clipping detected ({clipping_percent:.2f}% samples near max); mic gain may be too high"
        )

    result = {
        "warnings": warnings,
        "rms": rms,
        "peak": peak,
        "clipping_percent": clipping_percent,
    }

    if label == "noise":
        return result

    frame_rms = calculate_frame_rms(samples)
    active_frames, voice_threshold = estimate_voice_activity(frame_rms, rms)
    voiced_frames = sum(1 for active in active_frames if active)
    voiced_ms = voiced_frames * QUALITY_FRAME_MS
    first_voice_frame = next((i for i, active in enumerate(active_frames) if active), None)
    last_voice_frame = next(
        (i for i in range(len(active_frames) - 1, -1, -1) if active_frames[i]),
        None,
    )

    result.update({
        "voiced_ms": voiced_ms,
        "voice_threshold": voice_threshold,
        "first_voice_ms": None if first_voice_frame is None else first_voice_frame * QUALITY_FRAME_MS,
        "last_voice_ms": None if last_voice_frame is None else (last_voice_frame + 1) * QUALITY_FRAME_MS,
    })

    if peak < MIN_COMMAND_PEAK or rms < MIN_COMMAND_RMS:
        warnings.append(
            f"too quiet (rms {rms:.0f}, peak {peak}); move closer or increase gain"
        )

    min_voiced_ms = MIN_COMMAND_VOICE_MS.get(label, 300)
    if voiced_ms < min_voiced_ms:
        warnings.append(
            f"mostly silence or unclear speech ({voiced_ms} ms detected)"
        )

    head_frames = max(1, int(COMMAND_HEAD_CHECK_MS / QUALITY_FRAME_MS))
    tail_frames = max(1, int(COMMAND_TAIL_CHECK_MS / QUALITY_FRAME_MS))

    if first_voice_frame is not None and first_voice_frame < head_frames:
        warnings.append(
            f"speech starts in first {COMMAND_HEAD_CHECK_MS} ms; beginning may be cut"
        )

    if count_active_frames(active_frames, tail_frames) >= 3:
        warnings.append(
            f"speech still active in last {COMMAND_TAIL_CHECK_MS} ms; command may be cut off"
        )

    return result

def print_quality_report(label, quality):
    print(
        "Quality:",
        f"rms={quality['rms']:.0f}",
        f"peak={quality['peak']}",
        f"clipping={quality['clipping_percent']:.2f}%",
    )

    if label != "noise":
        first_voice = quality.get("first_voice_ms")
        last_voice = quality.get("last_voice_ms")
        first_text = "none" if first_voice is None else f"{first_voice} ms"
        last_text = "none" if last_voice is None else f"{last_voice} ms"
        print(
            "Voice:",
            f"detected={quality.get('voiced_ms', 0)} ms",
            f"first={first_text}",
            f"last={last_text}",
        )

    if quality["warnings"]:
        print("Quality warnings:")
        for warning in quality["warnings"]:
            print(f"- {warning}")
    else:
        print("Quality: OK")

def confirm_save_with_warnings():
    while True:
        answer = input("Save this sample anyway? [y/N]: ").strip().lower()

        if answer in ("y", "yes"):
            return True
        if answer in ("", "n", "no"):
            return False

        print("Enter y to save or n to discard.")

def get_next_filename(label):
    os.makedirs(label, exist_ok=True)
    existing = [f for f in os.listdir(label) if f.endswith(".wav")]
    number = len(existing) + 1
    return os.path.join(label, f"{label}_{number:03d}.wav")

def read_exact(ser, byte_count, timeout_seconds):
    data = bytearray()
    deadline = time.time() + timeout_seconds

    while len(data) < byte_count and time.time() < deadline:
        chunk = ser.read(byte_count - len(data))
        if chunk:
            data.extend(chunk)

    return bytes(data)

def wait_for_marker(ser, marker, timeout_seconds):
    deadline = time.time() + timeout_seconds

    while time.time() < deadline:
        line = ser.readline().decode(errors="ignore").strip()
        if line:
            print("ESP32:", line)
        if line == marker:
            return True

    return False

def wait_for_start_audio(ser, command):
    print("Sending record command and waiting for START_AUDIO...")

    start_time = time.time()
    last_command_time = 0

    while True:
        now = time.time()

        if now - start_time > START_TIMEOUT_SECONDS:
            print("Timeout: ESP32 did not send START_AUDIO")
            return False

        if now - last_command_time >= COMMAND_RETRY_SECONDS:
            ser.write(command)
            ser.flush()
            last_command_time = now

        line = ser.readline().decode(errors="ignore").strip()
        if line:
            print("ESP32:", line)
        if line == "START_AUDIO":
            return True

def record_sample(ser, label, duration_seconds, command):
    filename = get_next_filename(label)
    bytes_to_read = int(SAMPLE_RATE * duration_seconds * BYTES_PER_SAMPLE)

    ser.reset_input_buffer()
    ser.reset_output_buffer()

    if not wait_for_start_audio(ser, command):
        return

    print(f"Receiving {duration_seconds:g} second audio clip...")
    pcm_data = read_exact(ser, bytes_to_read, duration_seconds + 20)

    if len(pcm_data) != bytes_to_read:
        print(f"Error: expected {bytes_to_read} bytes, got {len(pcm_data)} bytes")
        return

    if not wait_for_marker(ser, "END_AUDIO", 5):
        print("Warning: audio bytes received but END_AUDIO marker was not seen")

    quality = analyze_audio_quality(label, pcm_data)
    print_quality_report(label, quality)

    if quality["warnings"] and not confirm_save_with_warnings():
        print("Discarded sample")
        return

    save_wav(filename, pcm_data)
    print(f"Saved: {filename}")

def main():
    print("Available labels:")
    for i, label in enumerate(LABELS, start=1):
        print(f"{i}. {label}")

    ser = serial.Serial(PORT, BAUD, timeout=0.1, write_timeout=1)
    ser.dtr = True
    ser.rts = False
    time.sleep(3)
    ser.reset_input_buffer()
    ser.reset_output_buffer()

    while True:
        print("\nChoose label:")
        print("1 = marvin")
        print("2 = turn_on")
        print("3 = turn_off")
        print("4 = noise")
        print("q = quit")

        choice = input("Enter choice: ").strip()

        if choice == "q":
            break

        if choice not in ["1", "2", "3", "4"]:
            print("Invalid choice")
            continue

        label = LABELS[int(choice) - 1]

        duration_seconds, command = LABEL_CONFIG[label]

        while True:
            if label == "noise":
                prompt = "Press Enter to record 2.5 seconds of real background noise, or any other key for menu: "
            else:
                prompt = f"Press Enter, then say: {label}. Any other key returns to menu: "

            repeat_choice = input(prompt).strip()
            if repeat_choice:
                break

            record_sample(ser, label, duration_seconds, command)

    ser.close()

if __name__ == "__main__":
    main()
