"""
stress_test.py  —  Pylon solenoid + telemetry stress test
==========================================================
Solenoid cycle  : every 6 s, hold open 4.5 s then off 1.5 s
Telemetry log   : every ~2 s, write all fields to CSV with PC timestamp
CSV flush       : every 60 s
Run             : python stress_test.py [host]
Stop            : Ctrl-C

Default host: http://testnodex.local
"""

import csv
import datetime
import sys
import threading
import time
import urllib.error
import urllib.request

# ---------------------------------------------------------------------------
# Config
# ---------------------------------------------------------------------------
HOST            = sys.argv[1] if len(sys.argv) > 1 else "http://testnodex.local"
SOLENOID_PERIOD = 6.0    # seconds between cycle starts
SOLENOID_ON     = 4.5    # seconds solenoid held open
TELEMETRY_PERIOD = 2.0   # seconds between telemetry samples
FLUSH_PERIOD    = 60.0   # seconds between CSV flushes
HTTP_TIMEOUT    = 5.0    # seconds for each HTTP call

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def api(path, method="GET"):
    url = HOST.rstrip("/") + path
    req = urllib.request.Request(url, method=method)
    try:
        with urllib.request.urlopen(req, timeout=HTTP_TIMEOUT) as r:
            import json
            return json.loads(r.read().decode())
    except Exception as exc:
        print(f"  [HTTP {method} {path}] {exc}")
        return None


def flatten(data, prefix=""):
    """Recursively flatten nested dicts.  Lists become semicolon-joined strings."""
    out = {}
    for k, v in data.items():
        key = f"{prefix}{k}" if prefix else k
        if isinstance(v, dict):
            out.update(flatten(v, key + "."))
        elif isinstance(v, list):
            out[key] = ";".join(str(x) for x in v)
        else:
            out[key] = v
    return out


def csv_filename():
    ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    return f"pylon_stress_{ts}.csv"


# ---------------------------------------------------------------------------
# Shared state
# ---------------------------------------------------------------------------
stop_event = threading.Event()

# ---------------------------------------------------------------------------
# Solenoid thread
# ---------------------------------------------------------------------------

def solenoid_thread():
    print(f"[solenoid] cycle: on {SOLENOID_ON}s / off {SOLENOID_PERIOD - SOLENOID_ON}s, period {SOLENOID_PERIOD}s")
    while not stop_event.is_set():
        cycle_start = time.monotonic()

        resp = api("/api/solenoid/on", method="POST")
        if resp:
            print(f"  [solenoid] ON  -> {resp.get('solenoid_active')}")

        # Wait SOLENOID_ON seconds (interruptible)
        stop_event.wait(SOLENOID_ON)
        if stop_event.is_set():
            break

        resp = api("/api/solenoid/off", method="POST")
        if resp:
            print(f"  [solenoid] OFF -> {resp.get('solenoid_active')}")

        # Sleep remainder of the period
        elapsed = time.monotonic() - cycle_start
        remaining = SOLENOID_PERIOD - elapsed
        if remaining > 0:
            stop_event.wait(remaining)


# ---------------------------------------------------------------------------
# Telemetry / CSV thread
# ---------------------------------------------------------------------------

def telemetry_thread():
    filename = csv_filename()
    print(f"[telemetry] logging to {filename}")

    csvfile = open(filename, "w", newline="", buffering=1)
    writer = None
    fieldnames = None
    last_flush = time.monotonic()
    sample_count = 0

    try:
        while not stop_event.is_set():
            loop_start = time.monotonic()

            data = api("/api/telemetry")
            if data is not None:
                row = {"_timestamp": datetime.datetime.now().isoformat(timespec="milliseconds")}
                row.update(flatten(data))

                if writer is None:
                    fieldnames = list(row.keys())
                    writer = csv.DictWriter(csvfile, fieldnames=fieldnames, extrasaction="ignore")
                    writer.writeheader()
                else:
                    # Add any new keys that weren't in the first row
                    for key in row:
                        if key not in fieldnames:
                            fieldnames.append(key)

                writer.writerow(row)
                sample_count += 1
                batt = row.get("battery_voltage_v", row.get("telemetry.battery_voltage_v", "?"))
                temp = row.get("temperature_f",     row.get("telemetry.temperature_f",     "?"))
                ping = row.get("telemetry.ping.last_ms", "?")
                sol  = row.get("solenoid_active", "?")
                print(f"  [telemetry #{sample_count}] batt={batt}V  temp={temp}°F  ping={ping}ms  solenoid={sol}")

            # Flush CSV every FLUSH_PERIOD seconds
            if time.monotonic() - last_flush >= FLUSH_PERIOD:
                csvfile.flush()
                last_flush = time.monotonic()
                print(f"  [csv] flushed ({sample_count} rows so far)")

            # Sleep the remainder of the 2-second interval
            elapsed = time.monotonic() - loop_start
            remaining = TELEMETRY_PERIOD - elapsed
            if remaining > 0:
                stop_event.wait(remaining)

    finally:
        csvfile.flush()
        csvfile.close()
        print(f"[telemetry] closed {filename}  ({sample_count} rows total)")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    print(f"Pylon stress test  ->  {HOST}")
    print(f"  Solenoid: on {SOLENOID_ON}s every {SOLENOID_PERIOD}s")
    print(f"  Telemetry: every {TELEMETRY_PERIOD}s  |  CSV flush every {FLUSH_PERIOD}s")
    print("  Ctrl-C to stop\n")

    # Confirm connectivity
    info = api("/api/telemetry")
    if info is None:
        print(f"ERROR: cannot reach {HOST}  — check host and Wi-Fi")
        sys.exit(1)
    print(f"  Connected: {info.get('pylon_id','?')}  FW {info.get('fw_version','?')}\n")

    t_solenoid  = threading.Thread(target=solenoid_thread,  daemon=True, name="solenoid")
    t_telemetry = threading.Thread(target=telemetry_thread, daemon=True, name="telemetry")

    t_solenoid.start()
    t_telemetry.start()

    try:
        while t_solenoid.is_alive() or t_telemetry.is_alive():
            time.sleep(0.25)
    except KeyboardInterrupt:
        print("\nStopping…")
        stop_event.set()
        # Make sure solenoid ends up off
        api("/api/solenoid/off", method="POST")
        t_solenoid.join(timeout=3)
        t_telemetry.join(timeout=5)
        print("Done.")


if __name__ == "__main__":
    main()
