"""
BleuIO + HibouAir + Cloudflare D1

Reads CO2, temperature and humidity from a HibouAir BLE advertisement
using BleuIO over USB serial, then sends the decoded values to a
Cloudflare Worker backed by D1.

Sensor scans start approximately every 30 seconds.
"""

import re
import time
from pathlib import Path

import requests
import serial


# ---------------------------------------------------------
# Application settings
# ---------------------------------------------------------

CONFIG_FILE = Path(__file__).with_name("info.txt")

SCAN_SECONDS = 5
SERIAL_READ_EXTRA_SECONDS = 2



SERIAL_TIMEOUT_SECONDS = 0.5
HTTP_TIMEOUT_SECONDS = 10

HIBOUAIR_ADV_PREFIX = "5B070"

# Long hexadecimal BLE advertising payload
LONG_HEX_PATTERN = re.compile(r"[0-9A-Fa-f]{40,}")

DEBUG_RAW_SCAN_LINES = False

# BleuIO USB CDC has been used at different configured baud rates
# in BleuIO examples. We test both automatically.
BAUD_RATES_TO_TRY = (115200, 9600)


# ---------------------------------------------------------
# Configuration
# ---------------------------------------------------------

def load_config():
    config = {}

    if not CONFIG_FILE.exists():
        raise RuntimeError(
            f"Configuration file not found: {CONFIG_FILE}"
        )

    with CONFIG_FILE.open("r", encoding="utf-8") as file:
        for raw_line in file:
            line = raw_line.strip()

            if not line or line.startswith("#"):
                continue

            if "=" not in line:
                continue

            key, value = line.split("=", 1)
            config[key.strip()] = value.strip()

    required = [
        "BLEUIO_PORT",
        "HIBOUAIR_SENSOR_ID",
        "CLOUDFLARE_API_URL",
        "CLOUDFLARE_API_KEY",
        "SCAN_INTERVAL_SECONDS"
    ]

    missing = [
        key for key in required
        if not config.get(key)
    ]

    if missing:
        raise RuntimeError(
            "Missing configuration: " + ", ".join(missing)
        )

    return config


# ---------------------------------------------------------
# BleuIO serial communication
# ---------------------------------------------------------

def send_command(connection, command):
    """
    Send an AT command to BleuIO.
    """
    connection.write((command + "\r\n").encode("ascii"))
    connection.flush()


def bleuio_responds(connection):
    """
    Check that the currently opened serial connection is BleuIO.
    """
    connection.reset_input_buffer()

    # Give the serial device a moment after opening.
    time.sleep(0.2)

    send_command(connection, "AT")

    deadline = time.monotonic() + 3

    while time.monotonic() < deadline:
        raw_line = connection.readline()

        if not raw_line:
            continue

        line = raw_line.decode(
            "utf-8",
            errors="replace"
        ).strip()

        if line:
            print(f"  BleuIO: {line}")

        if "OK" in line.upper().split():
            return True

    return False


def open_bleuio(port):
    """
    Try known BleuIO serial baud rates and return
    the first working serial connection.
    """

    for baud_rate in BAUD_RATES_TO_TRY:

        print(
            f"Trying BleuIO on {port} "
            f"at {baud_rate} baud..."
        )

        try:
            connection = serial.Serial(
                port=port,
                baudrate=baud_rate,
                timeout=SERIAL_TIMEOUT_SECONDS,
                write_timeout=2,
            )

        except serial.SerialException as error:
            raise RuntimeError(
                f"Could not open BleuIO port {port}: {error}"
            )

        try:
            if bleuio_responds(connection):
                print(
                    f"BleuIO connected successfully "
                    f"at {baud_rate} baud."
                )

                return connection

        except Exception:
            connection.close()
            raise

        connection.close()

    raise RuntimeError(
        "Serial port opened, but BleuIO did not respond "
        "to the AT command."
    )


def set_central_mode(connection):
    """
    Put BleuIO into central mode.
    """
    connection.reset_input_buffer()

    send_command(connection, "AT+CENTRAL")

    deadline = time.monotonic() + 2

    while time.monotonic() < deadline:
        raw_line = connection.readline()

        if not raw_line:
            continue

        line = raw_line.decode(
            "utf-8",
            errors="replace"
        ).strip()

        if not line:
            continue

        upper = line.upper()

        if "OK" in upper or "ERROR" in upper:
            return


# ---------------------------------------------------------
# HibouAir decoder
# ---------------------------------------------------------

def read_uint16_le(hex_string, start):
    """
    Read two bytes from a hexadecimal string
    as an unsigned little-endian 16-bit integer.
    """
    raw = hex_string[start:start + 4]

    if len(raw) != 4:
        return None

    try:
        raw_bytes = bytes.fromhex(raw)
        return int.from_bytes(
            raw_bytes,
            byteorder="little",
            signed=False
        )

    except ValueError:
        return None


def read_int16_le(hex_string, start):
    """
    Read two bytes from a hexadecimal string
    as a signed little-endian 16-bit integer.
    """
    raw = hex_string[start:start + 4]

    if len(raw) != 4:
        return None

    try:
        raw_bytes = bytes.fromhex(raw)

        return int.from_bytes(
            raw_bytes,
            byteorder="little",
            signed=True
        )

    except ValueError:
        return None


def decode_hibouair_payload(hex_payload):
    """
    Decode HibouAir CO2, temperature and humidity.

    HibouAir advertisement layout used here:

        temperature : pos + 22, 2-byte LE signed, /10
        humidity    : pos + 26, 2-byte LE unsigned, /10
        CO2         : pos + 46, 2-byte hexadecimal value
    """

    if not hex_payload:
        return None

    payload = re.sub(
        r"[^0-9A-Fa-f]",
        "",
        hex_payload
    ).upper()

    pos = payload.find(HIBOUAIR_ADV_PREFIX)

    if pos == -1:
        return None

    # Need enough data to reach CO2 field.
    if len(payload) < pos + 50:
        return None

    temperature_raw = read_int16_le(
        payload,
        pos + 22
    )

    humidity_raw = read_uint16_le(
        payload,
        pos + 26
    )

    try:
        co2 = int(
            payload[pos + 46:pos + 50],
            16
        )
    except ValueError:
        return None

    if temperature_raw is None:
        return None

    if humidity_raw is None:
        return None

    temperature = temperature_raw / 10.0
    humidity = humidity_raw / 10.0

    # Basic sanity checking prevents corrupt/incomplete
    # advertisements from reaching the cloud.

    if not -40 <= temperature <= 85:
        return None

    if not 0 <= humidity <= 100:
        return None

    if not 0 <= co2 <= 10000:
        return None

    return {
        "co2": co2,
        "temperature": round(temperature, 1),
        "humidity": round(humidity, 1),
    }


def extract_hex_payload(line):
    """
    Extract BLE advertising hexadecimal data
    from a BleuIO response line.
    """

    matches = LONG_HEX_PATTERN.findall(line)

    if not matches:
        return None

    # Prefer a payload containing the known HibouAir prefix.
    for match in matches:
        if HIBOUAIR_ADV_PREFIX in match.upper():
            return match.upper()

    return max(matches, key=len).upper()


def latest_hibouair_reading(scan_lines):
    """
    Return the latest valid HibouAir advertisement
    found during the scan.
    """

    latest = None

    for line in scan_lines:

        payload = extract_hex_payload(line)

        if not payload:
            continue

        if HIBOUAIR_ADV_PREFIX not in payload:
            continue

        reading = decode_hibouair_payload(payload)

        if reading:
            latest = reading

    return latest


# ---------------------------------------------------------
# BLE scan
# ---------------------------------------------------------

def scan_hibouair(connection, board_id):
    """
    Ask BleuIO to search specifically for the
    HibouAir board ID.
    """

    command = (
        f"AT+FINDSCANDATA="
        f"{board_id}="
        f"{SCAN_SECONDS}"
    )

    connection.reset_input_buffer()

    print(f"Command: {command}")

    send_command(connection, command)

    deadline = (
        time.monotonic()
        + SCAN_SECONDS
        + SERIAL_READ_EXTRA_SECONDS
    )

    scan_lines = []

    while time.monotonic() < deadline:

        raw_line = connection.readline()

        if not raw_line:
            continue

        line = raw_line.decode(
            "utf-8",
            errors="replace"
        ).strip()

        if line:
            scan_lines.append(line)

    if DEBUG_RAW_SCAN_LINES:
        print("\nRaw BleuIO response:")

        for line in scan_lines:
            print(line)

    return latest_hibouair_reading(scan_lines)


# ---------------------------------------------------------
# Cloudflare
# ---------------------------------------------------------

def upload_to_cloudflare(config, reading):
    """
    Upload one decoded HibouAir measurement
    to our Cloudflare Worker.
    """

    payload = {
        "board_id": config["HIBOUAIR_SENSOR_ID"],
        "co2": reading["co2"],
        "temperature": reading["temperature"],
        "humidity": reading["humidity"],
    }

    headers = {
        "Content-Type": "application/json",
        "X-API-Key": config["CLOUDFLARE_API_KEY"],
    }

    try:

        response = requests.post(
            config["CLOUDFLARE_API_URL"],
            json=payload,
            headers=headers,
            timeout=HTTP_TIMEOUT_SECONDS,
        )

        if response.ok:

            try:
                result = response.json()
            except ValueError:
                result = {}

            if result.get("success"):
                return True

        print(
            f"Cloudflare upload failed "
            f"({response.status_code}): "
            f"{response.text}"
        )

        return False

    except requests.RequestException as error:

        print(
            f"Cloudflare request failed: {error}"
        )

        return False


# ---------------------------------------------------------
# Terminal output
# ---------------------------------------------------------

def print_reading(reading):
    print()
    print("=" * 44)
    print("          HIBOUAIR AIR QUALITY")
    print("=" * 44)
    print()
    print(f"  CO2           {reading['co2']:>8} ppm")
    print(
        f"  Temperature   "
        f"{reading['temperature']:>8.1f} °C"
    )
    print(
        f"  Humidity      "
        f"{reading['humidity']:>8.1f} %RH"
    )
    print()
    print("=" * 44)


# ---------------------------------------------------------
# Main program
# ---------------------------------------------------------

def main():

    print()
    print("BleuIO + HibouAir + Cloudflare D1")
    print("---------------------------------")
    print()

    config = load_config()
    

    port = config["BLEUIO_PORT"]
    board_id = config["HIBOUAIR_SENSOR_ID"]
    scan_interval_seconds = int(
        config.get("SCAN_INTERVAL_SECONDS", "30")
    )

    print(f"BleuIO port : {port}")
    print(f"HibouAir ID : {board_id}")
    print(f"Scan interval: {scan_interval_seconds} seconds")
    print()

    connection = open_bleuio(port)

    try:

        print("Setting BleuIO to central mode...")
        set_central_mode(connection)

        print("Ready.")

        while True:

            cycle_started = time.monotonic()

            print()
            print(
                f"Scanning for HibouAir "
                f"{board_id}..."
            )

            reading = scan_hibouair(
                connection,
                board_id
            )

            if reading is None:

                print(
                    "No valid HibouAir advertisement "
                    "found during this scan."
                )

            else:

                print_reading(reading)

                print("Uploading to Cloudflare D1...")

                if upload_to_cloudflare(
                    config,
                    reading
                ):
                    print(
                        "✓ Reading stored successfully."
                    )

                else:
                    print(
                        "✗ Reading was not stored."
                    )

            elapsed = (
                time.monotonic()
                - cycle_started
            )

            sleep_seconds = max(
                0,
                scan_interval_seconds - elapsed
            )

            print(
                f"Next scan in "
                f"{sleep_seconds:.0f} seconds..."
            )

            time.sleep(sleep_seconds)

    except KeyboardInterrupt:

        print()
        print()
        print("Stopped by user.")

    finally:

        if connection.is_open:
            connection.close()

        print("BleuIO serial port closed.")


if __name__ == "__main__":
    main()