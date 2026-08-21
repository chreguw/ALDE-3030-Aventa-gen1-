"""
Alde 3030 Plus + Truma Aventa - Red Bus MQTT Bridge with Decoder v17
================================================================
- Original Alde functionality (PID 0x55 → Glykol + Warmwasser + External Start State)
- Decoder for Aventa status (PID 0x97)
- Supports AirconMode + AirconVentMode enums
- Fallback to "Alde Auto" when no AirconMode value is seen
- Clean MQTT Discovery for Home Assistant 

Run this script and operate the Aventa on your device.
"""

import pigpio
import time
import json
import threading
import paho.mqtt.client as mqtt


# ── GPIO / LIN ────────────────────────────────────────────────────────────────
GPIO_PIN = 22 #hier deine RX Pin angeben
TX_PIN   = 27 #hier deine TX Pin angeben, wie das Modul angeschlossen hast
BAUD = 9600

# ── MQTT ──────────────────────────────────────────────────────────────────────
MQTT_HOST = '150.15.150.150'      #IP des MQTT Brokers
MQTT_PORT = 1883                  #dein Standard Port
MQTT_USER = ''                    #falls ein username hier setzen, sonst so lassen
MQTT_PASS = ''                    #falls ein password vorhanden, sonst so lassen
MQTT_CLIENT = 'alde_red_bus_mqtt'

# ── Device ────────────────────────────────────────────────────────────────────
DEVICE = {
    "identifiers": ["alde_3030_plus"],
    "name": "Alde 3030 Plus + Aventa",
    "model": "Compact 3030 Plus + Truma Aventa",
    "manufacturer": "Alde / Truma",
    "sw_version": "v17"
}

# ── Topics ────────────────────────────────────────────────────────────────────
AVAIL_TOPIC = 'alde/red/available'

# Alde
GLYCOL_DISC  = 'homeassistant/sensor/alde_red/alde_glycol/config'
GLYCOL_STATE = 'alde/red/glycol_temp'
HW_DISC      = 'homeassistant/sensor/alde_red/alde_hot_water/config'
HW_STATE     = 'alde/red/hot_water_temp'

# Aventa
AVENTA_AIRCON_MODE_DISC  = 'homeassistant/sensor/alde_red/aventa_aircon_mode/config'
AVENTA_AIRCON_MODE_STATE = 'alde/red/aventa_aircon_mode'
AVENTA_TEMP_DISC  = 'homeassistant/sensor/alde_red/aventa_temp/config'
AVENTA_TEMP_STATE = 'alde/red/aventa_temp'
YELLOW_CLIMATE_STATE = 'alde/climate/state'
YELLOW_AC_MODE_CMD   = 'alde/climate/cmd/ac_mode'

AVENTA_VENT_MODE_DISC    = 'homeassistant/sensor/alde_red/aventa_vent_mode/config'
AVENTA_VENT_MODE_STATE   = 'alde/red/aventa_vent_mode'
CMD_VENT   = 'alde/red/aventa_vent_mode/set'
CMD_AIRCON = 'alde/red/aventa_aircon_mode/set'
CMD_TEMP = 'alde/red/aventa_temp/set'

# ── Shared state ──────────────────────────────────────────────────────────────
last_glycol = None
last_hot_water = None
last_aventa_aircon_mode = None
last_aventa_vent_mode = None
last_aircon_mode_time = 0
last_vent_mode_time = 0
pending_cmd = None
last_aventa_temp = None
last_aventa_temp_time = 0
pi = None
cmd_lock = threading.Lock()
state_lock = threading.Lock()
tx_ready_at = 0.0
TX_WARMUP_S = 5.0
cmd_retry_until = 0.0
cmd_retry_cmd = None
CMD_RETRY_MAX_S = 12.0
CMD_RETRY_EVERY_S = 0.8
last_retry_at = 0.0
CMD_DISPLAY = 'alde/display/cmd'
last_yellow_ac_mode  = None        
pending_after_manual = None          

def arm_tx_warmup(reason=""):
    global tx_ready_at
    tx_ready_at = time.time() + TX_WARMUP_S
    print(f"[TX] Warmup {TX_WARMUP_S}s ({reason})", flush=True)
# ── LIN checksum ──────────────────────────────────────────────────────────────
def chksum_enhanced(pid, data):
    total = pid + sum(data)
    while total > 0xFF:
        total = (total & 0xFF) + (total >> 8)
    return (~total) & 0xFF

def log_frame(pid, data_bytes, chksum, direction="RX"):
    hex_data = ' '.join(f'{b:02X}' for b in data_bytes)
    print(f"[{direction}] PID=0x{pid:02X} data=[{hex_data}] cs=0x{chksum:02X}", flush=True)

# ── Decoder Funktionen ────────────────────────────────────────────────────────
def decode_aircon_mode(value):
    """AirconMode Enum"""
    modes = {
        0x00: "OFF",
        0x04: "VENTILATION",
        0x05: "COOLING",
        0x06: "HEATING",
        0x07: "AUTO",
    }
    return modes.get(value, f"UNKNOWN_0x{value:02X}")

def decode_vent_mode(value):
    """AirconVentMode Enum"""
    modes = {
        0x70: "OFF",          # ← NEU hinzugefügt
        0x71: "LOW",
        0x72: "MID",
        0x73: "HIGH",
        0x74: "NIGHT",
        0x77: "AUTO",
    }
    return modes.get(value, f"UNKNOWN_0x{value:02X}")
# header, 0x0B, vent, mode, t_lo, t_hi, extra, 0xFF
FRAMES = {
    "OFF": [0xA4, 0x0B, 0x70, 0x00, 0x00, 0x00, 0x00, 0xFF],

    # Kühlen + Lüfter  (21°C = 7C 0B)
    "COOLING,LOW":  [0x7C, 0x0B, 0x71, 0x05, 0x00, 0x00, 0x7B, 0xFF],
    "COOLING,MID":  [0x7C, 0x0B, 0x72, 0x05, 0x00, 0x00, 0x7B, 0xFF],
    "COOLING,HIGH": [0x7C, 0x0B, 0x73, 0x05, 0x00, 0x00, 0x7B, 0xFF],

    # nur Lüften
    "VENTILATION,LOW":  [0x7C, 0x0B, 0x71, 0x04, 0x00, 0x00, 0x7B, 0xFF],
    "VENTILATION,MID":  [0x7C, 0x0B, 0x72, 0x04, 0x00, 0x00, 0x7B, 0xFF],
    "VENTILATION,HIGH": [0x7C, 0x0B, 0x73, 0x04, 0x00, 0x00, 0x7B, 0xFF],

    # Heizen
    "HEATING,LOW":  [0x7C, 0x0B, 0x71, 0x06, 0x00, 0x00, 0x7B, 0xFF],
    "HEATING,MID":  [0x7C, 0x0B, 0x72, 0x06, 0x00, 0x00, 0x7B, 0xFF],
    "HEATING,HIGH": [0x7C, 0x0B, 0x73, 0x06, 0x00, 0x00, 0x7B, 0xFF],

    "AUTO": [0x7C, 0x0B, 0x77, 0x07, 0x00, 0x00, 0x7B, 0xFF],

    "COOLING,NIGHT":     [0x7C, 0x0B, 0x74, 0x05, 0x00, 0x00, 0x7B, 0xFF],
    "VENTILATION,NIGHT": [0x7C, 0x0B, 0x74, 0x04, 0x00, 0x00, 0x7B, 0xFF],
    "HEATING,NIGHT":     [0x7C, 0x0B, 0x74, 0x06, 0x00, 0x00, 0x7B, 0xFF],
}
def lin_pid(id_):
    id_ &= 0x3F
    p0 = ((id_ >> 0) ^ (id_ >> 1) ^ (id_ >> 2) ^ (id_ >> 4)) & 1
    p1 = (~((id_ >> 1) ^ (id_ >> 3) ^ (id_ >> 4) ^ (id_ >> 5))) & 1
    return id_ | (p0 << 6) | (p1 << 7)

def build_frame(mode, vent, temp_c=None):
    if temp_c is None:
        with state_lock:
            temp_c = last_aventa_temp if last_aventa_temp is not None else 21.0
    raw = int(round(float(temp_c) * 10) + 2730)
    vents = {"OFF":0x70,"LOW":0x71,"MID":0x72,"HIGH":0x73,"NIGHT":0x74,"AUTO":0x77}
    modes = {"OFF":0x00,"VENTILATION":0x04,"COOLING":0x05,"HEATING":0x06,"AUTO":0x07}
    return [
        raw & 0xFF, (raw >> 8) & 0xFF,
        vents.get(vent, 0x73),
        modes.get(mode, 0x05),
        0x00, 0x00, 0x7B, 0xFF
    ]

def send_lin_frame(pi, pid_id, data8):
    """Break + Sync + PID + 8 Data + enhanced checksum – RX kurz pausieren"""
    pid = lin_pid(pid_id)
    cs = chksum_enhanced(pid, data8)
    frame = [0x55, pid] + list(data8) + [cs]

    bit_us = int(1_000_000 / BAUD)
    break_us = 18 * bit_us

    try:
        pi.bb_serial_read_close(GPIO_PIN)
    except Exception:
        pass

    try:
        pi.wave_clear()
        pi.set_mode(TX_PIN, pigpio.OUTPUT)
        pi.write(TX_PIN, 1)

        wf = []
        wf.append(pigpio.pulse(0, 1 << TX_PIN, break_us))
        wf.append(pigpio.pulse(1 << TX_PIN, 0, bit_us))

        for byte in frame:
            wf.append(pigpio.pulse(0, 1 << TX_PIN, bit_us))
            for i in range(8):
                if (byte >> i) & 1:
                    wf.append(pigpio.pulse(1 << TX_PIN, 0, bit_us))
                else:
                    wf.append(pigpio.pulse(0, 1 << TX_PIN, bit_us))
            wf.append(pigpio.pulse(1 << TX_PIN, 0, bit_us))

        pi.wave_add_generic(wf)
        wid = pi.wave_create()
        pi.wave_send_once(wid)
        while pi.wave_tx_busy():
            time.sleep(0.001)
        pi.wave_delete(wid)
        pi.write(TX_PIN, 1)
        print(f"[TX] PID=0x{pid_id:02X} data={[f'{b:02X}' for b in data8]} cs=0x{cs:02X}", flush=True)
    except Exception as e:
        print(f"[TX] Fehler: {e}", flush=True)
    finally:
        try:
            pi.bb_serial_read_open(GPIO_PIN, BAUD, 8)
        except Exception as e:
            print(f"[GPIO] reopen: {e}", flush=True)
# ── MQTT Discovery ────────────────────────────────────────────────────────────
def publish_discovery(client):
    # Alde
    client.publish(GLYCOL_DISC, json.dumps({
        "name": "Glycol Temperature",
        "unique_id": "alde_red_glycol",
        "object_id": "alde_glycol",
        "state_topic": GLYCOL_STATE,
        "device_class": "temperature",
        "unit_of_measurement": "°C",
        "state_class": "measurement",
        "availability_topic": AVAIL_TOPIC,
        "payload_available": "online",
        "payload_not_available": "offline",
        "device": DEVICE
    }), retain=True)

    client.publish(HW_DISC, json.dumps({
        "name": "Hot Water Temperature",
        "unique_id": "alde_red_hot_water",
        "object_id": "alde_hot_water",
        "state_topic": HW_STATE,
        "device_class": "temperature",
        "unit_of_measurement": "°C",
        "state_class": "measurement",
        "availability_topic": AVAIL_TOPIC,
        "payload_available": "online",
        "payload_not_available": "offline",
        "device": DEVICE
    }), retain=True)

# External Start
    client.publish("homeassistant/binary_sensor/alde_red/external_start/config", json.dumps({
        "name": "External Start",
        "unique_id": "alde_red_external_start",
        "object_id": "alde_external_start",
        "state_topic": "alde/red/external_start",
        "payload_on": "ON",
        "payload_off": "OFF",
        "device_class": "power",
        "availability_topic": AVAIL_TOPIC,
        "payload_available": "online",
        "payload_not_available": "offline",
        "device": DEVICE
    }), retain=True)

    client.publish('homeassistant/select/alde_red/aventa_aircon_mode/config', json.dumps({
        "name": "Aventa Aircon Mode",
        "unique_id": "alde_red_aventa_aircon_mode",
        "state_topic": AVENTA_AIRCON_MODE_STATE,
        "command_topic": CMD_AIRCON,
        "options": ["OFF", "VENTILATION", "COOLING", "HEATING", "AUTO"],
        "availability_topic": AVAIL_TOPIC,
        "payload_available": "online",
        "payload_not_available": "offline",
        "device": DEVICE,
        "icon": "mdi:air-conditioner",
    }), retain=True)

    client.publish('homeassistant/select/alde_red/aventa_vent_mode/config', json.dumps({
        "name": "Aventa Vent Mode",
        "unique_id": "alde_red_aventa_vent_mode",
        "state_topic": AVENTA_VENT_MODE_STATE,
        "command_topic": CMD_VENT,
        "options": ["OFF", "LOW", "MID", "HIGH", "NIGHT", "AUTO"],
        "availability_topic": AVAIL_TOPIC,
        "payload_available": "online",
        "payload_not_available": "offline",
        "device": DEVICE,
        "icon": "mdi:fan",
    }), retain=True)

    client.publish('homeassistant/number/alde_red/aventa_temp/config', json.dumps({
        "name": "Aventa Setpoint",
        "unique_id": "alde_red_aventa_temp_set",
        "state_topic": AVENTA_TEMP_STATE,
        "command_topic": CMD_TEMP,
        "min": 16,
        "max": 30,
        "step": 0.5,
        "unit_of_measurement": "°C",
        "device_class": "temperature",
        "availability_topic": AVAIL_TOPIC,
        "payload_available": "online",
        "payload_not_available": "offline",
        "device": DEVICE,
        "icon": "mdi:thermometer",
    }), retain=True)

    print("[MQTT] Discovery configs published")

# ── Publish helpers ───────────────────────────────────────────────────────────
def publish_state(client, glycol, hot_water):
    client.publish(GLYCOL_STATE, f"{glycol:.1f}", retain=True)
    client.publish(HW_STATE, f"{hot_water:.1f}", retain=True)

def publish_aventa_modes(client, aircon_mode, vent_mode):
    client.publish(AVENTA_AIRCON_MODE_STATE, aircon_mode, retain=True)
    client.publish(AVENTA_VENT_MODE_STATE, vent_mode, retain=True)

# ── Alde Processor (original) ─────────────────────────────────────────────────
def process_0x55(data_bytes, chksum, client):
    global last_glycol, last_hot_water
    if chksum_enhanced(0x55, data_bytes) != chksum:
        return
    glycol = (data_bytes[0] + data_bytes[1] * 256) / 10.0
    hot_water = (data_bytes[2] + data_bytes[3] * 256) / 10.0

    if not (0.0 <= glycol <= 120.0) or not (0.0 <= hot_water <= 90.0):
        return

    with state_lock:
        changed = (glycol != last_glycol or hot_water != last_hot_water)
        last_glycol = glycol
        last_hot_water = hot_water

    if changed:
        print(f"[ALDE] glycol={glycol:.1f}°C  hot_water={hot_water:.1f}°C")
        if client.is_connected():
            publish_state(client, glycol, hot_water)

last_external_start = None

def process_d3(data_bytes, chksum, client):
    global last_external_start
    if chksum_enhanced(0xD3, data_bytes) != chksum:
        return
    if len(data_bytes) < 1:
        return

    external_start = bool(data_bytes[0] & 0x20)

    with state_lock:
        changed = (external_start != last_external_start)
        last_external_start = external_start

    if changed:
        state = "ON" if external_start else "OFF"
        print(f"[ALDE] External Start = {state}")
        if external_start:
            arm_tx_warmup("external_start")
        if client.is_connected():
            client.publish("alde/red/external_start", state, retain=True)

# ── Vent Mode Decoder (nur Lüfterstufe) ──────────────────────────────────────
def process_aventa_vent(data_bytes, chksum, client):
    """0x97: Mode + Vent + Temp aus einem Frame"""
    global pending_cmd, pi
    global last_aventa_aircon_mode, last_aventa_vent_mode, last_aventa_temp
    global last_aircon_mode_time, last_vent_mode_time, last_aventa_temp_time

    if chksum_enhanced(0x97, data_bytes) != chksum:
        return
    if len(data_bytes) < 6:
        return

    now = time.time()
    vent_mode   = decode_vent_mode(data_bytes[4])
    aircon_mode = decode_aircon_mode(data_bytes[5])
    raw_temp    = data_bytes[2] + (data_bytes[3] << 8)
    temp_c      = round((raw_temp - 2730) / 10.0, 1)

    if not (5.0 <= temp_c <= 35.0):
        temp_c = None

    last_vent_mode_time   = now
    last_aircon_mode_time = now
    if temp_c is not None:
        last_aventa_temp_time = now

    with state_lock:
        changed = (
            vent_mode != last_aventa_vent_mode or
            aircon_mode != last_aventa_aircon_mode or
            temp_c != last_aventa_temp
        )
        last_aventa_vent_mode   = vent_mode
        last_aventa_aircon_mode = aircon_mode
        if temp_c is not None:
            last_aventa_temp = temp_c

    if changed:
        print(f"[AVENTA] Mode={aircon_mode} Vent={vent_mode} Temp={temp_c}°C")
        if client.is_connected():
            publish_aventa_modes(client, aircon_mode, vent_mode)
            if temp_c is not None:
                client.publish(AVENTA_TEMP_STATE, f"{temp_c:.1f}", retain=True)

    with cmd_lock:
        cmd = pending_cmd
        pending_cmd = None
    if not cmd or pi is None:
        return
    if time.time() < tx_ready_at:
        print(f"[TX] BLOCK (0x97) {tx_ready_at - time.time():.1f}s", flush=True)
        with cmd_lock:
            if pending_cmd is None:
                pending_cmd = cmd
        return
    try:
        parts = cmd.split(",")
        if len(parts) == 3:
            mode, vent, temp = parts[0], parts[1], float(parts[2])
        elif len(parts) == 2:
            mode, vent, temp = parts[0], parts[1], None
        else:
            mode, vent, temp = cmd, "MID", None
        data = build_frame(mode, vent, temp)
        print(f"[CMD] nach 0x97 → {mode},{vent},{temp}", flush=True)
        send_lin_frame(pi, 0x08, data)
    except Exception as e:
        print(f"[TX] Fehler: {e}", flush=True)

# ── MQTT Callbacks ────────────────────────────────────────────────────────────
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print(f"[MQTT] Connected to {MQTT_HOST}")
        publish_discovery(client)
        client.publish(AVAIL_TOPIC, "online", retain=True)

        client.subscribe(CMD_VENT)
        client.subscribe(CMD_AIRCON)
        client.subscribe(CMD_TEMP)
        client.subscribe(CMD_DISPLAY)
        client.subscribe(YELLOW_CLIMATE_STATE)

        with state_lock:
            g = last_glycol
            hw = last_hot_water
            am = last_aventa_aircon_mode
            vm = last_aventa_vent_mode

        if g is not None:
            publish_state(client, g, hw)
        if am is not None or vm is not None:
            publish_aventa_modes(client, am or "Alde Auto", vm or "UNKNOWN")
    else:
        print(f"[MQTT] Connection failed rc={rc}")

def on_message(client, userdata, msg):
    global pending_cmd, last_aventa_temp, last_yellow_ac_mode, pending_after_manual
    payload = msg.payload.decode().strip()
    topic = msg.topic
    print(f"[MSG] {topic} → {payload}", flush=True)

    # Display ON → 5 s keine Aventa-TX
    if topic == CMD_DISPLAY:
        if payload.upper() == 'ON':
            arm_tx_warmup("display")
        return

    # ── Yellow Klima-Mode (JSON, nicht uppercasen) ──
    if topic == YELLOW_CLIMATE_STATE:
        try:
            data = json.loads(payload)
            m = (data.get('ac_mode') or '').lower()
            last_yellow_ac_mode = m
            print(f"[YELLOW] ac_mode={m}", flush=True)
            if m == 'manual' and pending_after_manual:
                arm_tx_warmup("nach manual")
                with cmd_lock:
                    pending_cmd = pending_after_manual
                print(f"[CMD] nach Manual → {pending_after_manual}", flush=True)
                pending_after_manual = None
        except Exception:
            pass
        return
    payload = payload.upper()
   
    with state_lock:
        cur_mode = (last_aventa_aircon_mode or "COOLING").upper()
        cur_vent = (last_aventa_vent_mode or "MID").upper()
        cur_temp = last_aventa_temp if last_aventa_temp is not None else 21.0

    if cur_mode in ("STANDBY", "ALDE AUTO") or cur_mode.startswith("UNKNOWN"):
        cur_mode = "COOLING"
    if cur_vent.startswith("UNKNOWN"):
        cur_vent = "MID"

    mode, vent, temp = cur_mode, cur_vent, cur_temp

    if topic == CMD_AIRCON:
        if payload == "OFF":
            mode, vent = "OFF", "OFF"
        elif payload == "AUTO":
            mode, vent = "AUTO", "AUTO"
        else:
            mode = payload
    elif topic == CMD_VENT:
        if payload == "OFF":
            mode, vent = "OFF", "OFF"
        else:
            vent = payload
            if mode == "OFF":
                mode = "COOLING"
    elif topic == CMD_TEMP:
        try:
            t = float(payload)
            if 16.0 <= t <= 30.0:
                temp = t
                with state_lock:
                    last_aventa_temp = t          # sofort merken
            else:
                print(f"[CMD] Temp ausserhalb 16–30: {t}")
                return
        except ValueError:
            print(f"[CMD] ungültige Temp: {payload}")
            return

    key = f"{mode},{vent},{temp}"

    ymode = (last_yellow_ac_mode or '').lower()
    if ymode in ('alde auto', 'auto') and topic in (CMD_AIRCON, CMD_VENT, CMD_TEMP):
        pending_after_manual = key
        client.publish(YELLOW_AC_MODE_CMD, 'manual')
        print(f"[SCHUTZ] Yellow=auto → Manual anfordern, Aventa wartet: {key}", flush=True)
        return

    with cmd_lock:
        pending_cmd = key
    print(f"[CMD] queued: {key}", flush=True)
    if topic == CMD_TEMP and temp is not None and client.is_connected():
        client.publish(AVENTA_TEMP_STATE, f"{temp:.1f}", retain=True)

# ── Main ──────────────────────────────────────────────────────────────────────
def main():
    global pi
    global pending_cmd
    global last_aventa_aircon_mode, last_aventa_vent_mode, last_aventa_temp
    global last_aircon_mode_time, last_vent_mode_time
    global cmd_retry_cmd, cmd_retry_until, last_retry_at
    print("Alde 3030 + Aventa Decoder")
    print("=" * 50)
    pi = pigpio.pi()
    if not pi.connected:
        print("ERROR: pigpio not connected — is pigpiod running?")
        return

    pi.bb_serial_read_open(GPIO_PIN, BAUD, 8)
    print(f"[GPIO] Listening on GPIO{GPIO_PIN} @ {BAUD} baud")

    client = mqtt.Client(client_id=MQTT_CLIENT)
    client.username_pw_set(MQTT_USER, MQTT_PASS)
    client.will_set(AVAIL_TOPIC, "offline", retain=True)
    client.on_connect = on_connect
    client.user_data_set({"pi": pi})
    client.on_message = on_message
    client.connect(MQTT_HOST, MQTT_PORT, keepalive=60)
    client.loop_start()

    print("Waiting for MQTT connection...")
    for _ in range(20):
        if client.is_connected():
            break
        time.sleep(0.5)
    if not client.is_connected():
        print("ERROR: Could not connect to MQTT broker")
        pi.bb_serial_read_close(GPIO_PIN)
        pi.stop()
        return

    print("Listening on red bus...")
    print()

    buf = []
    last_t = time.time()
    sm_state = 'IDLE'
    current_pid = None

    try:
        while True:
            count, data = pi.bb_serial_read(GPIO_PIN)
            now = time.time()
            if count and data:
                for b in data:
                    gap = now - last_t
                    last_t = now

                    if gap > 0.010 and sm_state == 'DATA':
                        buf = []
                        current_pid = None
                        sm_state = 'IDLE'

                    if sm_state == 'IDLE':
                        if b == 0x00:
                            sm_state = 'GOT_BREAK'
                    elif sm_state == 'GOT_BREAK':
                        if b == 0x55:
                            sm_state = 'GOT_SYNC'
                        elif b == 0x00:
                            pass
                        else:
                            sm_state = 'IDLE'
                    elif sm_state == 'GOT_SYNC':
                        current_pid = b
                        buf = []
                        sm_state = 'DATA'
                    elif sm_state == 'DATA':
                        buf.append(b)
                        if len(buf) == 9:
                            chksum = buf[8]
                            data_bytes = buf[:8]
                            #log_frame(current_pid, data_bytes, chksum)

                            if current_pid == 0x55:
                                process_0x55(data_bytes, chksum, client)
                            elif current_pid == 0x97:
                                process_aventa_vent(data_bytes, chksum, client)
                            elif current_pid == 0xD3:
                                process_d3(data_bytes, chksum, client)   # ← neu
                            elif current_pid in (0x08, 0x3C, 0x7D, 0x3D, 0x18):
                                pass 
                                    #data = bytes(data_bytes)
                                    #print(f"[DEBUG] PID 0x03 empfangen: {data.hex()}")   # ← temporär
                                    #scan_for_aircon_mode(data_bytes, current_pid, client)                            
                            buf = []
                            current_pid = None
                            sm_state = 'IDLE'
            # === Timeout Checks ===
            now = time.time()

            # 1. VentMode Timeout → "OFF"  (wird zuerst geprüft)
            if last_vent_mode_time > 0 and (now - last_vent_mode_time) > 30:
                with state_lock:
                    if last_aventa_vent_mode != "OFF":
                        last_aventa_vent_mode = "OFF"
                        print("[AVENTA] Timeout → OFF")
                        if client.is_connected():
                            publish_aventa_modes(client, last_aventa_aircon_mode or "Alde Auto", "OFF")

            # 2. AirconMode Timeout → "Alde Auto"
            if last_aircon_mode_time > 0 and (now - last_aircon_mode_time) > 30:
                with state_lock:
                    if last_aventa_aircon_mode != "Alde Auto":
                        last_aventa_aircon_mode = "Alde Auto"
                        print("[AVENTA] Timeout → Alde Auto")
                        if client.is_connected():
                            publish_aventa_modes(client, "Alde Auto", last_aventa_vent_mode or "UNKNOWN")
            if cmd_retry_cmd and now < cmd_retry_until and now >= tx_ready_at:
                parts = cmd_retry_cmd.split(",")
                want_mode = parts[0]
                want_vent = parts[1] if len(parts) > 1 else None
                with state_lock:
                    got_mode = (last_aventa_aircon_mode or "").upper()
                    got_vent = (last_aventa_vent_mode or "").upper()
                ok = (got_mode == want_mode.upper())
                if want_vent and want_mode.upper() != "OFF":
                    ok = ok and (got_vent == want_vent.upper())
                if ok:
                    print(f"[TX] bestätigt: {got_mode}/{got_vent}", flush=True)
                    cmd_retry_cmd = None
                elif now - last_retry_at >= CMD_RETRY_EVERY_S:
                    last_retry_at = now
                    with cmd_lock:
                        if pending_cmd is None:
                            pending_cmd = cmd_retry_cmd
                    print(f"[TX] retry: {cmd_retry_cmd}", flush=True)                    
            # (einfach: timestamp last_retry mitführen)           
            # ── pending TX (Gap, 1×) ──
            with cmd_lock:
                cmd = pending_cmd
                pending_cmd = None
            if cmd and pi is not None:
                now = time.time()
                if now < tx_ready_at:
                    if not hasattr(arm_tx_warmup, "_last_block_log") or now - arm_tx_warmup._last_block_log >= 1.0:
                        print(f"[TX] BLOCK {tx_ready_at - now:.1f}s übrig, cmd={cmd}", flush=True)
                        arm_tx_warmup._last_block_log = now
                    with cmd_lock:
                        if pending_cmd is None:
                            pending_cmd = cmd
                else:
                    print(f"[TX] SEND (ready_at={tx_ready_at:.0f} now={now:.0f})", flush=True)
                    parts = cmd.split(",")
                    if len(parts) == 3:
                        mode, vent, temp = parts[0], parts[1], float(parts[2])
                    elif len(parts) == 2:
                        mode, vent, temp = parts[0], parts[1], None
                    else:
                        mode, vent, temp = cmd, "MID", None
                    data = build_frame(mode, vent, temp)
                    print(f"[CMD] gap → {mode},{vent},{temp}", flush=True)
                    quiet_since = time.time()
                    deadline = quiet_since + 0.25
                    while time.time() < deadline:
                        count, rx = pi.bb_serial_read(GPIO_PIN)
                        if count and rx:
                            quiet_since = time.time()
                        if (time.time() - quiet_since) >= 0.012:
                            break
                        time.sleep(0.001)
                    try:
                        for i in range(3):
                            send_lin_frame(pi, 0x08, data)
                            if i < 2:
                                time.sleep(0.030)
                        if cmd_retry_cmd is None:
                            cmd_retry_until = time.time() + CMD_RETRY_MAX_S
                        cmd_retry_cmd = cmd
                        last_retry_at = time.time()
                        print(f"[TX] warte auf Bestätigung: {cmd}", flush=True)
                    except Exception as e:
                        print(f"[TX] Fehler: {e}", flush=True)
            time.sleep(0.003)
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        client.publish(AVAIL_TOPIC, "offline", retain=True)
        client.loop_stop()
        client.disconnect()
        pi.bb_serial_read_close(GPIO_PIN)
        pi.stop()

if __name__ == '__main__':
    main()