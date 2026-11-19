#!/usr/bin/env python3
import json
import select
import socket
import sys
import termios
import threading
import time
import tty
import RPi.GPIO as GPIO

try:
    import smbus
except ImportError:
    try:
        import smbus2 as smbus
    except ImportError:
        smbus = None

try:
    import spidev
except ImportError:
    spidev = None


HOST_IP = "192.168.1.1"
TX_PORT = 5005
RX_PORT = 5006

# HC-SR04 + buzzer
TRIG_PIN = 16
ECHO_PIN = 12
BUZZER_PIN = 18

# Scale desk hand movement to gazebo arena scale (13.5 cm -> 1.35 m)
DIST_SCALE = 10.0
HAND_LIMIT_M = 1.35
BEEP_LIMIT_M = 0.65

# Stepper uses board pins 29, 31, 33, 35 (BCM 5, 6, 13, 19).
# BCM 13 and 19 clash with the Down/Right buttons on the Joy-Pi, so those
# switches stay off and only BCM 26 / 25 are wired as hardware buttons.
STEPPER_PINS = [5, 6, 13, 19]
BUTTONS = {"ESTOP": 26, "AUTO": 25}

# Right DIP switch 1 enables the MAX7219 matrix on SPI 0, CE1
MATRIX_BUS = 0
MATRIX_DEV = 1

HALF_STEPS = (
    (1, 0, 0, 0),
    (1, 1, 0, 0),
    (0, 1, 0, 0),
    (0, 1, 1, 0),
    (0, 0, 1, 0),
    (0, 0, 1, 1),
    (0, 0, 0, 1),
    (1, 0, 0, 1),
)

SEG_FONT = {
    "0": 0x3F, "1": 0x06, "2": 0x5B, "3": 0x4F, "4": 0x66,
    "5": 0x6D, "6": 0x7D, "7": 0x07, "8": 0x7F, "9": 0x6F,
    "-": 0x40, " ": 0x00,
}


class SevenSeg:
    def __init__(self, addr=0x70):
        self.addr = addr
        self.dev = None
        self.bus = None
        self.backend = None
        self._last = None

        try:
            from Adafruit_LED_Backpack import SevenSegment
            self.dev = SevenSegment.SevenSegment(address=addr)
            self.dev.begin()
            self.dev.clear()
            self.dev.write_display()
            self.backend = "adafruit"
            return
        except Exception:
            pass

        if smbus is not None:
            try:
                self.bus = smbus.SMBus(1)
                self.bus.write_byte(addr, 0x21)
                self.bus.write_byte(addr, 0x81)
                self.bus.write_byte(addr, 0xEF)
                self.backend = "smbus"
                self.clear()
            except Exception:
                self.backend = None

    def write_dist(self, val_m):
        if not self.backend:
            return
        v = max(0.0, min(99.9, float(val_m)))
        s = f"{v:5.1f}" if v >= 10.0 else f"{v:5.2f}"
        if s == self._last:
            return
        self._last = s

        chars = []
        for c in s:
            if c == "." and chars:
                chars[-1] = (chars[-1][0], True)
            else:
                chars.append((c, False))
        chars = chars[-4:]
        while len(chars) < 4:
            chars.insert(0, (" ", False))

        try:
            if self.backend == "adafruit":
                self.dev.clear()
                for i, (c, dp) in enumerate(chars):
                    if c != " ":
                        self.dev.set_digit(i, c, decimal=dp)
                self.dev.write_display()
            else:
                for reg, (c, dp) in zip((0x00, 0x02, 0x06, 0x08), chars):
                    bits = SEG_FONT.get(c, 0x00) | (0x80 if dp else 0x00)
                    self.bus.write_byte_data(self.addr, reg, bits)
                self.bus.write_byte_data(self.addr, 0x04, 0x00)
        except Exception:
            pass

    def clear(self):
        self._last = None
        if self.backend == "adafruit":
            try:
                self.dev.clear()
                self.dev.write_display()
            except Exception:
                pass
        elif self.backend == "smbus":
            try:
                for reg in (0x00, 0x02, 0x04, 0x06, 0x08):
                    self.bus.write_byte_data(self.addr, reg, 0x00)
            except Exception:
                pass


class LedMatrix:
    def __init__(self, port=0, device=1):
        self.dev = None
        self.spi = None
        self.backend = None
        self._last_n = -1

        try:
            from luma.core.interface.serial import spi, noop
            from luma.led_matrix.device import max7219
            from luma.core.render import canvas
            self._canvas = canvas
            self.dev = max7219(spi(port=port, device=device, gpio=noop()),
                               cascaded=1, block_orientation=0, rotate=0)
            self.dev.contrast(120)
            self.backend = "luma"
            self.set_bars(0)
            return
        except Exception:
            pass

        if spidev is not None:
            try:
                self.spi = spidev.SpiDev()
                self.spi.open(port, device)
                self.spi.max_speed_hz = 1000000
                for reg, val in ((0x0F, 0x00), (0x09, 0x00), (0x0A, 0x07),
                                 (0x0B, 0x07), (0x0C, 0x01)):
                    self.spi.xfer2([reg, val])
                self.backend = "spidev"
                self.set_bars(0)
            except Exception:
                self.backend = None

    def set_bars(self, n):
        if not self.backend:
            return
        n = max(0, min(8, int(n)))
        if n == self._last_n:
            return
        self._last_n = n

        try:
            if self.backend == "luma":
                with self._canvas(self.dev) as draw:
                    for i in range(n):
                        y = 7 - i
                        draw.line((0, y, 7, y), fill="white")
            else:
                for row in range(1, 9):
                    self.spi.xfer2([row, 0xFF if row <= n else 0x00])
        except Exception:
            pass

    def clear(self):
        self._last_n = -1
        self.set_bars(0)
        if self.spi is not None:
            try:
                self.spi.close()
            except Exception:
                pass


class CharLCD:
    def __init__(self, addr=0x21):
        self.addr = addr
        self.dev = None
        self.bus = None
        self.backend = None
        self._last = ("", "")

        try:
            import Adafruit_CharLCD as LCD
            self.dev = LCD.Adafruit_CharLCDBackpack(address=addr)
            self.dev.set_backlight(0)
            self.dev.clear()
            self.backend = "adafruit"
            return
        except Exception:
            pass

        if smbus is not None:
            try:
                self.bus = smbus.SMBus(1)
                self.bus.write_byte_data(addr, 0x00, 0x00)
                time.sleep(0.05)
                for _ in range(3):
                    self._pulse(0x03, 0)
                    time.sleep(0.005)
                self._pulse(0x02, 0)
                for cmd in (0x28, 0x0C, 0x06, 0x01):
                    self._byte(cmd, 0)
                    time.sleep(0.002)
                self.backend = "smbus"
            except Exception:
                self.backend = None

    def _pulse(self, nib, rs):
        b = 0x80 | (0x02 if rs else 0x00) | ((nib & 0x0F) << 3)
        self.bus.write_byte_data(self.addr, 0x09, b | 0x04)
        time.sleep(0.0005)
        self.bus.write_byte_data(self.addr, 0x09, b)
        time.sleep(0.0005)

    def _byte(self, val, rs):
        self._pulse((val >> 4) & 0x0F, rs)
        self._pulse(val & 0x0F, rs)

    def write(self, top, bottom):
        if not self.backend:
            return
        t = str(top)[:16].ljust(16)
        b = str(bottom)[:16].ljust(16)
        if (t, b) == self._last:
            return
        self._last = (t, b)

        try:
            if self.backend == "adafruit":
                self.dev.home()
                self.dev.message(f"{t}\n{b}")
            else:
                self._byte(0x80, 0)
                for c in t:
                    self._byte(ord(c), 1)
                self._byte(0xC0, 0)
                for c in b:
                    self._byte(ord(c), 1)
        except Exception:
            pass


GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(TRIG_PIN, GPIO.OUT)
GPIO.setup(ECHO_PIN, GPIO.IN)
GPIO.setup(BUZZER_PIN, GPIO.OUT, initial=GPIO.LOW)

for pin in BUTTONS.values():
    GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

for pin in STEPPER_PINS:
    GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)

# Sample resting pin states at boot so a noisy pin doesn't latch a command
btn_idle = {name: GPIO.input(pin) for name, pin in BUTTONS.items()}

tx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
rx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
rx_sock.bind(("0.0.0.0", RX_PORT))
rx_sock.setblocking(False)

lcd = CharLCD(0x21)
seg = SevenSeg(0x70)
matrix = LedMatrix(MATRIX_BUS, MATRIX_DEV)

stop_flag = False
host_spin = True
host_mode = None
host_obs_m = None
local_ok = True


def coils_off():
    for pin in STEPPER_PINS:
        GPIO.output(pin, GPIO.LOW)


def rotor_worker():
    idx = 0
    energized = False
    while not stop_flag:
        if host_spin and local_ok:
            for pin, state in zip(STEPPER_PINS, HALF_STEPS[idx]):
                GPIO.output(pin, state)
            idx = (idx + 1) % len(HALF_STEPS)
            energized = True
            time.sleep(0.0025)
        else:
            if energized:
                coils_off()
                energized = False
            time.sleep(0.02)


def read_ultrasonic():
    GPIO.output(TRIG_PIN, True)
    time.sleep(0.00001)
    GPIO.output(TRIG_PIN, False)

    t0 = time.time()
    while GPIO.input(ECHO_PIN) == 0:
        if time.time() - t0 > 0.03:
            return 10.0

    t_rise = time.time()
    while GPIO.input(ECHO_PIN) == 1:
        if time.time() - t_rise > 0.03:
            return 10.0

    meters = ((time.time() - t_rise) * 343.0) / 2.0
    return round(min(max(meters * DIST_SCALE, 0.12), 10.0), 3)


def drain_host_udp():
    global host_spin, host_mode, host_obs_m
    pkt = None
    # Drain queued packets so displays don't lag behind Gazebo
    while True:
        try:
            raw, _ = rx_sock.recvfrom(512)
            pkt = json.loads(raw.decode("utf-8"))
        except BlockingIOError:
            break
        except Exception:
            break

    if pkt is not None:
        host_spin = (pkt.get("motor") == "SPIN")
        if "mode" in pkt:
            host_mode = str(pkt["mode"])
        if "obs_dist" in pkt:
            host_obs_m = float(pkt["obs_dist"])


def proximity_bars(hand_m, obs_m):
    if hand_m < HAND_LIMIT_M:
        return 8, True

    if obs_m is not None:
        d = max(0.45, min(2.50, obs_m))
        bars = int(round(((2.50 - d) / 2.05) * 8))
        return bars, (obs_m < BEEP_LIMIT_M)

    d = max(HAND_LIMIT_M, min(4.0, hand_m))
    bars = int(round(((4.0 - d) / (4.0 - HAND_LIMIT_M)) * 8))
    return bars, False


def main():
    global stop_flag, local_ok
    cmd = "AUTO"
    was_estop = False
    tick = 0

    threading.Thread(target=rotor_worker, daemon=True).start()

    old_tty = None
    if sys.stdin.isatty():
        old_tty = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

    print(f"joypi ready (lcd={lcd.backend}, seg={seg.backend}, matrix={matrix.backend})")
    lcd.write("UAV HIL READY", "WAIT ROS2...")

    try:
        while not stop_flag:
            tick += 1

            for name, pin in BUTTONS.items():
                if GPIO.input(pin) != btn_idle[name]:
                    cmd = name
                    break

            if sys.stdin.isatty() and select.select([sys.stdin], [], [], 0)[0]:
                key = sys.stdin.read(1).upper()
                if key == " ":
                    cmd = "ESTOP"
                elif key == "M":
                    cmd = "AUTO"
                elif key in ("W", "A", "S", "D", "H"):
                    cmd = key

            hand_m = read_ultrasonic()
            local_ok = (hand_m >= HAND_LIMIT_M) and (cmd != "ESTOP")

            drain_host_udp()

            out = json.dumps({"dist_m": hand_m, "cmd": cmd}).encode("utf-8")
            tx_sock.sendto(out, (HOST_IP, TX_PORT))

            if cmd == "ESTOP" or host_mode == "STOP":
                mode_str = "STOP"
            elif hand_m < HAND_LIMIT_M or host_mode == "HAND STOP":
                mode_str = "HAND STOP"
            elif host_mode is not None:
                mode_str = host_mode
            elif cmd in ("W", "A", "S", "D"):
                mode_str = "MANUAL"
            elif cmd == "H":
                mode_str = "HOVER"
            else:
                mode_str = "AUTO"

            rotor_lbl = "SPIN" if (host_spin and local_ok) else "STOP"

            if hand_m < HAND_LIMIT_M or host_obs_m is None:
                disp_m = hand_m
            else:
                disp_m = host_obs_m

            seg.write_dist(disp_m)
            bars, alarm = proximity_bars(hand_m, host_obs_m)
            matrix.set_bars(bars)
            lcd.write(f"MODE: {mode_str}", f"OBS:{disp_m:4.2f}m {rotor_lbl}")

            in_estop = (mode_str == "STOP")
            if alarm:
                GPIO.output(BUZZER_PIN, GPIO.HIGH if (tick % 2 == 0) else GPIO.LOW)
            elif in_estop and not was_estop:
                GPIO.output(BUZZER_PIN, GPIO.HIGH)
                time.sleep(0.12)
                GPIO.output(BUZZER_PIN, GPIO.LOW)
            else:
                GPIO.output(BUZZER_PIN, GPIO.LOW)
            was_estop = in_estop

            print(
                f"\rhand={hand_m:4.2f}m obs={disp_m:4.2f}m mode={mode_str:<9} bars={bars}/8 rotor={rotor_lbl}",
                end="",
                flush=True,
            )
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\nstopping...")
    finally:
        stop_flag = True
        time.sleep(0.05)
        GPIO.output(BUZZER_PIN, GPIO.LOW)
        coils_off()
        seg.clear()
        matrix.clear()
        lcd.write("UAV HIL OFFLINE", "STOPPED")
        GPIO.cleanup()
        tx_sock.close()
        rx_sock.close()
        if old_tty is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_tty)


if __name__ == "__main__":
    main()
