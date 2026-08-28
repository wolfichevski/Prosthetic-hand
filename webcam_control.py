import math
import sys
import time
from collections import deque
from pathlib import Path

import cv2
import mediapipe as mp
import numpy as np
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
from scscl_package.scscl import SCSCL


MODEL_PATH = (Path(__file__).parent / "hand_landmarker.task").resolve()

# ─────────────────────────── PARAMETRI GLAĐENJA ───────────────────────────
SEND_INTERVAL = 0.05     # razdoblje slanja naredbi [s]  (20 naredbi/s)
MOVE_TIME_MS = 100       # vrijeme gibanja u naredbi [ms]; > razdoblja slanja
                         #   pa se uzastopna gibanja neprekidno stapaju

EURO_MIN_CUTOFF = 0.6    # [Hz] granična frekvencija pri mirovanju
                         #   manje = mirnije pri mirovanju, veći zaostatak
EURO_BETA = 0.03         # koliko brzina pokreta ubrzava filtar
                         #   veće = brži odziv na nagle pokrete
EURO_D_CUTOFF = 1.0      # [Hz] glađenje procjene brzine

DEADBAND = 8             # [koraka] promjene manje od ovoga se ne šalju
SLEW_NORMAL = 1800       # [koraka/s] najveća dopuštena brzina promjene
SLEW_REACQUIRE = 500     # [koraka/s] sniženo ograničenje nakon ponovne
REACQUIRE_TIME = 0.6     #   detekcije šake, tijekom ovoliko sekundi

OPEN_POSE = [50, 50, 50, 50, 50]   # logički položaji potpuno otvorene šake


# ─────────────────────────────── FILTRI ───────────────────────────────────
class Median3:
    """Klizni medijan od 3 uzorka — uklanja pojedinačne skokove."""

    def __init__(self):
        self.buf = deque(maxlen=3)

    def apply(self, x):
        self.buf.append(x)
        return sorted(self.buf)[len(self.buf) // 2]

    def reset(self):
        self.buf.clear()


class OneEuro:
    """Prilagodljivi niskopropusni filtar (Casiez i sur., 2012).

    Granična frekvencija raste s brzinom signala:
        fc = min_cutoff + beta * |dx/dt|
    pa je glađenje jako pri mirovanju, a slabo pri brzom pokretu.
    """

    def __init__(self, min_cutoff=EURO_MIN_CUTOFF, beta=EURO_BETA,
                 d_cutoff=EURO_D_CUTOFF):
        self.min_cutoff = min_cutoff
        self.beta = beta
        self.d_cutoff = d_cutoff
        self.x_prev = None
        self.dx_prev = 0.0
        self.t_prev = None

    @staticmethod
    def _alpha(cutoff, dt):
        tau = 1.0 / (2.0 * math.pi * cutoff)
        return 1.0 / (1.0 + tau / dt)

    def reset(self):
        self.x_prev = None
        self.dx_prev = 0.0
        self.t_prev = None

    def apply(self, x, t):
        if self.t_prev is None:
            self.x_prev, self.t_prev = x, t
            return x
        dt = max(t - self.t_prev, 1e-3)
        self.t_prev = t

        dx = (x - self.x_prev) / dt
        a_d = self._alpha(self.d_cutoff, dt)
        self.dx_prev = a_d * dx + (1.0 - a_d) * self.dx_prev

        cutoff = self.min_cutoff + self.beta * abs(self.dx_prev)
        a = self._alpha(cutoff, dt)
        self.x_prev = a * x + (1.0 - a) * self.x_prev
        return self.x_prev


def slew(prev, target, max_rate, dt):
    """Ograniči promjenu vrijednosti na max_rate [jedinica/s]."""
    step = max_rate * dt
    return prev + max(-step, min(step, target - prev))


# ───────────────────────── GEOMETRIJA I PRESLIKAVANJE ─────────────────────
def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def angle_between(a, b, c):
    a = np.array(a, dtype=np.float32)
    b = np.array(b, dtype=np.float32)
    c = np.array(c, dtype=np.float32)

    ba = a - b
    bc = c - b
    denom = (np.linalg.norm(ba) * np.linalg.norm(bc)) + 1e-6
    cosang = np.dot(ba, bc) / denom
    cosang = np.clip(cosang, -1.0, 1.0)
    return float(np.degrees(np.arccos(cosang)))


def map_angle_to_logical_pos(angle_deg, open_angle=165, closed_angle=75):
    """Dugi prsti: 50 = otvoreno, 900 = zatvoreno (float, bez zaokruživanja)."""
    t = (open_angle - angle_deg) / (open_angle - closed_angle)
    t = clamp(t, 0.0, 1.0)
    return 50.0 + t * (900.0 - 50.0)


def map_thumb_angle_to_logical_pos(angle_deg, open_angle=155, closed_angle=70):
    """Palac: 50 = otvoreno, 420 = zatvoreno (float, bez zaokruživanja)."""
    t = (open_angle - angle_deg) / (open_angle - closed_angle)
    t = clamp(t, 0.0, 1.0)
    return 50.0 + t * (420.0 - 50.0)


# indeksi karakterističnih točaka (MCP/PIP, vrh) za svaki prst
FINGER_POINTS = [(2, 3, 4),      # palac
                 (5, 6, 8),      # kažiprst
                 (9, 10, 12),    # srednjak
                 (13, 14, 16),   # prstenjak
                 (17, 18, 20)]   # mali prst

CONNECTIONS = [
    (0, 1), (1, 2), (2, 3), (3, 4),
    (0, 5), (5, 6), (6, 7), (7, 8),
    (5, 9), (9, 10), (10, 11), (11, 12),
    (9, 13), (13, 14), (14, 15), (15, 16),
    (13, 17), (17, 18), (18, 19), (19, 20),
    (0, 17),
]


def draw_landmarks(frame, landmarks):
    h, w, _ = frame.shape
    pts = []
    for lm in landmarks:
        x, y = int(lm.x * w), int(lm.y * h)
        pts.append((x, y))
        cv2.circle(frame, (x, y), 4, (0, 255, 0), -1)
    for i, j in CONNECTIONS:
        cv2.line(frame, pts[i], pts[j], (255, 0, 0), 2)


# ─────────────────────────────── GLAVNA PETLJA ────────────────────────────
def main():
    print("Trazim serijski prikljucak...")
    port = SCSCL.find_port()
    if port is None:
        print("Serijski prikljucak nije pronadjen!")
        sys.exit(1)
    print(f"Prikljucak: {port}")
    print(f"Model: {MODEL_PATH}")

    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Kamera se ne moze otvoriti")
        sys.exit(1)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)      # uvijek najsvjezija slika

    base_options = python.BaseOptions(model_asset_path=str(MODEL_PATH))
    options = vision.HandLandmarkerOptions(
        base_options=base_options,
        running_mode=vision.RunningMode.VIDEO,
        num_hands=1,
        min_hand_detection_confidence=0.7,
        min_hand_presence_confidence=0.7,
        min_tracking_confidence=0.7,
    )

    with vision.HandLandmarker.create_from_options(options) as landmarker, \
         SCSCL(port, baudrate=1000000) as servo:

        servo.with_groups({
            "hand": [1, 2, 3, 4, 5]
        })
        servo.with_limits({
            1: (50, 420),
            2: (50, 900, True),
            3: (50, 900, True),
            4: (50, 900, True),
            5: (50, 900, True),
        })

        print("Ukljucujem moment i otvaram saku...")
        servo.group_enable_torque("hand", True)
        servo.group_write_pos_safe("hand", OPEN_POSE, time_ms=800)
        time.sleep(0.9)

        # stanje filtara i upravljanja
        medians = [Median3() for _ in range(5)]
        euros = [OneEuro() for _ in range(5)]
        last_cmd = [float(p) for p in OPEN_POSE]   # posljednje poslano
        desired = last_cmd[:]                      # posljednji filtrirani cilj
        last_send = 0.0
        hand_was_visible = False
        reacquire_until = 0.0
        raw_angles = [0.0] * 5

        print("Za izlaz pritisnite 'q'.")
        try:
            while True:
                ok, frame = cap.read()
                if not ok:
                    print("Slika nije dohvacena")
                    break

                frame = cv2.flip(frame, 1)
                rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)

                now = time.monotonic()
                result = landmarker.detect_for_video(mp_image, int(now * 1000))

                visible = bool(result.hand_landmarks)
                if visible:
                    if not hand_was_visible:
                        # ponovna detekcija: filtri kreću iznova, a brzina
                        # promjene je privremeno dodatno ogranicena
                        for m, e in zip(medians, euros):
                            m.reset()
                            e.reset()
                        reacquire_until = now + REACQUIRE_TIME

                    landmarks = result.hand_landmarks[0]
                    draw_landmarks(frame, landmarks)
                    pts = [(lm.x, lm.y) for lm in landmarks]

                    for i, (p1, p2, p3) in enumerate(FINGER_POINTS):
                        ang = angle_between(pts[p1], pts[p2], pts[p3])
                        raw_angles[i] = ang
                        ang = medians[i].apply(ang)        # 1) medijan
                        ang = euros[i].apply(ang, now)     # 2) One Euro
                        if i == 0:
                            desired[i] = map_thumb_angle_to_logical_pos(ang)
                        else:
                            desired[i] = map_angle_to_logical_pos(ang)
                # ako saka nije vidljiva: desired ostaje nepromijenjen,
                # prototip mirno zadrzava posljednji polozaj

                hand_was_visible = visible

                # ── slanje naredbe najvise svakih SEND_INTERVAL sekundi
                if now - last_send >= SEND_INTERVAL:
                    dt = now - last_send if last_send > 0 else SEND_INTERVAL
                    rate = (SLEW_REACQUIRE if now < reacquire_until
                            else SLEW_NORMAL)

                    new_cmd = [slew(c, d, rate, dt)
                               for c, d in zip(last_cmd, desired)]

                    # mrtva zona: prst se ne pomice za sitne promjene
                    changed = False
                    for i in range(5):
                        if abs(new_cmd[i] - last_cmd[i]) >= DEADBAND:
                            changed = True
                        else:
                            new_cmd[i] = last_cmd[i]

                    if changed:
                        servo.group_write_pos_safe(
                            "hand", [int(round(c)) for c in new_cmd],
                            time_ms=MOVE_TIME_MS)
                        last_cmd = new_cmd
                    last_send = now

                # ── ispis stanja na sliku
                if visible:
                    status, col = "PRACENJE", (0, 255, 0)
                    if now < reacquire_until:
                        status, col = "PONOVNO HVATANJE", (0, 255, 255)
                else:
                    status, col = "SAKA NIJE PRONADJENA", (0, 0, 255)
                cv2.putText(frame, status, (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, col, 2)
                labels = "TIMRP"
                for i in range(5):
                    cv2.putText(
                        frame,
                        f"{labels[i]}:{int(round(last_cmd[i])):4d}"
                        f"  A:{int(raw_angles[i]):3d}",
                        (10, 60 + 26 * i),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)

                cv2.imshow("Webcam Hand Control", frame)
                if (cv2.waitKey(1) & 0xFF) == ord("q"):
                    break

        finally:
            print("Otvaram saku prije izlaska...")
            try:
                servo.group_write_pos_safe("hand", OPEN_POSE, time_ms=500)
                time.sleep(0.7)
                servo.group_enable_torque("hand", False)
            except Exception as e:
                print("Upozorenje pri iskljucivanju:", e)
            cap.release()
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
