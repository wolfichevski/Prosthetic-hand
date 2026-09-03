# Prosthetic Hand Controlled by Computer Vision

A 3D-printed prosthetic hand that mirrors your hand movements in real time.
The only sensor is a laptop webcam — nothing is attached to the user.

Camera → hand landmarks (MediaPipe) → finger flexion angles → servo positions.

## Hardware

- 5 × Waveshare SC-09 serial bus servos (IDs 1–5, thumb → pinky)
- 5 × SCS-Y4-2012 bus distribution boards
- 1 × Waveshare Bus Servo Adapter (A) — USB to servo bus
- External 6 V DC supply (FNIRSI DPS-150 used here)
- InMoov Hand i2 parts, SLS-printed from PA12
- Laptop with a webcam

⚠️ **Power the servos from the external supply, not USB.** Five stalled servos draw
about 5 A. Set the supply to **6 V** and limit current to **2–3 A** before the first run.

## Install

Tested on Python 3.9.

```bash
git clone https://github.com/wolfichevski/Prosthetic-hand.git
cd Prosthetic-hand

python3 -m venv .venv
source .venv/bin/activate          # Windows: .venv\Scripts\activate

pip install mediapipe opencv-python numpy pyserial
```

Run everything from the repository root — `scscl_package/` and `hand_landmarker.task`
are resolved relative to it.

On Linux, add yourself to the serial group once: `sudo usermod -a -G dialout $USER`
(log out and back in).

## Run

Go in this order — each step checks one layer, so a fault is easy to locate.

```bash
python test.py                 # 1. is a servo talking? (default ID 2)
python one_finger_control.py   # 2. move one finger, limits applied
python group_move.py           # 3. open/close all five together
python gesture_move.py         # 4. gestures: open, close, peace, rock, quit
python webcam_control.py       # 5. full system — control by webcam
```

**`webcam_control.py`** opens a window showing the detected landmarks and each finger's
angle and commanded position. Hold your hand in front of the camera, palm toward the
lens, ~0.5–1 m away, in decent light.

**Quit with the `q` key** in that window, not Ctrl+C — on exit the program opens the hand
and releases servo torque.

If no hand is detected, the prototype holds its last position rather than snapping open.

Servo IDs must be 1–5. If a servo does not answer, `test.py` scans IDs 1–20 and prints
what it finds.

## Layout

```
webcam_control.py        main program — computer vision control
gesture_move.py          predefined gestures
group_move.py            all five fingers, synchronised
one_finger_control.py    single finger
test.py                  communication and feedback check
hand_landmarker.task     MediaPipe Hand Landmarker model
scscl_package/scscl/     serial bus servo library (SCSCL class)
```

Smoothing parameters (send interval, adaptive filter, slew limit, dead band) are at the
top of `webcam_control.py`.

## Credits

Hand design: [InMoov Hand i2](https://inmoov.fr/hand-i2/) by Gaël Langevin (CC BY-NC) ·
Detection: [MediaPipe Hand Landmarker](https://ai.google.dev/edge/mediapipe/solutions/vision/hand_landmarker) ·
Servos: [Waveshare SC09](https://www.waveshare.com/sc09-servo.htm) ·
Adaptive filter: Casiez, Roussel & Vogel (2012), *1€ filter*, CHI '12.

Educational and research use. Not a medical device.
