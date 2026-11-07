# STM32 + K210 Intelligent Line-Following Vehicle

An autonomous line-following vehicle built on a **dual-core split**: an STM32F103 handles
motion control, a K210 (Sipex Maix Bit) handles vision. The two boards talk over a
one-way UART link.

Each frame the K210 reports *how many pixels the lane centre is offset from the image
axis*; the STM32 turns that number into a wheel-speed differential through a cascade PID.

```
        ┌──────────────┐   UART 115200    ┌──────────────┐
        │  K210 (Maix) │ ───────────────> │  STM32F103   │
        │  vision      │  4 bytes/frame    │  control     │
        └──────────────┘                  └──────┬───────┘
              │ camera                           │ PWM + direction
              ▼                                  ▼
          lane markings                     TB6612 → 2× DC motor
```

The track geometry and the rules the vehicle must satisfy are documented in
[`Doc/Design_Specification.pdf`](Doc/Design_Specification.pdf).

---

## 1. Repository layout

```
11_CoconutCar/
├── Control/                     STM32 firmware (Keil MDK project)
│   ├── AlgorithmLayer/          PID algorithms: velocity PI, line-tracking PD
│   ├── AppLayer/                Service layer: scheduling and application logic
│   ├── DebugLayer/              Non-blocking debug output (DMA + snprintf)
│   ├── HardwareLayer/           Motor / encoder / key / Bluetooth / K210 RX / voice
│   ├── MiddleLayer/             Frame protocol parser
│   ├── SystemLayer/             1 ms system time base
│   ├── User/                    main.c and interrupt vectors
│   ├── Library/                 ST standard peripheral library
│   └── Start/                   Startup files and CMSIS
│
├── Visual/                      K210 vision side (MaixPy scripts)
│   ├── config.py                Every tunable parameter lives here
│   ├── vision.py                Camera geometry + line extraction
│   ├── judge.py                 Error / state decision logic
│   └── main.py                  Entry point
│
├── Doc/                         Track specification and pin-out table
└── README.md
```

---

## 2. Control architecture

The STM32 firmware is organised in layers and runs from a single software timer in
`main()`:

| Period | Task | What it does |
|---|---|---|
| **5 ms** | `ServiceTask_5ms` | Velocity loop: encoder delta → PI → `Motor_SetDuty` |
| **10 ms** | `ServiceTask_10ms` | Key scanning (20 ms debounce) |
| **30 ms** | `ServiceTask_30ms` | Steering loop: K210 error → PD → wheel-speed differential |
| **100 ms** | `ServiceTask_100ms` | Run/stop state machine, link-loss protection, DMA debug frame |

**Cascade structure**: the outer PD loop decides *how fast each wheel should spin*, the
inner PI loop makes sure *the wheel actually reaches that speed*.

```
K210 error ──> line PD ──> target speed ──> velocity PI ──> PWM ──> motor
                 ▲                          ▲
                 │                          │
            pixel offset              encoder counts
```

### PID implementation details

| Feature | File | Purpose |
|---|---|---|
| Anti-windup | `PID.c` | Integration stops while the output is saturated and the error pushes further |
| Integral clamp | `PID.c` | Limited to `output_max / Ki`, not a hard-coded constant |
| First-cycle guard | `PID.c` | `isFirstRun` aligns `err_last` so the D term cannot kick at power-up |
| Integral gate | `PID.c` | Integration only while `|error| ≤ 20`, suppressing start-up build-up |
| Output floor | `Service.c` | Wheel targets clamped to ≥ 0 so a single wheel can never reverse |

---

## 3. Communication protocol

K210 → STM32, fixed **4-byte frame**, USART1 at 115200 8N1:

| Byte | Content | Range |
|---|---|---|
| 0 | `0xAA` header | — |
| 1 | Error, `int8` | −128 … 127 (pixels) |
| 2 | State, `uint8` | 0 / 1 / 2 / 3 |
| 3 | `0x55` tail | — |

**Sign convention**: positive means the lane centre appears to the *right* of the image
axis, i.e. the vehicle sits left of the lane → steer right.

**State meaning**:

| State | Trigger | STM32 base speed |
|---|---|---|
| 0 | Straight | 40 |
| 1 | Shallow corner (90° / 141.5°), or far field lost | 25 |
| 2 | Sharp corner (77°), or corner confirmed | 15 |
| 3 | Finish line (reserved) | Stop |

**Link-loss protection**: if no valid frame arrives for four consecutive task periods
(≈120 ms) the vehicle stops, so a dead camera cannot send it off the track.

**Line-loss policy**: a brief loss never triggers state 3. Up to 10 frames the previous
error is repeated; beyond that the vehicle crawls forward at reduced speed.

---

## 4. Vision algorithm

The K210 does not use `find_lines` or `find_blobs`. It scans **row by row** — a row is a
single black/white decision, which keeps the perspective relationship separable and the
behaviour predictable.

### Pipeline

1. **Capture** — 160×120 grayscale (QQVGA), speed over resolution
2. **Scan rows** — computed at start-up from camera height, pitch and field of view; not hard-coded
3. **Per-row adaptive threshold** — `threshold = darkest + (brightest − darkest) × 0.40`
   Each row thresholds itself, so uneven lighting and auto-exposure drift do not break it
4. **Run detection** — two runs with a plausible gap → take the midpoint;
   a single run → reconstruct the centre using the known half-lane spacing
5. **Weighted average** — near rows carry more weight (position information is firmer),
   far rows provide corner preview
6. **Corner prediction** — inner boundary gone while the vehicle is still centred →
   declare an approaching right-hand corner and raise the error to a fixed value

### Why the single-line fallback matters

Driving clockwise, the **outer boundary is always on the vehicle's left and is never
interrupted**, while the inner boundary bends away roughly half a metre before each
corner. A "midpoint of two lines" follower goes blind exactly at the corner entrance.

### Self-calibration

Scan-row positions and pixel spacing are derived from three geometric parameters at
start-up. During operation, any row that sees both boundaries keeps correcting its own
spacing estimate by 1/16 per frame, so a camera height measured a couple of centimetres
off still converges.

---

## 5. Debugging

| Method | Implementation | Note |
|---|---|---|
| Serial waveform | `DebugLayer`, USART2 + **DMA** | Non-blocking; never steals time from the 5 ms loop |
| Frame assembly | `snprintf` into a buffer, then hand the buffer to DMA | Avoids the byte-at-a-time blocking of `printf` |
| Breakpoints | SWD | JTAG is explicitly disabled in `Encoder_Init` |

One frame every 100 ms:

```
Err:<pixel error>, St:<state>, TL:<left target>, TR:<right target>, Run:<run flag>
```

Feed it to any serial plotting tool using the FireWater format (comma-separated numbers,
`\n` terminated). The `fputc` retarget is still present, but never mix `printf` and DMA
on the same UART — the output will interleave.

---

## 6. Pin assignment

The full table is in [`Doc/引脚分配表.xlsx`](Doc/), including peripheral, signal, GPIO mode
and the mating connector pin.

| Module | Pins |
|---|---|
| TB6612 motor driver | PA8, PA11 (PWM), PB5 (STBY), PB12–PB15 (direction) |
| Left encoder (TIM2) | PA15, PB3 |
| Right encoder (TIM4) | PB6, PB7 |
| K210 (USART1) | PA10 |
| Bluetooth (USART2) | PA2, PA3 |
| Voice module (USART3) | PB10, PB11 |
| Push button | PA0 |

> `Encoder_Init()` disables JTAG, so **downloads must use SWD**.
> PA15 / PB3 are JTDI / JTDO after reset; a 1 kΩ series resistor on PB3 is recommended.

---

## 7. Building and running

### STM32 side

1. Open `Control/11_CoconutCar.uvprojx` in Keil MDK
2. Make sure the debug adapter is set to **SWD**, not JTAG
3. Build and download

Build outputs (`Control/Objects/`, `Control/Listings/`) and Keil per-user layout files
are excluded by `.gitignore`.

### K210 side

With MaixPy IDE connected, **all four files must first be saved onto the board's
`/flash`** — the IDE's Run button only sends the file currently open in the editor.

| Order | File |
|---|---|
| 1 | `config.py` |
| 2 | `vision.py` |
| 3 | `judge.py` |
| 4 | `main.py` ← run this one last |

Wiring:

```
K210 IO24 (TX) ──> STM32 PA10 (USART1_RX)
K210 IO25 (RX) ──  not connected
K210 GND       ──> STM32 GND        ← common ground is mandatory
```

---

## 8. Tuning entry points

All tunables are collected in [`Visual/config.py`](Visual/config.py):

| Parameter | Meaning |
|---|---|
| `CAM_H_CM` / `CAM_PITCH_DEG` / `HFOV_DEG` | Camera mounting geometry (drives the scan-row selection) |
| `CALIB_OFFSET` | Zero-offset calibration with the vehicle centred |
| `CURV_T1` / `CURV_T2` | Corner speed-reduction thresholds |
| `CORNER_BIAS` | Differential increment applied on corner prediction |
| `LOST_KEEP` | Number of frames a line loss is tolerated |

On the STM32 side, base speeds live at the top of
[`Control/AppLayer/Service.c`](Control/AppLayer/Service.c), PID gains in `Service_Init()`.

---

## 9. Status and known limitations

- ✅ Dual-core split, frame protocol, link-loss protection and the DMA debug path all work
- ⚠️ The line-tracking PID gains still need on-vehicle tuning
- ⚠️ The **77° apex** (103° of heading change, 0.25 m turn radius) is the hardest case;
  the camera loses the lane for roughly the last 0.3 m and the vehicle currently relies on
  "hold the last error at minimum speed" to carry through
- ⏳ The voice module is implemented in `HardwareLayer/Voice.c` but not yet wired into the
  main control flow
- ⏳ Finish-line detection is scaffolded (`FINISH_ENABLE`) and disabled by default

---

## 10. Notes

Personal learning project. Code is shared for reference only.
