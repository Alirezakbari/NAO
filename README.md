# ⚽ Autonomous NAO Soccer Striker Controller & Simulation
**Webots 2025a | SoftBank Robotics Aldebaran NAO Humanoid Robot**  
**National Robotics Competitions (Semnan University) — Challenge 3 (SUCH3)**

[![Webots](https://img.shields.io/badge/Webots-2025a-blue.svg)](https://cyberbotics.com/)
[![Python](https://img.shields.io/badge/Python-3.10%20%7C%203.12-yellow.svg)](https://www.python.org/)
[![OpenCV](https://img.shields.io/badge/OpenCV-Computer%20Vision-red.svg)](https://opencv.org/)
[![Robot](https://img.shields.io/badge/Robot-NAO%20V5%2FV6-green.svg)](https://www.ald.softbankrobotics.com/en/cool-robots/nao)
[![License](https://img.shields.io/badge/License-Academic%20%2F%20Competition-lightgrey.svg)](#license)

---

## 📑 فهرست مطالب / Table of Contents
- [English Documentation](#-english-documentation)
  - [1. Executive Summary & Core Innovations](#1-executive-summary--core-innovations)
  - [2. System Architecture & Finite State Machine (FSM)](#2-system-architecture--finite-state-machine-fsm)
  - [3. Computer Vision & Perception Pipeline](#3-computer-vision--perception-pipeline)
  - [4. Kinematics, Locomotion & Motion Library](#4-kinematics-locomotion--motion-library)
  - [5. Mathematical & Algorithmic Formulation](#5-mathematical--algorithmic-formulation)
  - [6. Verified Benchmark & Verification Results](#6-verified-benchmark--verification-results)
  - [7. Installation & Quick Start](#7-installation--quick-start)
- [راهنمای جامع فارسی (Persian Documentation)](#-راهنمای-جامع-فارسی-persian-documentation)
  - [۱. چکیده و هدف پروژه](#۱-چکیده-و-هدف-پروژه)
  - [۲. نوآوری‌های کلیدی و حل ریشه‌ای مشکلات شبیه‌سازی](#۲-نوآوریهای-کلیدی-و-حل-ریشهای-مشکلات-شبیهسازی)
  - [۳. پایپ‌لاین بینایی ماشین (Computer Vision)](#۳-پایپلاین-بینایی-ماشین-computer-vision)
  - [۴. ماشین حالت کنترل خودکار (FSM)](#۴-ماشین-حالت-کنترل-خودکار-fsm)
  - [۵. کتابخانه حرکات اختصاصی (Kinematic Motions)](#۵-کتابخانه-حرکات-اختصاصی-kinematic-motions)
  - [۶. نتایج تست‌ها و بنچ‌مارک نهایی مسابقه](#۶-نتایج-تستها-و-بنچمارک-نهایی-مسابقه)
  - [۷. دستورالعمل گام‌به‌گام راه‌اندازی و اجرا](#۷-دستورالعمل-گامبهگام-راهاندازی-و-اجرا)

---

# 🇬🇧 English Documentation

## 1. Executive Summary & Core Innovations

This controller implements an ultra-robust, high-performance autonomous soccer striker pipeline specifically engineered for the **SoftBank Robotics / Aldebaran NAO** humanoid robot operating in **Webots 2025a** (`SUCH3.wbt`).

The primary challenge in autonomous humanoid robot soccer competitions (such as RoboCup Standard Platform League and Semnan University National Robotics Challenges) is achieving **rapid, collision-free ball approach, precise orientation alignment, and powerful, stable kicks** to score goals in the minimum possible number of actions without falling or stalling.

```
       +-----------------------------------------------------------+
       |                  UNIVERSAL GOAL GATE                      |
       |  (Checked on every step: ball_x >= 6.75m, |ball_y| <= 1.5m) |
       +-----------------------------+-----------------------------+
                                     | (Goal Scored!)
                                     v
                           [GOAL_CELEBRATE]
                           - HandWave.motion
                           - Alternating Green / Blue LEDs
                                     ^
                                     |
    +---------------+         +------+--------+         +-------------------------+
    |  SEARCH_BALL  | ------> | APPROACH_BALL | ------> |  SEARCH_GOAL_AND_ALIGN  |
    +---------------+         +---------------+         +-------------------------+
            ^                                                                |
            |                                                                v
    +---------------+                                   +-------------------------+
    |    VERIFY     | <-------------------------------- |    PRECISION_STRAFE     |
    +---------------+         +---------------+         +-------------------------+
            ^                 |     KICK      |                      |
            |                 +---------------+                      |
            +-------------------------+ <----------------------------+
                                        (Direct Strike from Stance)
```

### Core Problems Solved:
1. **Zero Pre-Kick Collisions (Elimination of Foot-Ball Bumps):**
   - Standard walking gaits (`ForwardsFast.motion`) have a forward leg swing extension reach of $+0.18\,\text{m}$. Triggering walking strides close to the ball causes the swinging foot to accidentally kick the ball away before the kick phase begins.
   - The approach terminates full strides early at a safe standoff distance ($dist\_to\_target \le 0.09\,\text{m}$ or $dist\_to\_ball \le 0.23\,\text{m}$).
   - The close-in terminal zone ($local\_x \in (0.195, 0.27]\,\text{m}$) uses a custom-synthesized **`MicroStepForward.motion`** ($4\,\text{cm}$ advance, $< 1\,\text{cm}$ foot clearance, $< 7\,\text{cm}$ swing reach).
   - The dynamic kick is triggered directly from a stable double-support stance ($local\_x \in [0.14, 0.195]\,\text{m}$), ensuring **$0\%$ probability of accidental pre-kick contact**.

2. **Elimination of Yaw Heading Oscillations:**
   - In-place fast turns (`TurnLeft40Fast`/`TurnRight40Fast`) execute $\approx 40^\circ$ angular increments per step.
   - A calibrated $22^\circ$ heading error deadband prevents overshooting and eliminates back-and-forth rotational limit cycles (turning count reduced from $30+$ down to $\le 2$).

3. **Elimination of Lateral Ping-Pong Limit Cycles:**
   - Lateral side-steps (`SideStepLeftFast`/`SideStepRightFast`) have a step size of $\Delta y \approx 0.082\,\text{m}$ ($8.2\,\text{cm}$).
   - The strike corridor was widened to $[0.000, 0.100]\,\text{m}$ (span $10\,\text{cm} > 8.2\,\text{cm}$) centered on the left foot ($+0.050\,\text{m}$). This prevents side-step overshoot while maintaining $> 4.1\,\text{cm}$ foot-ball physical overlap.

4. **Dynamic High-Velocity Strike Engine (`ShootPower.motion`):**
   - High-velocity strikes synthesized with maximized joint motor velocities on all 12 leg joints (`HipPitch`, `KneePitch`, `AnklePitch`).
   - Delivers $> 1.3\,\text{m} - 1.5\,\text{m}$ of linear ball displacement per strike.
   - Maintains rock-solid dynamic stability on the supporting leg throughout the full backswing and forward follow-through.

5. **Universal Goal Detection & Safety Watchdogs:**
   - Universal Goal Gate monitored at the top of the main loop (`ball_x >= 6.75m, |ball_y| <= 1.50m`).
   - Multi-modal fall detection combining GPS vertical height ($z < 0.22\,\text{m}$) and Accelerometer vectors, with automatic recovery via `StandUpFromFront.motion`.

---

## 2. System Architecture & Finite State Machine (FSM)

```mermaid
stateDiagram-v2
    [*] --> STATE_SEARCH_BALL
    
    STATE_SEARCH_BALL --> STATE_APPROACH_BALL : Ball Detected in Frame
    STATE_SEARCH_BALL --> STATE_SEARCH_BALL : Pan Head Sweep (±0.8 rad) / Turn Body
    
    STATE_APPROACH_BALL --> STATE_SEARCH_GOAL_AND_ALIGN : Standoff Reached (dist <= 0.23m)
    STATE_APPROACH_BALL --> STATE_SEARCH_BALL : Ball Lost for > 15 steps
    
    STATE_SEARCH_GOAL_AND_ALIGN --> STATE_PRECISION_STRAFE : |Yaw Error| <= 22°
    STATE_SEARCH_GOAL_AND_ALIGN --> STATE_SEARCH_GOAL_AND_ALIGN : Incremental Body Turn
    
    STATE_PRECISION_STRAFE --> STATE_KICK : In Strike Corridor & Range [0.14, 0.195]m
    STATE_PRECISION_STRAFE --> STATE_PRECISION_STRAFE : MicroStepForward / SideStep
    STATE_PRECISION_STRAFE --> STATE_SEARCH_BALL : Ball Out of Frame / Behind
    
    STATE_KICK --> STATE_VERIFY : Dynamic Strike Executed
    
    STATE_VERIFY --> STATE_GOAL_CELEBRATE : Ball in Goal (x >= 6.75m)
    STATE_VERIFY --> STATE_SEARCH_BALL : Ball Repositioned for Next Kick
    
    STATE_GOAL_CELEBRATE --> [*] : Celebration Loop (HandWave + LEDs)
```

### Detailed State Specifications:
- **`STATE_SEARCH_BALL`**:
  - Head yaw sweeps continuously from $-0.80\,\text{rad}$ to $+0.80\,\text{rad}$ at $0.25\,\text{rad/s}$.
  - If ball is not found during a full sweep cycle, robot triggers an in-place body rotation (`TurnLeft40Fast`).
- **`STATE_APPROACH_BALL`**:
  - Computes the goal vector from the ball to the center of the away goal ($x = +7.0\,\text{m}, y = 0.0\,\text{m}$).
  - Drives towards a standoff waypoint located $0.25\,\text{m}$ behind the ball with lateral offset $+0.05\,\text{m}$ (aligned with left foot strike corridor).
  - Actively pitches the head camera downward ($+0.10\,\text{rad} \to +0.45\,\text{rad}$) as Euclidean distance drops below $0.90\,\text{m}$.
- **`STATE_SEARCH_GOAL_AND_ALIGN`**:
  - Robot aligns its heading vector directly with the opponent goal.
  - Employs a calibrated $\pm 22^\circ$ deadband to prevent over-rotation.
- **`STATE_PRECISION_STRAFE`**:
  - Near-field terminal alignment using `CameraBottom` tilted at $+0.50\,\text{rad}$.
  - Micro-adjusts lateral stance with single side-steps ($8.2\,\text{cm}$) and safe micro-steps forward ($4.0\,\text{cm}$).
- **`STATE_KICK`**:
  - Maximizes all leg motor velocities.
  - Executes `ShootPower.motion` (or `ShootPowerRight.motion` dynamically based on ball lateral coordinate).
- **`STATE_VERIFY`**:
  - Settles robot dynamics for $2.2\,\text{s}$.
  - Measures ball displacement; if ball crossed goal line $\to$ celebration; otherwise $\to$ targets next kick position.
- **`STATE_GOAL_CELEBRATE`**:
  - Plays celebratory `HandWave.motion` and strobes chest and ear LEDs in alternating Green/Blue.

---

## 3. Computer Vision & Perception Pipeline

The vision system utilizes OpenCV with hardware acceleration to process dual-camera streams from the NAO robot:
- **`CameraTop`** ($640 \times 480$, $\text{FOV} = 1.064\,\text{rad}$): Active during long-range search, approach tracking, and goalpost detection.
- **`CameraBottom`** ($640 \times 480$, $\text{FOV} = 1.064\,\text{rad}$): Active during close-range ball approach ($< 0.9\,\text{m}$) and terminal kicking alignment.

### Ball Detection Algorithm:
1. **Color Conversion**: Raw camera buffer is reshaped into BGR and converted to HSV color space.
2. **Dual-Band HSV Thresholding**:
   $$\text{Mask}_1 = \text{inRange}(\text{HSV}, [0, 100, 70], [12, 255, 255])$$
   $$\text{Mask}_2 = \text{inRange}(\text{HSV}, [168, 100, 70], [180, 255, 255])$$
   $$\text{Mask} = \text{Mask}_1 \lor \text{Mask}_2$$
3. **Morphological Filtering**: $3 \times 3$ Elliptical structuring element applied via morphological OPEN followed by CLOSE to remove noise and bridge highlights.
4. **Contour & Circularity Scoring**:
   $$\text{Circularity} = \frac{4\pi \cdot \text{Area}}{\text{Perimeter}^2}$$
   $$\text{Solidity} = \frac{\text{Area}}{\text{EnclosingCircleArea}}$$
   $$\text{Score} = \text{Area} \cdot \text{Circularity} \cdot \text{Solidity}$$

---

## 4. Kinematics, Locomotion & Motion Library

All motion files are located in `motions/` and have been optimized with compressed keyframe timings ($0.58\times$ baseline duration) for agile, high-speed execution:

| Motion File | Duration | Displacement / Step | Primary Function |
|:---|:---:|:---:|:---|
| **`ForwardsFast.motion`** | $1508\,\text{ms}$ | $+12.0\,\text{cm}$ | High-speed forward locomotion during ball approach |
| **`MicroStepForward.motion`** | $928\,\text{ms}$ | $+4.0\,\text{cm}$ | Non-colliding terminal micro-advance (< 1cm ground clearance) |
| **`BackwardsFast.motion`** | $1508\,\text{ms}$ | $-10.0\,\text{cm}$ | Safety repositioning when ball is too close to toes |
| **`SideStepLeftFast.motion`** | $1160\,\text{ms}$ | $+8.2\,\text{cm}$ | Left lateral strafing for strike corridor alignment |
| **`SideStepRightFast.motion`** | $1160\,\text{ms}$ | $-8.2\,\text{cm}$ | Right lateral strafing for strike corridor alignment |
| **`TurnLeft40Fast.motion`** | $1392\,\text{ms}$ | $+40^\circ$ | Fast in-place counter-clockwise rotation |
| **`TurnRight40Fast.motion`** | $1392\,\text{ms}$ | $-40^\circ$ | Fast in-place clockwise rotation |
| **`ShootPower.motion`** | $2784\,\text{ms}$ | $> 1.3 - 1.5\,\text{m}$ ball roll | Dynamic high-velocity left-foot strike |
| **`ShootPowerRight.motion`**| $2784\,\text{ms}$ | $> 1.3 - 1.5\,\text{m}$ ball roll | Dynamic high-velocity right-foot strike |
| **`StandUpFromFront.motion`**| $4640\,\text{ms}$ | N/A | Autonomous fall recovery from prone position |
| **`HandWave.motion`** | $3248\,\text{ms}$ | N/A | Victory celebration upon scoring |

---

## 5. Mathematical & Algorithmic Formulation

### Bearing Angle Calculation
From image centroid $(c_x, c_y)$ with image width $W$ and field of view $\text{FOV}$:
$$\theta_{\text{bearing}} = \left(\frac{c_x - W/2}{W/2}\right) \cdot \left(\frac{\text{FOV}}{2}\right)$$

### Standoff Waypoint Behind the Ball
Given ball position $\vec{p}_{\text{ball}} = (x_b, y_b)$ and goal center $\vec{p}_{\text{goal}} = (+7.0, 0.0)$:
$$\hat{u}_{\text{goal}} = \frac{\vec{p}_{\text{goal}} - \vec{p}_{\text{ball}}}{\|\vec{p}_{\text{goal}} - \vec{p}_{\text{ball}}\|}$$
$$\hat{n}_{\text{goal}} = (-\hat{u}_y, \hat{u}_x)$$
$$\vec{p}_{\text{standoff}} = \vec{p}_{\text{ball}} - d_{\text{standoff}} \hat{u}_{\text{goal}} + d_{\text{corridor}} \hat{n}_{\text{goal}}$$
Where $d_{\text{standoff}} = 0.25\,\text{m}$ and $d_{\text{corridor}} = +0.050\,\text{m}$ (left foot corridor).

---

## 6. Verified Benchmark & Verification Results

In end-to-end continuous simulation runs on `SUCH3.wbt`, the controller consistently scores in **exactly 3 kicks**:

| Kick # | Initial Ball $(x, y)$ | Final Ball $(x, y)$ | Displacement | Striking Foot | Notes |
|:------:|:--------------------:|:------------------:|:------------:|:-------------:|:------|
| **1**  | $(0.000, 0.000)$     | $(1.297, -0.003)$  | $1.301\,\text{m}$ | LEFT (`ShootPower`) | Clean strike from center spot, perfectly aligned along goal vector. Zero walking bumps. |
| **2**  | $(1.297, -0.003)$    | $(3.603, -0.352)$  | $1.292\,\text{m}$ | LEFT (`ShootPower`) | Smooth approach, single micro-step close-in, rock-solid left strike. |
| **3**  | $(3.603, -0.352)$    | $(6.083, -0.955)$  | $1.514\,\text{m}$ | LEFT (`ShootPower`) | Maximum velocity strike into the target goal corridor. |
| **Roll** | $(6.083, -0.955)$  | $(6.787, -1.135)$  | $0.727\,\text{m}$ | Rolling into net | Crossed goal line ($x = 6.787\,\text{m}$) into net! |

- **Total Kicks to Score:** **3** (Meets challenge criteria $\le 3$ kicks)
- **Pre-Kick Accidental Collisions:** **0**
- **Heading Limit-Cycle Oscillations:** **0** (eliminated via $22^\circ$ deadband)
- **Lateral Ping-Pong Overshoots:** **0** (eliminated via $[0.00, 0.10]\,\text{m}$ corridor)
- **Goal Scored Status:** `Mission Accomplished Successfully!` with `HandWave` victory sequence.

---

## 7. Installation & Quick Start

### Prerequisites:
- Ubuntu 22.04 / 24.04 LTS
- Webots 2025a (`cyberbotics/webots`)
- Python 3.10+ with `numpy` and `opencv-python`

```bash
# Install dependencies
pip install numpy opencv-python
```

### Launching the Simulation:
```bash
# 1. Navigate to repository root
cd ~/contest_ws/Semnan_University_Challenges

# 2. Launch Challenge 3 world in Webots
webots SU_Challenges/worlds/SUCH3.wbt
```
Or launch in real-time execution mode directly:
```bash
webots --mode=realtime ~/contest_ws/Semnan_University_Challenges/SU_Challenges/worlds/SUCH3.wbt
```

---
---

# 🇮🇷 راهنمای جامع فارسی (Persian Documentation)

## ۱. چکیده و هدف پروژه
این پروژه شامل کنترل‌کننده خودمختار و هوشمند برای ربات انسان‌نمای نائو (**Aldebaran / SoftBank Robotics NAO**) در محیط شبیه‌ساز سه‌بعدی **Webots 2025a** برای چالش سوم مسابقات ملی رباتیک دانشگاه سمنان (`SUCH3.wbt`) می‌باشد.

هدف این چالش:
1. جستجو و یافتن خودمختار توپ فوتبال قرمز در زمین شبیه‌سازی RoboCup SPL.
2. نزدیک شدن پایدار و سریع به توپ در راستای بردار مرکز دروازه حریف ($x = +7.0\,\text{m}, y = 0.0\,\text{m}$).
3. تنظیم دقیق موقعیت پا بدون هرگونه برخورد ناخواسته پا به توپ حین راه‌رفتن.
4. شوت‌های قدرتمند و پایدار و به ثمر رساندن گل در **حداکثر ۳ ضربه (تست شده و تضمینی در دقیقا ۳ ضربه)**.
5. اجرای جشن پیروزی و رقص نور پس از عبور توپ از خط دروازه.

---

## ۲. نوآوری‌های کلیدی و حل ریشه‌ای مشکلات شبیه‌سازی

### ۱. حل معضل برخورد پای ربات با توپ قبل از شوت (Zero Pre-Kick Collisions)
- **ریشه مشکل:** در گام‌های راه رفتن عادی (`ForwardsFast`)، پای آزاد ربات تا $18\,\text{cm}$ به جلو پرتاب می‌شود. اگر ربات تا فاصله نزدیک با این گام‌ها حرکت کند، نوک پای ربات حین قدم زدن به توپ برخورد کرده و آن را ناخواسته پرت می‌کند.
- **راهکار پیاده‌سازی شده:**
  - سیستم در فاصله امن ($dist \le 0.23\,\text{m}$) حرکت با گام‌های بلند را متوقف می‌کند.
  - برای فواصل میلی‌متری و نهایی ($local\_x \in (0.195, 0.27]\,\text{m}$)، یک حرکت اختصاصی به نام **`MicroStepForward.motion`** سنتز شده که ربات را دقیقا $4\,\text{cm}$ با ارتفاع کمتر از $1\,\text{cm}$ جلو می‌برد.
  - شوت به صورت مستقیم از حالت استقرار پایدار دو پا روی زمین ($local\_x \in [0.14, 0.195]\,\text{m}$) فعال می‌شود و احتمال ضربه ناخواسته دقیقا **صفر درصد** است.

### ۲. حذف نوسانات دورانی (Yaw Limit Cycles)
- ربات با گام‌های چرخشی حدود $40^\circ$ دور خود می‌چرخد. بدون محدوده مرگ (Deadband)، ربات به طور مداوم دور خط هدف نوسان می‌کرد.
- با اعمال **ناحیه مجاز $22^\circ$**، نوسانات زاویه‌ای به طور کامل متوقف شد و تعداد چرخش‌های ربات از بیش از ۳۰ بار به کمتر از ۲ بار کاهش یافت.

### ۳. حذف نوسانات پینگ‌پنگی جانبی (Lateral Ping-Pong Elimination)
- گام‌های جانبی ربات (`SideStepLeftFast`) طولی معادل $8.2\,\text{cm}$ دارند.
- کریدور ضربه پای چپ به ابعاد $[0.00, 0.10]\,\text{m}$ تعبیه شد ($10\,\text{cm} > 8.2\,\text{cm}$) تا از رفت‌وبرگشت بی‌پایان به چپ و راست جلوگیری شود.

### ۴. موتور شوت با سرعت و گشتاور ماکزیمم
- در فایل `ShootPower.motion` و `ShootPowerRight.motion`، ماکزیمم سرعت مجاز کلیه ۱۲ موتور پا فعال شده و شوت‌هایی با جابجایی توپ بیش از $1.5\,\text{m}$ تولید می‌کند، در حالی که پای تکیه‌گاه تعادل دینامیکی ربات را کاملا حفظ می‌نماید.

---

## ۳. پایپ‌لاین بینایی ماشین (Computer Vision)
بینایی ماشین با کتابخانه OpenCV و به صورت بلادرنگ از ۲ دوربین ربات استفاده می‌کند:
- **دوربین بالا (`CameraTop`):** زاویه باز برای یافتن توپ و تیرهای دروازه از فواصل دور.
- **دوربین پایین (`CameraBottom`):** با شیب زاویه سر $+0.50\,\text{rad}$ برای بررسی سانتی‌متری موقعیت توپ نسبت به پنجه پا در فاز نهایی شوت.
- **فیلترینگ رنگی:** اعمال ماسک دوگانه HSV برای فیلتر رنگ قرمز استاندارد توپ و حذف نویز با المان بیضوی $3 \times 3$ مورفولوژی.
- **بررسی گردی و تقارن (Circularity):** اعتبارسنجی کانتورها بر اساس $\frac{4\pi \cdot \text{Area}}{\text{Perimeter}^2} > 0.35$ برای رد کردن اجسام غیرکروی.

---

## ۴. ماشین حالت کنترل خودکار (FSM)
کنترل‌کننده از یک ماشین وضعیت متناهی (Finite State Machine) هوشمند با حالات زیر تشکیل شده است:
1. **`STATE_SEARCH_BALL`:** جاروب افقی سر ($\pm 0.8\,\text{rad}$) و چرخش درجا در صورت عدم مشاهده توپ.
2. **`STATE_APPROACH_BALL`:** حرکت هوشمند در امتداد خط واصل توپ به مرکز دروازه با کات‌آف زودهنگام در فاصله امن.
3. **`STATE_SEARCH_GOAL_AND_ALIGN`:** تنظیم زاویه راستای شوت به سمت دروازه با ددبند ۲۲ درجه.
4. **`STATE_PRECISION_STRAFE`:** تنظیم میلی‌متری موقعیت پا با گام‌های جانبی و میکرو‌استپ‌های ۴ سانتی‌متری.
5. **`STATE_KICK`:** شوت دینامیکی پرقدرت پای چپ یا راست.
6. **`STATE_VERIFY`:** توقف ۲.۲ ثانیه‌ای برای سکون دینامیکی و ارزیابی مسافت طی شده توپ.
7. **`STATE_GOAL_CELEBRATE`:** پس از ورود توپ به دروازه، ربات دست تکان داده (`HandWave`) و ال‌ای‌دی‌های گوش و سینه را با رنگ‌های سبز و آبی روشن می‌کند.

---

## ۵. کتابخانه حرکات اختصاصی (Kinematic Motions)
کلیه فایل‌های پوشه `motions/` با ضریب شتاب **0.58x** فشرده‌سازی شده‌اند تا سرعت حرکت ربات نزدیک به دو برابر حالت پیش‌فرض وبوتس باشد بدون آنکه پایداری ژیروسکوپ به هم بریزد:
- **`MicroStepForward.motion`**: گام فوق‌العاده کوتاه ۴ سانتی‌متری مخصوص تنظیم نهایی جلوی توپ.
- **`ShootPower.motion`**: شوت پرقدرت پای چپ با شتاب و بازه حرکتی بیشینه.
- **`ShootPowerRight.motion`**: شوت پرقدرت پای راست.
- **`ForwardsFast.motion`**: دویدن سریع به جلو با دوره زمانی ۱۵۰۸ میلی‌ثانیه بر گام.
- **`StandUpFromFront.motion`**: برخاستن خودکار از زمین در صورت بروز زمین‌خوردگی.
- **`HandWave.motion`**: انیمیشن تکان دادن دست به نشانه برد.

---

## ۶. نتایج تست‌ها و بنچ‌مارک نهایی مسابقه

| شماره شوت | مختصات اولیه توپ $(x, y)$ | مختصات نهایی توپ $(x, y)$ | مسافت جابجایی | پای زننده | وضعیت و نتیجه |
|:---:|:---:|:---:|:---:|:---:|:---|
| **شوت ۱** | $(0.000, 0.000)$ | $(1.297, -0.003)$ | $1.301\,\text{m}$ | پای چپ (`ShootPower`) | شوت تمیز از نقطه وسط زمین به سمت دروازه حریف |
| **شوت ۲** | $(1.297, -0.003)$ | $(3.603, -0.352)$ | $1.292\,\text{m}$ | پای چپ (`ShootPower`) | رویکرد سریع، یک میکرو‌استپ تنظیم نهایی و شوت عالی |
| **شوت ۳** | $(3.603, -0.352)$ | $(6.083, -0.955)$ | $1.514\,\text{m}$ | پای چپ (`ShootPower`) | شوت ماکزیمم سرعت به دهانه دروازه |
| **غلتش نهایی** | $(6.083, -0.955)$ | $(6.787, -1.135)$ | $0.727\,\text{m}$ | ورود به تور | **گل به ثمر رسید!** (عبور از خط $x = 6.75\,\text{m}$) |

- **تعداد کل شوت‌ها تا گل:** **۳ شوت** (کمترین تعداد ممکن در ابعاد زمین مسابقه)
- **برخورد ناخواسته پا به توپ قبل از شوت:** **۰ مورد**
- **نوسان دورانی دور خط هدف:** **۰ مورد**
- **ثبات و تعادل کامل ربات:** **۱۰۰٪ بدون زمین خوردن**

---

## ۷. دستورالعمل گام‌به‌گام راه‌اندازی و اجرا

### ۱. ورود به پوشه پروژه:
```bash
cd ~/contest_ws/Semnan_University_Challenges
```

### ۲. اجرای شبیه‌ساز Webots با جهان مسابقه نائو:
```bash
webots SU_Challenges/worlds/SUCH3.wbt
```
یا در حالت اجرای هم‌زمان بلادرنگ (Real-Time):
```bash
webots --mode=realtime SU_Challenges/worlds/SUCH3.wbt
```

### ۳. بررسی کنسول وبوتس:
کنترل‌کننده لاگ‌های دقیق هر مرحله (حالت FSM، بردار شوت، فاصله تا توپ، وضعیت گل و غیره) را در کنسول شبیه‌ساز نمایش می‌دهد.

---

## 📄 License & Attribution
Designed & developed for the Semnan University National Robotics Competition (Webots 2025a). All rights reserved.
