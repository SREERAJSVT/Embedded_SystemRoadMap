# AVR Programming (ATmega328P) – Module 3 Assignments

This repository contains **15 embedded C programs** written for the
**ATmega328P** microcontroller (16 MHz). All programs are developed in a
**modular fashion** – driver source (`.c`) and header (`.h`) files are
separated from the application logic (`main.c`).

The assignments are grouped by peripheral module:

| Module | Topics |
|--------|--------|
| GPIO & Timers | Digital I/O, software delays, Timer1 CTC |
| Interrupts | External interrupts, Timer Compare Match ISRs |
| PWM | Fast PWM, Phase Correct PWM, duty‑cycle control |
| ADC | LM35 temperature sensor, potentiometer, servo mapping |
| LCD | 16×2 parallel LCD, 9‑digit counter, long/short press |
| UART | Serial command parser, PWM brightness control |
| SPI & I²C | BMP280 sensor, SSD1306 OLED, software I²C |

---

## Table of Contents

1. [Hardware & Build Notes](#hardware--build-notes)
2. [Module 3: After GPIO](#module-3-after-gpio)
   - [1. LED Toggle (3 s ON, 5 s OFF)](#1-led-toggle-3s-on-5s-off)
   - [2. Red/Green LED Toggle with Button](#2-redgreen-led-toggle-with-button)
   - [3. Train Animation (8 LEDs)](#3-train-animation-8-leds)
   - [4. 4‑Bit Binary Counter with Button](#4-4-bit-binary-counter-with-button)
3. [Module 3: Timers](#module-3-timers)
   - [5. Traffic Light Controller (Timer1, no `_delay`)](#5-traffic-light-controller-timer1-no-_delay)
4. [Module 3: Interrupts](#module-3-interrupts)
   - [6. Traffic Light with Emergency INT0](#6-traffic-light-with-emergency-int0)
   - [7. Independent Status & Heartbeat LEDs (Timer1 + Timer2)](#7-independent-status--heartbeat-leds-timer1--timer2)
5. [Module 3: PWM](#module-3-pwm)
   - [8/9. PWM Brightness Control with Button](#89-pwm-brightness-control-with-button)
6. [Module 3: ADC](#module-3-adc)
   - [10. Automated Cooling System (LM35 + DC Motor)](#10-automated-cooling-system-lm35--dc-motor)
   - [11. Servo Motor Control with Potentiometer](#11-servo-motor-control-with-potentiometer)
7. [Module 3: LCD (16×2)](#module-3-lcd-16x2)
   - [12. 9‑Digit Counter with Short/Long Press](#12-9-digit-counter-with-shortlong-press)
8. [Module 3: UART](#module-3-uart)
   - [13. UART‑Controlled LED Brightness](#13-uart-controlled-led-brightness)
9. [Module 3: SPI & I²C](#module-3-spi--ic)
   - [14. Motor Speed Control (Pot → ADC → PWM)](#14-motor-speed-control-pot--adc--pwm)
   - [15. BMP280 Temperature + OLED Display (I²C)](#15-bmp280-temperature--oled-display-ic)
10. [License](#license)

---

## Hardware & Build Notes

- **MCU**: ATmega328P @ 16 MHz (`F_CPU = 16000000UL`)
- **IDE**: Atmel Studio 7.0 (`.atsln` solutions provided)
- **Toolchain**: AVR‑GCC (`avr-gcc`)
- **Programming**: Any ISP programmer or Arduino‑as‑ISP
- **Pull‑ups**: All push‑buttons use **internal pull‑ups** on PORTD2
- **LED current**: Always use a series resistor (220 Ω – 1 kΩ)

> If your project does not compile, ensure `F_CPU` is defined **before**
> `<util/delay.h>` is included, either in `main.c` or via the compiler’s
> symbol definition (`-DF_CPU=16000000UL`).

---

## Module 3: After GPIO

### 1. LED Toggle (3 s ON, 5 s OFF)

**Folder**: `GccApplication1`

| Item | Detail |
|------|--------|
| **Objective** | Toggle an LED on PORTD5 with 3 s ON / 5 s OFF repeating forever. |
| **LED pin** | PD5 |
| **Files** | `main.c` |

**Behaviour**: LED turns ON immediately after reset, stays ON for 3 s,
then OFF for 5 s. The loop repeats indefinitely.

---

### 2. Red/Green LED Toggle with Button

**Folder**: `2RGLEDTg`

| Item | Detail |
|------|--------|
| **Objective** | Each button press toggles between a Red and a Green LED. |
| **Red LED** | PB0 |
| **Green LED** | PB1 |
| **Button** | PD2 (internal pull‑up) |
| **Files** | `main.c` |

**Key features**:
- Software debounce (20 ms delay)
- Wait‑for‑release logic ensures exactly **one** transition per press

---

### 3. Train Animation (8 LEDs)

**Folder**: `Train_animation_on_PB0_PB7`

| Item | Detail |
|------|--------|
| **Objective** | A single lit LED moves from PB0 to PB7 and wraps around. |
| **LEDs** | PB0 – PB7 |
| **Delay** | 200 ms per step |
| **Files** | `main.c` |

**Implementation**: Uses a `uint8_t` pattern shifted left each iteration.

---

### 4. 4‑Bit Binary Counter with Button

**Folder**: `4_BitBinaryCounterwithButton`

| Item | Detail |
|------|--------|
| **Objective** | Display a 4‑bit counter on PB0–PB3, increment on button press, wrap at 15. |
| **LEDs** | PB0 – PB3 |
| **Button** | PD2 |
| **Files** | `main.c` |

**Dedicated function** `updateLEDs(count)` writes only the lower nibble
of PORTB, preserving the upper bits.

---

## Module 3: Timers

### 5. Traffic Light Controller (Timer1, no `_delay`)

**Folder**: `5TrafficLightControllerN0delay`

| Item | Detail |
|------|--------|
| **Objective** | Green 10 s → Orange 3 s → Red 10 s loop without `_delay_ms`. |
| **Green LED** | PB0 |
| **Orange LED** | PB1 |
| **Red LED** | PB2 |
| **Files** | `main.c`, `timer.c`, `timer.h` |

**Architecture**:
- Timer1 in CTC mode, prescaler 64, OCR1A = 249 → **1 ms interrupt**
- `delay_ms_timer()` polls a global tick counter
- `timer.h` declares `extern volatile uint16_t timer1_ticks`

---

## Module 3: Interrupts

### 6. Traffic Light with Emergency INT0

**Folder**: `TrafficLightwithE_INT0`

| Item | Detail |
|------|--------|
| **Objective** | Normal cycle with emergency override to Green. |
| **Emergency button** | PD2 (INT0, falling edge) |
| **Files** | `main.c`, `timer.c`, `timer.h` |

**Key logic**:
- INT0 ISR sets `emergency_flag = 1`
- Main loop checks flag **after** the current light’s duration ends
- If flag is set, Green runs for an extra 10 s before resuming normal cycle

---

### 7. Independent Status & Heartbeat LEDs (Timer1 + Timer2)

**Folder**: `7_TITSHLE`

| Item | Detail |
|------|--------|
| **Objective** | Two LEDs toggle at independent intervals using Compare Match ISRs. |
| **Status LED** | PB1 – toggles every **2 s** (Timer1 CTC, OCR1A = 31249) |
| **Heartbeat LED** | PB2 – toggles every **500 ms** (Timer2 CTC, OCR2A = 124) |
| **Files** | `main.c` |

**Note**: Timer2 is 8‑bit and cannot generate a 500 ms interrupt directly
at 16 MHz. The solution uses a **software prescaler** inside the ISR
(counting 63 interrupts of ~8 ms each) to achieve ~500 ms.

---

## Module 3: PWM

### 8/9. PWM Brightness Control with Button

**Folder**: `PWMBCB`

| Item | Detail |
|------|--------|
| **Objective** | Button cycles duty cycle: 25 % → 50 % → 75 % → OFF → 25 % … |
| **PWM output** | PD6 (OC0A) |
| **Button** | PD2 |
| **Files** | `main.c`, `pwm.c`, `pwm.h` |

**PWM configuration**:
- Timer0, Fast PWM, non‑inverting mode
- Prescaler 8
- `OCR0A` values: 64 (25 %), 128 (50 %), 192 (75 %), 0 (OFF)

---

## Module 3: ADC

### 10. Automated Cooling System (LM35 + DC Motor)

**Folder**: `AC_LM35_DCM`

| Item | Detail |
|------|--------|
| **Objective** | Read LM35 temperature; adjust motor speed in three bands. |
| **LM35 output** | ADC0 (PC0) |
| **Motor PWM** | PD6 (OC0A) |
| **On/Off button** | PD2 |
| **Files** | `main.c`, `adc.c`, `adc.h`, `pwm.c`, `pwm.h` |

**Temperature‑speed mapping**:
- < 25 °C → 30 % duty
- 25–35 °C → 70 % duty
- \> 35 °C → 100 % duty
- System OFF → 0 % duty

---

### 11. Servo Motor Control with Potentiometer

**Folder**: `11SMCwPot`

| Item | Detail |
|------|--------|
| **Objective** | Map potentiometer position to servo angle (0–180°). |
| **Potentiometer** | ADC0 (PC0) |
| **Servo signal** | PB1 (OC1A) |
| **Files** | `main.c`, `adc.c`, `adc.h`, `servo.c`, `servo.h` |

**Timer1 configuration**:
- 16‑bit Phase Correct PWM, ICR1 = 39999 → **50 Hz**
- ADC value (0–1023) mapped to pulse width **1000–2000 µs**

---

## Module 3: LCD (16×2)

### 12. 9‑Digit Counter with Short/Long Press

**Folder**: `12_lcd16x2_9digit_app`

| Item | Detail |
|------|--------|
| **Objective** | Short press increments counter; long press (≥ 2 s) resets to 0. |
| **LCD** | 16×2, 4‑bit mode, PB0‑PB5 |
| **Button** | PD2 |
| **Files** | `main.c`, `lcd.c`, `lcd.h`, `timer.c`, `timer.h` |

**Pin mapping (LCD)**:
| LCD pin | ATmega328P |
|---------|------------|
| RS | PB0 |
| EN | PB1 |
| D4 | PB2 |
| D5 | PB3 |
| D6 | PB4 |
| D7 | PB5 |

---

## Module 3: UART

### 13. UART‑Controlled LED Brightness

**Folder**: `13UART_CLB`

| Item | Detail |
|------|--------|
| **Objective** | Accept `ON`, `OFF`, or 0‑100 via UART; adjust PWM brightness. |
| **UART** | 9600 baud, 8N1 |
| **PWM LED** | PD6 (OC0A) |
| **Files** | `main.c`, `uart.c`, `uart.h`, `pwm.c`, `pwm.h` |

**Command parser**:
- `"ON"` → 100 % duty, reply `LED is now ON`
- `"OFF"` → 0 % duty, reply `LED is now OFF`
- `"0"`–`"100"` → scaled duty, reply `Brightness set to X%`

---

## Module 3: SPI & I²C

### 14. Motor Speed Control (Pot → ADC → PWM)

**Folder**: `14MSCwPADCPD6ADC0`

| Item | Detail |
|------|--------|
| **Objective** | Linear mapping from potentiometer to motor speed. |
| **Potentiometer** | ADC0 (PC0) |
| **Motor PWM** | PD6 (OC0A) |
| **Files** | `main.c`, `adc.c`, `adc.h`, `pwm.c`, `pwm.h` |

**Scaling**: `ADC_value (0–1023) → OCR0A (0–255)` using integer math.

---

### 15. BMP280 Temperature + OLED Display (I²C)

**Folder**: `15BMP280OLEDDisplayI2C`

| Item | Detail |
|------|--------|
| **Objective** | Read BMP280 temperature, display on 128×64 SSD1306 OLED every 2 s. |
| **I²C bus** | Software bit‑banged on PC4 (SDA), PC5 (SCL) |
| **BMP280 address** | 0x76 |
| **SSD1306 address** | 0x3C |
| **Files** | `main.c`, `i2c.c`, `i2c.h`, `bmp280.c`, `bmp280.h`, `ssd1306.c`, `ssd1306.h` |

**Note**: External pull‑up resistors (4.7 kΩ) are required on SDA and SCL.

---

## License

This collection is part of the
[Embedded System Roadmap](https://github.com/SREERAJSVT/Embedded_SystemRoadMap)
educational repository. You are free to use, modify, and distribute these
examples for learning purposes.