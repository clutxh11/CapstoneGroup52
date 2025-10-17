# **Component Selection for Omni Ball**

This engineering design note summarizes component selections and core calculations for the Omni Ball platform. Focus is placed on the two-drive roller layout, which balances a platform by actively repositioning the spherical ball beneath the center of mass (COM).

## **1\. Motor Calculations – Two‑Drive Layout**

Goal: determine torque, power, and current requirements for the two red motors driving the ball.

\[Insert diagram of two-drive layout here – red rollers driving sphere, with R\_ball, R\_omni, and H\_COM labeled.\]

### **1.1 Symbols and Parameters**

• m – total system mass (chair \+ user \+ frame)  
• g – gravitational acceleration (9.81 m/s²)  
• H\_COM – height of center of mass above ball contact plane  
• R\_ball – radius of main sphere  
• R\_omni – radius of drive rollers  
• n – number of drive rollers (n = 2)  
• η – mechanical efficiency (≈ 0.85)  
• μ – roller‑to‑ball friction coefficient (≈ 0.8 for rubber contact)

### **1.2 Required Horizontal Acceleration**

To keep the platform balanced, the motors must accelerate the ball fast enough to shift it under the COM. Two estimation approaches:

a) \*\*Inverted‑pendulum model:\*\*  a\_req ≥ (g / H\_COM) · x\_max  
b) \*\*Time‑to‑correct model:\*\*  a\_req ≈ 2 · x\_max / t\_r²  
Use the larger of (a) and (b).

### **1.3 Tangential Force and Motor Torque**

Overall drive force on platform:

F\_total \= m · a\_req

Each motor contributes:

F\_motor ≈ F\_total / n

Motor torque after efficiency:

τ\_motor \= (F\_motor · R\_omni) / η

### **1.4 Motor Speed and Power**

ω\_motor \= v / R\_omni  
P\_motor \= τ\_motor · ω\_motor

### **1.5 Traction Requirement**

To prevent slip at the roller–ball interface:

N\_preload ≥ F\_motor / μ

Add at least 25 % margin for manufacturing variation.

### **1.6 Electrical Sizing (KV and Kt Relationship)**

For a BLDC motor:

K\_t \[N·m/A\] = 60 / (2π · KV \[rpm/V\])

Required phase current:  I ≈ τ\_motor / K\_t

### **1.7 Worked Example**

Given: m = 80 kg, x\_max = 0.10 m, t\_r = 0.30 s, H\_COM = 0.60 m, R\_ball = 0.20 m, R\_omni = 0.05 m, η = 0.85, μ = 0.8.

a\_pend = (9.81 / 0.60) · 0.10 = 1.64 m/s²  
a\_time = 2 · 0.10 / 0.30² = 2.22 m/s²  
→ a\_req = 2.22 m/s² (use higher value).

F\_total = 80 · 2.22 = 178 N  
F\_motor ≈ 178 / 2 = 89 N  
τ\_motor = (89 · 0.05) / 0.85 = 5.2 N·m  
If v = 1 m/s → ω = 1 / 0.05 = 20 rad/s (≈ 190 rpm)  
P = 5.2 · 20 = 104 W (mechanical per motor)  
N\_preload ≥ 89 / 0.8 = 111 N (choose 150–200 N)  
For 149 KV → K\_t = 60 / (2π · 149) = 0.064 N·m/A → I ≈ 81 A peak.

⚙️ Summary: Each motor must deliver ≈ 5 N·m torque and ≈ 100 W average power. Implement torque limits (\~25 A continuous) and gear reduction (\~3:1) for safer control.

## **2\. Components Overview**

### **2.1 IMU**

• \*\*Model:\*\* Bosch BNO085  
• 9‑axis absolute orientation sensor  
• Interfaces: I²C / SPI  
• Provides low‑latency fused orientation data for real‑time balance control

### **2.2 Motors**

| Motor | KV | Voltage | Max Current | Power | Shaft | Notes |
| :---- | :---- | :---- | :---- | :---- | :---- | :---- |
| Turnigy SK8 6374 | 149 | 8–12 S | 80 A | \~3.5 kW | 8 mm | Low KV → high torque; reliable |
| Maytech MTO6374 | 170 | 2–12 S | 65 A | \~3.5 kW | 8 mm | Sealed; robust |
| Flipsky 6354 | 190 | Up to 12 S | 65 A | \~2.4 kW | 8 mm | Compact |
| Maytech MTO5065 | 170 | 2–10 S | 50 A | \~1.8 kW | 8 mm | Lightweight/safer power |

### **2.3 Controllers**

• \*\*Type:\*\* VESC‑6 class (50–60 A single)  
• Features: FOC mode, current & ERPM limits, Hall/ABI encoder input, CAN/USB  
• Notes: Allows fine control, torque ramping, and regenerative braking

### **2.4 Batteries**

| Pack | Type | Voltage | Capacity | Notes |
| :---- | :---- | :---- | :---- | :---- |
| 10S2P Li‑ion | 18650 | 36 V | 6 Ah | ≈ 216 Wh; safe starter pack |
| 12S2P Li‑ion | 21700 | 44.4 V | 8 Ah | Higher torque; monitor thermals |

### **2.5 Safety Considerations**

• Set motor current limits in VESC Tool (≈ 25 A cont, 60 A peak)  
• Apply ERPM caps for max speed ≈ 1 m/s  
• Integrate hardware E‑stop switch  
• Start testing at low torque and increase gradually

🟢 This layout and motor selection ensure stable, controllable motion of the Omni Ball with sufficient safety margin for human operation.