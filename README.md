# UTSHAS Airplane Brownie Tracker
 
> **UTSHAS Brownie** -- Onboard tracker firmware for the paper airplane high altitude balloon drop mission (Mission TreeChopper).
 
---
 
<!-- Project image placeholder -->
<p align="center">
  <img src="https://i.ibb.co/1fpx2X9p/image.png" alt="UTSHAS Brownie Tracker" width="600"/>
  <br/>
  <em>UTSHAS Brownie Tracker</em>
</p>
---
 
## What is the UTSHAS Brownie?
 
The UTSHAS Brownie is a compact, lightweight tracking module developed by the University of Toronto Schools High Altitude Society (UTSHAS). It is designed to be carried aboard a paper airplane that is deployed mid-flight from a high altitude balloon
 
Unlike the main payload, which handles primary mission data collection and recovery, the Brownie is a self-contained secondary tracker attached directly to the paper airplane. Its job is simple but critical: tell us where the airplane lands.
 
### Why a separate tracker?
 
When the paper airplane separates from the balloon payload at altitude, it follows its own unpredictable flight path. The Brownie ensures that even if the airplane drifts far from the main payload landing zone, we can still locate and recover it.
 
---
 
## About This Repository
 
This repository contains the **firmware and software running on the Brownie tracker unit itself**.
 
> ⚠️ This is **not** the code for the main balloon payload. For the main payload software, see the corresponding UTSHAS payload repository (yet to be created, check back soon!).
 
### What's in here:
 
- Tracker initialization and configuration
- GPS data acquisition and parsing
- Telemetry transmission via LoRa
- Power management routines
- Flight state detection logic
---
 
## Team
 
**UTSHAS — University of Toronto Schools High Altitude Society**
 
- *Sasha Timokhov*
- *Dev Goel*
---
 
 
<p align="center">
  Made by UTSHAS
</p>
 
