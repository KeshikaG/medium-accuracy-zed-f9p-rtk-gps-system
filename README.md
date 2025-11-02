# Medium-Accuracy ZED-F9P RTK GPS System

Code and configuration files for a **low-cost, medium-accuracy RTK GPS system** built around the **u-blox ZED-F9P** GNSS receiver and **ANN-MB1-00** antenna.  
The system supports **TrigNet-based RTK corrections** and **Virtual Reference Station (VRS)** testing, with onboard **SD card logging**, **OLED display output**, and **push-button controls** for interactive operation.

---

## System Overview

This project demonstrates a complete RTK GNSS workflow using affordable, open-source hardware and software tools.  
It enables high-precision positioning by receiving real-time correction data from a network source (NTRIP) and applying it on a mobile rover.

### Features
- **u-blox ZED-F9P** receiver with RTK capability  
- **ESP32 microcontroller** for data handling and communication  
- **NTRIP client** for streaming correction data (supports TrigNet and VRS)  
- **SD card logging** of NMEA/RTCM data  
- **0.96" OLED display** for live position and status updates  
- **Push-button interface** for display toggle and SD logging control   

---

## Hardware Components

| Component | Description |
|------------|-------------|
| u-blox ZED-F9P | Dual-frequency RTK GNSS receiver |
| ANN-MB1-00 | Active multi-band GNSS antenna |
| ESP32 Dev Board | Main controller for data processing and communication |
| SD Card Module | For local data logging |
| OLED Display (SSD1306) | To show status and coordinates |
| Push Buttons | Toggle display and logging |

---

## System Architecture

The system consists of:
- **Base/Correction Source** – TrigNet or VRS server providing RTCM correction streams.
- **Mobile Rover** – Receives corrections via NTRIP, applies them using the ZED-F9P, and logs data locally.

```text
   +------------------------+
   |    TrigNet / VRS Base  |
   |   (RTCM Corrections)   |
   +-----------+------------+
               |
          Internet (NTRIP)
               |
   +-----------v------------+
   |      ESP32 + ZED-F9P   |
   |    (Rover Processing)  |
   +-----------+------------+
               |
       OLED, SD Card, Buttons
