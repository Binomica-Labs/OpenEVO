# OpenEvo — Open Evolution Turbidostat

**Authors:** Zane Chan, Pin-Che Huang, Phillip Kyriakakis  
**License:** [MIT](https://opensource.org/licenses/MIT)  
**Latest Release:** [V1.1 — June 17, 2026](OpenEvo_2026-06-17_Release_V1.1.zip)

---

OpenEvo is an open-source turbidostat for continuous-culture directed evolution. It runs as a browser-based interface on any computer connected to an Arduino Mega 2560, with real-time OD monitoring, automated media changes, and optional one-click global remote access via Cloudflare Tunnel — no cloud account or router configuration needed.

---

## Download & Run

**The easiest way to get started is to download the release zip — it contains everything you need:**

➡️ [`OpenEvo_2026-06-17_Release_V1.1.zip`](OpenEvo_2026-06-17_Release_V1.1.zip) (~60 MB — includes Python interface, Arduino firmware, software interface manual, launchers, installers, and Cloudflare tunnel binaries for remote access)

**Full archive including hardware assembly manual** also available on Zenodo:  
➡️ [https://doi.org/10.5281/zenodo.20694532](https://doi.org/10.5281/zenodo.20694532)

Inside the zip you will find a `README.txt` with full installation instructions. The short version:

1. Extract the zip to a permanent location
2. Run `Windows_Install_OpenEvo.bat` or `Mac_Install_OpenEvo.command` to install Python packages
3. Upload `2026-06-04_OpenEvo_Firmware_V1.ino` to your Arduino Mega 2560 via Arduino IDE
4. Run `Windows_Run_OpenEvo.bat` or `Mac_Run_OpenEvo.command`
5. Open **http://localhost:8080** in your browser and click **Connect**

---

## What's in This Repository

| File | Description |
|------|-------------|
| `2026-06-17_OpenEvo_Interface_V1.1.py` | Main Python web interface (latest) |
| `2026-06-04_OpenEvo_Firmware_V1.ino` | Arduino Mega 2560 firmware |
| `OpenEVO Interface Manual.pdf` | Software interface manual |
| `Windows_Run_OpenEvo.bat` | Windows launcher |
| `Mac_Run_OpenEvo.command` | Mac launcher |
| `Windows_Install_OpenEvo.bat` | Windows package installer |
| `Mac_Install_OpenEvo.command` | Mac package installer |
| `requirements.txt` | Python dependencies |
| `OpenEvo_TEST_CHECKLIST.html` | Interactive release test checklist |
| `OpenEvo_2026-06-17_Release_V1.1.zip` | Full release zip (tracked via Git LFS) |

---

## Features

- Real-time OD monitoring with live plotting
- Automated media dilution at a configurable OD threshold
- LED cycling for directed evolution experiments
- 2–4 point OD calibration using an inverse least-squares model (`OD = a/IR + b`)
- SD card persistence — config and data survive power cycles
- Manual pump control and a cycle program editor
- CSV data export with UTC timestamps
- **One-click global remote access** via Cloudflare Tunnel (bundled, no account required)
  - Generates a public URL instantly
  - All viewers — local and remote — see the same live interface

---

## Hardware

- Arduino Mega 2560
- IR optical density sensor
- NTC thermistor temperature sensors
- Peristaltic pumps
- Stirrer motor
- LEDs
- SD card module
- USB data cable

---

## Software Requirements

- Python 3.10 or newer
- NiceGUI 3.4.1 (`pip install nicegui==3.4.1`)
- PySerial (`pip install pyserial`)
- Arduino IDE 2.x
- Windows 10/11, macOS 10.15+, or Linux

---

## Release History

| Release | Date | Notes |
|---------|------|-------|
| V1 | 2026-06-11 | Current release |

---

## License

MIT — see [https://opensource.org/licenses/MIT](https://opensource.org/licenses/MIT)
