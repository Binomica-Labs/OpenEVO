"""
================================================================================
OpenEvo (Open Evolution) - Web Interface for Arduino-Based Turbidostat
================================================================================
A browser-based control system for automated directed evolution experiments.

SYSTEM ARCHITECTURE:
  - Frontend: NiceGUI (Python-based reactive web framework)
  - Backend: Python asyncio for non-blocking serial communication
  - Hardware: Arduino Mega 2560 running turbidostat firmware
  - Connection: USB serial (auto-detected)

KEY FEATURES:
  - Real-time OD monitoring with live plotting
  - Automated media changes at threshold
  - LED cycling for evolution experiments  
  - 2-4 point OD calibration (inverse or linear models for light scattering)
  - SD card persistence (config & data)
  - Manual pump control
  - Cycler program editor
  - Data export to CSV
  - UTC timestamps for all data logging

MEMORY OPTIMIZATIONS (for long-running experiments):
  - Adaptive downsampling: keeps recent data at full resolution
  - Max 20,000 data points in memory (~24 hours at 2-sec rate)
  - Periodic garbage collection (every 5 minutes)
  - Limited peak detection (30 peaks max)
  - 2-second update interval

CALIBRATION:
  - Inverse (preferred): OD = a/IR + b
  - 2-point linear fallback: OD = slope*IR + intercept
  - Auto-switches based on number of calibration points

Authors: Zane Chan, Pin-Che Huang, Phillip Kyriakakis
Version: 0.8117 (2026-05-26)
License: MIT License (https://opensource.org/licenses/MIT)
================================================================================
"""

from nicegui import ui, app
import serial
import serial.tools.list_ports
import asyncio
import time
import csv
import os
from datetime import datetime, timedelta
import json
import math # For slope calculation
import subprocess
import threading
import urllib.request
import traceback
import gc  # For garbage collection and memory monitoring
from fastapi import Request
from fastapi.responses import JSONResponse
import tempfile
import webbrowser
import platform
import atexit
import re
import queue

print("--- OpenEvo Interface 2026-06-04 V1 ---")
print("[BUILD: V1 - 2026-06-11] (Production 1.0 - pairs with Firmware 2026-06-04 V1)")

# ============================================================================
# EXPERIMENT SCANNING & DATA MANAGEMENT
# ============================================================================

def scan_for_experiments(filepath):
    """
    SCAN CSV FILE FOR EXPERIMENT DATA
    ==================================
    Searches through a CSV file to identify experiment runs based on headers.
    Each experiment is detected by finding header rows containing 'OD940'.
    
    DETECTION LOGIC:
      - Looks for lines with 'uptime' or 'unixtime' AND 'od940'
      - Counts data rows following each header
      - Requires at least 3 data rows to qualify as an experiment
    
    RETURNS:
      List of experiment dictionaries containing:
        - header_line: Line number of header
        - data_rows: Number of data rows
        - time_idx: Column index for time data
        - od_idx: Column index for OD values
        - media_idx: Column index for media type
        - unix_time_idx: Column index for Unix timestamps
    
    USE CASE:
      - Loading historical data from SD card
      - Resuming interrupted experiments
      - Analyzing multiple experiment runs in one file
    """
    experiments = []
    try:
        with open(filepath, 'r') as f:
            all_lines = f.readlines()
        
        print(f"📄 Scanning {os.path.basename(filepath)} ({len(all_lines)} lines)...")

        for line_num, line in enumerate(all_lines):
            if ('uptime' in line.lower() or 'unixtime' in line.lower()) and 'od940' in line.lower():
                header = next(csv.reader([line.strip()]))
                
                data_count = 0
                for i in range(line_num + 1, len(all_lines)):
                    data_line = all_lines[i].strip()
                    if ('uptime' in data_line.lower() or 'unixtime' in data_line.lower()) and 'od940' in data_line.lower():
                        break
                    if data_line and ',' in data_line:
                        data_count += 1
                
                if data_count >= 3:
                    time_idx, od_idx, media_idx, unix_time_idx = None, None, None, None
                    for i, col in enumerate(header):
                        col_l = col.lower()
                        if 'uptime' in col_l: time_idx = i
                        if 'unixtime' in col_l: unix_time_idx = i
                        if 'od940' in col_l: od_idx = i
                        if 'mediatype' in col_l: media_idx = i

                    if time_idx is None: time_idx = unix_time_idx

                    if time_idx is not None and od_idx is not None:
                        experiments.append({
                            'header_line': line_num + 1,
                            'data_rows': data_count,
                            'time_idx': time_idx,
                            'od_idx': od_idx,
                            'media_idx': media_idx,
                            'unix_time_idx': unix_time_idx,
                            'start_line': line_num + 1,
                            'header_columns': header
                        })
    except Exception as e:
        print(f"❌ Error scanning for experiments: {e}")
        traceback.print_exc()
        
    print(f"✅ Found {len(experiments)} experiments in scan.")
    return experiments

# ============================================================================
# GLOBAL STATE VARIABLES
# ============================================================================
# These variables maintain the application's runtime state across functions.

# SERIAL COMMUNICATION
arduino = None  # pySerial connection object to Arduino

# EXPERIMENT STATE FLAGS
is_running = False  # True when experiment is active
is_paused = False   # True when experiment is paused (can resume)
config_updating = False  # True during config upload (pauses plotting to prevent invalid OD)
is_configuring = False  # True during configuration dialog operations
connect_in_progress = False  # V2.4: single-flight guard - blocks overlapping connect/sync flows that race for serial responses

# DATA LOGGING
csv_file = None  # Open file handle for CSV data logging
csv_writer = None  # CSV writer object
current_session_csv = None  # Path to current session's CSV file

# NETWORK/TUNNELING
is_global = False  # True if running with public URL (tunnel)
cloudflared_process = None  # Track cloudflared process (Mac/Linux)
tunnelmole_process = None  # Track tunnelmole process (Windows)
active_tunnel_type = None  # 'cloudflare' or 'tunnelmole'

# FILE SYSTEM
temp_dir = tempfile.mkdtemp()  # Temporary directory for session files
last_directory = None  # Last directory used in file dialogs

# UI REFERENCES (set during UI initialization)
# NiceGUI's auto-index page (no @ui.page decorator) shares all UI elements
# across every connected browser automatically, so plain module-level
# references are all that's needed for multi-client mirroring.
sd_card_status_label = None  # Status label for SD card indicator
start_button = None  # Reference to start/stop button
lcd_mirror = None  # Display widget that mirrors Arduino's LCD
connection_status = None  # Connection status label

# ============================================================================
# CONFIGURATION DATA
# ============================================================================
# Default experiment parameters. Loaded from oevo_config.json if available.
# These values are synced to Arduino on startup and when changed.

config_data = {
    # HARDWARE SETTINGS
    'temperature': 30.0,      # <----- Culture temperature setpoint (°C)
    'stirring_speed': 90,     # <----- Motor PWM (0-255, typically 90)
    'led_brightness': 500,    # <----- IR LED brightness (0-4095, affects OD sensitivity)
    'threshold': 5.0,         # <----- OD threshold for media change (higher = denser culture)
    
    # 2-4 POINT INVERSE CALIBRATION 
    # Measure blank media, low/med/high OD to get these values
    # More points = more robust fit (least-squares averaging reduces measurement error)
    'ir_low': 13825,     # <----- IR reading at low OD (blank media)
    'od_low': 0.0,       # <----- Known OD at low point (usually 0)
    # NOTE: Mid1 = low OD (high IR), Mid2 = high OD (low IR)
    'ir_mid1': 5496,     # <----- Point 2: IR at low-mid OD (higher IR = less cells)
    'od_mid1': 1.0,      # <----- Point 2: Known OD (low-mid)
    'ir_mid2': 3000,     # <----- Point 3: IR at high-mid OD (lower IR = more cells)
    'od_mid2': 2.5,      # <----- Point 3: Known OD (high-mid)
    'ir_high': 1224,     # <----- IR reading at high OD
    'od_high': 5.0,      # <----- Known OD at high point
    
    # BACKWARD COMPATIBILITY (old 3-point names)
    'ir_mid': 5496,      # Alias for ir_mid1
    'od_mid': 1.0,       # Alias for od_mid1
    
    # BACKWARD COMPATIBILITY (2-point linear calibration)
    # These are aliases maintained for older code
    'ir_zero': 6800,    # Alias for ir_low
    'od_zero': 1.0,     # Alias for od_low
    'ir_target': 1300,  # Alias for ir_high
    'od_target': 5.0,   # Alias for od_high
    
    # PUMP CONFIGURATION
    'pump_speeds': [1161, 1162, 1162, 1161],  # <----- PWM values [Neutral, Positive, Negative, Waste]
    'max_dispensations': 400,  # <----- Pump pulses per media change (~40mL at 0.1mL/pulse)
    
    # CALIBRATION COEFFICIENTS (calculated from points above)
    # Linear fallback: OD = slope*IR + intercept
    'slope': -0.00072727,
    'intercept': 5.945455,
    
    # NOTE: cycles_per_led_change is stored in working_program, not here
}

def downsample_to_n(data, n=5000):
    """Downsample to exactly n points for chart performance.
    
    Args:
        data: List of data points (can be simple list or list of [x,y] pairs)
        n: Target number of points (default 5000)
    
    Returns:
        Downsampled list with exactly n points (or original if shorter)
    """
    if len(data) <= n:
        return data
    
    # Take every Nth point to get close to target
    step = len(data) / n
    indices = [int(i * step) for i in range(n)]
    return [data[i] for i in indices]

# REMOVED: oevo_config.json was causing stale/corrupted data to override clean defaults
# Configuration hierarchy is now:
#   1. Hard-coded defaults in config_data dict (lines 184-231)
#   2. Arduino SD card (config.txt) when resuming experiments
# No local JSON file needed - it was redundant and problematic

# ============================================================================
# CONFIGURATION SYNCHRONIZATION
# ============================================================================

def sync_config_to_arduino():
    """
    SYNCHRONIZE CONFIGURATION TO ARDUINO
    =====================================
    Sends all configuration parameters from Python to Arduino over serial.
    This ensures Arduino uses the same settings as the interface.
    
    SYNCHRONIZATION SEQUENCE:
      1. Send 4-point calibration (triggers Arduino's auto-calculation)
      2. Send inverse calibration coefficients (the active OD model)
      3. Send hardware parameters (threshold, temperature, stirring, LED)
      4. Send pump speeds (4 pumps)
      5. Save to SD card (persists across power cycles)
    
    TIMING:
      - 0.2-0.3s delays between commands prevent serial buffer overflow
      - 0.5s delay after SD save ensures write completes
    
    RETURNS:
      bool - True if sync successful, False if Arduino not connected
    """
    if not arduino.connected:
        print("⚠️ Arduino not connected, cannot sync config")
        return False
    
    try:
        print("🔧 Synchronizing configuration to Arduino...")
        # Send 4-point calibration data
        print(f"📤 Sending 4-point calibration (slow mode):")
        print(f"   Point 1 (Low):     IR={config_data['ir_low']}, OD={config_data['od_low']}")
        print(f"   Point 2 (Mid-Low): IR={config_data['ir_mid1']}, OD={config_data['od_mid1']}")
        print(f"   Point 3 (Mid-High):IR={config_data['ir_mid2']}, OD={config_data['od_mid2']}")
        print(f"   Point 4 (High):    IR={config_data['ir_high']}, OD={config_data['od_high']}")
        
        # CRITICAL: Send each point slowly to prevent serial buffer corruption
        arduino.send_command(f"SET_CALIBRATION_POINT_1,{config_data['ir_low']},{config_data['od_low']:.3f}")
        time.sleep(0.8)
        arduino.read_data()  # Clear buffer
        
        arduino.send_command(f"SET_CALIBRATION_POINT_2,{config_data['ir_mid1']},{config_data['od_mid1']:.3f}")
        time.sleep(0.8)
        arduino.read_data()  # Clear buffer
        
        arduino.send_command(f"SET_CALIBRATION_POINT_3,{config_data['ir_mid2']},{config_data['od_mid2']:.3f}")
        time.sleep(0.8)
        arduino.read_data()  # Clear buffer
        
        arduino.send_command(f"SET_CALIBRATION_POINT_4,{config_data['ir_high']},{config_data['od_high']:.3f}")
        time.sleep(0.8)
        arduino.read_data()  # Clear buffer
        
        # Send inverse calibration coefficients (the active OD model)
        # Calculate inverse calibration first if not already done
        if 'inverseA' not in config_data or 'inverseB' not in config_data:
            calculate_inverse_calibration()
        a = config_data.get('inverseA', 0)
        b = config_data.get('inverseB', 0)
        print(f"📤 Sending inverse calibration: a={a:.2f}, b={b:.4f} → OD = {a:.2f}/IR + {b:.4f}")
        arduino.send_command(f"SET_INVERSE_CALIBRATION,{a:.6f},{b:.6f}")
        time.sleep(0.3)
        
        # Send other config parameters
        arduino.send_command(f"SET_THRESHOLD,{config_data['threshold']}")
        time.sleep(0.3)
        arduino.send_command(f"SET_TEMPERATURE,{config_data['temperature']}")
        time.sleep(0.3)
        arduino.send_command(f"SET_MAX_DISPENSATIONS,{config_data.get('max_dispensations', 400)}")
        time.sleep(0.3)
        for i, speed in enumerate(config_data.get('pump_speeds', [1161, 1162, 1162, 1161])):
            arduino.send_command(f"SET_PUMP_SPEED,{i+1},{speed}")
            time.sleep(0.2)
        arduino.send_command(f"SET_STIRRING_SPEED,{config_data.get('stirring_speed', 90)}")
        time.sleep(0.3)
        # Send LED brightness for OD940 measurement LED
        arduino.send_command(f"SET_LED_BRIGHTNESS,{config_data.get('led_brightness', 500)}")
        time.sleep(0.3)
        # NOTE: Do NOT auto-save config on startup - this would overwrite SD card settings!
        # Config is only saved when user explicitly changes values in the interface.
        # arduino.send_command("SAVE_CONFIG_TO_SD")  # COMMENTED OUT - don't overwrite SD card
        time.sleep(0.5)
        print(f"📤 Config synced: threshold={config_data['threshold']}, temp={config_data['temperature']}°C, LED={config_data.get('led_brightness', 500)}")
        return True
    except Exception as e:
        print(f"⚠️ Error syncing config to Arduino: {e}")
        return False

# ============================================================================
# CALIBRATION CALCULATION
# ============================================================================

def calculate_inverse_calibration():
    """
    CALCULATE INVERSE CALIBRATION COEFFICIENTS (LEAST-SQUARES FIT)
    ================================================================
    Computes a, b for the inverse formula: OD = a/IR + b
    This empirically models LIGHT SCATTERING (turbidity) measurements.
    Supports 2-5 calibration points using least-squares regression.
    

    Our system measures scattered light intensity, not transmitted light absorbance.
    
    WHY INVERSE MODEL (OD ∝ 1/IR)?
      - Empirically accurate across full OD range (0.1 to 5.0+)
      - Captures non-linear behavior at high cell densities
      - More stable than polynomial fits for wide IR ranges
      - Works well for turbidity/scattering measurements
    
    PRIMARY MODEL: Inverse fit (OD = A/IR + B), attempted with 2–4 points.
    FALLBACK MODEL: Linear (OD = slope*IR + intercept), used when the inverse
    fit fails (singular matrix or exception). Both Python and firmware have this
    two-path logic. The linear fallback is real — it is not conceptual.

    2-POINT vs 4-POINT (inverse fit):
      - 2 points: Accurate AT the calibration points, less accurate between
      - 4 points: Accurate across full range due to more constraints
    
    MATHEMATICAL APPROACH:
      - Least-squares fit: minimizes sum of (OD - (a/IR + b))²
      - Handles 2 to 5 calibration points
      - Solves using normal equations
      - No numpy dependency (pure Python math)
    
    UPDATES config_data:
      - inverseA, inverseB (inverse coefficients)
      - useInverse (True if using inverse, False otherwise)
      - r_squared (goodness of fit)
    """
    global config_data
    try:
        # ===== RETRIEVE ALL CALIBRATION POINTS =====
        points = []
        
        ir_low = config_data.get('ir_low', config_data.get('ir_zero', 6800))
        od_low = config_data.get('od_low', config_data.get('od_zero', 0.0))
        if ir_low > 0:
            points.append((ir_low, od_low))
        
        ir_mid1 = config_data.get('ir_mid1', config_data.get('ir_mid', 5496))
        od_mid1 = config_data.get('od_mid1', config_data.get('od_mid', 1.0))
        if ir_mid1 > 0:
            points.append((ir_mid1, od_mid1))
        
        ir_mid2 = config_data.get('ir_mid2', 0)
        od_mid2 = config_data.get('od_mid2', 2.5)
        if ir_mid2 > 0:
            points.append((ir_mid2, od_mid2))
        
        ir_high = config_data.get('ir_high', config_data.get('ir_target', 1300))
        od_high = config_data.get('od_high', config_data.get('od_target', 5.0))
        if ir_high > 0:
            points.append((ir_high, od_high))
        
        n = len(points)
        print(f"📊 Inverse least-squares calibration with {n} points:")
        for i, (ir, od) in enumerate(points, 1):
            print(f"   Point {i}: IR={ir}, OD={od}")
        
        # Need at least 2 points
        if n < 2:
            print("⚠️ Need at least 2 calibration points for inverse fit")
            config_data['useInverse'] = False
            return
        
        # ===== LEAST-SQUARES INVERSE FIT =====
        # Solve: minimize sum of (OD - (a/IR + b))²
        # Using normal equations with X = [1/IR, 1]
        
        sum_1_ir2 = sum(1/(ir**2) for ir, od in points)
        sum_1_ir = sum(1/ir for ir, od in points)
        sum_1 = n
        
        sum_od_1_ir = sum(od * (1/ir) for ir, od in points)
        sum_od = sum(od for ir, od in points)
        
        # Solve 2x2 system:
        # | sum_1_ir2  sum_1_ir |   | a |   | sum_od_1_ir |
        # | sum_1_ir   sum_1    | * | b | = | sum_od      |
        
        det = sum_1_ir2 * sum_1 - sum_1_ir * sum_1_ir
        
        if abs(det) < 1e-10:
            print("⚠️ Matrix is singular, cannot solve inverse calibration")
            config_data['useInverse'] = False
            return
        
        a = (sum_od_1_ir * sum_1 - sum_1_ir * sum_od) / det
        b = (sum_1_ir2 * sum_od - sum_1_ir * sum_od_1_ir) / det
        
        # ===== UPDATE CONFIGURATION =====
        config_data['inverseA'] = float(a)
        config_data['inverseB'] = float(b)
        config_data['useInverse'] = True
        
        # Calculate R² (goodness of fit)
        mean_od = sum_od / n
        ss_tot = sum((od - mean_od)**2 for ir, od in points)
        ss_res = sum((od - (a/ir + b))**2 for ir, od in points)
        r_squared = 1 - (ss_res / ss_tot) if ss_tot > 0 else 0
        config_data['r_squared'] = r_squared
        
        # Keep linear fallback
        config_data['slope'] = (points[-1][1] - points[0][1]) / (points[-1][0] - points[0][0])
        config_data['intercept'] = points[0][1] - config_data['slope'] * points[0][0]
        
        print(f"✅ Inverse Fit ({n} points): OD = {a:.2f}/IR + {b:.4f}")
        print(f"   R² = {r_squared:.6f} (1.0 = perfect fit, >0.95 = good)")
        print(f"   useInverse = {config_data['useInverse']}")
        
        # Show predictions at key points
        if n >= 2:
            for i, (ir, od_expected) in enumerate([points[0], points[-1]], 1):
                od_predicted = a/ir + b
                print(f"   Verification Point {i}: IR={ir} → OD={od_predicted:.3f} (expected {od_expected:.3f})")
    
    except Exception as e:
        print(f"⚠️ Could not calculate inverse calibration: {e}")
        traceback.print_exc()
        config_data['useInverse'] = False

def calculate_od_from_ir(ir_value):
    """Calculate OD from IR using inverse calibration (primary) or linear fallback"""
    global config_data
    try:
        # Try inverse first (most physically accurate for light absorption)
        if config_data.get('useInverse', False) and 'inverseA' in config_data and 'inverseB' in config_data:
            a, b = config_data['inverseA'], config_data['inverseB']
            if abs(a) > 1e-10 and ir_value > 0:  # Prevent division by zero
                od = a / ir_value + b
                return max(0.0, min(10.0, od))  # Clamp to 0-10 OD range
        
        # Fallback to linear
        slope = config_data.get('slope', -0.00072727)
        intercept = config_data.get('intercept', 5.945455)
        od = slope * ir_value + intercept
        return max(0.0, od)  # OD can't be negative
        
    except Exception as e:
        print(f"⚠️ OD calculation error: {e}")
        # Emergency fallback
        return max(0.0, -0.00072727 * ir_value + 5.945455)

def calculate_and_update_calibration():
    """
    Calculate calibration coefficients from calibration points.
    This is called when loading config from Arduino or when user changes calibration points.
    Automatically enables inverse calibration for multi-point data.
    """
    # Count valid calibration points
    valid_points = []
    ir_low = config_data.get('ir_low', config_data.get('ir_zero', 0))
    od_low = config_data.get('od_low', config_data.get('od_zero', 0.0))
    ir_mid1 = config_data.get('ir_mid1', 0)
    od_mid1 = config_data.get('od_mid1', 0.0)
    ir_mid2 = config_data.get('ir_mid2', 0)
    od_mid2 = config_data.get('od_mid2', 0.0)
    ir_high = config_data.get('ir_high', config_data.get('ir_target', 0))
    od_high = config_data.get('od_high', config_data.get('od_target', 0.0))
    
    if ir_low > 0: valid_points.append((ir_low, od_low))
    if ir_mid1 > 0: valid_points.append((ir_mid1, od_mid1))
    if ir_mid2 > 0: valid_points.append((ir_mid2, od_mid2))
    if ir_high > 0: valid_points.append((ir_high, od_high))
    
    # Auto-enable inverse calibration for 2+ points
    if len(valid_points) >= 2:
        config_data['useInverse'] = True
        calculate_inverse_calibration()
    # Fallback to linear calibration (single point or invalid data)
    else:
        # Linear calibration from two points (low and high)
        if ir_high > 0 and ir_low > 0 and ir_high != ir_low:
            config_data['slope'] = (od_high - od_low) / (ir_high - ir_low)
            config_data['intercept'] = od_low - config_data['slope'] * ir_low
        else:
            # Fallback to hardcoded defaults if points are invalid
            config_data['slope'] = -0.00072727
            config_data['intercept'] = 5.945455

# Only calculate calibration if slope/intercept are missing (shouldn't happen with defaults)
if 'slope' not in config_data or 'intercept' not in config_data:
    print("⚠️ Missing calibration values, calculating from defaults...")
    calculate_and_update_calibration()
else:
    # Dynamic calibration point counting
    points_msg = []
    if config_data.get('ir_low', 0) > 0: points_msg.append(f"Low: ({config_data['ir_low']}, {config_data['od_low']})")
    if config_data.get('ir_mid1', 0) > 0: points_msg.append(f"Mid1: ({config_data['ir_mid1']}, {config_data['od_mid1']})")
    elif config_data.get('ir_mid', 0) > 0: points_msg.append(f"Mid1: ({config_data['ir_mid']}, {config_data['od_mid']})") # Legacy
    if config_data.get('ir_mid2', 0) > 0: points_msg.append(f"Mid2: ({config_data['ir_mid2']}, {config_data['od_mid2']})")
    if config_data.get('ir_high', 0) > 0: points_msg.append(f"High: ({config_data['ir_high']}, {config_data['od_high']})")
    
    print(f"✅ Using {len(points_msg)}-point calibration:")
    for p in points_msg:
        print(f"   - {p}")

# Global access functions
def start_cloudflare_tunnel():
    """Start Cloudflare Tunnel and return the public URL (no account needed!)"""
    global cloudflared_process
    try:
        print("🌍 Starting Cloudflare Tunnel (no signup required)...")
        
        # Try to find cloudflared executable
        cloudflared_path = None
        
        # Check local folder first. IMPORTANT (V1.8): pick the binary that
        # matches THIS platform. Release folders ship BOTH cloudflared (Mac/Linux)
        # and cloudflared.exe (Windows), so we must not blindly grab the .exe —
        # doing that on a Mac launches the Windows binary and fails with
        # "Permission denied" / cannot execute.
        local_cf = os.path.join(os.getcwd(), 'cloudflared')
        local_cf_exe = os.path.join(os.getcwd(), 'cloudflared.exe')
        
        if platform.system() == "Windows":
            # Windows: prefer the .exe, fall back to the extensionless binary.
            if os.path.exists(local_cf_exe):
                cloudflared_path = local_cf_exe
            elif os.path.exists(local_cf):
                cloudflared_path = local_cf
        else:
            # Mac/Linux: use the extensionless binary, never the .exe.
            if os.path.exists(local_cf):
                cloudflared_path = local_cf

        if cloudflared_path is None:
            # Not found in the local folder above — check the system PATH.
            # Check system PATH
            if platform.system() == "Windows":
                check_result = subprocess.run(['where', 'cloudflared'], capture_output=True, text=True)
            else:
                check_result = subprocess.run(['which', 'cloudflared'], capture_output=True, text=True)
            
            if check_result.returncode == 0:
                cloudflared_path = 'cloudflared'
            else:
                return ("❌ Cloudflare Tunnel not found.\n"
                       "The installer should have included cloudflared.\n"
                       "Download from: https://github.com/cloudflare/cloudflared/releases\n"
                       " Or run the installer again.")

        # On Mac/Linux make sure the bundled binary is executable (zip transfers
        # can strip the +x bit, which also surfaces as "Permission denied").
        if cloudflared_path and cloudflared_path != 'cloudflared' and platform.system() != "Windows":
            try:
                import stat as _stat
                _mode = os.stat(cloudflared_path).st_mode
                os.chmod(cloudflared_path, _mode | _stat.S_IXUSR | _stat.S_IXGRP | _stat.S_IXOTH)
            except Exception as _chmod_err:
                print(f"   ⚠️ Could not set execute permission on cloudflared: {_chmod_err}")
        
        # Kill any existing cloudflared processes before starting new one
        print("   🧹 Cleaning up any old cloudflared processes...")
        try:
            if platform.system() == "Windows":
                result = subprocess.run(['taskkill', '/f', '/im', 'cloudflared.exe'], capture_output=True, text=True)
            else:
                result = subprocess.run(['pkill', '-9', '-f', 'cloudflared'], capture_output=True, text=True)
            time.sleep(2)  # Wait for processes to die
            print("   ✅ Cleanup complete")
        except Exception as e:
            print(f"   ⚠️ Cleanup error (probably no processes to clean): {e}")
        
        print("🚀 Starting Cloudflare Tunnel on port 8080...")
        print("   ⏳ Generating public URL (takes ~10 seconds)...")
        print("   💡 No signup needed - powered by Cloudflare's network")
        
        # Start cloudflared tunnel and track the process.
        # Note: previously we forced --protocol http2 (TCP) to avoid UDP/QUIC
        # firewall issues, but on Windows that path was dropping WebSocket
        # frames to remote clients (page rendered but charts/status never
        # mirrored).  Letting cloudflared auto-negotiate (default = QUIC with
        # automatic fallback to http2 if UDP is blocked) fixes that and still
        # works behind restrictive firewalls because of the fallback.
        # V10 fix: isolate cloudflared from our console. On Windows a child that
        # shares the console also shares Ctrl+C / console-close events, so calling
        # .terminate() on it later sent a control event back to THIS Python process
        # and froze the whole server when the user clicked "Stop Global Access".
        # CREATE_NEW_PROCESS_GROUP + CREATE_NO_WINDOW fully detaches it so stopping
        # the tunnel never touches the server.
        popen_kwargs = dict(
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            bufsize=1,
        )
        if platform.system() == "Windows":
            popen_kwargs['creationflags'] = (
                subprocess.CREATE_NO_WINDOW | subprocess.CREATE_NEW_PROCESS_GROUP
            )
        cloudflared_process = subprocess.Popen(
            [cloudflared_path, 'tunnel', '--url', 'http://127.0.0.1:8080'],
            **popen_kwargs
        )
        cf_process = cloudflared_process  # Keep local reference too
        
        # Cloudflare prints URL to stderr (not stdout)
        url_found = None
        start_time = time.time()
        
        while time.time() - start_time < 30:  # Cloudflare can take up to 30 seconds
            # Check if process died
            if cf_process.poll() is not None:
                stdout, stderr = cf_process.communicate()
                error_msg = stderr.strip() if stderr else "Process exited without error message"
                stdout_msg = stdout.strip() if stdout else ""
                return (f"❌ Cloudflare Tunnel process died:\n"
                       f"Exit code: {cf_process.returncode}\n"
                       f"stderr: {error_msg}\n"
                       f"stdout: {stdout_msg}\n"
                       f"💡 Try manually: ./cloudflared tunnel --url http://127.0.0.1:8080")
            
            # Read stderr for URL (cloudflared outputs to stderr)
            try:
                line = cf_process.stderr.readline()
                if line:
                    line = line.strip()
                    print(f"   [CF] {line}")
                    
                    # Cloudflare prints: "... https://example.trycloudflare.com"
                    if 'https://' in line and 'trycloudflare.com' in line:
                        import re
                        match = re.search(r'https://[^\s]+\.trycloudflare\.com', line)
                        if match:
                            url_found = match.group(0)
                            break
            except:
                pass
            
            time.sleep(0.5)
        
        if url_found:
            print(f"✅ Cloudflare Tunnel URL: {url_found}")
            print(f"   🔓 No password protection")
            print(f"   ⚠️  URL changes each session")
            print(f"   ⚡ Powered by Cloudflare's global network")
            return url_found
        else:
            return "⚠️ Cloudflare Tunnel started but URL not captured.\n   Check terminal for URL."
            
    except FileNotFoundError:
        return ("❌ cloudflared not found.\n"
               "The installer should have included it.\n"
               "Download from: https://github.com/cloudflare/cloudflared/releases")
    except Exception as e:
        return f"❌ Cloudflare Tunnel error: {str(e)}"

def stop_cloudflare_tunnel():
    """Stop Cloudflare Tunnel properly"""
    global cloudflared_process
    try:
        print("🛑 Stopping Cloudflare Tunnel...")
        
        # Try to terminate the tracked process first
        if cloudflared_process and cloudflared_process.poll() is None:
            print("   Terminating tracked process...")
            cloudflared_process.terminate()
            
            # Wait up to 5 seconds for graceful shutdown
            import time
            for i in range(10):
                time.sleep(0.5)
                if cloudflared_process.poll() is not None:
                    print("   ✅ Process terminated gracefully")
                    cloudflared_process = None
                    return True
            
            # Force kill if still running
            print("   Force killing process...")
            cloudflared_process.kill()
            cloudflared_process = None
        
        # Also kill any orphaned cloudflared processes (cleanup)
        if platform.system() == "Windows":
            subprocess.run(['taskkill', '/f', '/im', 'cloudflared.exe'], capture_output=True)
        else:
            subprocess.run(['pkill', '-9', '-f', 'cloudflared'], capture_output=True)
        
        print("   ✅ Cloudflare Tunnel stopped")
        return True
        
    except Exception as e:
        print(f"   ⚠️ Error stopping tunnel: {e}")
        # Try brute force cleanup
        if platform.system() == "Windows":
            subprocess.run(['taskkill', '/f', '/im', 'cloudflared.exe'], capture_output=True)
        else:
            subprocess.run(['pkill', '-9', '-f', 'cloudflared'], capture_output=True)
        cloudflared_process = None
        return False

def start_tunnelmole_tunnel():
    """Start Tunnelmole tunnel and return the public URL (no account needed, works on Windows)."""
    global tunnelmole_process
    try:
        print("🌍 Starting Tunnelmole Tunnel (no signup required)...")
        
        tmole_path = None
        local_tmole = os.path.join(os.getcwd(), 'tmole.exe')
        local_tmole_mac = os.path.join(os.getcwd(), 'tmole')
        
        if platform.system() == "Windows":
            if os.path.exists(local_tmole):
                tmole_path = local_tmole
            else:
                check = subprocess.run(['where', 'tmole'], capture_output=True, text=True)
                if check.returncode == 0:
                    tmole_path = 'tmole'
                else:
                    check = subprocess.run(['where', 'npx'], capture_output=True, text=True)
                    if check.returncode == 0:
                        tmole_path = 'npx'
        else:
            if os.path.exists(local_tmole_mac):
                tmole_path = local_tmole_mac
            else:
                check = subprocess.run(['which', 'tmole'], capture_output=True, text=True)
                if check.returncode == 0:
                    tmole_path = 'tmole'
                else:
                    check = subprocess.run(['which', 'npx'], capture_output=True, text=True)
                    if check.returncode == 0:
                        tmole_path = 'npx'
        
        if not tmole_path:
            return ("❌ Tunnelmole (tmole.exe) not found. "
                   "Place tmole.exe in the same folder as this script, or install via: npm install -g tunnelmole")
        
        # Kill any existing tmole processes
        try:
            if platform.system() == "Windows":
                subprocess.run(['taskkill', '/f', '/im', 'tmole.exe'], capture_output=True)
            else:
                subprocess.run(['pkill', '-9', '-f', 'tmole'], capture_output=True)
            time.sleep(1)
        except Exception:
            pass
        
        print("🚀 Starting Tunnelmole on port 8080...")
        env = os.environ.copy()
        env['TUNNELMOLE_QUIET_MODE'] = '0'
        
        if tmole_path == 'npx':
            tunnelmole_process = subprocess.Popen(
                ['npx', 'tunnelmole', '8080'],
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                text=True, bufsize=1, env=env, cwd=os.getcwd()
            )
        else:
            tunnelmole_process = subprocess.Popen(
                [tmole_path, '8080'],
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                text=True, bufsize=1, env=env, cwd=os.getcwd()
            )

        # Read stdout in a background thread so a silent process can't block our timeout
        line_queue: "queue.Queue[str]" = queue.Queue()

        def _drain_pipe(pipe, q):
            try:
                for raw in iter(pipe.readline, ''):
                    q.put(raw)
            except Exception:
                pass

        reader = threading.Thread(
            target=_drain_pipe, args=(tunnelmole_process.stdout, line_queue), daemon=True
        )
        reader.start()

        url_found = None
        start_time = time.time()
        # Accept the current .tunnelmole.net domain as well as any future
        # tunnelmole.* host (e.g. .com) so the URL doesn't get missed.
        url_pattern = re.compile(r'https://[A-Za-z0-9.-]+\.tunnelmole\.[A-Za-z]+')

        while time.time() - start_time < 15:
            if tunnelmole_process.poll() is not None:
                return "❌ Tunnelmole process exited. Check if port 8080 is in use."
            try:
                line = line_queue.get(timeout=0.3).strip()
            except queue.Empty:
                continue
            if not line:
                continue
            print(f"   [TM] {line}")
            match = url_pattern.search(line)
            if match:
                url_found = match.group(0)
                break

        if url_found:
            print(f"✅ Tunnelmole URL: {url_found}")
            return url_found

        # No URL captured in time - kill the orphan process so it doesn't keep running silently
        print("⚠️ Tunnelmole did not report a URL within 15s. Stopping process.")
        try:
            stop_tunnelmole_tunnel()
        except Exception:
            pass
        return "⚠️ Tunnelmole started but URL not captured. Check terminal."

    except Exception as e:
        return f"❌ Tunnelmole error: {str(e)}"

def stop_tunnelmole_tunnel():
    """Stop Tunnelmole tunnel."""
    global tunnelmole_process
    try:
        print("🛑 Stopping Tunnelmole...")
        if tunnelmole_process and tunnelmole_process.poll() is None:
            tunnelmole_process.terminate()
            for _ in range(10):
                time.sleep(0.5)
                if tunnelmole_process.poll() is not None:
                    break
            else:
                tunnelmole_process.kill()
        tunnelmole_process = None
        if platform.system() == "Windows":
            subprocess.run(['taskkill', '/f', '/im', 'tmole.exe'], capture_output=True)
        else:
            subprocess.run(['pkill', '-9', '-f', 'tmole'], capture_output=True)
        print("   ✅ Tunnelmole stopped")
    except Exception as e:
        print(f"   ⚠️ Error stopping Tunnelmole: {e}")
    tunnelmole_process = None

def start_tunnel():
    """Start tunnel - Cloudflare on all platforms (handles WebSockets reliably)."""
    global active_tunnel_type
    active_tunnel_type = 'cloudflare'
    return start_cloudflare_tunnel()

def stop_tunnel():
    """Stop whichever tunnel is active."""
    global active_tunnel_type
    if active_tunnel_type == 'tunnelmole':
        stop_tunnelmole_tunnel()
    else:
        stop_cloudflare_tunnel()
    active_tunnel_type = None

# Global access password
global_access_password = None

async def show_global_password_dialog():
    """Show confirmation dialog for global access (tunnel - no password required)"""
    global global_access_password

    tunnel_label = "Cloudflare Tunnel"

    # Create an event to signal when dialog is closed
    dialog_closed = asyncio.Event()
    user_cancelled = False
    
    with ui.dialog() as password_dialog, ui.card().style('min-width: 450px;'):
        ui.label('🌍 Start Global Access?').classes('text-h6 q-mb-md')

        with ui.column().classes('q-mb-md'):
            ui.label(f'Create a public URL using {tunnel_label}.').classes('text-body2')
            ui.separator().classes('q-my-sm')

            ui.label('  • URL is temporary (changes each session)').classes('text-body2')
            ui.separator().classes('q-my-sm')
            ui.label('💡 Anyone with the URL can access your OpenEvo').classes('text-caption text-grey-6')
            ui.label('   Only share with trusted users').classes('text-caption text-grey-6')

        def confirm_start():
            global global_access_password
            global_access_password = None  # Tunnel services don't require a password
            password_dialog.close()
            dialog_closed.set()

        def cancel_dialog():
            nonlocal user_cancelled
            user_cancelled = True
            password_dialog.close()
            dialog_closed.set()

        with ui.row().classes('w-full justify-center q-gutter-md q-mt-lg'):
            ui.button('Cancel', on_click=cancel_dialog).props('flat size=md')
            ui.button('✅ Start Tunnel', on_click=confirm_start).props('color=primary size=lg')
    
    password_dialog.open()
    
    # Wait for the dialog to be closed
    await dialog_closed.wait()
    
    # Return True if user proceeded, False if cancelled
    return not user_cancelled

# Data storage
time_data = []
od_data = []
ir_data = []
temp_data = []
unix_time_data = []
media_data = []  # Track media type for each data point
peak_times = []
peak_media_types = []  # Track media type at each peak (when dilution triggered)
peak_intervals = []
media_areas = []
last_media_type = "NEUTRAL"
last_media_start_time = 0
od_chart_x_range = [0, 24] # For slope calculation

# Chart objects
od_chart = None
peak_chart = None
csv_data_pending_chart_update = False

# Status tracking
current_status = "Ready"
current_cycle = 1
current_step = 1
current_media_temp = 0.0
current_ambient_temp = 0.0
current_heater_temp = 0.0
current_heater_pwm = 0
current_led_brightness = 0  # Arduino PWM value (0-4095), will be updated from Arduino state
current_media_type = "NEUTRAL"
previous_media_type = "NEUTRAL"  # Track previous media type for peak coloring
total_program_steps = 9  # Default value, will be updated from cycler.csv
current_led_state = "OFF"

# Light dose tracking
# LED is OFF for ~115ms every 1000ms during OD measurement (88.5% duty cycle)
OD_MEASUREMENT_DUTY_CYCLE = 0.885  # <-- fraction (unitless, 0-1)

# LED Calibration - measured intensity at 100% PWM for each LED channel
# Channels 1-6 correspond to different stimulation LEDs
# Users measure with a light meter and enter values here
led_calibration = {
    1: {'intensity': 0.0, 'wavelength': ''},  # LED 1: intensity in mW/cm² <--, wavelength in nm <--
    2: {'intensity': 0.0, 'wavelength': ''},  # LED 2: intensity in mW/cm² <--, wavelength in nm <--
    3: {'intensity': 0.0, 'wavelength': ''},  # LED 3: intensity in mW/cm² <--, wavelength in nm <--
    4: {'intensity': 0.0, 'wavelength': ''},  # LED 4: intensity in mW/cm² <--, wavelength in nm <--
    5: {'intensity': 0.0, 'wavelength': ''},  # LED 5: intensity in mW/cm² <--, wavelength in nm <--
    6: {'intensity': 0.0, 'wavelength': ''},  # LED 6: intensity in mW/cm² <--, wavelength in nm <--
}
current_led_channel = 1  # Which LED channel is currently active (for irradiance calculation)

# ============================================================================
# STARTUP: Load config from local file (LED calibration comes from Arduino SD card)
# ============================================================================
def load_config_on_startup():
    """Load config from local JSON file on startup (NOT LED calibration - that's on Arduino SD card)"""
    global config_data
    try:
        if os.path.exists('oevo_config.json'):
            with open('oevo_config.json', 'r') as f:
                saved_config = json.load(f)
                # NOTE: LED calibration is NOT loaded from local file anymore
                # It will be loaded from Arduino's SD card when connected
                # Only load other config values (calibration points, thresholds, etc.)
                for key in config_data.keys():
                    if key in saved_config and key != 'led_calibration':
                        config_data[key] = saved_config[key]
                print("✅ Loaded config from oevo_config.json on startup (LED calibration will come from Arduino SD)")
        else:
            print("ℹ️ No oevo_config.json found, using defaults")
    except Exception as e:
        print(f"⚠️ Could not load config on startup: {e}")

# Call on module load
load_config_on_startup()

# Global working program - preserves unsaved changes
def validate_and_fix_working_program():
    """Ensure all steps in working_program have valid data"""
    global working_program
    
    if 'steps' not in working_program:
        working_program['steps'] = []
    
    valid_media_types = ['NEUTRAL', 'POSITIVE', 'NEGATIVE']
    fixed_count = 0
    
    for i, step in enumerate(working_program['steps']):
        # Fix media type
        if 'media_type' not in step or step['media_type'] not in valid_media_types or not step['media_type']:
            print(f"⚠️ Fixed step {i+1} media type: '{step.get('media_type', 'MISSING')}' → 'NEUTRAL'")
            step['media_type'] = 'NEUTRAL'
            fixed_count += 1
        
        # Fix LED brightness - must be 0-100%
        if 'led_brightness' not in step or not isinstance(step['led_brightness'], (int, float)):
            step['led_brightness'] = 0
            fixed_count += 1
        elif step['led_brightness'] < 0 or step['led_brightness'] > 100:
            old_val = step['led_brightness']
            step['led_brightness'] = max(0, min(100, int(step['led_brightness'])))
            print(f"⚠️ Fixed step {i+1} LED brightness: {old_val}% → {step['led_brightness']}%")
            fixed_count += 1
            
        # Temperature removed - now controlled globally via config, not per-step
    
    if fixed_count > 0:
        print(f"✅ Fixed {fixed_count} corrupted step parameters")
    
    return working_program

working_program = {
    'steps': [
        {'media_type': 'NEUTRAL', 'led_brightness': 0},
        {'media_type': 'NEUTRAL', 'led_brightness': 0},
        {'media_type': 'NEGATIVE', 'led_brightness': 0},
        {'media_type': 'NEGATIVE', 'led_brightness': 0},
        {'media_type': 'NEUTRAL', 'led_brightness': 0},
        {'media_type': 'NEUTRAL', 'led_brightness': 0},
        {'media_type': 'POSITIVE', 'led_brightness': 50},
        {'media_type': 'POSITIVE', 'led_brightness': 50},
        {'media_type': 'POSITIVE', 'led_brightness': 50}
    ],
    'cycles_per_led_change': 1  # <----- CHANGE CYCLES PER LED CHANGE HERE
}

# Validate program on startup
validate_and_fix_working_program()

# Program capture variables
program_capture_active = False
program_capture_started = False
program_capture_steps = []
program_capture_cycles = 1

# Current state capture variables
current_state_cycle = 0
current_state_step = 0
current_ir = 0
current_od = 0.0  # <----- Current OD from Arduino (updated every data point, not downsampled)
sd_card_status = "Checking..."
sd_card_free_space = 0
has_previous_experiment_data = False

# ============================================================================
# ARDUINO SERIAL COMMUNICATION CLASS
# ============================================================================
# Handles USB serial connection, command sending, and data reading.
# Uses auto-detection to find Arduino on any available port.

class ArduinoConnection:
    """
    ARDUINO USB SERIAL CONNECTION MANAGER
    ======================================
    Manages bidirectional serial communication with Arduino turbidostat.
    
    KEY FEATURES:
      - Auto-detection of Arduino port (no manual port selection needed)
      - Connection verification via PING/PONG handshake
      - Command queueing with newline-terminated protocol
      - Line-based data reading with UTF-8 decoding
      - Error handling and automatic reconnection support
    
    ATTRIBUTES:
      ser: pySerial Serial object (None if not connected)
      connected: bool - True when connected and verified
      port: str - Device path of connected port (e.g., '/dev/ttyUSB0')
    
    PROTOCOL:
      - Commands: Text strings terminated with '\n'
      - Responses: '$' prefix for structured data, other for debug
      - Baud rate: 115200
      - Timeout: 1 second for read operations
    """
    def __init__(self):
        self.ser = None
        self.connected = False
        self.port = None
        self.last_known_port = None  # Remember last working port for faster reconnection
        self.last_data_time = 0  # Track last successful communication (0 = never)
        self.last_keepalive_time = 0  # Track last keep-alive ping
        self._ever_received_data = False  # True once we've received real data
        self._read_error_count = 0  # Track consecutive read errors
        self.monitor_thread = None
        self.monitor_running = False
        self.lock = threading.Lock()
        self.user_initiated_connection = False  # Track if user explicitly connected

    def start_monitor(self):
        """Start background thread to monitor and restore connection"""
        if self.monitor_running:
            return
        self.monitor_running = True
        self.monitor_thread = threading.Thread(target=self._monitor_loop, daemon=True)
        self.monitor_thread.start()
        print("🛡️ USB Watchdog started")

    def _monitor_loop(self):
        """Background loop to check connection health and reconnect"""
        while self.monitor_running:
            try:
                current_time = time.time()
                
                # CASE 1: Disconnected - Try to reconnect (ONLY if user previously connected)
                if not self.connected and self.user_initiated_connection:
                    # Only scan every 5 seconds to avoid spamming logs
                    if current_time - self.last_keepalive_time > 5:
                        print("🛡️ Watchdog: Scanning for Arduino...")
                        # We use find_arduino() but need to be careful about threading
                        # find_arduino manages its own connection logic
                        if self.find_arduino():
                            print("🛡️ Watchdog: Reconnection successful!")
                            self.last_data_time = time.time()
                        self.last_keepalive_time = current_time
                
                # CASE 2: Connected but quiet - nudge with a PING to keep alive.
                # IMPORTANT (V2.2): do NOT force-close on "no PONG". That heuristic
                # repeatedly killed perfectly healthy connections during the
                # connect/resume command bursts (SET_CALIBRATION_POINT / SET_STEP /
                # START_TURBIDOSTAT), when the Arduino is briefly busy and not
                # streaming data, so last_data_time goes momentarily stale. Genuine
                # disconnects (USB unplugged, port gone) are already detected by
                # read_data(), which raises and calls close(). So here we only send
                # a keep-alive PING and let real I/O errors handle true disconnects.
                elif self.connected and (current_time - self.last_data_time > 10):
                    print(f"🛡️ Watchdog: No data for {int(current_time - self.last_data_time)}s - sending PING")
                    self.send_command("PING")
                
                # CASE 3: Healthy - Just keep alive
                else:
                    # Send keep-alive every 2 seconds to be aggressive against USB sleep
                    if current_time - self.last_keepalive_time > 2:
                        self.send_keepalive()
                        
                time.sleep(1) # Check every second
                
            except Exception as e:
                print(f"⚠️ Watchdog error: {e}")
                time.sleep(5) # Back off on error

    def find_arduino(self):
        """
        AUTO-DETECT AND CONNECT TO ARDUINO
        ===================================
        Scans all serial ports and attempts connection to Arduino-like devices.
        
        DETECTION LOGIC:
          - Looks for common Arduino USB-to-serial chips (CH340, CP210x, FTDI)
          - Checks device name patterns (usbserial, usbmodem, ttyUSB, ttyACM)
          - Excludes system ports (Bluetooth, debug, incoming, Intel AMT)
          - Verifies connection with PING/PONG handshake
        
        CONNECTION SEQUENCE:
          1. Scan all COM/tty ports
          2. Filter for Arduino-like characteristics
          3. Open serial connection (115200 baud)
          4. Wait 3 seconds for Arduino reset
          5. Read initialization messages
          6. Send PING command
          7. Wait for PONG response
          8. Mark as connected if verified
        
        PREVIOUS EXPERIMENT DETECTION:
          - Monitors init messages for SD card data
          - Sets has_previous_experiment_data flag if found
          - Enables "Resume" option in UI
        
        RETURNS:
          bool - True if Arduino found and verified, False otherwise
        """
        ports = serial.tools.list_ports.comports()
        print(f"🔍 Found {len(ports)} serial ports: {[p.device for p in ports]}")
        
        # Sort ports to prioritize known Arduino chips (CH340, etc.)
        def port_priority(port):
            desc = port.description.lower()
            # Highest priority: Known Arduino USB chips
            if 'ch340' in desc or 'ch341' in desc:
                return 0
            if 'cp210' in desc:
                return 1
            if 'ftdi' in desc:
                return 2
            if 'arduino' in desc:
                return 3
            if 'usb serial' in desc or 'usb-serial' in desc:
                return 4
            # Lower priority for generic COM ports
            return 10
        
        sorted_ports = sorted(ports, key=port_priority)
        
        for port in sorted_ports:
            print(f"🔍 Checking port: {port.device} (description: {port.description})")
            desc_lower = port.description.lower()
            
            # Only connect to actual Arduino/USB serial ports - MUST have USB serial chip indicator
            is_arduino_port = (
                # macOS/Linux Arduino USB-to-serial chips
                'usbserial' in port.device.lower() or 
                'usbmodem' in port.device.lower() or
                'ttyUSB' in port.device.lower() or 
                'ttyACM' in port.device.lower() or
                'wchusbserial' in port.device.lower() or
                # Arduino in description
                'arduino' in desc_lower or
                # Common USB-to-serial chip descriptions (REQUIRED for Windows COM ports)
                'ch340' in desc_lower or
                'ch341' in desc_lower or
                'cp210' in desc_lower or
                'ftdi' in desc_lower or
                'usb serial' in desc_lower or
                'usb-serial' in desc_lower or
                'usbser' in desc_lower
            )
            
            # Exclude system/debug/bluetooth/Intel AMT ports
            is_system_port = (
                'debug' in port.device.lower() or
                'bluetooth' in port.device.lower() or
                'incoming' in port.device.lower() or
                'jbl' in port.device.lower() or  # Bluetooth devices
                'airpods' in port.device.lower() or
                # Windows built-in serial ports (NOT USB!)
                'communications port' in desc_lower or
                'com1)' in desc_lower or  # Built-in COM1
                # Intel AMT (Active Management Technology) - NOT an Arduino!
                'intel' in desc_lower or
                'amt' in desc_lower or
                'active management' in desc_lower or
                'sol' in desc_lower or  # Serial Over LAN
                # Other system ports
                'modem' in desc_lower and 'usb' not in desc_lower or
                'fax' in desc_lower
            )
            
            if is_arduino_port and not is_system_port:
                print(f"✅ Arduino-like port detected: {port.device}")
                try:
                    print(f"🔍 Attempting to connect to {port.device}...")
                    self.ser = serial.Serial(
                        port.device, 
                        115200, 
                        timeout=2,  # Increased timeout for reliability
                        write_timeout=2,
                        inter_byte_timeout=None
                    )
                    # Keep USB port active (prevent Windows USB suspend)
                    self.ser.dtr = True
                    self.ser.rts = False
                    print(f"🔍 Serial port opened, waiting for Arduino to reset and initialize...")
                    
                    # Wait for Arduino to reset and send initialization messages
                    time.sleep(3)
                    
                    # Clear any initialization messages and check for previous experiment data
                    global has_previous_experiment_data
                    has_previous_experiment_data = False  # Reset flag - will be set if SD card has data
                    while self.ser.in_waiting > 0:
                        line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                        if line:
                            print(f"📥 Init: {line}")
                            # Detect previous experiment data from Arduino initialization messages
                            if ("OEVO.csv found on SD card" in line or 
                                "Restored state" in line or 
                                "Last line content:" in line):
                                has_previous_experiment_data = True
                                print("📊 Previous experiment data detected during Arduino initialization")
                    
                    self.connected = True
                    self.port = port.device # Store the port name
                    print(f"✅ Connected to Arduino on {self.port}")
                    # VERIFY it's the right device
                    if self.verify_connection():
                        print(f"✅ Connection verified via PING/PONG")
                        self.user_initiated_connection = True  # Mark that user connected
                        return True
                    else:
                        print(f"❌ Verification failed. Wrong device or not ready.")
                        self.close()
                        continue
                except Exception as e:
                    print(f"❌ Failed to connect to {port.device}: {e}")
                    if self.ser: self.ser.close()
                    self.connected = False
                    continue
            else:
                if is_system_port:
                    print(f"⏭️ Skipping system port: {port.device}")
                else:
                    print(f"⏭️ Skipping non-Arduino port: {port.device}")
        print("❌ No Arduino found on any port")
        return False

    def verify_connection(self, timeout=10):
        """Send a PING and wait for a PONG to verify it's the correct device."""
        if not self.ser or not self.connected:
            print("❌ Verification failed - no serial connection")
            return False
        
        print("🔍 Verifying connection with PING...")
        self.send_command("PING")
        
        start_time = time.time()
        pong_received = False
        
        while time.time() - start_time < timeout:
            lines = self.read_data() # Use the existing read_data method
            for line in lines:
                print(f"🔍 Verification received: {line}")
                if "PONG" in line:
                    print("✅ PONG received - connection verified!")
                    pong_received = True
                    return True
                # Also accept any Arduino response as a sign of life
                if line.startswith("$DEBUG") or line.startswith("$SD_STATUS"):
                    print("✅ Arduino responding - connection verified!")
                    return True
            time.sleep(0.1) # Small delay to prevent busy-waiting
        
        print(f"❌ Verification timeout after {timeout}s - no PONG received")
        return False

    def send_command(self, command):
        # Try to send if serial port exists - don't require connected flag
        if self.ser:
            try:
                # V2.3: no lock - matches the proven-working 2026-03-04 baseline.
                # The V2.1 self.lock caused watchdog/read-loop contention that
                # starved read_data() during the connect burst and tripped the
                # watchdog. Direct write+flush is what reliably connects.
                self.ser.write(f"{command}\n".encode())
                self.ser.flush()  # Force immediate transmission to prevent buffer concatenation
                print(f"📤 SENT: {command}")
            except (OSError, serial.SerialException) as e:
                # Only disconnect on serious errors (port gone, permission denied, etc.)
                print(f"❌ Connection lost during send_command '{command}': {e}")
                self.close()
            except Exception as e:
                # For other errors, just log and continue (might be temporary)
                print(f"⚠️ Temporary send error '{command}': {e}")

    def read_data(self):
        lines = []
        # Try to read if serial port exists - don't check self.connected flag!
        # The connected flag can be wrong, but if the port works, we're connected.
        if self.ser:
            try:
                # V2.3: no lock - matches the proven-working 2026-03-04 baseline.
                while self.ser.in_waiting:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        # Only print non-data lines (data lines start with @)
                        if not line.startswith('@'):
                            print(f"📡 ARDUINO: {line}")
                        lines.append(line)
                        self.last_data_time = time.time()  # Update last successful communication
                        self._ever_received_data = True    # Mark that we've received real data
                        # If we got data, we're definitely connected - fix the flag!
                        if not self.connected:
                            print("🔄 Connection restored - data received, updating connected flag")
                            self.connected = True
                # Reset error counter on successful read
                self._read_error_count = 0
            except (OSError, serial.SerialException) as e:
                # Disconnect on serious errors (port gone, permission denied, etc.)
                print(f"❌ Serial read error - disconnecting: {e}")
                self.close()
            except TypeError as e:
                # "argument must be an int, or have a fileno() method" = port is invalid
                print(f"❌ Serial port invalid - disconnecting: {e}")
                self.close()
            except Exception as e:
                # Track repeated errors - disconnect after 5 consecutive failures
                if not hasattr(self, '_read_error_count'):
                    self._read_error_count = 0
                self._read_error_count += 1
                if self._read_error_count >= 5:
                    print(f"❌ Too many read errors - disconnecting: {e}")
                    self._read_error_count = 0
                    self.close()
                else:
                    print(f"⚠️ Read error ({self._read_error_count}/5): {e}")
        return lines
    
    def send_keepalive(self):
        """Send periodic keep-alive ping to prevent USB suspend"""
        global config_updating
        # Skip keep-alive during config updates to prevent serial buffer corruption
        if config_updating:
            return
        current_time = time.time()
        # Send keep-alive every 30 seconds
        if current_time - self.last_keepalive_time > 30:
            if self.ser:  # Don't require connected flag - just need serial port
                try:
                    self.ser.write(b"PING\n")
                    self.last_keepalive_time = current_time
                    print(f"💓 Keep-alive ping sent (last data: {int(current_time - self.last_data_time)}s ago)")
                except:
                    pass  # Don't disconnect on keep-alive failure

    def close(self):
        """Close serial connection and stop watchdog"""
        # Stop the watchdog first
        self.monitor_running = False
        
        # Close serial port
        if self.ser:
            try:
                self.ser.close()
                print("🔌 Serial port closed")
            except:
                pass
        self.ser = None  # Clear serial object so status check knows we're disconnected
        self.connected = False
        self.user_initiated_connection = False
        self._ever_received_data = False  # Reset for next connection
    
    def shutdown(self):
        """Complete shutdown - call this when exiting the application"""
        print("🛑 Shutting down Arduino connection...")
        self.monitor_running = False
        
        # Wait for watchdog thread to stop (max 2 seconds)
        if self.monitor_thread and self.monitor_thread.is_alive():
            self.monitor_thread.join(timeout=2)
        
        # Close serial port
        if self.ser:
            try:
                self.ser.close()
                print("🔌 Serial port released")
            except:
                pass
        self.connected = False
        print("✅ Arduino connection shutdown complete")

    async def send_command_async(self, command):
        # Try to send if serial port exists - don't require connected flag
        if self.ser:
            try:
                # V2.3: no lock - matches the proven-working 2026-03-04 baseline.
                self.ser.write(f"{command}\n".encode())
                self.ser.flush()  # Force immediate transmission to prevent buffer concatenation
                print(f"📤 SENT: {command}")
            except (OSError, serial.SerialException) as e:
                # Only disconnect on serious errors (port gone, permission denied, etc.)
                print(f"❌ Connection lost during send_command_async '{command}': {e}")
                self.close()
            except Exception as e:
                # For other errors, just log and continue (might be temporary)
                print(f"⚠️ Temporary async send error '{command}': {e}")

# Initialize Arduino
arduino = ArduinoConnection()

def safe_notify(message, **kwargs):
    """Wrapper for ui.notify that won't crash if the browser tab was closed"""
    try:
        ui.notify(message, **kwargs)
    except RuntimeError:
        pass

def handle_state_handshake(state_line):
    """Handle state handshake from Arduino to restore experiment state"""
    global is_running, is_paused, start_button, current_led_channel, current_cycle, current_step, current_media_type
    try:
        # Parse: $STATE,isRunning,isPaused,currentCycle,currentStep,currentLEDPin,currentLEDBrightness,currentLEDState,currentMediaType,dynamicThreshold,incubationSetpointTemp,totalProgramSteps,maxDispensations,stirringSpeed,ledBrightness,irZero,odZero,irTarget,odTarget,pumpSpeed1,pumpSpeed2,pumpSpeed3,pumpSpeed4
        parts = state_line.split(',')
        if len(parts) >= 23:  # Backward compatible with current firmware
            is_running_state = parts[1] == '1'
            is_paused_state = parts[2] == '1'
            current_cycle = int(parts[3])
            current_step = int(parts[4])
            current_led_pin = int(parts[5])
            current_led_brightness = int(parts[6])
            current_led_state = parts[7] == '1'
            current_media_type = parts[8].strip() if parts[8].strip() else "NEUTRAL"  # Default to NEUTRAL if empty
            threshold = float(parts[9])
            temp = float(parts[10])
            total_steps = int(parts[11])
            max_dispensations = int(parts[12])
            
            # Track active LED channel for irradiance calculation
            if current_led_pin >= 1 and current_led_pin <= 6:
                current_led_channel = current_led_pin
            
            # Check if this is a previous experiment
            # ONLY consider it a previous experiment if:
            # 1. Experiment is currently running, OR
            # 2. Cycle is > 1 (has advanced past first cycle), OR
            # 3. Step is > 1 (has advanced past first step)
            # NOTE: Do NOT use media_type != "" as a condition - Arduino always has default media type!
            # NOTE: Do NOT use has_previous_experiment_data from init messages alone - needs SD card verification
            has_previous_data = (is_running_state or 
                               (current_cycle > 1) or 
                               (current_cycle == 1 and current_step > 1))
            
            if is_running_state:
                print(f"🔄 RECONNECTING to running experiment:")
                print(f"   📊 Cycle {current_cycle}, Step {current_step}")
                led_percent_reconnect = int(current_led_brightness/4095*100) if current_led_brightness > 0 else 50  # Default to 50% if not reported
                print(f"   💡 LED{current_led_pin}: {led_percent_reconnect}%")
                print(f"   🧪 Media: {current_media_type}")
                print(f"   ⏸️ Paused: {'Yes' if is_paused_state else 'No'}")
                
                # Update global state
                is_running = True
                is_paused = is_paused_state
                
                # Update button state
                if start_button is not None:
                    if is_paused_state:
                        start_button.text = 'Resume'
                        start_button.props('color=positive')
                        start_button.props('icon=play_arrow')
                    else:
                        start_button.text = 'Pause'
                        start_button.props('color=warning')
                        start_button.props('icon=pause')
                    start_button.update()

                # Load configuration from Arduino to sync with saved settings
                try:
                    # Parse additional config values from extended $STATE response
                    stirring_speed = int(parts[13])
                    led_brightness = int(parts[14])
                    # Adaptive parsing for old (23/25) vs new (27) firmware
                    if len(parts) >= 27:
                        # New firmware with 4-point calibration
                        ir_zero = int(parts[15])      # irLower (Point 1)
                        od_zero = float(parts[16])    # odLower
                        ir_mid1 = int(parts[17])      # irMidLow (Point 2)
                        od_mid1 = float(parts[18])    # odMidLow
                        ir_mid2 = int(parts[19])      # irMidHigh (Point 3)
                        od_mid2 = float(parts[20])    # odMidHigh
                        ir_target = int(parts[21])    # irUpper (Point 4)
                        od_target = float(parts[22])  # odUpper
                        pump_speed_1 = int(parts[23])
                        pump_speed_2 = int(parts[24])
                        pump_speed_3 = int(parts[25])
                        pump_speed_4 = int(parts[26])
                        ir_mid = ir_mid2  # Use mid2 for backward compat
                        od_mid = od_mid2
                    elif len(parts) >= 25:
                        # Old firmware with 3-point calibration
                        ir_zero = int(parts[15])  # irLower
                        od_zero = float(parts[16])  # odLower
                        ir_mid = int(parts[17])     # irMid
                        od_mid = float(parts[18])   # odMid
                        ir_target = int(parts[19])  # irUpper
                        od_target = float(parts[20]) # odUpper
                        pump_speed_1 = int(parts[21])
                        pump_speed_2 = int(parts[22])
                        pump_speed_3 = int(parts[23])
                        pump_speed_4 = int(parts[24])
                    else:
                        # Old firmware with 2-point calibration
                        ir_zero = int(parts[15])  # irLower
                        od_zero = float(parts[16])  # odLower
                        ir_mid = 4000  # Default middle point
                        od_mid = 3.0   # Default middle point
                        ir_target = int(parts[17])  # irUpper
                        od_target = float(parts[18]) # odUpper
                        pump_speed_1 = int(parts[19])
                        pump_speed_2 = int(parts[20])
                        pump_speed_3 = int(parts[21])
                        pump_speed_4 = int(parts[22])
                    
                    # Update local config with Arduino's current values
                    config_data['threshold'] = threshold
                    config_data['temperature'] = temp
                    config_data['max_dispensations'] = max_dispensations
                    config_data['stirring_speed'] = stirring_speed
                    config_data['led_brightness'] = led_brightness
                    # 3-point calibration data
                    config_data['ir_low'] = ir_zero    # irLower
                    config_data['od_low'] = od_zero    # odLower
                    config_data['ir_mid'] = ir_mid     # irMid
                    config_data['od_mid'] = od_mid     # odMid
                    config_data['ir_high'] = ir_target # irUpper
                    config_data['od_high'] = od_target # odUpper
                    # Backward compatibility
                    config_data['ir_zero'] = ir_zero
                    config_data['od_zero'] = od_zero
                    config_data['ir_target'] = ir_target
                    config_data['od_target'] = od_target
                    config_data['pump_speeds'] = [pump_speed_1, pump_speed_2, pump_speed_3, pump_speed_4]
                    
                    # Recalculate calibration from loaded values
                    calculate_and_update_calibration()
                    
                    print(f"📋 Synced ALL config from Arduino: threshold={threshold}, temperature={temp}°C, max_disp={max_dispensations}")
                    print(f"   stirring_speed={stirring_speed}, led_brightness={led_brightness}")
                    print(f"   calibration: ir_zero={ir_zero}, od_zero={od_zero}, ir_target={ir_target}, od_target={od_target}")
                    print(f"   pump_speeds=[{pump_speed_1}, {pump_speed_2}, {pump_speed_3}, {pump_speed_4}]")
                except Exception as e:
                    print(f"⚠️ Error syncing config from Arduino: {e}")
                
                # Don't send new header - experiment is continuing
                safe_notify(f'🔄 Reconnected to running experiment (Cycle {current_cycle}, Step {current_step})', type='positive')
            elif has_previous_data:
                print(f"📊 DETECTED previous experiment data:")
                print(f"   📊 Last position: Cycle {current_cycle}, Step {current_step}")
                print(f"   🧪 Last media: {current_media_type}")
                print(f"   💾 Previous experiment data available for continuation")
                
                # Don't set is_running=True since it's not currently running
                # But the interface should offer to continue from this point
                
                # Set a global flag to indicate previous data exists
                global has_previous_experiment_data
                has_previous_experiment_data = True
            else:
                print("🔍 No running experiment found - will use system defaults for new experiment")
                # Don't load config from SD card for new experiments - use defaults
            
            # Only sync config from Arduino state if experiment is running
            try:
                # Parse additional config values from extended $STATE response
                stirring_speed = int(parts[13])
                led_brightness = int(parts[14])
                # Adaptive parsing for old (23/25) vs new (27) firmware
                if len(parts) >= 27:
                    # New firmware with 4-point calibration
                    ir_zero = int(parts[15])      # irLower
                    od_zero = float(parts[16])    # odLower
                    ir_mid1 = int(parts[17])      # irMidLow
                    od_mid1 = float(parts[18])    # odMidLow
                    ir_mid2 = int(parts[19])      # irMidHigh
                    od_mid2 = float(parts[20])    # odMidHigh
                    ir_target = int(parts[21])    # irUpper
                    od_target = float(parts[22])  # odUpper
                    pump_speed_1 = int(parts[23])
                    pump_speed_2 = int(parts[24])
                    pump_speed_3 = int(parts[25])
                    pump_speed_4 = int(parts[26])
                    # Use mid2 as the primary mid point for backward compatibility
                    ir_mid = ir_mid2
                    od_mid = od_mid2
                elif len(parts) >= 25:
                    # Old firmware with 3-point calibration
                    ir_zero = int(parts[15])  # irLower
                    od_zero = float(parts[16])  # odLower
                    ir_mid1 = int(parts[17])     # irMid
                    od_mid1 = float(parts[18])   # odMid
                    ir_mid2 = 0                  # Not available
                    od_mid2 = 0.0                # Not available
                    ir_target = int(parts[19])  # irUpper
                    od_target = float(parts[20]) # odUpper
                    pump_speed_1 = int(parts[21])
                    pump_speed_2 = int(parts[22])
                    pump_speed_3 = int(parts[23])
                    pump_speed_4 = int(parts[24])
                    ir_mid = ir_mid1
                    od_mid = od_mid1
                else:
                    # Very old firmware with 2-point calibration
                    ir_zero = int(parts[15])  # irLower
                    od_zero = float(parts[16])  # odLower
                    ir_mid1 = 0                  # Not available
                    od_mid1 = 0.0                # Not available
                    ir_mid2 = 0                  # Not available
                    od_mid2 = 0.0                # Not available
                    ir_target = int(parts[17])  # irUpper
                    od_target = float(parts[18]) # odUpper
                    pump_speed_1 = int(parts[19])
                    pump_speed_2 = int(parts[20])
                    pump_speed_3 = int(parts[21])
                    pump_speed_4 = int(parts[22])
                    ir_mid = 0
                    od_mid = 0.0
                
                # Update local config with Arduino's current values
                config_data['threshold'] = threshold
                config_data['temperature'] = temp
                config_data['max_dispensations'] = max_dispensations
                config_data['stirring_speed'] = stirring_speed
                config_data['led_brightness'] = led_brightness
                # 4-point calibration data
                config_data['ir_low'] = ir_zero       # irLower (Point 1)
                config_data['od_low'] = od_zero       # odLower
                config_data['ir_mid1'] = ir_mid1      # irMidLow (Point 2)
                config_data['od_mid1'] = od_mid1      # odMidLow
                config_data['ir_mid2'] = ir_mid2      # irMidHigh (Point 3)
                config_data['od_mid2'] = od_mid2      # odMidHigh
                config_data['ir_high'] = ir_target    # irUpper (Point 4)
                config_data['od_high'] = od_target    # odUpper
                # Backward compatibility
                config_data['ir_zero'] = ir_zero
                config_data['od_zero'] = od_zero
                config_data['ir_mid'] = ir_mid if ir_mid > 0 else (ir_mid2 if ir_mid2 > 0 else ir_mid1)
                config_data['od_mid'] = od_mid if od_mid > 0 else (od_mid2 if od_mid2 > 0 else od_mid1)
                config_data['ir_target'] = ir_target
                config_data['od_target'] = od_target
                config_data['pump_speeds'] = [pump_speed_1, pump_speed_2, pump_speed_3, pump_speed_4]
                
                # Recalculate calibration from loaded values
                calculate_and_update_calibration()
                
                print(f"📋 Synced ALL config from Arduino state: threshold={threshold}, temperature={temp}°C, max_disp={max_dispensations}")
                print(f"   stirring_speed={stirring_speed}, led_brightness={led_brightness}")
                if ir_mid1 > 0 or ir_mid2 > 0:
                    print(f"   calibration (4-point): ir_low={ir_zero}, od_low={od_zero}")
                    if ir_mid1 > 0: print(f"                          ir_mid1={ir_mid1}, od_mid1={od_mid1}")
                    if ir_mid2 > 0: print(f"                          ir_mid2={ir_mid2}, od_mid2={od_mid2}")
                    print(f"                          ir_high={ir_target}, od_high={od_target}")
                else:
                    print(f"   calibration (2-point): ir_low={ir_zero}, od_low={od_zero}, ir_high={ir_target}, od_high={od_target}")
                print(f"   pump_speeds=[{pump_speed_1}, {pump_speed_2}, {pump_speed_3}, {pump_speed_4}]")
                if config_data.get('useInverse', False):
                    print(f"   📐 Using INVERSE calibration: OD = {config_data.get('inverseA', 0):.2f}/IR + {config_data.get('inverseB', 0):.4f}, R²={config_data.get('r_squared', 0):.4f}")
                else:
                    print(f"   📐 Using LINEAR fallback: slope={config_data.get('slope', 0):.8f}, intercept={config_data.get('intercept', 0):.6f}")
            except Exception as e:
                print(f"⚠️ Error syncing config from Arduino state: {e}")
                
    except Exception as e:
        print(f"❌ Error parsing state handshake: {e}")
        print(f"   Raw line: {state_line}")
async def update_data():
    """Update data from Arduino and update charts while preserving zoom."""
    global current_cycle, current_step, current_media_temp, current_ambient_temp, current_heater_temp, current_heater_pwm, current_led_brightness, current_od
    global current_media_type, previous_media_type, current_led_state, current_ir, last_media_type, last_media_start_time, media_areas
    global csv_writer, csv_file, sd_card_status, sd_card_free_space, is_running, sd_card_status_label
    global time_data, od_data, temp_data, ir_data, unix_time_data, peak_times, peak_intervals
    global connection_status  # Add connection_status to globals

    # Send periodic keep-alive is now handled by the background watchdog
    # arduino.send_keepalive()

    # ===== TRY to read data even if arduino.connected is False =====
    # This prevents showing DISCONNECTED prematurely due to race conditions with watchdog
    # The read_data() function will handle the actual connection check
    data_lines = []
    try:
        if arduino and arduino.ser:
            data_lines = arduino.read_data()
    except Exception as e:
        print(f"⚠️ Error reading Arduino data: {e}")
    
    # Update connection status based on actual data flow, not just flags
    # This prevents false DISCONNECTED status when data is actually flowing
    data_was_received = bool(data_lines)
    
    # Check if we've received data recently (within 30 seconds)
    # BUT only if we have a serial connection (arduino.ser exists)
    received_data_recently = False
    if arduino and arduino.ser and hasattr(arduino, 'last_data_time'):
        seconds_since_data = time.time() - arduino.last_data_time
        # Only count as "recent" if we've actually received data at least once
        # (last_data_time > startup time + 5 seconds means we got real data)
        if hasattr(arduino, '_ever_received_data') and arduino._ever_received_data:
            received_data_recently = seconds_since_data < 30
    
    if connection_status:
        if data_was_received:
            # Got data right now - definitely connected
            connection_status.text = '🟢 CONNECTED'
            if hasattr(update_data, 'disconnect_counter'):
                update_data.disconnect_counter = 0
        elif arduino and arduino.connected:
            # Connected flag is True - show connected
            connection_status.text = '🟢 CONNECTED'
        elif received_data_recently:
            # No data this cycle, but we got data recently - still connected
            # This catches the case where arduino.connected flag is wrong
            connection_status.text = '🟢 CONNECTED'
            if hasattr(update_data, 'disconnect_counter'):
                update_data.disconnect_counter = 0
        else:
            # No recent data at all - truly disconnected
            connection_status.text = '🔴 DISCONNECTED'
            if not hasattr(update_data, 'disconnect_counter'):
                update_data.disconnect_counter = 0
            update_data.disconnect_counter += 1
            if update_data.disconnect_counter % 10 == 0:
                print(f"⚠️ Arduino not connected - skipping data update ({update_data.disconnect_counter}s)")
            return
    new_data_point = False
    
    # Data reception logged only on first connect (removed per-cycle spam)
    
    # Program capture during GET_CURRENT_STATE (global variables)
    global working_program, total_program_steps, program_capture_active, program_capture_started, program_capture_steps, program_capture_cycles
    global current_state_cycle, current_state_step
    
    if data_lines:
        for line in data_lines:
            # Capture current state for startup dialog and sync configuration
            if line.startswith("$STATE,"):
                try:
                    parts = line.split(',')
                    if len(parts) >= 5:  # Need at least 5 parts to get cycle and step
                        current_state_cycle = int(parts[3])  # currentCycle is at index 3
                        current_state_step = int(parts[4])   # currentStep is at index 4
                        print(f"🔍 CAPTURED STATE: Cycle {current_state_cycle}, Step {current_state_step}")
                        
                        # Handle state handshake to sync configuration and experiment state
                        handle_state_handshake(line)
                        
                except (ValueError, IndexError) as e:
                    print(f"⚠️ Error parsing current state: {e}")
            
            # Program capture logic
            if program_capture_active:
                if line.startswith("$PROGRAM_START"):
                    program_capture_started = True
                    parts = line.split(',')
                    if len(parts) >= 3:
                        try:
                            program_capture_cycles = int(parts[2])
                            print(f"🔍 Program cycles: {program_capture_cycles}")
                        except Exception:
                            program_capture_cycles = 1
                elif line.startswith("$PROGRAM_STEP") and program_capture_started:
                    parts = line.split(',')
                    if len(parts) >= 5:
                        try:
                            media_type = parts[2]
                            led_pwm = int(parts[3])
                            # Temperature from Arduino is ignored - use global config temperature instead
                            led_pct = int((led_pwm / 4095.0) * 100)
                            # Clamp LED percentage to valid range 0-100
                            led_pct = max(0, min(100, led_pct))
                            program_capture_steps.append({'media_type': media_type, 'led_brightness': led_pct})
                            print(f"🔍 Added step: {media_type}, {led_pct}% LED")
                        except Exception as e:
                            print(f"⚠️ Error parsing step: {e}")
                elif line.startswith("$PROGRAM_END") and program_capture_started:
                    if program_capture_steps:
                        try:
                            working_program = {'steps': program_capture_steps, 'cycles_per_led_change': program_capture_cycles}
                            total_program_steps = len(program_capture_steps)
                            print(f"✅ Program captured: {total_program_steps} steps, {program_capture_cycles} cycles per LED")
                            print(f"🔍 Global working_program now has {len(working_program['steps'])} steps")
                        except Exception as _e:
                            print(f"⚠️ Failed to capture program: {_e}")
                    else:
                        print(f"⚠️ No steps to capture!")
                    program_capture_active = False
                    program_capture_started = False

    # Check for physical button pause/resume messages (any button)
    global is_running, is_paused
    for line in data_lines:
        # Check for new debug format pause/unpause
        if "$DEBUG,PAUSED" in line:
            print(f"🔍 Physical button PAUSE detected! Line: {line.strip()}")
            is_paused = True  # Keep is_running=True, set paused flag
            # Update button to show Resume
            if start_button is not None:
                start_button.text = 'Resume'
                start_button.props('color=positive')
                start_button.props('icon=play_arrow')
                start_button.update()
            ui.notify(f'⏸️ Experiment paused', type='info')
            
        elif "$DEBUG,UNPAUSED" in line:
            print(f"🔍 Physical button UNPAUSE detected! Line: {line.strip()}")
            print(f"🔍 Setting is_paused from {is_paused} to False")
            is_paused = False
            print(f"🔍 is_paused is now: {is_paused}")
            # Update button to show Pause
            if start_button is not None:
                start_button.text = 'Pause'
                start_button.props('color=warning')
                start_button.props('icon=pause')
                start_button.update()
            ui.notify(f'▶️ Experiment resumed', type='positive')
            
        # Pump sequencing debug messages - Print to console, minimal UI notifications
        elif "$DEBUG,EMPTYING_COMPLETE" in line:
            print(f"🔍 PUMP: Emptying phase complete")
            # No notification - progress bar shows this
            
        elif "$DEBUG,STARTING_FILL" in line:
            print(f"🔍 PUMP: Starting fill phase")
            # No notification - progress bar shows this
            
        elif "$DEBUG,FILLING_COMPLETE" in line:
            print(f"🔍 PUMP: Filling phase complete")
            # No notification - progress bar shows this
            
        elif "$DEBUG,MEDIA_CHANGE_STARTING," in line:
            max_disp = line.split("max_dispensations=")[1].strip()
            print(f"🔍 PUMP: Media change starting with {max_disp} max dispensations")
            # No notification - too spammy during normal operation
            
        elif "$DEBUG,MEDIA_CHANGE_COMPLETE" in line:
            print(f"🔍 PUMP: Media change complete")
            # No notification - dilution event is already logged
            
        # Pump progress messages - Enhanced to show ALL pumping activity
        elif line.startswith("$PUMP_PROGRESS,"):
            try:
                parts = line.split(',')
                if len(parts) >= 4:
                    operation = parts[1]  # EMPTYING, FILLING, or MANUAL_PUMP_X
                    current_vol = float(parts[2])  # Current volume dispensed
                    total_vol = float(parts[3])    # Total volume to dispense
                    progress_pct = (current_vol / total_vol) * 100
                    
                    # Handle different operation types with enhanced display
                    if operation.startswith("MANUAL_PUMP_"):
                        pump_num = operation.split("_")[-1]  # Extract pump number
                        # Get pump name for progress display
                        pump_names = {1: 'Neutral', 2: 'Positive', 3: 'Negative', 4: 'Empty'}
                        pump_name = pump_names.get(int(pump_num), f'Pump {pump_num}')
                        operation_display = f"🔧 Manual Pump {pump_num} ({pump_name})"
                        # Show every 1.0mL for manual pumps (prevent notification spam)
                        if abs(round(current_vol, 1) - round(current_vol)) < 0.15:
                            ui.notify(f'{operation_display}: {current_vol:.1f}mL / {total_vol:.1f}mL ({progress_pct:.0f}%)', type='info', timeout=2)
                    elif operation == "EMPTYING":
                        operation_display = "🔄 Emptying Vessel"
                        # Show every 1.0mL for emptying
                        if abs(round(current_vol, 1) - round(current_vol)) < 0.15:
                            ui.notify(f'{operation_display}: {current_vol:.1f}mL / {total_vol:.1f}mL ({progress_pct:.0f}%)', type='warning', timeout=2)
                    elif operation == "FILLING":
                        operation_display = "🔄 Filling Vessel"
                        # Show every 1.0mL for filling
                        if abs(round(current_vol, 1) - round(current_vol)) < 0.15:
                            ui.notify(f'{operation_display}: {current_vol:.1f}mL / {total_vol:.1f}mL ({progress_pct:.0f}%)', type='positive', timeout=2)
                    else:
                        operation_display = f"🔄 {operation.title()}"
                        # Show progress notification every 1.0mL
                        if abs(round(current_vol, 1) - round(current_vol)) < 0.15:
                            ui.notify(f'{operation_display}: {current_vol:.1f}mL / {total_vol:.1f}mL ({progress_pct:.0f}%)', type='info', timeout=2)
                    
                    pass  # Silent - pump progress logged via UI notification
            except Exception as e:
                print(f"⚠️ Error parsing pump progress: {e}")
            
        # SD file debugging messages - Only notify on errors, not normal operations
        elif "$DEBUG,SD_FILE_OPENED_FOR_LOGGING" in line:
            print(f"🔍 SD: File opened for logging successfully")
            # No notification - normal operation
            
        elif "$DEBUG,SD_FILE_OPEN_FAILED" in line:
            print(f"🔍 SD: File open failed for logging")
            ui.notify('❌ SD file open failed - Check SD card', type='negative', timeout=10)
            
        elif "$DEBUG,FILENAME:" in line:
            filename = line.split("FILENAME:")[1].strip()
            print(f"🔍 SD: Attempted filename: {filename}")
            # No notification - only relevant during errors
            
        elif "$DEBUG,DATA_FILE_OPEN_FAILED" in line:
            print(f"🔍 SD: Data file open failed")
            ui.notify('❌ Data file open failed - Check SD card', type='negative', timeout=10)
            
        elif "$DEBUG,AVERAGED_DATA_FILE_OPEN_FAILED" in line:
            print(f"🔍 SD: Averaged data file open failed")
            ui.notify('❌ Averaged data file open failed - Check SD card', type='negative', timeout=10)
            
        elif "$DEBUG,SD_HEADER_WRITTEN" in line:
            print(f"🔍 SD: Header written successfully")
            # No notification - normal operation
            
        elif "$DEBUG,SD_HEADER_WRITE_FAILED" in line:
            print(f"🔍 SD: Header write failed")
            ui.notify('❌ SD header write failed - Check SD card', type='negative', timeout=10)
            
        # LCD update messages - sync is_paused state from Arduino to prevent state mismatch
        elif "$DEBUG,LCD_UPDATE," in line:
            # Parse isPaused state from Arduino and sync Python state
            # Format: $DEBUG,LCD_UPDATE,isPaused:FALSE,watchdog:FALSE,sdCard:TRUE
            try:
                if "isPaused:TRUE" in line and not is_paused:
                    print(f"🔄 SYNC: Arduino reports PAUSED, updating Python state")
                    is_paused = True
                    if start_button is not None:
                        start_button.text = 'Resume'
                        start_button.props('color=positive icon=play_arrow')
                        start_button.update()
                elif "isPaused:FALSE" in line and is_paused:
                    print(f"🔄 SYNC: Arduino reports UNPAUSED, updating Python state")
                    is_paused = False
                    if start_button is not None:
                        start_button.text = 'Pause'
                        start_button.props('color=warning icon=pause')
                        start_button.update()
            except Exception as e:
                print(f"⚠️ Error syncing pause state: {e}")
            
        elif "$DEBUG,LCD_DISPLAY," in line:
            display_type = line.split("LCD_DISPLAY,")[1].strip()
            # Only notify on errors, not normal operation
            if display_type == "SD_ERROR":
                ui.notify('❌ SD Card Error - Check LCD display', type='negative', timeout=5)
            elif display_type == "WATCHDOG":
                ui.notify('⚠️ Watchdog Alert - Check LCD display', type='warning', timeout=5)
        
        # Arduino state messages - Print to console (UI already handles these via other mechanisms)
        elif "$DEBUG,EXPERIMENT_STARTED" in line:
            print(f"🔍 ARDUINO: Experiment started")
            # No notification - UI already shows this via start button
            
        elif "$DEBUG,EXPERIMENT_STOPPED" in line:
            print(f"🔍 ARDUINO: Experiment stopped")
            # No notification - UI already shows this via start button
            
        elif "$DEBUG,EXPERIMENT_PAUSED" in line:
            print(f"🔍 ARDUINO: Experiment paused")
            # No notification - already handled by physical button pause message
            
        elif "$DEBUG,EXPERIMENT_UNPAUSED" in line:
            print(f"🔍 ARDUINO: Experiment unpaused")
            # No notification - already handled by physical button unpause message
        
        # LED Calibration responses - parse and store globally
        # This catches responses that might be missed by the dedicated load function
        elif line.startswith("$LED_CAL,"):
            try:
                # Parse: $LED_CAL,<channel>,<intensity>,<wavelength>
                parts = line.split(',')
                if len(parts) >= 4:
                    channel = int(parts[1])
                    intensity = float(parts[2])
                    wavelength = int(parts[3]) if parts[3] else 0
                    if 1 <= channel <= 6:
                        led_calibration[channel] = {
                            'intensity': intensity,
                            'wavelength': str(wavelength) if wavelength > 0 else ''
                        }
                        if intensity > 0:
                            print(f"💡 LED{channel} calibration loaded: {intensity} mW/cm² @ {wavelength}nm")
            except (ValueError, IndexError) as e:
                print(f"⚠️ Error parsing LED calibration: {e}")
        
        # LCD Content messages - Mirror the physical LCD display
        elif line.startswith("$LCD_CONTENT,"):
            try:
                lcd_content = line.split("$LCD_CONTENT,")[1].strip()
                lcd_lines = lcd_content.split("|")
                
                # Extract LED channel from LCD content (e.g., "Media:POS LED6:20%")
                global current_led_channel
                for lcd_line in lcd_lines:
                    if 'LED' in lcd_line and ':' in lcd_line:
                        import re
                        led_match = re.search(r'LED(\d+):', lcd_line)
                        if led_match:
                            current_led_channel = int(led_match.group(1))
                
                # Update the LCD mirror display
                if 'lcd_mirror' in globals() and lcd_mirror:
                    lcd_mirror.text = ""
                    for line_content in lcd_lines:
                        if line_content.strip():  # Only add non-empty lines
                            lcd_mirror.text += f"{line_content.strip()}\n"
                    lcd_mirror.text = lcd_mirror.text.rstrip()  # Remove trailing newline
                
            except Exception as e:
                print(f"⚠️ Error parsing LCD content: {e}")
        
        # Debug: Check for any LCD-related messages
        elif "$DEBUG,LCD" in line:
            print(f"🔍 LCD DEBUG: {line.strip()}")
        
        # Temperature and PWM debug messages - UPDATE GLOBALS for accurate status bar display
        elif "$DEBUG,TEMPS," in line:
            try:
                # Parse temperature debug message: $DEBUG,TEMPS,Media:XX.XX,Heater:XX.XX,Ambient:XX.XX,Setpoint:XX.XX,PWM:XXX,RawMedia:XXXX,RawHeater:XXXX
                parts = line.split("TEMPS,")[1].strip()
                temp_parts = parts.split(',')
                
                for part in temp_parts:
                    if part.startswith("PWM:"):
                        pwm_float = float(part.split(":")[1])
                        current_heater_pwm = int(round(pwm_float))
                    elif part.startswith("Media:"):
                        current_media_temp = float(part.split(":")[1])  # Update global for status bar
                    elif part.startswith("Heater:"):
                        current_heater_temp = float(part.split(":")[1])  # Update global for status bar
                    elif part.startswith("Ambient:"):
                        current_ambient_temp = float(part.split(":")[1])  # Update global for status bar
                    elif part.startswith("Setpoint:"):
                        setpoint_debug = float(part.split(":")[1])
                    elif part.startswith("RawMedia:"):
                        raw_media = int(part.split(":")[1])
                    elif part.startswith("RawHeater:"):
                        raw_heater = int(part.split(":")[1])
                
            except Exception as e:
                print(f"⚠️ Error parsing temperature debug: {e}")
                print(f"   Raw line: {line.strip()}")
        
        # Max dispensations debug messages
        elif "$DEBUG,MAX_DISPENSATIONS_SET," in line:
            try:
                max_disp = int(line.split("MAX_DISPENSATIONS_SET,")[1].strip())
                print(f"✅ MAX DISPENSATIONS SET: {max_disp}")
                ui.notify(f'✅ Max dispensations set to {max_disp}', type='positive')
            except Exception as e:
                print(f"⚠️ Error parsing max dispensations set: {e}")
        
        elif "$DEBUG,MAX_DISPENSATIONS_INVALID," in line:
            try:
                invalid_value = int(line.split("MAX_DISPENSATIONS_INVALID,")[1].strip())
                print(f"❌ MAX DISPENSATIONS INVALID: {invalid_value} (must be 10-500)")
                ui.notify(f'❌ Invalid max dispensations: {invalid_value} (must be 10-500)', type='negative')
            except Exception as e:
                print(f"⚠️ Error parsing max dispensations invalid: {e}")
        
        # Media type update debug messages
        elif "$DEBUG,MEDIA_TYPE_UPDATED," in line:
            try:
                parts = line.split("MEDIA_TYPE_UPDATED,")[1].strip()
                print(f"🔄 MEDIA TYPE UPDATED: {parts}")
            except Exception as e:
                print(f"⚠️ Error parsing media type update: {e}")
        
        elif "$DEBUG,MEDIA_TYPE_UPDATED_JUMP," in line:
            try:
                parts = line.split("MEDIA_TYPE_UPDATED_JUMP,")[1].strip()
                print(f"🔄 MEDIA TYPE UPDATED (JUMP): {parts}")
            except Exception as e:
                print(f"⚠️ Error parsing media type update jump: {e}")
        
        # Media filling debug messages
        elif "$DEBUG,FILLING_WITH_POSITIVE_MEDIA" in line:
            print("✅ FILLING WITH POSITIVE MEDIA")
            ui.notify('✅ Filling with POSITIVE media', type='positive')
        
        elif "$DEBUG,FILLING_WITH_NEGATIVE_MEDIA" in line:
            print("✅ FILLING WITH NEGATIVE MEDIA")
            ui.notify('✅ Filling with NEGATIVE media', type='positive')
        
        elif "$DEBUG,FILLING_WITH_NEUTRAL_MEDIA" in line:
            print("✅ FILLING WITH NEUTRAL MEDIA")
            ui.notify('✅ Filling with NEUTRAL media', type='positive')
        
        elif "$DEBUG,CALIBRATION_POINTS_SET," in line:
            try:
                parts = line.split("CALIBRATION_POINTS_SET,")[1].strip()
                point_parts = parts.split(',')
                if len(point_parts) >= 4:
                    ir_zero_set = int(point_parts[0])
                    od_zero_set = float(point_parts[1])
                    ir_target_set = int(point_parts[2])
                    od_target_set = float(point_parts[3])
                    print(f"✅ CALIBRATION POINTS SET: irZero={ir_zero_set}, odZero={od_zero_set:.1f}, irTarget={ir_target_set}, odTarget={od_target_set:.1f}")
            except Exception as e:
                print(f"⚠️ Error parsing calibration points set: {e}")
        
        elif "$DEBUG,MEDIA_CHANGE_PUMP_SELECTION," in line:
            try:
                parts = line.split("MEDIA_CHANGE_PUMP_SELECTION,")[1].strip()
                print(f"🔍 PUMP SELECTION: {parts}")
            except Exception as e:
                print(f"⚠️ Error parsing pump selection: {e}")
        
        elif "$DEBUG,FILLING_WITH_NEUTRAL_MEDIA,PUMP_ONE" in line:
            print("🔍 PUMP SELECTION: NEUTRAL → PUMP_ONE")
        
        elif "$DEBUG,FILLING_WITH_POSITIVE_MEDIA,PUMP_TWO" in line:
            print("🔍 PUMP SELECTION: POSITIVE → PUMP_TWO")
        
        elif "$DEBUG,FILLING_WITH_NEGATIVE_MEDIA,PUMP_THREE" in line:
            print("🔍 PUMP SELECTION: NEGATIVE → PUMP_THREE")
        
        elif "$DEBUG,FILLING_WITH_DEFAULT_NEUTRAL_MEDIA,PUMP_ONE" in line:
            print("🔍 PUMP SELECTION: DEFAULT → PUMP_ONE (NEUTRAL)")
        
        elif "$DEBUG,APPLY_STEP_SETTINGS," in line:
            try:
                parts = line.split("APPLY_STEP_SETTINGS,")[1].strip()
                print(f"🔍 STEP SETTINGS: {parts}")
            except Exception as e:
                print(f"⚠️ Error parsing step settings: {e}")
        
        elif "$DEBUG,APPLY_STEP_SETTINGS_ERROR," in line:
            try:
                parts = line.split("APPLY_STEP_SETTINGS_ERROR,")[1].strip()
                print(f"❌ STEP SETTINGS ERROR: {parts}")
            except Exception as e:
                print(f"⚠️ Error parsing step settings error: {e}")
        
        # General debug message catch-all for any other debug messages
        elif "$DEBUG," in line and "TEMPS" not in line and "WATCHDOG" not in line and "EXPERIMENT" not in line and "LCD" not in line and "MAX_DISPENSATIONS" not in line and "MEDIA_TYPE" not in line and "FILLING_WITH" not in line:
            pass  # Silent - verbose debug removed
        
        # Watchdog status messages - Enhanced with detailed information
        elif "$DEBUG,WATCHDOG_STATUS," in line:
            try:
                # Parse the enhanced watchdog status message
                parts = line.split("WATCHDOG_STATUS,")[1].strip()
                status_parts = parts.split(',')
                watchdog_status = status_parts[0]
                
                print(f"🔍 WATCHDOG: Status = {watchdog_status}")
                
                if watchdog_status == "ACTIVE":
                    # Parse detailed information for watchdog trigger
                    od_drop = "N/A"
                    required = "N/A"
                    pre_od = "N/A"
                    current_od = "N/A"
                    
                    for part in status_parts[1:]:
                        if part.startswith("OD_DROP:"):
                            od_drop = part.split(":")[1]
                        elif part.startswith("REQUIRED:"):
                            required = part.split(":")[1]
                        elif part.startswith("PRE_OD:"):
                            pre_od = part.split(":")[1]
                        elif part.startswith("CURRENT_OD:"):
                            current_od = part.split(":")[1]
                    
                    # Red banner removed - LCD display is sufficient for watchdog alerts
                    # ui.notify(f'⚠️ WATCHDOG TRIGGERED! OD drop insufficient (Drop: {od_drop}, Required: {required})', type='negative')
                    print(f"🔍 WATCHDOG DETAILS: Pre-OD: {pre_od}, Current OD: {current_od}, Drop: {od_drop}, Required: {required}")
                    
                elif watchdog_status == "INACTIVE":
                    # Parse detailed information for watchdog clear
                    od_drop = "N/A"
                    required = "N/A"
                    
                    for part in status_parts[1:]:
                        if part.startswith("OD_DROP:"):
                            od_drop = part.split(":")[1]
                        elif part.startswith("REQUIRED:"):
                            required = part.split(":")[1]
                    
                    # Only show the toast for a REAL auto-clear (OD data present).
                    # The bare WATCHDOG_STATUS,INACTIVE sent on PAUSE has no OD_DROP/
                    # REQUIRED fields, which previously produced a misleading
                    # "OD drop sufficient (Drop: N/A, Required: N/A)" message.
                    if od_drop != "N/A" and required != "N/A":
                        ui.notify(f'✅ Watchdog cleared - OD drop sufficient (Drop: {od_drop}, Required: {required})', type='positive')
                    print(f"🔍 WATCHDOG CLEARED: Drop: {od_drop}, Required: {required}")
                    
            except Exception as e:
                print(f"⚠️ Error parsing watchdog status: {e}")
                # Fallback to simple parsing
                watchdog_status = line.split("WATCHDOG_STATUS,")[1].strip().split(',')[0]
                if watchdog_status == "ACTIVE":
                    # Red banner removed - LCD display is sufficient
                    # ui.notify('⚠️ WATCHDOG TRIGGERED! OD drop insufficient after media change', type='negative')
                    pass
                elif watchdog_status == "INACTIVE":
                    ui.notify('✅ Watchdog cleared - OD drop sufficient', type='positive')
            
        # Legacy format support (old debug messages)
        elif ": Experiment paused" in line and "$DEBUG," in line:
            # Extract button name (UP, SEL, or DOWN)
            button_name = "Physical"
            if "$DEBUG,UP:" in line:
                button_name = "UP"
            elif "$DEBUG,SEL:" in line:
                button_name = "SEL"
            elif "$DEBUG,DOWN:" in line:
                button_name = "DOWN"
            
            print(f"🔍 Physical button {button_name} PAUSE detected!")
            is_paused = True  # Keep is_running=True, set paused flag
            # Update button to show Resume
            if start_button is not None:
                start_button.text = 'Resume'
                start_button.props('color=positive')
                start_button.props('icon=play_arrow')
                start_button.update()
            ui.notify(f'⏸️ Experiment paused via {button_name} button', type='info')
            
        elif ": Experiment resumed" in line and "$DEBUG," in line:
            # Extract button name (UP, SEL, or DOWN)
            button_name = "Physical"
            if "$DEBUG,UP:" in line:
                button_name = "UP"
            elif "$DEBUG,SEL:" in line:
                button_name = "SEL"
            elif "$DEBUG,DOWN:" in line:
                button_name = "DOWN"
                
            print(f"🔍 Physical button {button_name} RESUME detected!")
            is_paused = False  # Update paused state
            # Update button to show Pause
            if start_button is not None:
                start_button.text = 'Pause'
                start_button.props('color=warning')
                start_button.props('icon=pause')
                start_button.update()
            ui.notify(f'▶️ Experiment resumed via {button_name} button', type='positive')
        
        # Note: PEAK_DETECTED from Arduino is deprecated - now using Dilution_Event flag from data stream
                
    for line in data_lines:
        if line.startswith('@'):
            try:
                parts = line[1:].split(',')
                # New 11-field format: unixTime,upTime,OD940,infraredReading,ambientTemp,mediaTemp,heaterPlateTemp,totalCycleCount,currentCycle,currentStep,mediaType
                if len(parts) < 11: 
                    print(f"❌ Invalid data format ({len(parts)} < 11 parts), skipping")
                    continue

                # Save the PREVIOUS media type before updating (for peak coloring)
                previous_media_type = current_media_type

                # Parse data - NEW 11-FIELD FORMAT
                unix_time = int(parts[0])     # unixTime
                time_ms = int(parts[1])       # upTime
                od = float(parts[2])          # OD940
                current_od = od               # Store for real-time status bar (not downsampled!)
                current_ir = int(parts[3])    # infraredReading
                current_ambient_temp = float(parts[4])  # ambientTemp
                current_media_temp = float(parts[5])  # mediaTemp
                current_heater_temp = float(parts[6])  # heaterPlateTemp
                # parts[7] is totalCycleCount (we use current_cycle)
                current_cycle = int(parts[8])         # currentCycle
                current_step = int(parts[9])          # currentStep
                # mediaType - default to NEUTRAL if missing or empty
                media_type_raw = parts[10].strip() if len(parts) > 10 else ""
                current_media_type = media_type_raw if media_type_raw else "NEUTRAL"
                
                # current_led_channel is set from $STATE (currentLEDPin) - do not recalculate here
                
                # Dilution_Event (index 11) - V3+
                is_dilution_event = False
                if len(parts) > 11:
                    try:
                        dilution_flag = parts[11].strip()
                        is_dilution_event = (dilution_flag == '1' or dilution_flag.lower() == 'true')
                        if is_dilution_event:
                            print(f"🔍 DEBUG: Dilution_Event flag detected! parts[11]='{dilution_flag}', len(parts)={len(parts)}")
                    except Exception as e:
                        print(f"⚠️ Error parsing Dilution_Event: {e}")
                else:
                    # Debug: show why we're not checking
                    if len(parts) == 11:
                        print(f"🔍 DEBUG: Data line has only {len(parts)} parts, need >11 for Dilution_Event. Last part: '{parts[10]}'")


                # Update data lists for plotting
                current_time_hours = time_ms / 3600000.0
                
                # Handle Dilution Event from Data Stream (Backup for PEAK_DETECTED)
                if is_dilution_event:
                    # Add peak if not already recorded (deduplicate by time)
                    # Check if we have a recent peak (within 0.0033 hours = 12 seconds) to avoid duplicates
                    # The Dilution_Event flag stays high for ~10 seconds until SD write
                    has_recent_peak = False
                    for pt in peak_times:
                        if abs(pt - current_time_hours) < 0.0033: 
                            has_recent_peak = True
                            break
                    
                    if not has_recent_peak:
                        # Use previous_media_type (saved at line 1886 before parsing this line)
                        # current_media_type is already the NEW media (after change)
                        print(f"🔔 Dilution Event at {current_time_hours:.4f}h | GREW IN: {previous_media_type} | CHANGED TO: {current_media_type} | PEAK COLOR: {previous_media_type}")
                        peak_times.append(current_time_hours)
                        peak_media_types.append(previous_media_type)
                        
                        # Sort and manage peaks
                        paired = list(zip(peak_times, peak_media_types))
                        paired.sort(key=lambda x: x[0])
                        peak_times[:] = [p[0] for p in paired]
                        peak_media_types[:] = [p[1] for p in paired]
                        
                        # Keep only last 50 peaks
                        MAX_PEAKS = 50
                        if len(peak_times) > MAX_PEAKS:
                            peak_times[:] = peak_times[-MAX_PEAKS:]
                            peak_media_types[:] = peak_media_types[-MAX_PEAKS:]
                        
                        # Recalculate intervals
                        peak_intervals.clear()
                        if len(peak_times) >= 2:
                            for i in range(1, len(peak_times)):
                                interval = peak_times[i] - peak_times[i-1]
                                if interval > 0:
                                    peak_intervals.append(interval)
                        
                        # Update chart
                        if peak_chart:
                             # Trigger update logic (via new_data_point or direct call)
                             pass

                # Write to CSV if experiment is running AND not paused
                if is_running and not is_paused and csv_writer:
                    try:
                        # Write data in the full 14-field format, matching the SD card:
                        # unixTime,upTime,OD940,infraredReading,ambientTemp,mediaTemp,heaterPlateTemp,totalCycleCount,currentCycle,currentStep,mediaType,Dilution_Event,LED_Channel,LED_Percent
                        # LED_Percent mirrors the firmware: PWM% × 0.885 (effective dose,
                        # accounting for the LED being off during OD measurements).
                        led_percent_log = int((current_led_brightness / 4095.0) * 100)
                        effective_led_percent_log = int(led_percent_log * 0.885)
                        csv_writer.writerow([
                            unix_time,  # unixTime
                            time_ms,    # upTime
                            od,         # OD940
                            current_ir, # infraredReading
                            parts[4],   # ambientTemp
                            parts[5],   # mediaTemp
                            parts[6],   # heaterPlateTemp
                            parts[7],   # totalCycleCount
                            parts[8],   # currentCycle
                            parts[9],   # currentStep
                            parts[10],  # mediaType
                            1 if is_dilution_event else 0, # Dilution_Event
                            current_led_channel,           # LED_Channel (active LED, from $STATE)
                            effective_led_percent_log      # LED_Percent (effective dose)
                        ])
                        csv_file.flush()  # Ensure data is written immediately
                    except Exception as e:
                        print(f"⚠️ CSV write error: {e}")

                # Add to data arrays - only keep every 5th point to save memory
                # SD card saves every 10s, Python displays every 10s - same resolution!
                # Skip data points during config updates to avoid plotting with inconsistent calibration
                # Also skip during pause to avoid misleading flat lines in the plot
                if not config_updating and not is_paused:
                    # Downsample: keep every 5th data point (10 seconds at 2-sec rate)
                    # This creates a 5-point moving average effect
                    if not hasattr(update_data, 'downsample_counter'):
                        update_data.downsample_counter = 0
                    update_data.downsample_counter += 1
                    
                    if update_data.downsample_counter >= 5:
                        time_data.append(current_time_hours)
                        od_data.append(od)
                        ir_data.append(current_ir)
                        unix_time_data.append(unix_time)
                        media_data.append(current_media_type)  # Track media type for peak coloring
                        print(f"📊 Point added: {current_time_hours:.1f}h, OD={od:.3f}, NIR={current_ir}, Cycle={current_cycle}, Step={current_step}")
                        update_data.downsample_counter = 0
                        new_data_point = True  # Signal that we added a point (will trigger chart update)
                    else:
                        # Don't print every skip - too much console spam
                        new_data_point = False  # No new data added, don't update chart
                else:
                    new_data_point = False  # Skipping this point, don't update chart
                    if config_updating:
                        print(f"⏸️ Skipping data point during config update (would have invalid calibration)")
                    elif is_paused:
                        print(f"⏸️ Skipping data point during pause (experiment paused) - is_paused={is_paused}, config_updating={config_updating}")
                
                # ADAPTIVE DATA RETENTION for month-long runs:
                # Keep more points early in run, fewer as time goes on
                # This keeps recent data detailed while compressing old data
                
                # For first 6 hours: keep all points (21,600 at 1/sec)
                # For 6-24 hours: keep every 2nd point (~27,000 total)
                # After 24 hours: keep every 5th point (~35,000 total for 7 days)
                # After 7 days: keep every 10th point (~44,000 total for 30 days)
                
                # Keep every 5th data point - reduces memory by 80%
                # At 2-sec rate with 5x downsample = 1 point per 10 seconds
                # 288 hours = 518,400 points → 103,680 points (~450-650 MB)
                # Matches SD card resolution (10-second averaged data)
                MAX_DATA_POINTS = 103680  # 288 hours (12 days) at 10-second intervals
                
                if len(time_data) > MAX_DATA_POINTS:
                    # Simple trim: remove oldest data
                    points_to_remove = len(time_data) - MAX_DATA_POINTS
                    oldest_time_removed = time_data[points_to_remove - 1]
                    
                    del time_data[:points_to_remove]
                    del od_data[:points_to_remove]
                    del ir_data[:points_to_remove]
                    del unix_time_data[:points_to_remove]
                    del media_data[:points_to_remove]
                    
                    # CRITICAL: Remove peaks from trimmed time range (keep media types synchronized)
                    paired_peaks = [(t, m) for t, m in zip(peak_times, peak_media_types) if t > oldest_time_removed]
                    peak_times[:] = [p[0] for p in paired_peaks]
                    peak_media_types[:] = [p[1] for p in paired_peaks]
                    
                    # Recalculate intervals after trimming peaks
                    peak_intervals.clear()
                    if len(peak_times) >= 2:
                        for i in range(1, len(peak_times)):
                            interval = peak_times[i] - peak_times[i-1]
                            if interval > 0:
                                peak_intervals.append(interval)
                    
                    print(f"🗜️ Trimmed {points_to_remove} oldest points, {len(peak_times)} peaks remain (~{MAX_DATA_POINTS*10/3600:.1f} hours at 10-sec intervals)")

                # SIMPLE & RELIABLE: Threshold crossing detection removed
                # We now rely exclusively on the firmware's Dilution_Event flag (parsed above)
                # to prevent phantom peaks and list synchronization issues.
                pass

            except Exception as e:
                print(f"❌ Data parsing error: {e}")
                print(f"❌ Line that caused error: {line.strip()}")
                print(f"❌ Parts: {parts if 'parts' in locals() else 'No parts available'}")
                import traceback
                traceback.print_exc()

        # Monitor SD card status
        elif line.startswith('$SD_STATUS'):
            print(f"🔍 Received SD status: {line}")
            try:
                # Format: $SD_STATUS,status,free_space_mb
                parts = line.split(',')
                print(f"🔍 SD status parts: {parts}")
                if len(parts) >= 3:
                    sd_card_status = parts[1]
                    sd_card_free_space = int(parts[2])
                    print(f"🔍 Parsed SD status: {sd_card_status}, {sd_card_free_space}MB")
                    
                    # Update SD card status display
                    print(f"🔍 Attempting to update SD label. Label exists: {sd_card_status_label is not None}")
                    if sd_card_status_label is not None:
                        try:
                            if sd_card_status.upper() == "OK":
                                sd_card_status_label.text = f'💾 SD: OK ({sd_card_free_space}MB)'
                                sd_card_status_label.style('color: green; font-weight: bold;')
                            elif sd_card_status.upper() in ["NOT_FOUND", "NOT_INSERTED"]:
                                sd_card_status_label.text = '💾 SD: NOT INSERTED'
                                sd_card_status_label.style('color: red; font-weight: bold;')
                                ui.notify('⚠️ No SD card detected! Please insert SD card for data logging.', type='warning')
                            else:
                                sd_card_status_label.text = f'💾 SD: {sd_card_status}'
                                sd_card_status_label.style('color: red; font-weight: bold;')
                                ui.notify(f'⚠️ SD Card Issue: {sd_card_status}', type='warning')
                            print(f"✅ SD status updated to: {sd_card_status_label.text}")
                        except Exception as e:
                            print(f"❌ Error updating SD status label: {e}")
                    else:
                        print(f"⚠️ SD status label not available yet. Current label: {sd_card_status_label}")
                    
                    print(f"SD Card Status: {sd_card_status}, Free Space: {sd_card_free_space}MB")
            except Exception as e:
                print(f"SD status parsing error: {e}")

        # Handle SD card errors/warnings
        elif line.startswith('$SD_ERROR'):
            ui.notify(f'⚠️ SD Card Error: {line[10:]}', type='negative')
            try:
                sd_card_status_label.text = '💾 SD: ERROR'
                sd_card_status_label.classes('text-weight-bold text-negative')
            except NameError:
                pass
            print(f"SD Card Error: {line}")
        
        # Handle SD card not found/missing
        elif line.startswith('$DEBUG,SD card not found'):
            sd_card_status = "NOT_FOUND"
            if sd_card_status_label is not None:
                try:
                    sd_card_status_label.text = '💾 SD: NOT INSERTED'
                    sd_card_status_label.classes('text-weight-bold text-negative')
                    print(f"✅ SD status updated to: NOT INSERTED")
                except Exception as e:
                    print(f"❌ Error updating SD status label: {e}")
            ui.notify('⚠️ No SD card detected in OpenEvo! Please insert SD card for data logging.', type='warning')
            print("SD Card: Not found")
        
        # Handle SD card successful detection
        elif line.startswith('$DEBUG,SD card initialization successful'):
            sd_card_status = "OK"
            if sd_card_status_label is not None:
                try:
                    sd_card_status_label.text = '💾 SD: OK'
                    sd_card_status_label.classes('text-weight-bold text-positive')
                    print(f"✅ SD status updated to: OK")
                except Exception as e:
                    print(f"❌ Error updating SD status label: {e}")
            ui.notify('✅ SD card detected and ready for data logging!', type='positive')
            print("SD Card: Successfully detected")

        elif line.startswith('$SD_WARNING'):
            ui.notify(f'⚠️ SD Card Warning: {line[12:]}', type='warning')
            print(f"SD Card Warning: {line}")

        # Handle SD card removed/lost during operation
        elif '$DEBUG,SD_LOST' in line or '$DEBUG,SD_CHECK,CARD_MISSING' in line:
            sd_card_status = "MISSING"
            if sd_card_status_label is not None:
                try:
                    sd_card_status_label.text = '💾 SD: CARD REMOVED!'
                    sd_card_status_label.style('color: red; font-weight: bold;')
                except:
                    pass
            ui.notify('❌ SD CARD REMOVED! Experiment auto-paused. Insert card and unpause to resume.', type='negative')
            print("❌ SD Card removed during operation")

        # Handle SD card recovery/insertion
        elif '$DEBUG,SD_RECOVERED' in line:
            sd_card_status = "OK"
            if sd_card_status_label is not None:
                try:
                    sd_card_status_label.text = '💾 SD: OK (Recovered)'
                    sd_card_status_label.style('color: green; font-weight: bold;')
                except:
                    pass
            ui.notify('✅ SD Card detected! Unpause to resume experiment.', type='positive')
            print("✅ SD Card recovered")

        # Handle unpause failure due to missing SD card
        elif '$ERROR,UNPAUSE_FAILED_NO_SD' in line:
            ui.notify('❌ Cannot unpause - SD card not detected! Please insert SD card first.', type='negative')
            print("❌ Unpause failed - No SD card")

        # Handle pump status messages
        elif line.startswith('$PUMP_STATUS'):
            try:
                # Format: $PUMP_STATUS,action,status[,volume]
                parts = line.split(',')
                if len(parts) >= 3:
                    action = parts[1]  # EMPTYING, FILLING, EMPTY_COMPLETE, FILL_COMPLETE
                    status = parts[2]  # STARTED, volume
                    
                    if action == "EMPTYING" and status == "STARTED":
                        ui.notify('🔽 Emptying reactor...', type='info')
                    elif action == "FILLING" and status == "STARTED":
                        ui.notify('🔼 Filling reactor...', type='info')
                    elif action == "EMPTY_COMPLETE":
                        volume = parts[2] if len(parts) > 2 else "Unknown"
                        ui.notify(f'✅ Emptying complete: {volume}mL', type='positive')
                    elif action == "FILL_COMPLETE":
                        volume = parts[2] if len(parts) > 2 else "Unknown"
                        ui.notify(f'✅ Filling complete: {volume}mL', type='positive')
            except Exception as e:
                print(f"Pump status parsing error: {e}")

        # Handle current state response for experiment detection
        elif line.startswith('$CURRENT_STATE'):
            try:
                # Format: $CURRENT_STATE,cycle,step
                parts = line.split(',')
                if len(parts) >= 3:
                    cycle = int(parts[1])
                    step = int(parts[2])
                    
                    # Update global state variables for startup dialog
                    current_state_cycle = cycle
                    current_state_step = step
                    
                    print(f"🔍 Current state received: Cycle {cycle}, Step {step}")
                    
                    # If this is a previous experiment (cycle > 1 or step > 1), show notification
                    if cycle > 1 or step > 1:
                        ui.notify(f'📋 Previous experiment detected: Cycle {cycle}, Step {step}', type='info')
            except Exception as e:
                print(f"Current state parsing error: {e}")

    if new_data_point:
        # Update charts when new data point is added (every 10 seconds)
        if od_chart:
            update_ui_elements()
        else:
            print("⚠️ Charts not initialized yet")
    
    # Connection status is now updated at the START of update_data() based on actual data reception
    # This prevents the issue where status showed DISCONNECTED even when data was flowing
    
    # Periodic SD status check (every 30 seconds)
    # NOTE: This is done AFTER status update so failures don't incorrectly show DISCONNECTED
    if not hasattr(update_data, 'sd_counter'):
        update_data.sd_counter = 0
    
    update_data.sd_counter += 1
    if update_data.sd_counter >= 30:  # 30 seconds at 1-second intervals
        print("📤 Periodic SD card status check...")
        arduino.send_command("CHECK_SD_STATUS")
        update_data.sd_counter = 0

def update_ui_elements():
    """Update all dynamic UI elements at once."""
    # Update Status Bar
    try:
        if status_labels:
            status_labels['cycle'].text = f'Cycle {current_cycle}'
            status_labels['step'].text = f'Step {current_step}/{total_program_steps}'
            status_labels['media'].text = f'{current_media_type}'
            # Use current_od (updated every data point) instead of od_data[-1] (downsampled)
            status_labels['od'].text = f'OD {current_od:.3f}' if isinstance(current_od, (int, float)) else 'OD --'
            status_labels['ir'].text = f'IR {current_ir}'
            status_labels['temp'].text = f'Media {current_media_temp:.1f}°C'
            status_labels['ambient'].text = f'Ambient {current_ambient_temp:.1f}°C'
            status_labels['heater'].text = f'Heater {current_heater_temp:.1f}°C'
            status_labels['pwm'].text = f'Heater PWM {current_heater_pwm}'
            
            # Calculate LED percentage - use expected value from program if Arduino brightness is 0
            led_percent_arduino = (current_led_brightness / 4095.0) * 100
            
            # Get expected LED brightness from current program step
            expected_led_percent = 0
            try:
                if working_program and 'steps' in working_program and current_step > 0 and current_step <= len(working_program['steps']):
                    expected_led_percent = working_program['steps'][current_step - 1].get('led_brightness', 0)
            except:
                pass
            
            # Use Arduino value if available, otherwise show expected value
            display_led_percent = led_percent_arduino if led_percent_arduino > 0 else expected_led_percent
            
            # Show LED status with channel number and intensity
            # Get calibration data for current LED channel
            cal_data = led_calibration.get(current_led_channel, {})
            cal_intensity = cal_data.get('intensity', 0)  # <-- mW/cm² (calibrated max intensity at 100% PWM)
            cal_wavelength = cal_data.get('wavelength', '')  # <-- nm (wavelength)
            
            if display_led_percent > 0:
                # Calculate actual irradiance at current PWM (before OD measurement duty cycle)
                pwm_irradiance = cal_intensity * (display_led_percent / 100.0) if cal_intensity > 0 else 0
                
                # Build LED status string
                if cal_intensity > 0 and cal_wavelength:
                    status_labels['led'].text = f'LED {current_led_channel} ({cal_wavelength}nm): {display_led_percent:.0f}% ({pwm_irradiance:.2f} mW/cm²)'
                elif cal_intensity > 0:
                    status_labels['led'].text = f'LED {current_led_channel}: {display_led_percent:.0f}% ({pwm_irradiance:.2f} mW/cm²)'
                else:
                    status_labels['led'].text = f'LED {current_led_channel}: {display_led_percent:.0f}%'
            else:
                status_labels['led'].text = f'LED OFF'
            
            # Calculate effective light dose (accounts for 88.5% duty cycle during OD readings)
            if display_led_percent > 0:
                effective_dose = display_led_percent * OD_MEASUREMENT_DUTY_CYCLE
                if cal_intensity > 0:
                    actual_irradiance = cal_intensity * (effective_dose / 100.0)
                    status_labels['light_dose'].text = f'Effective Dose: {effective_dose:.1f}% ({actual_irradiance:.2f} mW/cm²)'
                else:
                    status_labels['light_dose'].text = f'Effective Dose: {effective_dose:.1f}%'
            else:
                status_labels['light_dose'].text = ''

    except Exception as e:
        print(f"Status update error: {e}")

    # Update Charts
    try:
        if od_chart and time_data and od_data:
            # --- Media Area Coloring Logic (from reference) ---
            global last_media_type, last_media_start_time, media_areas
            if not time_data or last_media_start_time == 0:
                last_media_start_time = time_data[-1] if time_data else 0
                last_media_type = current_media_type

            if current_media_type != last_media_type:
                if last_media_type in ['POSITIVE', 'NEGATIVE']:
                    color = '#bbdefb' if last_media_type == 'POSITIVE' else '#ffcdd2'
                    media_areas.append([
                        {'xAxis': last_media_start_time, 'itemStyle': {'color': color, 'opacity': 0.3}},
                        {'xAxis': time_data[-1]}
                    ])

                last_media_type = current_media_type
                last_media_start_time = time_data[-1]

            current_areas = media_areas.copy()
            if current_media_type in ['POSITIVE', 'NEGATIVE']:
                color = '#bbdefb' if current_media_type == 'POSITIVE' else '#ffcdd2'
                current_areas.append([
                    {'xAxis': last_media_start_time, 'itemStyle': {'color': color, 'opacity': 0.3}},
                    {'xAxis': time_data[-1]}
                ])

            # --- Chart Update - data already downsampled at collection (every 5th point) ---
            # Reduced logging for performance
            
            od_chart_data = []
            ir_chart_data = []
            time_chart_data = []
            unix_chart_data = []
            
            # Plot all stored points (already downsampled at collection)
            # Ensure all arrays are same length (they should be, but verify)
            min_len = min(len(time_data), len(od_data), len(ir_data), len(unix_time_data))
            if min_len < len(time_data):
                print(f"⚠️ Data length mismatch! time:{len(time_data)}, od:{len(od_data)}, ir:{len(ir_data)}, unix:{len(unix_time_data)}")
            
            for i in range(min_len):
                t = time_data[i]
                od_chart_data.append([t, od_data[i]])
                ir_chart_data.append([t, ir_data[i]])
                
                # Time & Date data (invisible) - convert decimal hours to readable date/time
                start_time = datetime.now() - timedelta(hours=time_data[-1])
                current_time = start_time + timedelta(hours=t)
                formatted_time = current_time.strftime("%Y-%m-%d %H:%M:%S")
                time_chart_data.append([t, formatted_time])
                
                # Unix time data (invisible)
                unix_chart_data.append([t, unix_time_data[i]])
            
            # Downsample to exactly 5000 points for chart performance
            if len(od_chart_data) > 5000:
                od_chart_data = downsample_to_n(od_chart_data, 5000)
                ir_chart_data = downsample_to_n(ir_chart_data, 5000)
                time_chart_data = downsample_to_n(time_chart_data, 5000)
                unix_chart_data = downsample_to_n(unix_chart_data, 5000)
                print(f"📊 Downsampled {len(time_data)} points → 5000 points for chart rendering")
            
            # Update all series data
            od_chart.options['series'][0]['data'] = od_chart_data      # OD (visible, left y-axis)
            od_chart.options['series'][1]['data'] = ir_chart_data      # NIR (visible, right y-axis)
            od_chart.options['series'][2]['data'] = time_chart_data    # Time (invisible)
            od_chart.options['series'][3]['data'] = unix_chart_data    # Unix Time (invisible)
            od_chart.options['series'][0]['markArea']['data'] = current_areas
            
            # Update threshold series data (now index 4)
            od_chart.options['series'][4]['data'] = [
                [time_data[0] if time_data else 0, config_data['threshold']],
                [time_data[-1] if time_data else 1, config_data['threshold']]
            ]
            
            # Debug: Check IR data being sent to chart
            if ir_chart_data and len(ir_chart_data) > 0:
                print(f"📊 Chart update: {len(ir_chart_data)} IR points, last IR={ir_chart_data[-1][1]}, ir_data len={len(ir_data)}")
            
            od_chart.update()
        elif not od_chart and time_data and od_data:
            print(f"⚠️ Chart not initialized yet, but data loaded: {len(time_data)} points")
            print(f"📊 Time range: {time_data[0]:.4f}h to {time_data[-1]:.4f}h")
            print(f"📊 OD range: {min(od_data):.4f} to {max(od_data):.4f}")

        if peak_chart and peak_intervals:
            # Build single continuous dataset with media type info
            all_peak_data = []
            media_types_for_peaks = []
            
            for i, interval in enumerate(peak_intervals):
                # Skip truly problematic intervals
                if interval <= 0:
                    continue
                    
                # X-axis is peak number (i+1 = first peak to second peak interval)
                peak_number = i + 1
                
                # peak_intervals[i] is the interval between peak[i] and peak[i+1]
                # The cells grew in peak_media_types[i+1] during this interval
                # (the media they were in when they reached peak i+1)
                media_type_at_peak = "NEUTRAL"  # Default
                
                if i+1 < len(peak_media_types):
                    media_type_at_peak = peak_media_types[i+1]
                
                # Get timestamp for this peak (peak_times[i+1] is when interval ended)
                peak_timestamp = ""
                peak_time_hrs = 0
                if i+1 < len(peak_times) and unix_time_data:
                    peak_time_hrs = peak_times[i+1]
                    # Calculate absolute timestamp from peak time in hours
                    start_unix = unix_time_data[0] if unix_time_data else 0
                    peak_unix = start_unix + peak_times[i+1] * 3600
                    try:
                        peak_timestamp = datetime.utcfromtimestamp(peak_unix).strftime('%Y-%m-%d %H:%M:%S UTC')
                    except (ValueError, OSError, OverflowError):
                        peak_timestamp = f"{peak_times[i+1]:.2f}h"
                
                # Add to single dataset with timestamp
                all_peak_data.append({
                    'value': [peak_number, interval], 
                    'media': media_type_at_peak,
                    'timestamp': peak_timestamp,
                    'peak_time_hours': peak_time_hrs
                })
                media_types_for_peaks.append(media_type_at_peak)
            
            # Build properly formatted chart data with colors
            chart_data = []
            for point_data in all_peak_data:
                media = point_data['media']
                if media == "POSITIVE":
                    color = '#2196F3'  # Blue
                elif media == "NEGATIVE":
                    color = '#f44336'  # Red
                else:
                    color = '#000000'  # Black for NEUTRAL
                
                timestamp = point_data.get('timestamp', '')
                peak_hrs = point_data.get('peak_time_hours', 0)
                
                # Add point with value, itemStyle, and extra data for JavaScript tooltip formatter
                chart_data.append({
                    'value': point_data['value'],
                    'media': media,
                    'peakTimeHours': peak_hrs,
                    'timestamp': timestamp,
                    'itemStyle': {'color': color, 'borderWidth': 2, 'borderColor': '#fff'}
                })
            
            # Update chart with SINGLE continuous series, colored markers
            # Also set up tooltip with label formatter for HTML rendering
            peak_chart.options['series'] = [
                {
                    'name': 'Peak Intervals',
                    'type': 'line',
                    'data': chart_data,
                    'lineStyle': {'color': '#333333', 'width': 2.5, 'type': 'solid'},  # Continuous dark gray line
                    'symbol': 'circle',
                    'symbolSize': 12,
                    'showSymbol': True,
                    'connectNulls': True,  # Ensure line is continuous
                    'tooltip': {
                        'trigger': 'item'
                    },
                    'label': {
                        'show': False
                    }
                }
            ]
            
            # Update tooltip to use custom formatter via JavaScript
            # NiceGUI echart accepts :formatter for JavaScript functions
            peak_chart.options['tooltip'] = {
                'trigger': 'item',
                'backgroundColor': 'rgba(255, 255, 255, 0.95)',
                'borderColor': '#ccc', 
                'borderWidth': 1,
                'padding': [10, 15],
                'textStyle': {'color': '#333', 'fontSize': 13},
                'extraCssText': 'box-shadow: 0 2px 8px rgba(0,0,0,0.15); border-radius: 6px;',
                ':formatter': '''(params) => {
                    const data = params.data;
                    if (!data || !data.value) return '';
                    
                    const peakNum = data.value[0];
                    const interval = data.value[1];
                    const media = data.media || 'NEUTRAL';
                    const peakTimeHours = data.peakTimeHours || 0;
                    const timestamp = data.timestamp || '';
                    
                    const mediaColor = media === 'POSITIVE' ? '#2196F3' : 
                                      media === 'NEGATIVE' ? '#f44336' : '#000000';
                    
                    let html = '<b>Interval #' + peakNum + '</b><br/>';
                    html += 'Duration: <b>' + interval.toFixed(2) + ' hours</b><br/>';
                    html += '<span style="color: ' + mediaColor + '; font-weight: bold;">Media: ' + media + '</span><br/>';
                    if (peakTimeHours > 0) {
                        html += 'Peak at: <b>' + peakTimeHours.toFixed(2) + 'h</b><br/>';
                    }
                    if (timestamp) {
                        html += '<span style="color: #666; font-size: 11px;">' + timestamp + '</span>';
                    }
                    return html;
                }'''
            }
            
            # Auto-expand axes if data exceeds current range
            if len(peak_intervals) > 0:
                max_peak_num = len(peak_intervals)
                max_interval_value = max(peak_intervals)
                
                current_x_max = peak_chart.options.get('xAxis', {}).get('max', 10)
                current_y_max = peak_chart.options.get('yAxis', {}).get('max', 50)
                
                # Expand x-axis if peaks exceed visible range
                if max_peak_num > current_x_max * 0.9:  # 90% threshold
                    new_x_max = max_peak_num + 5  # Add buffer
                    peak_chart.options['xAxis']['max'] = new_x_max
                    print(f"📈 Auto-expanded peak chart x-axis to {new_x_max}")
                
                # Expand y-axis if interval exceeds visible range
                if max_interval_value > current_y_max * 0.9:  # 90% threshold
                    new_y_max = max_interval_value * 1.2  # Add 20% buffer
                    peak_chart.options['yAxis']['max'] = new_y_max
                    print(f"📈 Auto-expanded peak chart y-axis to {new_y_max:.2f}")
            
            peak_chart.update()
        elif not peak_chart and peak_intervals:
            print(f"⚠️ Peak chart not initialized yet, but {len(peak_intervals)} intervals calculated")
        
        # --- Slope Calculation (from reference) ---
        if od_slope_label and 'od_chart_x_range' in globals():
             od_slope = calculate_visible_slope(time_data, od_data, od_chart_x_range)
             od_slope_label.text = f'Slope: {od_slope:.4f} OD/h'

    except Exception as e:
        print(f"❌ Chart update failed: {e}")

def calculate_visible_slope(x_data, y_data, x_range):
    """Calculate slope for data points within the visible X range (from reference)."""
    if not x_data or not y_data or len(x_data) < 2: return 0.0
    
    visible_points = [(x, y) for x, y in zip(x_data, y_data) if x_range[0] <= x <= x_range[1]]
    if len(visible_points) < 2: return 0.0
    
    x_vals, y_vals = zip(*visible_points)
    n = len(x_vals)
    sum_x, sum_y = sum(x_vals), sum(y_vals)
    sum_xy = sum(x * y for x, y in zip(x_vals, y_vals))
    sum_x2 = sum(x * x for x in x_vals)
    
    denominator = n * sum_x2 - sum_x * sum_x
    return (n * sum_xy - sum_x * sum_y) / denominator if denominator != 0 else 0.0

def reset_od_chart_zoom():
    """Reset OD chart zoom to auto-scaling mode (shows all data and expands as new data arrives)"""
    global od_chart_x_range
    if od_chart:
        try:
            # Clear any user-set zoom by removing min/max constraints
            # This enables auto-scaling that will grow with new data
            od_chart.options['xAxis']['min'] = None
            od_chart.options['xAxis']['max'] = None
            
            # Keep Y-axis at default ranges
            od_chart.options['yAxis'][0]['min'] = 0
            od_chart.options['yAxis'][0]['max'] = 6
            od_chart.options['yAxis'][1]['min'] = 0
            od_chart.options['yAxis'][1]['max'] = 30000
            
            od_chart.update()
            
            # Reset the global x_range to track current data
            if len(time_data) > 0:
                od_chart_x_range = [min(time_data), max(time_data)]
            
            print("🔄 OD chart reset to auto-scaling mode (will expand with new data)")
            ui.notify('Chart reset to auto-scaling mode', type='positive')
        except Exception as e:
            print(f"❌ Failed to reset OD chart zoom: {e}")
            ui.notify(f'Reset failed: {e}', type='negative')

def reset_peak_chart_zoom():
    """Reset Peak chart zoom to auto-scaling mode"""
    if peak_chart:
        try:
            # Clear any user-set zoom constraints for auto-scaling
            peak_chart.options['xAxis']['min'] = None
            peak_chart.options['xAxis']['max'] = None
            peak_chart.options['yAxis']['min'] = None
            peak_chart.options['yAxis']['max'] = None
            
            peak_chart.update()
            
            print("🔄 Peak chart reset to auto-scaling mode")
            ui.notify('Peak chart reset to auto-scaling', type='positive')
        except Exception as e:
            print(f"❌ Failed to reset Peak chart zoom: {e}")
            ui.notify(f'Reset failed: {e}', type='negative')

def save_od_chart():
    """Save OD chart as image"""
    if od_chart:
        try:
            # Enable toolbox saveAsImage feature in chart options
            if 'toolbox' not in od_chart.options:
                od_chart.options['toolbox'] = {}
            od_chart.options['toolbox']['feature'] = {
                'saveAsImage': {'show': False}  # Hidden but functional
            }
            od_chart.update()
            
            # Trigger save via JavaScript
            ui.run_javascript(f'''
                const chartDom = document.querySelector('.nicegui-echart');
                if (chartDom && chartDom.__echarts_instance__) {{
                    const chart = chartDom.__echarts_instance__;
                    const url = chart.getDataURL({{
                        type: 'png',
                        pixelRatio: 2,
                        backgroundColor: '#fff'
                    }});
                    const link = document.createElement('a');
                    link.href = url;
                    link.download = 'OD_Chart_' + new Date().toISOString().slice(0,19).replace(/:/g,'-') + '.png';
                    link.click();
                }}
            ''')
            ui.notify('💾 Downloading OD chart...', type='positive')
            print("💾 OD chart saved as image")
        except Exception as e:
            ui.notify(f'❌ Save failed: {e}', type='negative')
            print(f"❌ Failed to save OD chart: {e}")

def save_peak_chart():
    """Save Peak chart as image"""
    if peak_chart:
        try:
            # Enable toolbox saveAsImage feature in chart options
            if 'toolbox' not in peak_chart.options:
                peak_chart.options['toolbox'] = {}
            peak_chart.options['toolbox']['feature'] = {
                'saveAsImage': {'show': False}  # Hidden but functional
            }
            peak_chart.update()
            
            # Trigger save via JavaScript
            ui.run_javascript(f'''
                const charts = document.querySelectorAll('.nicegui-echart');
                if (charts.length > 1) {{
                    const chartDom = charts[1];  // Second chart is peak chart
                    if (chartDom && chartDom.__echarts_instance__) {{
                        const chart = chartDom.__echarts_instance__;
                        const url = chart.getDataURL({{
                            type: 'png',
                            pixelRatio: 2,
                            backgroundColor: '#fff'
                        }});
                        const link = document.createElement('a');
                        link.href = url;
                        link.download = 'Peak_Chart_' + new Date().toISOString().slice(0,19).replace(/:/g,'-') + '.png';
                        link.click();
                    }}
                }}
            ''')
            ui.notify('💾 Downloading Peak chart...', type='positive')
            print("💾 Peak chart saved as image")
        except Exception as e:
            ui.notify(f'❌ Save failed: {e}', type='negative')
            print(f"❌ Failed to save Peak chart: {e}")

async def upload_default_program():
    """Uploads the current working program to the Arduino."""
    global working_program
    
    if working_program and 'steps' in working_program and len(working_program['steps']) > 0:
        ui.notify('🔄 Synchronizing loaded cycle program...', type='info')
        default_steps = working_program['steps']
    else:
        ui.notify('🔄 Synchronizing default cycle program...', type='info')
        default_steps = [
            {'media_type': 'NEUTRAL', 'led_brightness': 0},
            {'media_type': 'NEUTRAL', 'led_brightness': 0},
            {'media_type': 'NEGATIVE', 'led_brightness': 0},
            {'media_type': 'NEGATIVE', 'led_brightness': 0},
            {'media_type': 'NEUTRAL', 'led_brightness': 0},
            {'media_type': 'NEUTRAL', 'led_brightness': 0},
            {'media_type': 'POSITIVE', 'led_brightness': 50},
            {'media_type': 'POSITIVE', 'led_brightness': 50},
            {'media_type': 'POSITIVE', 'led_brightness': 50}
        ]
        working_program['steps'] = default_steps

    # Save default program to file
    try:
        program_data = {
            'name': 'Current Program',
            'description': 'Synchronized from interface',
            'steps': default_steps,
            'cycles_per_led': working_program.get('cycles_per_led_change', 1)
        }
        with open('oevo_default_program.json', 'w') as f:
            json.dump(program_data, f, indent=2)
        print("✅ Program saved to oevo_default_program.json")
    except Exception as e:
        print(f"⚠️ Program file save error: {e}")
    
    # Send total steps
    await arduino.send_command_async(f"SET_TOTAL_STEPS,{len(default_steps)}")
    await asyncio.sleep(0.5)

    # Send global cycles per LED change from working program
    cycles_per_led = working_program.get('cycles_per_led_change', 1)
    print(f"📤 Sending cycles per LED change: {cycles_per_led}")
    await arduino.send_command_async(f"SET_CYCLES_PER_LED,{cycles_per_led}")
    await asyncio.sleep(0.5)
    
    # Send each step
    for i, step in enumerate(default_steps):
        step_num = i + 1
        media = step['media_type']
        led_pin = i + 1  # LED pin (not used in 6-LED cycling mode)
        led_pwm = int((step['led_brightness'] / 100.0) * 4095)
        temp = config_data['temperature']  # Use global temperature from config
        await arduino.send_command_async(f"SET_STEP,{step_num},{media},{led_pin},{led_pwm},{temp}")
        await asyncio.sleep(0.5)
    
    # Sync current configuration to Arduino
    sync_config_to_arduino()
    
    # Save configuration to SD card for standalone operation
    await arduino.send_command_async("SAVE_CONFIG_TO_SD")
    await asyncio.sleep(0.3)
    
    safe_notify('✅ Program and configuration synchronized.', type='positive')

async def connect_and_sync():
    """Establish connection and sync state with Arduino (shared across all browsers)."""
    global arduino, working_program, total_program_steps
    
    local_cycle = 0
    local_step = 0

    try:
        # Check if Arduino is already connected (from another browser or previous connection)
        if arduino and arduino.connected:
            print("🔍 Arduino already connected (shared across all browsers)")
            safe_notify(f"✅ Already connected on {arduino.port}\n👥 Connection is shared - all browsers see same data", type='positive', timeout=3000)
            connection_status.text = '🟢 CONNECTED'
        
        if not arduino or not arduino.connected:
            # No connection yet - establish it (only happens once)
            print("🔍 No active connection, attempting to connect...")
            if not arduino:
                arduino = ArduinoConnection()
            
            if not arduino.find_arduino():
                connection_status.text = '🔴 DISCONNECTED'
                safe_notify("❌ Failed to connect to Arduino.", type='negative')
                return None, None
            
            connection_status.text = '🟢 CONNECTED'
            safe_notify(f"✅ Connected to Arduino on {arduino.port}", type='positive')
            
            # Start watchdog AFTER successful connection (not at startup)
            if not arduino.monitor_running:
                print("🛡️ Starting USB Watchdog...")
                arduino.start_monitor()

            # Check SD card status immediately after connection
            print("📤 Checking SD card status...")
            arduino.send_command("CHECK_SD_STATUS")
            await asyncio.sleep(0.5)  # Give Arduino time to respond

            print("📋 Step 1: Loading configuration from Arduino's SD card...")
            # Load config that Arduino loaded from SD card on startup
            await load_config_from_arduino_sd()
            
            print("📋 Step 2: Loading saved program from Arduino's SD card...")
            # Load program that Arduino loaded from SD card on startup
            saved_program = await load_cycle_program_from_arduino()
            if saved_program:
                working_program = saved_program
                total_program_steps = len(saved_program['steps'])
                print(f"✅ Loaded {len(saved_program['steps'])} steps from Arduino's SD card")
            else:
                print("⚠️ Failed to load program from Arduino, keeping current program")
            
            print("📊 Step 3: Reading last experiment state from Arduino's memory...")
            
            # Reset state capture
            global current_state_cycle, current_state_step
            current_state_cycle = 0
            current_state_step = 0
            
            # Request current state from Arduino
            arduino.send_command("GET_CURRENT_STATE")
            print("📤 Sent GET_CURRENT_STATE command")
            
            # Wait for the main data processing loop to handle the response
            for i in range(10):  # Wait up to 2 seconds
                await asyncio.sleep(0.2)
                print(f"🔍 Waiting for state response {i+1}/10...")
                
                # Check if we got a response
                if current_state_cycle > 0 or current_state_step > 0:
                    print(f"✅ Got state response: Cycle {current_state_cycle}, Step {current_state_step}")
                    break
            
            # Use the captured state from the main processing loop
            local_cycle = current_state_cycle if current_state_cycle > 0 else 1
            local_step = current_state_step if current_state_step > 0 else 1
            
            print(f"🔍 Final result: cycle={local_cycle}, step={local_step}")
            print("🚀 Step 3: Returning startup info...")
            return local_cycle, local_step
        
        # If already connected, still need to get current state for startup dialog
        print("📋 Loading program from already-connected Arduino...")
        saved_program = await load_cycle_program_from_arduino()
        if saved_program:
            working_program = saved_program
            total_program_steps = len(saved_program['steps'])
            print(f"✅ Loaded {len(saved_program['steps'])} steps")
        
        # Get current state from Arduino
        print("📊 Reading current state...")
        local_cycle = 0
        local_step = 0
        
        try:
            arduino.send_command("GET_CURRENT_STATE")
            
            # Wait for response
            for i in range(15):
                await asyncio.sleep(0.2)
                data = arduino.read_data()
                if data:
                    for line in data:
                        if line.startswith("$STATE,"):
                            parts = line.split(',')
                            if len(parts) >= 5:
                                local_cycle = int(parts[3])
                                local_step = int(parts[4])
                                print(f"🔍 Got state: cycle={local_cycle}, step={local_step}")
                                return local_cycle, local_step
            
            print("⚠️ No state response, using defaults")
            return 1, 1
            
        except Exception as e:
            print(f"❌ Error getting state: {e}")
            return 1, 1
            
    except Exception as e:
        print(f"❌ An error occurred during connect_and_sync: {e}")
        traceback.print_exc()
        ui.notify(f'Connection failed: {e}', type='negative')
        if arduino and arduino.connected:
            arduino.close()
        arduino = None
        return 0, 0

async def show_startup_dialog(last_cycle, last_step):
    with ui.dialog() as dialog, ui.card().classes('w-96'):
        ui.label('🚀 OpenEvo Startup').classes('text-h5 mb-4')
        
        # Check for previous experiment data (either advanced position OR data exists)
        global has_previous_experiment_data
        has_previous_data = (last_cycle > 1 or last_step > 1 or has_previous_experiment_data)
        
        # Only load saved config and program if there's a previous experiment
        if has_previous_data:
            print("📋 Loading saved configuration and program for previous experiment...")
            if arduino.connected:
                # For previous experiments, prioritize Arduino's SD card config
                print("🔍 Loading config from Arduino's SD card...")
                sd_config_loaded = await load_config_from_arduino_sd()
                if not sd_config_loaded:
                    print("⚠️ Failed to load from SD card, using hard-coded defaults...")
                    # Use hard-coded defaults already in config_data
                    sync_config_to_arduino()
                
                # Load program from Arduino's SD card
                print("🔍 Loading program from Arduino's SD card...")
                global working_program, total_program_steps, led_calibration
                saved_program = await load_cycle_program_from_arduino()
                if saved_program:
                    working_program = saved_program
                    total_program_steps = len(saved_program['steps'])
                    print(f"✅ Loaded {len(saved_program['steps'])} steps from Arduino's SD card")
                else:
                    print("⚠️ Failed to load program from Arduino, using current program")
            else:
                # Arduino not connected, try loading from local JSON file
                try:
                    if os.path.exists('oevo_config.json'):
                        with open('oevo_config.json', 'r') as f:
                            saved_config = json.load(f)
                            config_data.update(saved_config)
                            # Also load LED calibration if present
                            if 'led_calibration' in saved_config:
                                led_calibration.update(saved_config['led_calibration'])
                            print("✅ Loaded config from local oevo_config.json")
                    else:
                        print("ℹ️ No local config file, using hardcoded defaults")
                except Exception as e:
                    print(f"⚠️ Could not load local config: {e}, using hardcoded defaults")
        else:
            print("🔍 New experiment - keeping system defaults")
        
        # Show current state info with more details
        if has_previous_data:
            ui.label('📋 Previous Experiment Found').classes('text-h6 mb-2 text-blue')
            
            # Show detailed info in a card
            with ui.card().classes('bg-blue-50 p-3 mb-4'):
                ui.label(f'🔄 Cycle: {last_cycle}').classes('text-body1 font-bold')
                ui.label(f'📍 Step: {last_step} of {total_program_steps}').classes('text-body1 font-bold')
                
                # Show current step info if available
                if hasattr(arduino, 'connected') and arduino.connected:
                    try:
                        # Get current step details from the loaded program
                        if working_program and 'steps' in working_program and last_step <= len(working_program['steps']) and last_step > 0:
                            step_info = working_program['steps'][last_step - 1]
                            media_type = step_info.get('media_type', 'Unknown')
                            led_brightness = step_info.get('led_brightness', 0)
                            ui.label(f'🧪 Media Type: {media_type}').classes('text-body2')
                            ui.label(f'💡 LED: {led_brightness}%').classes('text-body2')
                    except:
                        pass
            
            ui.label('What would you like to do?').classes('text-body2 mb-4')
        else:
            ui.label('📋 No Previous Experiment Found').classes('text-h6 mb-2 text-gray')
            with ui.card().classes('bg-gray-50 p-3 mb-4'):
                ui.label('Ready to start a fresh experiment').classes('text-body2')
            ui.label('Ready to begin!').classes('text-body2 mb-4')
        
        with ui.row().classes('gap-4 justify-center'):
            with ui.column().classes('gap-2'):
                ui.button('🆕 Start Fresh Experiment', 
                         on_click=lambda: dialog.submit('new')).classes('bg-green text-white px-6 py-2')
                ui.label('(Resets to defaults & cycle 1)').classes('text-caption text-center text-grey')
            
            if has_previous_data:
                with ui.column().classes('gap-2'):
                    ui.button(f'▶️ Resume from Cycle {last_cycle}, Step {last_step}', 
                             on_click=lambda: dialog.submit('resume')).classes('bg-blue text-white px-6 py-2')
                    ui.label('(Continue from where left off)').classes('text-caption text-center text-grey')

    result = await dialog
    return result, last_cycle, last_step

async def start_new_experiment():
    """Starts a completely new experiment from cycle 1, step 1."""
    global is_running, is_paused, csv_writer, csv_file, csv_filename, working_program, led_calibration
    
    try:
        # ===== STEP 1: LOAD CONFIGURATION FROM SD CARD FIRST =====
        # Priority: SD card config > local oevo_config.json > hardcoded defaults
        print("🔄 Loading saved configuration from SD card for new experiment...")
        sd_config_loaded = await load_config_from_arduino_sd()
        
        if not sd_config_loaded:
            # Try loading from local JSON file as backup
            print("⚠️ SD card config not available, trying local oevo_config.json...")
            try:
                if os.path.exists('oevo_config.json'):
                    with open('oevo_config.json', 'r') as f:
                        saved_config = json.load(f)
                        config_data.update(saved_config)
                        # Also load LED calibration if present
                        if 'led_calibration' in saved_config:
                            led_calibration.update(saved_config['led_calibration'])
                        print("✅ Loaded config from local oevo_config.json")
                else:
                    print("ℹ️ No local config file, using hardcoded defaults")
            except Exception as e:
                print(f"⚠️ Could not load local config: {e}, using hardcoded defaults")
        
        # ===== STEP 2: LOAD CYCLER PROGRAM FROM SD CARD =====
        print("🔄 Loading saved program from SD card for new experiment...")
        saved_program = await load_cycle_program_from_arduino()
        if saved_program and saved_program.get('steps'):
            working_program = saved_program
            print(f"✅ Loaded {len(saved_program['steps'])} steps from SD card")
        else:
            # Fallback to hardcoded defaults only if SD load fails
            print("⚠️ Failed to load program from SD, using hardcoded defaults...")
            working_program = {
                'steps': [
                    {'media_type': 'NEUTRAL', 'led_brightness': 0},
                    {'media_type': 'NEUTRAL', 'led_brightness': 0},
                    {'media_type': 'NEGATIVE', 'led_brightness': 0},
                    {'media_type': 'NEGATIVE', 'led_brightness': 0},
                    {'media_type': 'NEUTRAL', 'led_brightness': 0},
                    {'media_type': 'NEUTRAL', 'led_brightness': 0},
                    {'media_type': 'POSITIVE', 'led_brightness': 50},
                    {'media_type': 'POSITIVE', 'led_brightness': 50},
                    {'media_type': 'POSITIVE', 'led_brightness': 50}
                ],
                'cycles_per_led_change': 1
            }
        
        # Update total program steps to match the loaded program
        global total_program_steps
        total_program_steps = len(working_program['steps'])
        print(f"🔄 Set total_program_steps to {total_program_steps}")
        if not arduino.connected:
            ui.notify('❌ Arduino not connected', type='negative')
            return

        # Reset Arduino to fresh state
        print("🔄 Resetting Arduino to fresh state...")
        arduino.send_command("STOP")  # Stop any running experiment
        await asyncio.sleep(0.5)
        arduino.send_command("RESET_CYCLE_STEP")  # Reset to cycle 1, step 1
        await asyncio.sleep(0.5)
        
        # Upload the reset program to Arduino
        await upload_default_program()
        
        # Reset state variables
        global current_cycle, current_step
        current_cycle = 1
        current_step = 1
        is_running = True
        is_paused = False
        print(f"🔄 Reset state: Cycle {current_cycle}, Step {current_step}")
        
        # Reset data arrays for charts
        # ... existing code ...

        # Create a new local CSV file for backup
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        csv_filename = f"OEVO_log_{timestamp}.csv"
        csv_file = open(csv_filename, 'w', newline='')
        csv_writer = csv.writer(csv_file)
        
        # Write the header to the new local file (full 14-field format, matches SD card)
        header = ["unixTime", "upTime", "OD940", "infraredReading", "ambientTemp", "mediaTemp", "heaterPlateTemp", "totalCycleCount", "currentCycle", "currentStep", "mediaType", "Dilution_Event", "LED_Channel", "LED_Percent"]
        csv_writer.writerow(header)
        csv_file.flush()  # Flush header immediately so the file is never 0 KB / unopenable
        
        # Tell Arduino to initialize the SD card and write its header
        await arduino.send_command_async("INIT_SD_CARD")
        await asyncio.sleep(2.0) # Give SD card time to initialize
        await arduino.send_command_async("SD_WRITE_HEADER")
        await asyncio.sleep(0.5)
        
        # RESET TO FRESH STATE FOR NEW EXPERIMENT
        print("🔄 Resetting Arduino to fresh default state for new experiment...")
        
        # Reset to cycle 1, step 1
        await arduino.send_command_async("JUMP_TO,1,1")
        await asyncio.sleep(0.2)
        
        # Send fresh default configuration and save to SD card for new experiment
        print("💾 Saving system defaults to SD card for new experiment...")
        # Send 4-point calibration using correct firmware commands (slow mode)
        print(f"📤 Sending 4-point calibration (slow mode):")
        print(f"   Point 1 (Low):     IR={config_data['ir_low']}, OD={config_data['od_low']}")
        print(f"   Point 2 (Mid-Low): IR={config_data['ir_mid1']}, OD={config_data['od_mid1']}")
        print(f"   Point 3 (Mid-High):IR={config_data['ir_mid2']}, OD={config_data['od_mid2']}")
        print(f"   Point 4 (High):    IR={config_data['ir_high']}, OD={config_data['od_high']}")
        
        # CRITICAL: Send each point slowly to prevent serial buffer corruption
        await arduino.send_command_async(f"SET_CALIBRATION_POINT_1,{config_data['ir_low']},{config_data['od_low']:.3f}")
        await asyncio.sleep(0.8)
        arduino.read_data()  # Clear buffer
        
        await arduino.send_command_async(f"SET_CALIBRATION_POINT_2,{config_data['ir_mid1']},{config_data['od_mid1']:.3f}")
        await asyncio.sleep(0.8)
        arduino.read_data()  # Clear buffer
        
        await arduino.send_command_async(f"SET_CALIBRATION_POINT_3,{config_data['ir_mid2']},{config_data['od_mid2']:.3f}")
        await asyncio.sleep(0.8)
        arduino.read_data()  # Clear buffer
        
        await arduino.send_command_async(f"SET_CALIBRATION_POINT_4,{config_data['ir_high']},{config_data['od_high']:.3f}")
        await asyncio.sleep(0.8)
        arduino.read_data()  # Clear buffer
        
        # Send inverse calibration
        # Calculate inverse calibration first if not already done
        if 'inverseA' not in config_data or 'inverseB' not in config_data:
            calculate_inverse_calibration()
        a = config_data.get('inverseA', 0)
        b = config_data.get('inverseB', 0)
        print(f"📤 Sending inverse calibration: a={a:.2f}, b={b:.4f} → OD = {a:.2f}/IR + {b:.4f}")
        await arduino.send_command_async(f"SET_INVERSE_CALIBRATION,{a:.6f},{b:.6f}")
        await asyncio.sleep(0.3)
        
        await arduino.send_command_async(f"SET_THRESHOLD,{config_data['threshold']}")
        await asyncio.sleep(0.3)
        await arduino.send_command_async(f"SET_TEMPERATURE,{config_data['temperature']}")
        await asyncio.sleep(0.3)
        for i, speed in enumerate(config_data['pump_speeds']):
            await arduino.send_command_async(f"SET_PUMP_SPEED,{i+1},{speed}")
            await asyncio.sleep(0.2)
        await arduino.send_command_async(f"SET_STIRRING_SPEED,{config_data['stirring_speed']}")
        await asyncio.sleep(0.3)
        await arduino.send_command_async(f"SET_MAX_DISPENSATIONS,{config_data.get('max_dispensations', 400)}")
        await asyncio.sleep(0.3)
        # Send LED brightness for OD940 measurement LED
        await arduino.send_command_async(f"SET_LED_BRIGHTNESS,{config_data.get('led_brightness', 500)}")
        await asyncio.sleep(0.3)
        
        # Save defaults to SD card so they persist
        print("💾 Saving defaults to SD card...")
        await arduino.send_command_async("SAVE_CONFIG_TO_SD")
        await asyncio.sleep(1.0)
        
        print("✅ Arduino reset to fresh defaults")
        await asyncio.sleep(0.5)
        
        # CRITICALLY, SEND THE START COMMAND with extra delay
        print("📤 SENDING START_TURBIDOSTAT COMMAND")
        await arduino.send_command_async("START_TURBIDOSTAT")
        await asyncio.sleep(0.2)
        
        # Check SD card status after starting
        print("📤 Checking SD card status...")
        await arduino.send_command_async("CHECK_SD_STATUS")
        await asyncio.sleep(0.2)
        
        # Update UI to reflect running state
        start_button.text = 'Pause'
        start_button.props('color=warning icon=pause')
        start_button.update()
        
        ui.notify(f'🚀 New experiment started - logging to {csv_filename}', type='positive')
        
    except Exception as e:
        ui.notify(f'❌ Failed to start new experiment: {e}', type='negative')

async def resume_experiment(cycle, step):
    """Resumes an existing experiment from a specific cycle and step."""
    global is_running, is_paused, csv_writer, csv_file, csv_filename
    
    try:
        if not arduino.connected:
            ui.notify('❌ Arduino not connected', type='negative')
            return
        
        # Create a new local CSV backup file for this resumed run (V2.5).
        # Previously only start_new_experiment() logged to the PC; resumed runs
        # had no local backup (data only went to the Arduino SD card). Now resume
        # opens its own OEVO_log_*.csv so resumed runs are backed up locally too.
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        csv_filename = f"OEVO_log_{timestamp}.csv"
        csv_file = open(csv_filename, 'w', newline='')
        csv_writer = csv.writer(csv_file)
        header = ["unixTime", "upTime", "OD940", "infraredReading", "ambientTemp", "mediaTemp", "heaterPlateTemp", "totalCycleCount", "currentCycle", "currentStep", "mediaType", "Dilution_Event", "LED_Channel", "LED_Percent"]
        csv_writer.writerow(header)
        csv_file.flush()
        print(f"📁 Resume logging to local file: {csv_filename}")
        
        # Load configuration and program from Arduino's SD card before resuming
        print("🔍 Loading saved configuration from Arduino's SD card...")
        sd_config_loaded = await load_config_from_arduino_sd()
        if not sd_config_loaded:
            print("⚠️ Failed to load config from SD card, using current interface config...")
            sync_config_to_arduino()
        
        # Load program from Arduino's SD card
        print("🔍 Loading saved program from Arduino's SD card...")
        global working_program, total_program_steps
        saved_program = await load_cycle_program_from_arduino()
        if saved_program:
            working_program = saved_program
            total_program_steps = len(saved_program['steps'])
            print(f"✅ Loaded {len(saved_program['steps'])} steps from Arduino's SD card for resume")
        else:
            print("⚠️ Failed to load program from Arduino, using current program")
        
        # CRITICAL: Send calibration and program to Arduino BEFORE starting
        # This ensures Arduino has correct calibration even if SD load was incomplete
        print("📤 Sending calibration to Arduino for resume...")
        
        # Calculate inverse calibration if not already done
        if 'inverseA' not in config_data or 'inverseB' not in config_data:
            calculate_inverse_calibration()
        a = config_data.get('inverseA', 0)
        b = config_data.get('inverseB', 0)
        print(f"📤 Sending inverse calibration: a={a:.2f}, b={b:.4f} → OD = {a:.2f}/IR + {b:.4f}")
        await arduino.send_command_async(f"SET_INVERSE_CALIBRATION,{a:.6f},{b:.6f}")
        await asyncio.sleep(0.3)
        
        print("📤 Uploading program to Arduino for resume...")
        await arduino.send_command_async(f"SET_TOTAL_STEPS,{len(working_program['steps'])}")
        await asyncio.sleep(0.3)
        await arduino.send_command_async(f"SET_CYCLES_PER_LED,{working_program.get('cycles_per_led_change', 1)}")
        await asyncio.sleep(0.3)
        
        for i, step_data in enumerate(working_program['steps']):
            step_num = i + 1
            media = step_data['media_type']
            led_pwm = int((step_data['led_brightness'] / 100.0) * 4095)
            temp = config_data['temperature']
            await arduino.send_command_async(f"SET_STEP,{step_num},{media},0,{led_pwm},{temp}")
            await asyncio.sleep(0.2)
        
        print("✅ Program uploaded to Arduino")
        await asyncio.sleep(0.3)
        
        # Send the START_TURBIDOSTAT command to resume the experiment
        print("📤 Sending START_TURBIDOSTAT to resume experiment...")
        await arduino.send_command_async("START_TURBIDOSTAT")
        await asyncio.sleep(0.3)
        
        # Update global state
        is_running = True
        is_paused = False
        
        # Update button text
        start_button.text = 'Pause'
        start_button.props('icon=pause')
        
        safe_notify(f'▶️ Resumed experiment at Cycle {cycle}, Step {step}\n📁 Logging to {csv_filename}', type='positive')
        print(f"✅ Resumed experiment: Cycle {cycle}, Step {step}")
        
    except Exception as e:
        safe_notify(f'❌ Failed to resume experiment: {e}', type='negative')
        print(f"❌ Resume experiment error: {e}")

async def start_stop_experiment(e):
    """Connect or Pause/Resume experiment"""
    global is_paused, start_button, is_running, arduino, connect_in_progress
    
    print(f"🔍 PAUSE/RESUME BUTTON CLICKED!")
    print(f"🔍 Current button text: '{start_button.text}'")
    print(f"🔍 is_paused: {is_paused}")
    
    current_text = start_button.text
    
    # If not connected OR button says "Connect", handle connection
    if not arduino or not arduino.connected or current_text == 'Connect':
        # V2.4: single-flight guard. The mirroring / pinned-client setup can
        # deliver this click more than once (and the user can double-click).
        # A second connect/sync flow running at the same time as the first
        # races it for serial bytes - both readers drain the port, so each
        # steals the other's $STATE / $PROGRAM replies and they false-timeout.
        # (The Arduino is fine; it's two concurrent readers fighting.) Only
        # ever allow ONE connect/sync/start sequence to run at a time.
        if connect_in_progress:
            print("⏸️ Connect/sync already in progress - ignoring duplicate click")
            safe_notify('⏳ Already connecting, please wait…', type='info')
            return
        connect_in_progress = True
        try:
            ui.notify('🔌 Connecting to OpenEvo...', timeout=2)
            # Get the local cycle and step from the sync function
            cycle, step = await connect_and_sync()
            
            # Now show the dialog with the safely returned values
            if cycle is not None and step is not None:
                dialog_result, last_cycle, last_step = await show_startup_dialog(cycle, step)
                # Call experiment functions here (main page context, not dialog context)
                # so that ui.notify works correctly after the dialog has been deleted.
                if dialog_result == 'new':
                    await start_new_experiment()
                elif dialog_result == 'resume':
                    await resume_experiment(last_cycle, last_step)

            # After dialog closes (whether user started experiment or not),
            # ensure button shows correct state based on actual running status
            if not is_running:
                start_button.text = 'Connect'
                start_button.props('color=secondary icon=link')
        finally:
            connect_in_progress = False
        return
        
    # Handle Pause/Resume only
    if current_text == 'Pause':
        print("🔍 PAUSE button clicked!")
        # Pause experiment and ensure SD card data is synced
        arduino.send_command("PAUSE")
        arduino.send_command("SD_SYNC")  # Ensure all data is written to SD card
        is_paused = True
        safe_notify('⏸️ Experiment paused - SD card data synced', type='info')
        start_button.text = 'Resume'
        start_button.props('color=positive')
        start_button.props('icon=play_arrow')
        start_button.update()
        print("🔍 Button changed to 'Resume'")
        
    elif current_text == 'Resume':
        print("🔍 RESUME button clicked!")
        print(f"🔍 is_paused was: {is_paused}")
        # Resume experiment
        arduino.send_command("UNPAUSE")
        is_paused = False
        print(f"🔍 is_paused now set to: {is_paused}")
        safe_notify('▶️ Experiment resumed', type='positive')
        start_button.text = 'Pause'
        start_button.props('color=warning')
        start_button.props('icon=pause')
        start_button.update()
        print("🔍 Button changed to 'Pause', data plotting should resume")
    elif current_text == 'Connect':
        print("🔍 CONNECT button clicked!")
        # This should have been handled by the connection logic above, 
        # but if we get here, try connecting again
        safe_notify('🔌 Connecting to OpenEvo...', timeout=2)
        cycle, step = await connect_and_sync()
        if cycle is not None and step is not None:
            await show_startup_dialog(cycle, step)
        if not is_running:
            start_button.text = 'Connect'
            start_button.props('color=secondary icon=link')
    else:
        print(f"🔍 UNEXPECTED: No matching condition! current_text='{current_text}'")


def skip_with_media():
    """Skip step with media change - SIMPLE LIKE WORKING VERSION"""
    if arduino.connected:
        arduino.send_command("SKIP_STEP")
        ui.notify('⏭️ Skipping step with media change', type='info')
    else:
        ui.notify('❌ Arduino not connected', type='negative')

def skip_no_media():
    """Skip step without media change - SIMPLE LIKE WORKING VERSION"""
    if arduino.connected:
        arduino.send_command("SKIP_NO_MEDIA")
        ui.notify('⏭️ Skipping step (no media change)', type='info')
    else:
        ui.notify('❌ Arduino not connected', type='negative')

def show_jump_dialog():
    """Show jump to cycle/step dialog"""
    with ui.dialog() as jump_dialog, ui.card():
        ui.label('Jump to Cycle/Step').classes('text-h6 q-mb-md')
        
        cycle_input = ui.number('Cycle Number', value=current_cycle, min=1, max=999).classes('w-full')
        step_input = ui.number('Step Number', value=current_step, min=1, max=50).classes('w-full')
        
        async def do_jump():
            try:
                cycle = int(cycle_input.value)
                step = int(step_input.value)
                
                # Global variables needed for chart update
                global time_data, peak_times, peak_media_types
                
                # Sync the target step configuration first to ensure media type is correct
                # This prevents "phantom neutral" plotting if user edited step but didn't upload
                if working_program and 'steps' in working_program and 0 < step <= len(working_program['steps']):
                    target_step_data = working_program['steps'][step-1]
                    media = target_step_data.get('media_type', 'NEUTRAL')
                    led_brightness = target_step_data.get('led_brightness', 0)
                    # Convert to PWM
                    led_pwm = int((led_brightness / 100.0) * 4095)
                    temp = config_data['temperature']
                    
                    print(f"📤 Syncing step {step} (Media={media}) before jump")
                    arduino.send_command(f"SET_STEP,{step},{media},0,{led_pwm},{temp}")
                    await asyncio.sleep(0.2)
                
                arduino.send_command(f"JUMP_TO,{cycle},{step}")
                ui.notify(f'🎯 Jumped to Cycle {cycle}, Step {step}', type='positive')
                jump_dialog.close()
            except Exception as e:
                ui.notify(f'❌ Jump failed: {e}', type='negative')
        
        with ui.row():
            ui.button('Jump', on_click=do_jump).props('color=primary')
            ui.button('Cancel', on_click=jump_dialog.close).props('flat')
    
    jump_dialog.open()

def show_pump_control():
    """Show manual pump control dialog"""
    with ui.dialog() as pump_dialog:
        with ui.card():
            ui.label('Manual Pump Control').classes('text-h6 q-mb-md')
            
            pump_options = {
                1: 'Pump 1 - Neutral',
                2: 'Pump 2 - Positive', 
                3: 'Pump 3 - Negative',
                4: 'Pump 4 - Empty'
            }
            pump_select = ui.select(pump_options, value=1, label='Select Pump').classes('w-full')
            volume_input = ui.number('Volume (mL)', value=1.0, min=0.1, max=50.0, step=0.1).classes('w-full')
            
            def do_pump():
                try:
                    if not arduino or not arduino.connected:
                        ui.notify('❌ Arduino not connected', type='negative')
                        return
                    
                    pump = int(pump_select.value)
                    volume = float(volume_input.value)
                    dispensations = int(volume / 0.1)  # 1 dispensation = 0.1mL
                    
                    # Get pump name for notification
                    pump_names = {1: 'Neutral', 2: 'Positive', 3: 'Negative', 4: 'Empty'}
                    pump_name = pump_names.get(pump, f'Pump {pump}')
                    
                    arduino.send_command(f"MANUAL_PUMP,{pump},{dispensations}")
                    ui.notify(f'🔧 Pump {pump} ({pump_name}): {volume}mL ({dispensations} dispensations)', type='positive')
                    pump_dialog.close()
                except Exception as e:
                    ui.notify(f'❌ Pump control failed: {e}', type='negative')
            
            with ui.row():
                ui.button('Run Pump', on_click=do_pump).props('color=primary')
                ui.button('Cancel', on_click=pump_dialog.close).props('flat')
    
    pump_dialog.open()

def sync_rtc_time():
    """Sync Arduino RTC with current system time"""
    global arduino
    
    if not arduino or not arduino.connected:
        ui.notify('❌ Arduino not connected - Connect first to sync time', type='negative')
        return
    
    import time
    import datetime
    
    # Get current UTC time as Unix timestamp
    # Use time.time() which always returns UTC timestamp
    current_timestamp = int(time.time())
    
    # Get UTC and local time for display purposes
    now_utc = datetime.datetime.utcfromtimestamp(current_timestamp)
    now_local = datetime.datetime.fromtimestamp(current_timestamp)
    
    ui.notify(f'🕐 Syncing RTC time to UTC...', type='info')
    
    # Send the SET_TIME command with UTC timestamp
    command = f"SET_TIME,{current_timestamp}"
    arduino.send_command(command)
    
    print(f"🕐 Sent time sync command: {command}")
    print(f"🕐 UTC timestamp: {current_timestamp}")
    print(f"🕐 UTC time: {now_utc.strftime('%Y-%m-%d %H:%M:%S')} UTC")
    print(f"🕐 Local time: {now_local.strftime('%Y-%m-%d %H:%M:%S %Z')}")
    
    # Show human-readable UTC time for confirmation
    readable_time_utc = now_utc.strftime('%Y-%m-%d %H:%M:%S UTC')
    readable_time_local = now_local.strftime('%Y-%m-%d %H:%M:%S %Z')
    ui.notify(f'📡 RTC set to: {readable_time_utc} (Local: {readable_time_local})', type='positive')

# ─── DIALOG DEFINITIONS (must be at top level) ───────────────────────────

class CycleEditor:
    def __init__(self, load_from_arduino=False):
        global working_program
        
        if load_from_arduino:
            # Only load from Arduino when explicitly requested (e.g., on startup)
            saved_program = load_cycle_program_from_arduino()
            if saved_program:
                working_program = saved_program
                print(f"📋 Loaded {len(saved_program['steps'])} steps from Arduino")
            else:
                # Fallback to CSV file if Arduino not available
                saved_program = load_cycle_program_from_csv()
                if saved_program:
                    working_program = saved_program
                    print(f"📋 Loaded {len(saved_program['steps'])} steps from CSV fallback")
                else:
                    print("📋 Using default 3-step program")
        
        # Always use the global working program
        self.steps = working_program['steps'].copy()  # Make a copy to avoid reference issues
        self.cycles_per_led_change = working_program['cycles_per_led_change']
        print(f"📋 Editor initialized with {len(self.steps)} steps")
        
        self.step_cards = []
        self.container = ui.column().classes('w-full')
    
    def create_step_card(self, step_index, step_data):
        """Create a compact card for editing a single step"""
        with ui.card().classes('w-full q-mb-xs').style('padding: 8px;') as card:
            with ui.row().classes('w-full items-center q-gutter-sm'):
                # Step number and delete button
                ui.label(f'Step {step_index + 1}').classes('text-subtitle2 text-weight-bold').style('min-width: 60px;')
                
                def delete_step():
                    global working_program, total_program_steps
                    if len(self.steps) > 1:
                        self.steps.pop(step_index)
                        # Update global working program
                        working_program['steps'] = self.steps.copy()
                        total_program_steps = len(self.steps)
                        self.refresh_editor()
                        ui.notify(f'🗑️ Deleted step {step_index + 1}', type='info')
                    else:
                        ui.notify('❌ Cannot delete last step', type='negative')
                
                ui.button('🗑️', on_click=delete_step).props('flat color=red size=xs dense')
                
                # Compact inputs in a row
                # Validate and clean media type - fix corrupted data
                valid_media_types = ['NEUTRAL', 'POSITIVE', 'NEGATIVE']
                current_media_type = step_data['media_type']
                if current_media_type not in valid_media_types:
                    print(f"⚠️ Invalid media type '{current_media_type}' found in step {step_index + 1}, defaulting to 'NEUTRAL'")
                    current_media_type = 'NEUTRAL'
                    step_data['media_type'] = 'NEUTRAL'  # Fix the data
                
                media_input = ui.select(valid_media_types, 
                                      value=current_media_type).props('dense').style('width: 120px;')
                
                led_input = ui.number(value=step_data['led_brightness'], 
                                    min=0, max=100, step=1).props('dense suffix="%"').style('width: 80px;')
                
                # Temperature removed - now controlled globally via config
            
            def update_step():
                global working_program
                # Clamp LED brightness to valid range 0-100%
                led_val = max(0, min(100, int(led_input.value or 0)))
                self.steps[step_index] = {
                    'media_type': media_input.value,
                    'led_brightness': led_val
                }
                # Update global working program
                working_program['steps'] = self.steps.copy()
            
            # Update step data when inputs change
            media_input.on('update:model-value', lambda: update_step())
            led_input.on('update:model-value', lambda: update_step())
            
        return card
    
    def refresh_editor(self):
        """Refresh the entire editor"""
        # Clear existing cards
        for card in self.step_cards:
            try:
                card.delete()
            except:
                pass
        self.step_cards.clear()
        
        # Clear the entire container to remove any duplicate headers
        self.container.clear()
        
        # Recreate cards
        with self.container:
            # Header row for step columns (appears above step cards)
            with ui.card().classes('w-full q-mb-sm').style('background: #e8f4fd; padding: 8px;'):
                with ui.row().classes('w-full items-center q-gutter-sm'):
                    ui.label('Step #').classes('text-weight-bold text-caption').style('min-width: 60px;')
                    ui.label('Delete').classes('text-weight-bold text-caption').style('min-width: 30px;')
                    ui.label('Media Type').classes('text-weight-bold text-caption').style('min-width: 120px;')
                    ui.label('LED %').classes('text-weight-bold text-caption').style('min-width: 80px;')

            
            # Create step cards below header
            for i, step in enumerate(self.steps):
                card = self.create_step_card(i, step)
                self.step_cards.append(card)

def show_cycle_editor():
    """Show cycle program editor"""
    with ui.dialog().style('width: 90vw; max-width: none;') as cycle_dialog:
        with ui.card().style('width: 100%; min-width: 150px; max-width: 400;'):
            ui.label('Cycle Program Editor').classes('text-h6 q-mb-md')
            
            # Global cycles per LED change setting
            with ui.card().classes('w-full q-mb-md').style('background: #f0f8ff;'):
                ui.label('Global Settings').classes('text-subtitle1 text-weight-bold q-mb-sm')
                cycles_per_led_input = ui.number('Dilutions per Step', 
                                               value=working_program.get('cycles_per_led_change', 1), min=1, max=100, step=1).classes('w-full')
                ui.label('Note: LED brightness and media type will advance to the next step after this many dilutions').classes('text-caption text-grey')
            
            editor = CycleEditor()
            
            # Update editor's global setting when input changes
            def update_global_cycles():
                global working_program
                editor.cycles_per_led_change = cycles_per_led_input.value
                working_program['cycles_per_led_change'] = cycles_per_led_input.value
            cycles_per_led_input.on('update:model-value', lambda: update_global_cycles())
            
            editor.refresh_editor() # Initial population
            
            # Add Step button
            def add_step():
                global working_program, total_program_steps
                editor.steps.append({'media_type': 'NEUTRAL', 'led_brightness': 0})
                # Update global working program
                working_program['steps'] = editor.steps.copy()
                total_program_steps = len(editor.steps)
                editor.refresh_editor()
                ui.notify(f'➕ Added step {len(editor.steps)}', type='positive')
            
            ui.button('➕ Add Step', on_click=add_step).props('color=positive dense').classes('q-mt-sm')
            
            def export_and_upload_program():
                try:
                    global working_program, total_program_steps
                    
                    # Update global working program before saving
                    working_program['steps'] = editor.steps.copy()
                    working_program['cycles_per_led_change'] = editor.cycles_per_led_change
                    total_program_steps = len(editor.steps)
                    
                    # Validate and fix any corrupted steps before sending
                    validate_and_fix_working_program()
                    print(f"📋 Validated program: {len(working_program['steps'])} steps ready for export")
                    
                    # Send total steps FIRST
                    arduino.send_command(f"SET_TOTAL_STEPS,{len(editor.steps)}")
                    time.sleep(0.2)
                    
                    # Send global cycles per LED change setting
                    print(f"📤 Sending cycles per LED change: {editor.cycles_per_led_change}")
                    arduino.send_command(f"SET_CYCLES_PER_LED,{editor.cycles_per_led_change}")
                    time.sleep(0.2)
                    
                    # Convert to Arduino format and send each step with validation
                    for i, step in enumerate(editor.steps):
                        # Validate and fix media type
                        media_type = step.get('media_type', 'NEUTRAL')
                        if media_type not in ['NEUTRAL', 'POSITIVE', 'NEGATIVE'] or not media_type:
                            print(f"⚠️ Step {i+1} has invalid media type '{media_type}', fixing to NEUTRAL")
                            media_type = 'NEUTRAL'
                            step['media_type'] = 'NEUTRAL'  # Fix the step data
                        
                        # Convert percentage to PWM (0-100% to 0-4095)
                        led_brightness = step.get('led_brightness', 0)
                        pwm_value = int((led_brightness / 100.0) * 4095)
                        temperature = config_data['temperature']  # Use global temperature from config
                        
                        # LED pin is not used in 6-LED cycling mode, but send step number as placeholder
                        led_pin = i + 1  # Use step number as LED pin (will be overridden by cycle-based logic)
                        
                        print(f"📤 Sending step {i+1}: media='{media_type}', led={led_brightness}%, temp={temperature}°C (global)")
                        arduino.send_command(f"SET_STEP,{i+1},{media_type},{led_pin},{pwm_value},{temperature}")
                        time.sleep(0.2)
                    
                    # Export to SD card (this will be the main function)
                    arduino.send_command("SAVE_CYCLER_TO_SD")
                    time.sleep(0.3)
                    
                    # Also save configuration to ensure OpenEvo can run independently
                    arduino.send_command("SAVE_CONFIG_TO_SD")
                    time.sleep(0.3)
                    
                    # Update total_program_steps after saving
                    read_total_steps_from_cycler()
                    
                    ui.notify(f'✅ Exported {len(editor.steps)} steps and configuration to SD card', type='positive')
                    cycle_dialog.close()
                except Exception as e:
                    ui.notify(f'❌ Export failed: {e}', type='negative')
                
            with ui.row().classes('w-full justify-end q-mt-md'):
                ui.button('Cancel', on_click=cycle_dialog.close)
                ui.button('Save', on_click=export_and_upload_program).props('color=primary')
    
    cycle_dialog.open()

def show_config_dialog():
    """Show configuration dialog"""
    with ui.dialog() as config_dialog, ui.card().style('width: 700px; max-width: 98vw; min-width: 700px'):
        ui.label('Configure OpenEvo').classes('text-h6 q-mb-sm')
        
        # OD Calibration  
        ui.separator().classes('q-my-sm')
        ui.label('2-4 Point Calibration (Least-Squares Inverse Fit - Always use zero and the threshold/high OD)').classes('text-subtitle2 q-mb-xs')
        ui.label('Set IR=0 to skip optional mid points.').classes('text-caption text-grey q-mb-sm')

        # First row: Low, Mid1, Mid2, High
        with ui.row().classes('w-full justify-between').style('flex-wrap: nowrap; gap: 0px;'):
            # Low OD Point (required)
            with ui.column().style('flex: 1; min-width: 0; max-width: 130px;'):
                ui.label('Point 1: Low OD').classes('text-weight-bold text-caption q-mb-xs')
                ir_low_input = ui.number('IR at Low OD', value=config_data['ir_low']).style('width: 100%').props('dense')
                od_low_input = ui.number('Low OD Value', value=config_data['od_low'], step=0.1).style('width: 100%').props('dense')
            # Mid1 OD Point (required)
            with ui.column().style('flex: 1; min-width: 0; max-width: 130px;'):
                ui.label('Point 2: Mid-Low OD').classes('text-weight-bold text-caption q-mb-xs')
                ir_mid1_input = ui.number('IR at Mid-Low OD', value=config_data.get('ir_mid1', config_data.get('ir_mid', 5496))).style('width: 100%').props('dense')
                od_mid1_input = ui.number('Mid-Low OD Value', value=config_data.get('od_mid1', config_data.get('od_mid', 1.0)), step=0.1).style('width: 100%').props('dense')
            # Mid2 OD Point (optional)
            with ui.column().style('flex: 1; min-width: 0; max-width: 130px;'):
                ui.label('Point 3: Mid-High OD').classes('text-weight-bold text-caption q-mb-xs')
                ir_mid2_input = ui.number('IR at Mid-High OD', value=config_data.get('ir_mid2', 0)).style('width: 100%').props('dense')
                od_mid2_input = ui.number('Mid-High OD Value', value=config_data.get('od_mid2', 2.5), step=0.1).style('width: 100%').props('dense')
            # High OD Point (required)
            with ui.column().style('flex: 1; min-width: 0; max-width: 130px;'):
                ui.label('Point 4: High OD').classes('text-weight-bold text-caption q-mb-xs')
                ir_high_input = ui.number('IR at High OD', value=config_data['ir_high']).style('width: 100%').props('dense')
                od_high_input = ui.number('High OD Value', value=config_data['od_high'], step=0.1).style('width: 100%').props('dense')
        
        ui.label('Formula: OD = a/IR + b (inverse fit)').classes('text-caption text-grey q-mt-xs')
        
        # Show current calculated equation
        def get_current_equation():
            if config_data.get('useInverse', False):
                a = config_data.get('inverseA', 0)
                b = config_data.get('inverseB', 0)
                return f'Current (Inverse): OD = {a:.2f}/IR + {b:.4f}'
            else:
                slope = config_data.get('slope', 0)
                intercept = config_data.get('intercept', 0)
                return f'Current (Linear): OD = {slope:.6f}×IR + {intercept:.4f}'
        
        calibration_equation_label = ui.label(get_current_equation()).classes('text-caption text-primary q-mt-xs').style('font-weight: bold;')
        
        # Show R² (goodness of fit)
        def get_r_squared_text():
            r_squared = config_data.get('r_squared', None)
            if r_squared is not None:
                if r_squared >= 0.99:
                    color = 'green'
                    quality = 'Excellent'
                elif r_squared >= 0.95:
                    color = 'blue'
                    quality = 'Good'
                elif r_squared >= 0.90:
                    color = 'orange'
                    quality = 'Fair'
                else:
                    color = 'red'
                    quality = 'Poor'
                return f'R² = {r_squared:.6f} ({quality} fit)', color
            else:
                return 'R² = Not calculated yet', 'grey'
        
        r_squared_text, r_squared_color = get_r_squared_text()
        r_squared_label = ui.label(r_squared_text).classes(f'text-caption text-{r_squared_color} q-mt-xs').style('font-weight: bold;')
        
        # Direct calibration section removed - handled automatically from 2-point calibration
        
    
        # Pump Speeds (moved above Other Settings)
        ui.separator().classes('q-my-sm')
        ui.label('Pump Speeds (RPM)').classes('text-subtitle2 q-mb-xs')
        pump_inputs = []
        with ui.row().classes('w-full q-gutter-sm q-mb-sm'):
            for i in range(4):
                pump_input = ui.number(f'Pump {i+1}', value=config_data['pump_speeds'][i], min=0, max=2000, step=1).style('width: 120px').props('dense')
                pump_inputs.append(pump_input)
        
        # LED Calibration Section
        ui.separator().classes('q-my-sm')
        with ui.row().classes('items-center'):
            ui.label('LED Intensity Calibration (mW/cm²)').classes('text-subtitle2')
            ui.label('💡').tooltip(
                'Measure LED intensity at 100% with a light meter.\n'
                'Click "Test" to turn on each LED, then enter the measured value.\n'
                'This allows calculating actual irradiance from effective dose %.'
            )
        
        led_intensity_inputs = {}
        led_wavelength_inputs = {}
        
        with ui.column().classes('w-full q-gutter-xs'):
            # Create 2 rows of 3 LEDs each
            for row_start in [1, 4]:
                with ui.row().classes('w-full q-gutter-sm'):
                    for led_num in range(row_start, row_start + 3):
                        with ui.card().classes('q-pa-sm').style('min-width: 180px;'):
                            with ui.row().classes('items-center q-gutter-xs'):
                                ui.label(f'LED {led_num}').classes('text-weight-bold')
                                
                                # Test button to turn on this LED at 100% for calibration
                                def make_test_handler(channel):
                                    async def test_led():
                                        if arduino and arduino.connected:
                                            # Send TEST_LED command (channel, brightness at 100% = 4095)
                                            # Firmware enters calibration mode and keeps LED on through OD cycles
                                            await arduino.send_command_async(f"TEST_LED,{channel},4095")
                                            ui.notify(f'💡 LED {channel} ON at 100% for calibration', type='info')
                                        else:
                                            ui.notify('❌ Arduino not connected', type='negative')
                                    return test_led
                                
                                ui.button('Test', on_click=make_test_handler(led_num)).props('size=sm color=primary dense')
                                
                                # Off button - exits calibration mode and resumes normal LED operation
                                def make_off_handler(channel):
                                    async def led_off():
                                        if arduino and arduino.connected:
                                            # Send brightness 0 to exit calibration mode
                                            await arduino.send_command_async(f"TEST_LED,{channel},0")
                                            ui.notify(f'LED {channel} OFF - Normal LED operation resumed', type='info')
                                        else:
                                            ui.notify('❌ Arduino not connected', type='negative')
                                    return led_off
                                
                                ui.button('Off', on_click=make_off_handler(led_num)).props('size=sm color=grey dense')
                            
                            with ui.row().classes('items-center q-gutter-xs q-mt-xs'):
                                intensity_input = ui.number(
                                    'mW/cm²', 
                                    value=led_calibration[led_num]['intensity'],
                                    min=0, max=1000, step=0.1
                                ).props('dense').style('width: 80px;')
                                led_intensity_inputs[led_num] = intensity_input
                                
                                wavelength_input = ui.input(
                                    'λ (nm)',
                                    value=led_calibration[led_num]['wavelength']
                                ).props('dense').style('width: 70px;')
                                led_wavelength_inputs[led_num] = wavelength_input
        
        # Other Settings
        ui.separator().classes('q-my-sm')
        ui.label('Other Settings').classes('text-subtitle2 q-mb-sm')
        
        # All settings on one row
        with ui.row().classes('w-full').style('gap: 16px;'):
            temp_input = ui.number('Media Temperature (°C)', value=config_data['temperature'], min=20, max=50).style('width: 120px').props('dense')
            threshold_input = ui.number('OD Threshold', value=config_data['threshold'], min=0.1, max=10.0, step=0.1).style('width: 120px').props('dense')
            stir_input = ui.number('Stirring Speed (RPM)', value=config_data['stirring_speed'], min=0, max=2000, step=1).style('width: 120px').props('dense')
            led_input = ui.number('OD940nm LED (1-4095)', value=config_data['led_brightness'], min=0, max=4095, step=1).style('width: 120px').props('dense')
            max_disp_input = ui.number('Dilution Media Vol (ml)', value=config_data['max_dispensations'], min=10, max=500, step=1).style('width: 120px').props('dense')


        with ui.row().classes('w-full q-mt-md justify-end'):
            ui.button('Cancel', on_click=config_dialog.close).props('flat')
            
            async def save_and_upload():
                """Nested async function to handle the save action."""
                try:
                    # Update config data from UI elements
                    config_data['temperature'] = temp_input.value
                    config_data['stirring_speed'] = int(stir_input.value)
                    config_data['led_brightness'] = int(led_input.value)
                    config_data['threshold'] = threshold_input.value
                    
                    # 3-5 point calibration with debugging
                    print(f"🔍 Reading calibration from UI:")
                    print(f"   Point 1 (Low): IR={ir_low_input.value}, OD={od_low_input.value}")
                    print(f"   Point 2 (Mid1): IR={ir_mid1_input.value}, OD={od_mid1_input.value}")
                    print(f"   Point 3 (Mid2): IR={ir_mid2_input.value}, OD={od_mid2_input.value}")
                    print(f"   Point 4 (High): IR={ir_high_input.value}, OD={od_high_input.value}")
                    
                    config_data['ir_low'] = int(ir_low_input.value)
                    config_data['od_low'] = od_low_input.value
                    config_data['ir_mid1'] = int(ir_mid1_input.value)
                    config_data['od_mid1'] = od_mid1_input.value
                    config_data['ir_mid2'] = int(ir_mid2_input.value)
                    config_data['od_mid2'] = od_mid2_input.value
                    config_data['ir_high'] = int(ir_high_input.value)
                    config_data['od_high'] = od_high_input.value
                    
                    # Backward compatibility (keep old 3-point names pointing to mid1)
                    config_data['ir_mid'] = int(ir_mid1_input.value)
                    config_data['od_mid'] = od_mid1_input.value
                    config_data['ir_zero'] = int(ir_low_input.value)
                    config_data['od_zero'] = od_low_input.value
                    config_data['ir_target'] = int(ir_high_input.value)
                    config_data['od_target'] = od_high_input.value
                    config_data['pump_speeds'] = [int(p.value) for p in pump_inputs]
                    config_data['max_dispensations'] = int(max_disp_input.value)
                    
                    # Save LED intensity calibration
                    global led_calibration
                    print(f"🔍 Reading LED calibration from UI inputs...")
                    for led_num in range(1, 7):
                        if led_num in led_intensity_inputs:
                            intensity_val = led_intensity_inputs[led_num].value
                            print(f"   LED{led_num} intensity input value: {intensity_val} (type: {type(intensity_val)})")
                            led_calibration[led_num]['intensity'] = float(intensity_val or 0)
                        if led_num in led_wavelength_inputs:
                            wavelength_val = led_wavelength_inputs[led_num].value
                            print(f"   LED{led_num} wavelength input value: {wavelength_val} (type: {type(wavelength_val)})")
                            led_calibration[led_num]['wavelength'] = str(wavelength_val or '')
                    print(f"💡 LED calibration after reading from UI: {led_calibration}")

                    # Calculate calibration using INVERSE (physically correct for light absorption)
                    calculate_inverse_calibration()
                    
                    # Update UI labels with new calibration
                    calibration_equation_label.set_text(get_current_equation())
                    r_squared_text, r_squared_color = get_r_squared_text()
                    r_squared_label.set_text(r_squared_text)
                    r_squared_label.classes(f'text-caption text-{r_squared_color} q-mt-xs', remove='text-caption text-grey q-mt-xs text-caption text-green q-mt-xs text-caption text-blue q-mt-xs text-caption text-orange q-mt-xs text-caption text-red q-mt-xs')
                    r_squared_label.style('font-weight: bold;')
                    
                    # Count active points
                    active_points = sum([
                        config_data['ir_low'] > 0,
                        config_data['ir_mid1'] > 0,
                        config_data['ir_mid2'] > 0,
                        config_data['ir_high'] > 0
                    ])
                    print(f"✅ {active_points}-Point calibration saved (least-squares fit)")
                    
                    # Save config to local JSON file (NOT including LED calibration - that's on Arduino SD card)
                    try:
                        save_data = config_data.copy()
                        # Don't save LED calibration to local file - it's stored on Arduino SD card
                        with open('oevo_config.json', 'w') as f:
                            json.dump(save_data, f, indent=2)
                        print("✅ Configuration saved to oevo_config.json (LED calibration is on Arduino SD)")
                    except Exception as e:
                        print(f"⚠️ Config file save error: {e}")
                    
                    # Send configuration to Arduino
                    if arduino.connected:
                        global config_updating
                        config_updating = True  # Pause data plotting during config update
                        ui.notify('📡 Uploading configuration to Arduino...', type='info')
                        
                        # Send 4-point calibration data FIRST
                        # CRITICAL: Send each point slowly and wait for Arduino to process
                        # Previous bug: commands were concatenating in serial buffer
                        print(f"📤 Dialog save: Sending 4-point calibration (slow mode):")
                        print(f"   Point 1 (Low):     IR={config_data['ir_low']}, OD={config_data['od_low']}")
                        print(f"   Point 2 (Mid-Low): IR={config_data['ir_mid1']}, OD={config_data['od_mid1']}")
                        print(f"   Point 3 (Mid-High):IR={config_data['ir_mid2']}, OD={config_data['od_mid2']}")
                        print(f"   Point 4 (High):    IR={config_data['ir_high']}, OD={config_data['od_high']}")
                        
                        # Send Point 1 and wait
                        await arduino.send_command_async(f"SET_CALIBRATION_POINT_1,{config_data['ir_low']},{config_data['od_low']:.3f}")
                        await asyncio.sleep(0.8)
                        arduino.read_data()  # Clear any pending responses
                        
                        # Send Point 2 and wait
                        await arduino.send_command_async(f"SET_CALIBRATION_POINT_2,{config_data['ir_mid1']},{config_data['od_mid1']:.3f}")
                        await asyncio.sleep(0.8)
                        arduino.read_data()  # Clear any pending responses
                        
                        # Send Point 3 and wait
                        await arduino.send_command_async(f"SET_CALIBRATION_POINT_3,{config_data['ir_mid2']},{config_data['od_mid2']:.3f}")
                        await asyncio.sleep(0.8)
                        arduino.read_data()  # Clear any pending responses
                        
                        # Send Point 4 and wait
                        await arduino.send_command_async(f"SET_CALIBRATION_POINT_4,{config_data['ir_high']},{config_data['od_high']:.3f}")
                        await asyncio.sleep(0.8)
                        arduino.read_data()  # Clear any pending responses
                        
                        # Send inverse calibration to Arduino (overwrites auto-calculated)
                        # Use inverse formula: OD = a/IR + b
                        print(f"📤 Dialog save: Sending inverse calibration coefficients")
                        a = config_data.get('inverseA', 0)
                        b = config_data.get('inverseB', 0)
                        print(f"   Inverse: a={a:.2f}, b={b:.4f} → OD = {a:.2f}/IR + {b:.4f}")
                        await arduino.send_command_async(f"SET_INVERSE_CALIBRATION,{a:.6f},{b:.6f}")
                        await asyncio.sleep(0.5)
                        # Send threshold with debug output
                        print(f"📤 Dialog save: Sending SET_THRESHOLD,{config_data['threshold']}")
                        await arduino.send_command_async(f"SET_THRESHOLD,{config_data['threshold']}")
                        await asyncio.sleep(0.8)  # Longer delay for threshold
                        # Send temperature with debug output
                        print(f"📤 Dialog save: Sending SET_TEMPERATURE,{config_data['temperature']}")
                        await arduino.send_command_async(f"SET_TEMPERATURE,{config_data['temperature']}")
                        await asyncio.sleep(0.8)  # Longer delay for temperature
                        for i, speed in enumerate(config_data['pump_speeds']):
                            await arduino.send_command_async(f"SET_PUMP_SPEED,{i+1},{speed}")
                            await asyncio.sleep(0.3)
                        await arduino.send_command_async(f"SET_STIRRING_SPEED,{config_data['stirring_speed']}")
                        await asyncio.sleep(0.3)
                        # Send max dispensations setting
                        max_disp_value = config_data.get('max_dispensations', 400)
                        print(f"📤 Sending SET_MAX_DISPENSATIONS,{max_disp_value}")
                        await arduino.send_command_async(f"SET_MAX_DISPENSATIONS,{max_disp_value}")
                        await asyncio.sleep(0.3)
                        # Send LED brightness for OD940 measurement LED
                        led_brightness_value = config_data.get('led_brightness', 500)
                        print(f"📤 Sending SET_LED_BRIGHTNESS,{led_brightness_value}")
                        await arduino.send_command_async(f"SET_LED_BRIGHTNESS,{led_brightness_value}")
                        await asyncio.sleep(0.3)
                        
                        # Extra delay before save to ensure all commands are processed
                        await asyncio.sleep(0.5)
                        # Persist to SD card so changes are written
                        print("📤 Sending SAVE_CONFIG_TO_SD command to Arduino...")
                        await arduino.send_command_async("SAVE_CONFIG_TO_SD")
                        await asyncio.sleep(1.0)  # Longer delay for SD write to complete
                        print("✅ Configuration save to SD card requested")
                        
                        config_updating = False  # Resume data plotting
                        ui.notify('✅ Configuration uploaded to Arduino!', type='positive')
                    else:
                        ui.notify('❌ Arduino not connected. Configuration saved locally.', type='warning')
                    
                    config_dialog.close()
                except Exception as e:
                    ui.notify(f'❌ Config save failed: {e}', type='negative')

            async def save_and_export_unified():
                """Save configuration both locally and to Arduino SD card"""
                try:
                    # Turn off any test LEDs before saving (exit calibration mode)
                    if arduino and arduino.connected:
                        await arduino.send_command_async("TEST_LED,1,0")  # Turn off all test LEDs
                        print("💡 Turned off test LEDs before saving")
                    
                    # First, save and upload all the new values to the Arduino's RAM.
                    await save_and_upload()
                    
                    # Then, wait a moment for the Arduino to process the new settings.
                    await asyncio.sleep(1.0) # CRITICAL DELAY
                    
                    # Save to Arduino SD card
                    if arduino.connected:
                        # Save main config to SD card
                        print("📤 Sending SAVE_CONFIG_TO_SD command to Arduino...")
                        arduino.send_command("SAVE_CONFIG_TO_SD")
                        await asyncio.sleep(2.0)  # INCREASED: Give Arduino more time to finish SD write
                        print("✅ Configuration save to SD card completed")
                        
                        # ===== SAVE LED CALIBRATION TO ARDUINO SD CARD =====
                        print("📤 Sending LED calibration to Arduino...")
                        print(f"   Current led_calibration dict: {led_calibration}")
                        has_nonzero_led = False
                        for led_num in range(1, 7):
                            intensity = led_calibration[led_num].get('intensity', 0)
                            wavelength_str = led_calibration[led_num].get('wavelength', '')
                            # Handle both string and numeric wavelength values
                            try:
                                wavelength = int(float(wavelength_str)) if wavelength_str else 0
                            except (ValueError, TypeError):
                                wavelength = 0
                            if intensity > 0:
                                has_nonzero_led = True
                            print(f"   LED{led_num}: intensity={intensity}, wavelength={wavelength} (from '{wavelength_str}')")
                            await arduino.send_command_async(f"SET_LED_CAL,{led_num},{intensity},{wavelength}")
                            await asyncio.sleep(0.2)  # INCREASED: More time between each LED cal
                        
                        if not has_nonzero_led:
                            print("⚠️ WARNING: All LED calibration values are 0! Did you enter values in the config dialog?")
                        
                        # Wait for all SET_LED_CAL commands to be processed
                        await asyncio.sleep(0.5)
                        
                        # Save LED calibration to SD card
                        print("📤 Sending SAVE_LED_CAL command to Arduino...")
                        await arduino.send_command_async("SAVE_LED_CAL")
                        await asyncio.sleep(1.5)  # INCREASED: Give SD card time to write
                        print("✅ LED calibration save to SD card completed")
                        
                        # Verify the save by requesting values back
                        print("📤 Verifying LED calibration was saved...")
                        arduino.send_command("GET_LED_CAL")
                        await asyncio.sleep(0.5)
                        
                        # ===== VERIFY CONFIG WAS SAVED CORRECTLY =====
                        print("📤 Verifying config was saved correctly to SD card...")
                        arduino.send_command("GET_CURRENT_STATE")
                        await asyncio.sleep(1.0)  # Wait for response
                        
                        # Read back the response
                        verification_lines = arduino.read_data()
                        config_verified = False
                        verification_errors = []
                        
                        for line in verification_lines:
                            if line.startswith("$STATE,"):
                                parts = line.split(",")
                                if len(parts) >= 23:
                                    try:
                                        # Parse calibration points from response
                                        # STATE format: ...,ledBrightness,irLow,odLow,irMid1,odMid1,irMid2,odMid2,irHigh,odHigh,...
                                        # Indices: 14=ledBrightness, 15=irLow, 16=odLow, 17=irMid1, etc.
                                        saved_ir_low = int(parts[15])
                                        saved_od_low = float(parts[16])
                                        saved_ir_mid1 = int(parts[17])
                                        saved_od_mid1 = float(parts[18])
                                        saved_ir_mid2 = int(parts[19])
                                        saved_od_mid2 = float(parts[20])
                                        saved_ir_high = int(parts[21])
                                        saved_od_high = float(parts[22])
                                        
                                        # Compare with what we sent
                                        if saved_ir_low != config_data['ir_low']:
                                            verification_errors.append(f"IR Low: sent {config_data['ir_low']}, got {saved_ir_low}")
                                        if abs(saved_od_low - config_data['od_low']) > 0.01:
                                            verification_errors.append(f"OD Low: sent {config_data['od_low']:.3f}, got {saved_od_low:.3f}")
                                        if saved_ir_mid1 != config_data['ir_mid1']:
                                            verification_errors.append(f"IR Mid1: sent {config_data['ir_mid1']}, got {saved_ir_mid1}")
                                        if abs(saved_od_mid1 - config_data['od_mid1']) > 0.01:
                                            verification_errors.append(f"OD Mid1: sent {config_data['od_mid1']:.3f}, got {saved_od_mid1:.3f}")
                                        if saved_ir_mid2 != config_data['ir_mid2']:
                                            verification_errors.append(f"IR Mid2: sent {config_data['ir_mid2']}, got {saved_ir_mid2}")
                                        if abs(saved_od_mid2 - config_data['od_mid2']) > 0.01:
                                            verification_errors.append(f"OD Mid2: sent {config_data['od_mid2']:.3f}, got {saved_od_mid2:.3f}")
                                        if saved_ir_high != config_data['ir_high']:
                                            verification_errors.append(f"IR High: sent {config_data['ir_high']}, got {saved_ir_high}")
                                        if abs(saved_od_high - config_data['od_high']) > 0.01:
                                            verification_errors.append(f"OD High: sent {config_data['od_high']:.3f}, got {saved_od_high:.3f}")
                                        
                                        config_verified = True
                                        print(f"✅ Verified calibration points from SD card:")
                                        print(f"   Point 1: IR={saved_ir_low}, OD={saved_od_low}")
                                        print(f"   Point 2: IR={saved_ir_mid1}, OD={saved_od_mid1}")
                                        print(f"   Point 3: IR={saved_ir_mid2}, OD={saved_od_mid2}")
                                        print(f"   Point 4: IR={saved_ir_high}, OD={saved_od_high}")
                                    except (ValueError, IndexError) as e:
                                        print(f"⚠️ Error parsing verification response: {e}")
                        
                        if verification_errors:
                            error_msg = "❌ CONFIG VERIFICATION FAILED:\n" + "\n".join(verification_errors)
                            print(error_msg)
                            # Show a very obvious error dialog that requires user acknowledgment
                            with ui.dialog() as error_dialog, ui.card().classes('items-center'):
                                ui.icon('error', size='80px', color='red')
                                ui.label('⚠️ CONFIGURATION SAVE FAILED!').classes('text-h4 text-negative text-weight-bold q-mt-md')
                                ui.label('Data corruption detected - the saved values do not match what was sent.').classes('text-subtitle1 q-mt-sm')
                                ui.separator().classes('q-my-md')
                                ui.label('Mismatched values:').classes('text-weight-bold')
                                for err in verification_errors:
                                    ui.label(f'• {err}').classes('text-negative')
                                ui.separator().classes('q-my-md')
                                ui.label('Please try saving again. If this persists, check SD card.').classes('text-subtitle2')
                                ui.button('OK - I Understand', on_click=error_dialog.close).props('color=negative size=lg').classes('q-mt-md')
                            error_dialog.open()
                        elif config_verified:
                            if has_nonzero_led:
                                ui.notify('✅ Config & LED calibration saved and VERIFIED!', type='positive')
                            else:
                                ui.notify('✅ Config saved and VERIFIED. LED calibration is all zeros.', type='warning')
                        else:
                            print("⚠️ Could not verify config save (no STATE response)")
                            if has_nonzero_led:
                                ui.notify('💾 Config saved (verification skipped)', type='info')
                            else:
                                ui.notify('💾 Config saved. LED calibration is all zeros.', type='warning')
                    else:
                        ui.notify('💾 Config saved locally (Arduino not connected for SD export)', type='warning')
                except Exception as e:
                    ui.notify(f'❌ Save & Export failed: {e}', type='negative')
            
            ui.button('💾 Save & Export Configuration', on_click=save_and_export_unified).props('color=primary')
    
    config_dialog.open()
# ─── UI Setup ───────────────────────────────────────────────────────────────────
def main_page():
    global start_button, connection_status, od_chart, peak_chart, status_labels, od_slope_label
    global x_min_input, x_max_input, y_min_input, y_max_input, ir_min_input, ir_max_input
    global peak_x_min_input, peak_x_max_input, peak_y_min_input, peak_y_max_input
    
    # No auto-connection - user must manually connect
    auto_connected = False
    
    # Browser close warning removed - no longer needed
    
    # ADD THIS CONTAINER TO CONTROL THE OVERALL PAGE WIDTH
    with ui.element('div').style('max-width: 1800px; width: 98%; margin: 0 auto;'):
        
        # Global Access Controls (at the top)
        with ui.card().classes('w-full q-mb-md'):
            # Header row with title and buttons
            with ui.row().classes('w-full items-center justify-between'):
               
                with ui.row().classes('items-center q-gutter-sm').style('margin-left: 100px;'):
                    async def toggle_global():
                        """Toggle global access on/off"""
                        global is_global
                        
                        if not is_global:
                            # Show password dialog first and wait for user response
                            proceed = await show_global_password_dialog()
                            if not proceed:
                                # User cancelled
                                ui.notify('❌ Global access cancelled', type='info')
                                return
                            
                            # Start global access
                            global_button.props('loading=true')
                            global_status_label.text = '🔄 Starting tunnel...'
                            
                            def start_in_thread():
                                try:
                                    result = start_tunnel()
                                    if result and result.startswith("http"):
                                        # Success - got a URL
                                        global is_global
                                        is_global = True
                                        global_status_label.text = f'🌍 Global Access: Online'
                                        global_url_input.value = result
                                        global_button.text = '🛑 Stop Global Access'
                                        global_button.props('color=negative loading=false')
                                        print(f'✅ Global access enabled! URL: {result}')
                                    else:
                                        # Error or failure
                                        global_status_label.text = '❌ Global Access: Failed'
                                        global_url_input.value = result or "Check console for details"
                                        global_button.props('loading=false')
                                        print(f'❌ Global access failed: {result}')
                                except Exception as e:
                                    global_status_label.text = '❌ Global Access: Error'
                                    global_url_input.value = f"Error: {str(e)}"
                                    global_button.props('loading=false')
                                    print(f"❌ Global access error: {e}")
                            
                            threading.Thread(target=start_in_thread, daemon=True).start()
                        else:
                            # Stop global access. V10 fix: this click may have
                            # arrived OVER the tunnel (i.e. from the remote browser).
                            # Killing the tunnel severs that browser's own websocket,
                            # so we must NOT block the event loop waiting for the kill
                            # inside this client-bound handler — doing so froze the
                            # whole server when stopping from the remote side. Instead:
                            # flip the UI state right now (event-loop thread, valid
                            # slot), then kill cloudflared in a detached daemon thread
                            # that is independent of this client's connection lifecycle.
                            is_global = False
                            global_status_label.text = '🔴 Global Access: Offline'
                            global_url_input.value = ''
                            global_button.text = '🌍 Go Global'
                            global_button.props('color=primary')
                            ui.notify('✅ Global access stopped', type='positive')

                            def _stop_tunnel_detached():
                                try:
                                    stop_tunnel()
                                except Exception as _e:
                                    print(f"⚠️ Tunnel stop error: {_e}")
                            threading.Thread(target=_stop_tunnel_detached, daemon=True).start()
                    
                    global_button = ui.button('🌍 Go Global', on_click=toggle_global).props('color=primary').style('margin-left: 0px;')
                    global_status_label = ui.label('🔴 Global Access: Offline').classes('text-weight-bold').style('margin-right: 10px;')
                    

            with ui.row().classes('w-full items-center q-gutter-sm'):
                ui.label('Global URL:').classes('text-weight-bold')
                global_url_input = ui.input(
                    placeholder='Click "Go Global" to generate URL...'
                ).props('readonly').style('flex-grow: 1;')
                
                def copy_url():
                    if global_url_input.value:
                        url_safe = json.dumps(global_url_input.value)
                        ui.run_javascript(f'navigator.clipboard.writeText({url_safe}); alert("URL copied to clipboard!");')
                    else:
                        ui.notify('No URL to copy', type='warning')
                
                ui.button('📋 Copy', on_click=copy_url).props('color=primary size=md').style('margin-right: 272px;')

        
                    # This is the new simplified header/control area
            with ui.card().classes('w-full q-mb-md'):
                # Row 1: Configuration and data buttons
                with ui.row().classes('w-full justify-start items-center q-gutter-sm q-mb-lg'):                                
                    ui.button('Configure OpenEvo', on_click=show_config_dialog).props('color=primary')
                    ui.button('Cycle Program', on_click=show_cycle_editor).props('color=primary')
                    ui.button('Sync Clocks', on_click=lambda: sync_rtc_time()).props('color=info icon=schedule')
                    ui.button('📊 Load CSV', on_click=show_file_picker).props('color=accent')
                
                # Row 2: Control buttons and status indicators
                with ui.row().classes('w-full justify-start items-center q-gutter-sm q-mb-md'):
                    # Start button (aligned with other buttons)
                    initial_text = 'Pause' if auto_connected else 'Connect'
                    initial_color = 'warning' if auto_connected else 'secondary'
                    global start_button
                    start_button = ui.button(initial_text, on_click=start_stop_experiment).props(f'color={initial_color} icon={"pause" if auto_connected else "link"}')
                    
                    # Status indicators
                    connection_status = ui.label('🔴 DISCONNECTED').classes('text-weight-bold')
                    global sd_card_status_label
                    sd_card_status_label = ui.label('💾 SD: Waiting for Arduino...').classes('text-weight-bold text-grey')
                    
                    # Close tab button
                    with ui.row().classes('items-center'):
                        
                        def close_tab_only():
                            """Close browser tab but leave server running"""
                            ui.run_javascript('window.close();')
                            ui.notify('If tab did not close, please close it manually. OpenEvo continues running.', type='info')
                        
                        async def shutdown_system():
                            """Show confirmation dialog for full shutdown"""
                            with ui.dialog() as shutdown_dialog, ui.card():
                                ui.label('Shutdown OpenEvo System?').classes('text-h6')
                                ui.label('This will stop the experiment, disconnect Arduino, and shut down the server.')
                                
                                async def do_shutdown():
                                    try:
                                        ui.notify('🔄 Shutting down OpenEvo system...', type='info')
                                        print("🔄 Starting complete system shutdown...")
                                        
                                        # Stop experiment and close Arduino
                                        if arduino and arduino.connected:
                                            try:
                                                arduino.send_command("SD_SYNC")  # Sync SD card
                                                time.sleep(0.5)  # Give time for sync
                                                arduino.send_command("STOP")
                                                time.sleep(0.5)  # Give time for stop
                                                arduino.close()
                                                print("✅ Arduino disconnected")
                                            except Exception as e:
                                                print(f"⚠️ Arduino shutdown error: {e}")
                                        
                                        # Stop tunnel only if it was started
                                        if is_global:
                                            try:
                                                stop_tunnel()
                                                print("✅ Tunnel stopped")
                                            except Exception as e:
                                                  print(f"⚠️ Tunnel stop error: {e}")
                                        # No message needed when no tunnel was active
                                        
                                        # Close any open files
                                        global csv_file, csv_writer
                                        if csv_file:
                                            try:
                                                if csv_writer:
                                                    csv_writer = None
                                                csv_file.close()
                                                csv_file = None
                                                print("✅ CSV file closed")
                                            except Exception as e:
                                                print(f"⚠️ CSV close error: {e}")
                                        
                                        # Close browser tab with multiple methods
                                        try:
                                            await ui.run_javascript('''
                                                // Clear all cached data
                                                if ('caches' in window) {
                                                    caches.keys().then(function(names) {
                                                        for (let name of names) caches.delete(name);
                                                    });
                                                }
                                                
                                                // Clear localStorage and sessionStorage
                                                try { localStorage.clear(); } catch(e) {}
                                                try { sessionStorage.clear(); } catch(e) {}
                                                
                                                // Show shutdown message with prominent close button
                                                document.body.innerHTML = '<div style="text-align: center; margin-top: 50px; font-family: Arial; background: #f44336; color: white; padding: 20px; min-height: 100vh;"><h1>🔌 OpenEvo System Shutdown</h1><p style="font-size: 18px; margin: 20px 0;">System is shutting down...<br/>Cache cleared. This tab will close automatically.</p><div style="margin: 30px 0;"><button onclick="window.close(); window.location.href=\\'about:blank\\'" style="background: white; color: #f44336; border: 2px solid white; padding: 15px 30px; border-radius: 8px; cursor: pointer; font-size: 18px; font-weight: bold; box-shadow: 0 4px 8px rgba(0,0,0,0.3);">CLOSE TAB NOW</button></div><p style="font-size: 14px; opacity: 0.8;">If the tab doesn\\'t close automatically, click the button above</p></div>';
                                                
                                                // Multiple aggressive attempts to close the tab
                                                let attempts = 0;
                                                const maxAttempts = 15;
                                                
                                                function attemptClose() {
                                                    attempts++;
                                                    console.log('Attempt ' + attempts + ' to close tab...');
                                                    
                                                    // Try window.close() first
                                                    try {
                                                        window.close();
                                                        if (window.closed) return; // Success!
                                                    } catch(e) {
                                                        console.log('window.close() failed:', e);
                                                    }
                                                    
                                                    // Try to navigate away
                                                    try {
                                                        window.location.href = 'about:blank';
                                                    } catch(e) {
                                                        console.log('location.href failed:', e);
                                                    }
                                                    
                                                    // Try to replace the current page
                                                    try {
                                                        window.location.replace('about:blank');
                                                    } catch(e) {
                                                        console.log('location.replace failed:', e);
                                                    }
                                                    
                                                    // Try history manipulation
                                                    try {
                                                        if (window.history.length > 1) {
                                                            window.history.back();
                                                        }
                                                        window.close();
                                                    } catch(e) {
                                                        console.log('history manipulation failed:', e);
                                                    }
                                                    
                                                    // Try opening a new window and closing this one
                                                    try {
                                                        const newWin = window.open('about:blank', '_self');
                                                        if (newWin) {
                                                            newWin.close();
                                                        }
                                                        window.close();
                                                    } catch(e) {
                                                        console.log('new window method failed:', e);
                                                    }
                                                    
                                                    // If we haven't succeeded and haven't reached max attempts, try again
                                                    if (attempts < maxAttempts) {
                                                        setTimeout(attemptClose, 300);
                                                    } else {
                                                        console.log('All close attempts failed. Tab must be closed manually.');
                                                        // Make the close button flash to get attention
                                                        const btn = document.querySelector('button');
                                                        if (btn) {
                                                            setInterval(() => {
                                                                btn.style.background = btn.style.background === 'yellow' ? 'white' : 'yellow';
                                                            }, 500);
                                                        }
                                                    }
                                                }
                                                
                                                // Start attempting to close immediately
                                                setTimeout(attemptClose, 500);
                                                
                                                // Also try to trigger browser's beforeunload
                                                setTimeout(() => {
                                                    try {
                                                        window.onbeforeunload = null;
                                                        window.close();
                                                    } catch(e) {}
                                                }, 1500);
                                            ''')
                                            print("✅ Browser shutdown message displayed and cache cleared")
                                        except Exception as e:
                                            print(f"⚠️ Browser close error: {e}")
                                        
                                        # Wait a moment for cleanup
                                        await asyncio.sleep(1.0)
                                        
                                        print("🔄 Forcing application exit...")
                                        
                                        # Multiple shutdown approaches for reliability
                                        try:
                                            app.shutdown()
                                        except:
                                            pass
                                        
                                        # Kill any related processes
                                        try:
                                            import subprocess
                                            import os
                                            
                                            # Get current process ID
                                            current_pid = os.getpid()
                                            print(f"🔍 Current process PID: {current_pid}")
                                            
                                            # Kill related processes (all OpenEvo interface versions)
                                            # Kill any OpenEvo interface processes
                                            subprocess.run(['pkill', '-f', 'python.*OpenEvo_Interface'], capture_output=True)
                                            subprocess.run(['pkill', '-f', 'cloudflared'], capture_output=True)
                                            print("🔍 Killed all OpenEvo interface processes")
                                            
                                            # Kill processes using port 8080
                                            result = subprocess.run(['lsof', '-ti:8080'], capture_output=True, text=True)
                                            if result.stdout.strip():
                                                pids = result.stdout.strip().split('\n')
                                                for pid in pids:
                                                    try:
                                                        subprocess.run(['kill', '-9', pid], capture_output=True)
                                                        print(f"✅ Killed process {pid}")
                                                    except:
                                                        pass
                                                        
                                        except Exception as kill_error:
                                            print(f"⚠️ Process kill error: {kill_error}")
                                        
                                        # Force exit with multiple methods
                                        import os
                                        import sys
                                        print("💀 Force exit with os._exit(0)")
                                        
                                        # Try graceful shutdown first
                                        try:
                                            sys.exit(0)
                                        except:
                                            pass
                                            
                                        # Force exit
                                        os._exit(0)
                                        
                                    except Exception as e:
                                        print(f"❌ Shutdown error: {e}")
                                        # Force exit even if there's an error
                                        import os
                                        print("💀 Emergency force exit")
                                        os._exit(0)
                                
                                with ui.row():
                                    ui.button('Cancel', on_click=shutdown_dialog.close).props('flat')
                                    ui.button('Shutdown', on_click=do_shutdown).props('color=negative')
                            
                            shutdown_dialog.open()
                        
                        # Custom close tab warning dialog
                        with ui.dialog() as close_warning_dialog, ui.card():
                            ui.label('⚠️ Close Tab?').classes('text-h6')
                            ui.label('This will close the tab on ALL connected devices (including remote connections).').classes('text-body2 q-mt-sm')
                            ui.label('The OpenEvo system will continue running in the background.').classes('text-body2 text-grey-7')
                            
                            with ui.row().classes('q-mt-md'):
                                ui.button('Cancel', on_click=close_warning_dialog.close).props('flat')
                                ui.button('Close Tab Anyway', on_click=lambda: ui.run_javascript('window.close();')).props('color=negative')
                        
                        ui.button('🚪 Close Tab', on_click=close_warning_dialog.open).props('color=red size=md')
                        
                # Row 3: Runtime operations (Skip, Jump, Manual Pump) and LCD Mirror
                with ui.row().classes('w-full justify-start items-center q-gutter-xs q-mt-xs'):
                    ui.button('Skip (Media)', on_click=skip_with_media).props('color=warning')
                    ui.button('Skip (No Media)', on_click=skip_no_media).props('color=warning')
                    ui.button('Jump', on_click=show_jump_dialog).props('color=warning')
                    ui.button('Manual Pump', on_click=show_pump_control).props('color=warning')
                    
                    # LCD Mirror display
                    global lcd_mirror
                    with ui.card().classes('q-ml-md').style('background: #000; color: #0f0; font-family: monospace; padding: 6px; border: 2px solid #333; border-radius: 5px; min-width: 200px;'):
                        lcd_mirror = ui.label('Waiting for Arduino...').classes('text-weight-bold').style('color: #0f0; font-size: 12px; line-height: 1.1; white-space: pre-line; margin: 0;')



    # Unified Chart Axis Controls (above both charts)
    with ui.card().classes('w-full q-mb-md'):
        with ui.row().classes('w-full items-start q-gutter-lg'):
            # Functions to update charts immediately when inputs change
            def update_od_chart_axes():
                """Update OD chart axes immediately when any input changes."""
                global od_chart_x_range
                try:
                    if od_chart:
                        od_chart.options['xAxis']['min'] = x_min_input.value or 0
                        od_chart.options['xAxis']['max'] = x_max_input.value or 24
                        od_chart.options['yAxis'][0]['min'] = y_min_input.value or 0
                        od_chart.options['yAxis'][0]['max'] = y_max_input.value or 6
                        od_chart.options['yAxis'][1]['min'] = ir_min_input.value or 0
                        od_chart.options['yAxis'][1]['max'] = ir_max_input.value or 30000
                        od_chart_x_range = [x_min_input.value or 0, x_max_input.value or 24]
                        od_chart.update()
                except Exception as e:
                    print(f"OD chart axis update error: {e}")
            
            def update_peak_chart_axes():
                """Update Peak chart axes immediately when any input changes."""
                try:
                    if peak_chart:
                        peak_chart.options['xAxis']['min'] = peak_x_min_input.value or 0
                        peak_chart.options['xAxis']['max'] = peak_x_max_input.value or 50
                        peak_chart.options['yAxis']['min'] = peak_y_min_input.value or 0
                        peak_chart.options['yAxis']['max'] = peak_y_max_input.value or 50
                        peak_chart.update()
                except Exception as e:
                    print(f"Peak chart axis update error: {e}")
            
            # OD Chart Axes - with immediate updates
            with ui.column().classes('q-gutter-xs'):
                ui.label('OD Chart Axes:').classes('text-weight-bold q-mb-xs').style('margin-right: 0px;')
                with ui.row().classes('q-gutter-sm'):
                    x_min_input = ui.number('X-Min (hr)', value=0).props('dense style=width:80px step=0.001').style('margin-right: 200px;')
                    x_min_input.on('update:model-value', lambda: update_od_chart_axes())
                    x_max_input = ui.number('X-Max (hr)', value=24).props('dense style=width:80px step=0.001')
                    x_max_input.on('update:model-value', lambda: update_od_chart_axes())
                    y_min_input = ui.number('Y-Min (OD)', value=0).props('dense style=width:80px step=0.001')
                    y_min_input.on('update:model-value', lambda: update_od_chart_axes())
                    y_max_input = ui.number('Y-Max (OD)', value=6).props('dense style=width:80px step=0.001')
                    y_max_input.on('update:model-value', lambda: update_od_chart_axes())
                with ui.row().classes('q-gutter-sm items-center'):
                    ir_min_input = ui.number('NIR-Min', value=0).props('dense style=width:80px step=100')
                    ir_min_input.on('update:model-value', lambda: update_od_chart_axes())
                    ir_max_input = ui.number('NIR-Max', value=30000).props('dense style=width:80px step=1000')
                    ir_max_input.on('update:model-value', lambda: update_od_chart_axes())
            
            # Peak Chart Axes - with immediate updates
            with ui.column().classes('q-gutter-xs'):
                ui.label('Peak Chart Axes:').classes('text-weight-bold q-mb-xs')
                with ui.row().classes('q-gutter-sm'):
                    # <----- PEAK INTERVAL CHART AXIS DEFAULTS (Live View)
                    peak_x_min_input = ui.number('X-Min (Peak #)', value=0).props('dense style=width:80px step=1')
                    peak_x_min_input.on('update:model-value', lambda: update_peak_chart_axes())
                    peak_x_max_input = ui.number('X-Max (Peak #)', value=50).props('dense style=width:80px step=1')
                    peak_x_max_input.on('update:model-value', lambda: update_peak_chart_axes())
                    peak_y_min_input = ui.number('Y-Min (Interval)', value=0).props('dense style=width:80px step=0.1')
                    peak_y_min_input.on('update:model-value', lambda: update_peak_chart_axes())
                    peak_y_max_input = ui.number('Y-Max (Interval)', value=50).props('dense style=width:80px step=0.1')
                    peak_y_max_input.on('update:model-value', lambda: update_peak_chart_axes())

        # Status Bar - Two rows, centered with more spacing
    with ui.card().classes('w-full q-mb-md').style('background: #f5f5f5; padding: 12px 24px;'):
        status_labels = {}
        # Row 1: Experiment status + LED (centered)
        with ui.row().classes('w-full justify-center items-center gap-6').style('margin-bottom: 8px;'):
            status_labels['cycle'] = ui.label('Cycle --').classes('text-weight-bold')
            status_labels['step'] = ui.label('Step --').classes('text-weight-bold')
            status_labels['media'] = ui.label('--').classes('text-weight-bold')
            ui.label('|').classes('text-grey-5')
            status_labels['led'] = ui.label('LED --').classes('text-weight-bold')
            status_labels['light_dose'] = ui.label('').classes('text-weight-bold').tooltip(
                'Effective dose = LED% × calibrated intensity × 88.5% duty cycle'
            )
        # Row 2: OD/IR on left + Temperatures + Heater PWM (centered)
        with ui.row().classes('w-full justify-center items-center gap-6'):
            status_labels['od'] = ui.label('OD --').classes('text-weight-bold').style('color: #1976d2;')
            status_labels['ir'] = ui.label('IR --').classes('text-weight-bold').style('color: #1976d2;')
            ui.label('|').classes('text-grey-5')
            status_labels['temp'] = ui.label('Media --°C').classes('text-weight-bold')
            status_labels['ambient'] = ui.label('Ambient --°C').classes('text-weight-bold')
            status_labels['heater'] = ui.label('Heater --°C').classes('text-weight-bold').style('color: red')
            status_labels['pwm'] = ui.label('Heater PWM --').classes('text-weight-bold').style('color: red')

    # OD Chart
    with ui.card().classes('w-full q-mb-md'):
        # Header row with slope and reset button
        with ui.row().classes('w-full justify-between items-center').style('margin-bottom: 10px;'):
            ui.element('div').style('width: 100px;')  # Spacer
            od_slope_label = ui.label('Slope: --').classes('text-weight-bold text-h6')
            ui.button('🔄 Reset Zoom', on_click=reset_od_chart_zoom).props('size=sm flat').style('margin-right: 10px;')
        
        global od_chart
        od_chart = ui.echart({
            'tooltip': {
                'trigger': 'axis',
                'axisPointer': {
                    'type': 'cross'
                }
            },
            'xAxis': {
                'type': 'value', 
                'name': 'Time (hours)', 
                'nameLocation': 'middle', 
                'nameGap': 35,
                'nameTextStyle': {'fontSize': 14, 'fontWeight': 'bold', 'color': '#333'},
                'axisLabel': {'fontSize': 12, 'color': '#666'}
            },
            'yAxis': [
                {
                    'type': 'value', 
                    'name': 'OD940', 
                    'min': 0, 
                    'max': 6, 
                    'nameLocation': 'middle', 
                    'nameGap': 50,
                    'nameTextStyle': {'fontSize': 14, 'fontWeight': 'bold', 'color': '#333'},
                    'axisLabel': {'fontSize': 12, 'color': '#666'},
                    'position': 'left'
                },
                {
                    'type': 'value', 
                    'name': 'NIR', 
                    'min': 0, 
                    'max': 25000, 
                    'nameLocation': 'middle', 
                    'nameGap': 50,
                    'nameTextStyle': {'fontSize': 14, 'fontWeight': 'bold', 'color': '#333'},
                    'axisLabel': {'fontSize': 12, 'color': '#666'},
                    'position': 'right'
                }
            ],
            'legend': {
                'data': [
                    {'name': 'NIR', 'itemStyle': {'color': '#0066FF'}},
                    {'name': 'OD940', 'itemStyle': {'color': '#00AA00'}}
                ],
                'top': 10,
                'left': 'center'
            },
            'series': [{
                'id': 'od_series',
                'data': [],
                'type': 'line',
                'name': 'OD940',
                'smooth': True,
                'symbol': 'none',
                'lineStyle': {'color': 'black', 'width': 2},
                'markArea': { 'data': [], 'silent': True }
            }, {
                'id': 'ir_series',
                'data': [],
                'type': 'line',
                'name': 'NIR',
                'smooth': True,
                'symbol': 'none',
                'lineStyle': {'color': '#0066FF', 'width': 2},
                'yAxisIndex': 1
            }, {
                'id': 'time_series',
                'data': [],
                'type': 'line',
                'name': 'Time & Date',
                'smooth': True,
                'symbol': 'none',
                'lineStyle': {'opacity': 0},
                'itemStyle': {'opacity': 0},
                'legendHoverLink': False,
                'showSymbol': False
            }, {
                'id': 'unix_series',
                'data': [],
                'type': 'line',
                'name': 'Unix Time',
                'smooth': True,
                'symbol': 'none',
                'lineStyle': {'opacity': 0},
                'itemStyle': {'opacity': 0},
                'legendHoverLink': False,
                'showSymbol': False
            }, {
                'id': 'threshold_series',
                'data': [[0, config_data['threshold']], [24, config_data['threshold']]],
                'type': 'line',
                'lineStyle': {'type': 'dashed', 'color': 'red', 'width': 2},
                'itemStyle': {'color': 'red'},
                'symbol': 'none',
                'silent': True,
                'showInLegend': False
            }],
            'grid': {
                'left': 90,
                'right': 90,
                'top': 40,
                'bottom': 90,
                'containLabel': False
            },
            'dataZoom': [
                {'type': 'inside', 'xAxisIndex': 0},
                {'type': 'slider', 'xAxisIndex': 0, 'bottom':5, 'height': 25}
            ]
        }).classes('w-full h-96')
        
        # Simple tooltip - no custom JavaScript needed
        
        
    # Peak Chart
    with ui.card().classes('w-full q-mb-md'):
        # Header row with title and reset button
        with ui.row().classes('w-full justify-between items-center').style('margin-bottom: 10px;'):
            ui.element('div').style('width: 100px;')  # Spacer
            ui.label('Peak-to-Peak Intervals').classes('text-h6 text-weight-bold')
            ui.button('🔄 Reset Zoom', on_click=reset_peak_chart_zoom).props('size=sm flat').style('margin-right: 10px;')
        
        global peak_chart
        peak_chart = ui.echart({
            'tooltip': {
                'trigger': 'item',
                'backgroundColor': 'rgba(255, 255, 255, 0.95)',
                'borderColor': '#ccc',
                'borderWidth': 1,
                'padding': [10, 15],
                'textStyle': {'color': '#333', 'fontSize': 13},
                'extraCssText': 'box-shadow: 0 2px 8px rgba(0,0,0,0.15); border-radius: 6px;'
            },
            'xAxis': {
                'type': 'value', 
                'name': 'Interval Number', 
                'nameLocation': 'middle', 
                'nameGap': 35,
                'min': 0,
                'max': 10,
                'splitNumber': 5,
                'nameTextStyle': {'fontSize': 14, 'fontWeight': 'bold', 'color': '#333'},
                'axisLabel': {'fontSize': 12, 'color': '#666'}
            },
            'yAxis': {
                'type': 'value', 
                'name': 'Interval (hours)', 
                'nameLocation': 'middle', 
                'nameGap': 50,
                'min': 0,
                'max': 24,
                'splitNumber': 4,
                'nameTextStyle': {'fontSize': 14, 'fontWeight': 'bold', 'color': '#333'},
                'axisLabel': {'fontSize': 12, 'color': '#666'}
            },
            'grid': {
                'left': 90,
                'right': 90,
                'top': 40,
                'bottom': 90,
                'containLabel': False,
                'show': True,
                'borderWidth': 1,
                'backgroundColor': 'rgba(0,0,0,0.02)'
            },
            'series': [{
                'id': 'peak_series', 
                'data': [],
                'type': 'line',
                'symbol': 'circle',
                'symbolSize': 6
            }],
            'dataZoom': [
                {'type': 'inside', 'xAxisIndex': 0},
                {'type': 'slider', 'xAxisIndex': 0, 'bottom': 5, 'height': 25}
            ]
        }).classes('w-full h-96')


def read_total_steps_from_cycler():
    """Read the total number of steps from cycler.csv file."""
    global total_program_steps
    try:
        cycler_path = os.path.join(os.getcwd(), 'cycler.csv')
        if os.path.exists(cycler_path):
            with open(cycler_path, 'r') as f:
                lines = f.readlines()
                # Skip header line, count data lines
                data_lines = [line.strip() for line in lines[1:] if line.strip()]
                if data_lines:
                    total_program_steps = len(data_lines)
                    print(f"📊 Read {total_program_steps} steps from cycler.csv")
                else:
                    print("⚠️ No data lines found in cycler.csv, using default")
        else:
            print("⚠️ cycler.csv not found, using default total_program_steps")
    except Exception as e:
        print(f"❌ Error reading cycler.csv: {e}")
async def load_config_from_arduino_sd():
    """Load configuration that Arduino loaded from its SD card on startup"""
    global config_data, led_calibration
    
    if not arduino.connected:
        print("⚠️ Arduino not connected, cannot load SD config")
        return False
    
    try:
        print("🔍 Loading config from Arduino (which loaded from SD card on startup)...")
        # Arduino automatically loads config from SD card on startup
        # We just need to get the current state which reflects that loaded config
        success = await load_config_from_arduino()
        
        # Also load LED calibration from Arduino's SD card
        print("🔍 Loading LED calibration from Arduino's SD card...")
        await load_led_calibration_from_arduino()
        
        if success:
            print("✅ Successfully loaded config from Arduino's SD card")
            # Also save this config to local file for backup (NOT including LED calibration - that's on SD card only)
            try:
                save_data = config_data.copy()
                # Don't save LED calibration to local file - it's on Arduino SD card
                with open('oevo_config.json', 'w') as f:
                    json.dump(save_data, f, indent=2)
                print("💾 Backed up SD card config to local file")
            except Exception as e:
                print(f"⚠️ Could not backup config to local file: {e}")
        return success
    except Exception as e:
        print(f"❌ Error loading config from Arduino SD card: {e}")
        return False

async def load_led_calibration_from_arduino():
    """Load LED calibration values from Arduino's SD card"""
    global led_calibration
    
    if not arduino.connected:
        print("⚠️ Arduino not connected, cannot load LED calibration")
        return False
    
    try:
        print("🔍 Requesting LED calibration from Arduino...")
        arduino.send_command("GET_LED_CAL")
        
        # Wait for response
        capture_active = False
        for i in range(10):  # Wait up to 2 seconds
            await asyncio.sleep(0.2)
            data = arduino.read_data()
            if data:
                for line in data:
                    line = line.strip()
                    if line == "$LED_CAL_START":
                        capture_active = True
                        print("📥 Receiving LED calibration data...")
                    elif line == "$LED_CAL_END":
                        capture_active = False
                        print("✅ LED calibration loaded from Arduino SD card")
                        return True
                    elif capture_active and line.startswith("$LED_CAL,"):
                        # Parse: $LED_CAL,<channel>,<intensity>,<wavelength>
                        parts = line.split(',')
                        if len(parts) >= 4:
                            try:
                                channel = int(parts[1])
                                intensity = float(parts[2])
                                wavelength = int(parts[3]) if parts[3] else 0
                                if 1 <= channel <= 6:
                                    led_calibration[channel] = {
                                        'intensity': intensity,
                                        'wavelength': str(wavelength) if wavelength > 0 else ''
                                    }
                                    if intensity > 0:
                                        print(f"   LED{channel}: {intensity} mW/cm² @ {wavelength}nm")
                            except (ValueError, IndexError) as e:
                                print(f"⚠️ Error parsing LED calibration: {e}")
        
        print("⚠️ No LED calibration response from Arduino (using defaults)")
        return False
    except Exception as e:
        print(f"❌ Error loading LED calibration from Arduino: {e}")
        return False

async def load_config_from_arduino():
    """Load configuration values from Arduino's memory (which loads from SD card on startup)"""
    global config_data
    
    if not arduino.connected:
        print("⚠️ Arduino not connected, cannot load config")
        return False
    
    try:
        print("🔍 Requesting current state from Arduino...")
        # Request current state which includes threshold and temperature
        arduino.send_command("GET_CURRENT_STATE")
        
        # Wait for response
        for i in range(10):  # Wait up to 2 seconds
            await asyncio.sleep(0.2)
            data = arduino.read_data()
            if data:
                for line in data:
                    print(f"🔍 Received line: {line.strip()}")
                    if line.startswith("$STATE,"):
                        print(f"🔍 Parsing STATE line: {line.strip()}")
                        # Parse state: $STATE,isRunning,isPaused,currentCycle,currentStep,currentLEDPin,currentLEDBrightness,currentLEDState,currentMediaType,dynamicThreshold,incubationSetpointTemp,totalProgramSteps,maxDispensations,stirring_speed,led_brightness,irLower,odLower,irMidLow,odMidLow,irMidHigh,odMidHigh,irUpper,odUpper,pumpSpeed1,pumpSpeed2,pumpSpeed3,pumpSpeed4
                        parts = line.split(',')
                        print(f"🔍 STATE parts count: {len(parts)}")
                        
                        # VALIDATION: Check for truncated/corrupt data
                        # Last part should be a complete number, not truncated
                        if parts and parts[-1].strip() == '':
                            print(f"⚠️ STATE line appears truncated (empty last field), skipping...")
                            continue
                        
                        if len(parts) >= 23:  # Backward compatible with current firmware
                            try:
                                # Original fields (indices 0-12)
                                threshold = float(parts[9])
                                temperature = float(parts[10])
                                max_dispensations = int(parts[12])
                                
                                # New config fields (indices 13-22)
                                stirring_speed = int(parts[13])
                                led_brightness = int(parts[14])
                                
                                # Adaptive parsing for old (23/25) vs new (27) firmware
                                if len(parts) >= 27:
                                    # New firmware with 4-point calibration
                                    ir_low = int(parts[15])      # irLower
                                    od_low = float(parts[16])    # odLower
                                    ir_mid1 = int(parts[17])     # irMidLow
                                    od_mid1 = float(parts[18])   # odMidLow
                                    ir_mid2 = int(parts[19])     # irMidHigh
                                    od_mid2 = float(parts[20])   # odMidHigh
                                    ir_high = int(parts[21])     # irUpper
                                    od_high = float(parts[22])   # odUpper
                                    pump_speed_1 = int(parts[23])
                                    pump_speed_2 = int(parts[24])
                                    pump_speed_3 = int(parts[25])
                                    pump_speed_4 = int(parts[26])
                                elif len(parts) >= 25:
                                    # Old firmware with 3-point calibration
                                    ir_low = int(parts[15])      # irLower
                                    od_low = float(parts[16])    # odLower
                                    ir_mid1 = int(parts[17])     # irMid
                                    od_mid1 = float(parts[18])   # odMid
                                    ir_mid2 = 0                  # Not available
                                    od_mid2 = 0.0                # Not available
                                    ir_high = int(parts[19])     # irUpper
                                    od_high = float(parts[20])   # odUpper
                                    pump_speed_1 = int(parts[21])
                                    pump_speed_2 = int(parts[22])
                                    pump_speed_3 = int(parts[23])
                                    pump_speed_4 = int(parts[24])
                                else:
                                    # Very old firmware with 2-point calibration
                                    ir_low = int(parts[15])      # irLower
                                    od_low = float(parts[16])    # odLower
                                    ir_mid1 = 0                  # Not available
                                    od_mid1 = 0.0                # Not available
                                    ir_mid2 = 0                  # Not available
                                    od_mid2 = 0.0                # Not available
                                    ir_high = int(parts[17])     # irUpper
                                    od_high = float(parts[18])   # odUpper
                                    pump_speed_1 = int(parts[19])
                                    pump_speed_2 = int(parts[20])
                                    pump_speed_3 = int(parts[21])
                                    pump_speed_4 = int(parts[22])
                                
                                # Update local config with Arduino's values
                                config_data['threshold'] = threshold
                                config_data['temperature'] = temperature
                                config_data['max_dispensations'] = max_dispensations
                                config_data['stirring_speed'] = stirring_speed
                                config_data['led_brightness'] = led_brightness
                                # 4-point calibration data
                                config_data['ir_low'] = ir_low       # irLower (Point 1)
                                config_data['od_low'] = od_low       # odLower
                                config_data['ir_mid1'] = ir_mid1     # irMidLow (Point 2)
                                config_data['od_mid1'] = od_mid1     # odMidLow
                                config_data['ir_mid2'] = ir_mid2     # irMidHigh (Point 3)
                                config_data['od_mid2'] = od_mid2     # odMidHigh
                                config_data['ir_high'] = ir_high     # irUpper (Point 4)
                                config_data['od_high'] = od_high     # odUpper
                                # Backward compatibility
                                config_data['ir_zero'] = ir_low
                                config_data['od_zero'] = od_low
                                config_data['ir_mid'] = ir_mid2 if ir_mid2 > 0 else ir_mid1  # Use mid2 if available, else mid1
                                config_data['od_mid'] = od_mid2 if od_mid2 > 0 else od_mid1
                                config_data['ir_target'] = ir_high
                                config_data['od_target'] = od_high
                                config_data['pump_speeds'] = [pump_speed_1, pump_speed_2, pump_speed_3, pump_speed_4]
                                
                                # VALIDATION: Only catch obviously corrupt data (negative or impossibly large)
                                # We use very wide ranges to avoid rejecting valid configs
                                values_valid = True
                                validation_errors = []
                                
                                # Check for negative values (always invalid)
                                if threshold < 0:
                                    validation_errors.append(f"threshold={threshold} negative")
                                    values_valid = False
                                if temperature < 0:
                                    validation_errors.append(f"temperature={temperature} negative")
                                    values_valid = False
                                
                                # Check IR values (should be positive, max 65535 for 16-bit ADC)
                                for name, val in [('ir_low', ir_low), ('ir_high', ir_high)]:
                                    if val < 0:
                                        validation_errors.append(f"{name}={val} negative")
                                        values_valid = False
                                
                                # Check OD values (should be non-negative)
                                for name, val in [('od_low', od_low), ('od_high', od_high)]:
                                    if val < 0:
                                        validation_errors.append(f"{name}={val} negative")
                                        values_valid = False
                                
                                # Check pump speeds (should be non-negative)
                                for name, val in [('pump1', pump_speed_1), ('pump2', pump_speed_2), 
                                                  ('pump3', pump_speed_3), ('pump4', pump_speed_4)]:
                                    if val < 0:
                                        validation_errors.append(f"{name}={val} negative")
                                        values_valid = False
                                
                                if not values_valid:
                                    print(f"⚠️ CONFIG VALIDATION FAILED: {', '.join(validation_errors)}")
                                    print(f"⚠️ Keeping existing config to prevent corruption")
                                    return False
                                
                                # Recalculate calibration from loaded values
                                calculate_and_update_calibration()
                                
                                print(f"📋 Loaded ALL config from Arduino (validated):")
                                print(f"   threshold={threshold}, temperature={temperature}°C, max_disp={max_dispensations}")
                                print(f"   stirring_speed={stirring_speed}, led_brightness={led_brightness}")
                                print(f"   calibration (4-point): ir_low={ir_low}, od_low={od_low}")
                                print(f"                          ir_mid1={ir_mid1}, od_mid1={od_mid1}")
                                print(f"                          ir_mid2={ir_mid2}, od_mid2={od_mid2}")
                                print(f"                          ir_high={ir_high}, od_high={od_high}")
                                print(f"   pump_speeds=[{pump_speed_1}, {pump_speed_2}, {pump_speed_3}, {pump_speed_4}]")
                                if config_data.get('useInverse', False):
                                    print(f"   📐 Using INVERSE calibration: OD = {config_data.get('inverseA', 0):.2f}/IR + {config_data.get('inverseB', 0):.4f}")
                                    print(f"   R² = {config_data.get('r_squared', 0):.6f} (goodness of fit)")
                                else:
                                    print(f"   📐 Using LINEAR fallback: slope={config_data['slope']:.8f}, intercept={config_data['intercept']:.6f}")
                                return True
                            except (ValueError, IndexError) as e:
                                print(f"⚠️ Error parsing Arduino config: {e}")
                                print(f"   Parts: {parts}")
                                continue
                        else:
                            print(f"⚠️ STATE line too short: {len(parts)} parts, need 23")
                            print(f"   Line: {line.strip()}")
        
        print("⚠️ No config response from Arduino, using local defaults")
        return False
        
    except Exception as e:
        print(f"❌ Error loading config from Arduino: {e}")
        return False

async def load_cycle_program_from_arduino():
    """Load the complete cycle program from Arduino by reading the serial port DIRECTLY.

    IMPORTANT (V1.3): This function used to rely on the background update_data() timer
    to read and parse the $PROGRAM_* lines. That timer can stall (e.g. during a
    websocket disconnect/reconnect or when the shared mirroring client's lifecycle
    is disrupted), which left the program lines unread in the serial buffer until the
    request timed out and the watchdog killed the connection. We now read + parse the
    serial port directly here, exactly like load_config_from_arduino() does, so this
    no longer depends on the timer running.
    """
    try:
        if not arduino or not arduino.connected:
            print("⚠️ Arduino not connected, using default program")
            return None
        
        print("📡 Requesting current program from Arduino...")
        
        # Send the command
        arduino.send_command("GET_CURRENT_PROGRAM")
        
        # Read + parse the response directly (do NOT depend on the background timer)
        import asyncio
        captured_started = False
        captured_steps = []
        captured_cycles = 1
        timeout = 10.0  # 10 second timeout
        start_time = asyncio.get_event_loop().time()
        
        while (asyncio.get_event_loop().time() - start_time) < timeout:
            data = arduino.read_data()
            if data:
                for line in data:
                    if line.startswith("$PROGRAM_START"):
                        captured_started = True
                        parts = line.split(',')
                        if len(parts) >= 3:
                            try:
                                captured_cycles = int(parts[2])
                            except Exception:
                                captured_cycles = 1
                    elif line.startswith("$PROGRAM_STEP") and captured_started:
                        parts = line.split(',')
                        if len(parts) >= 5:
                            try:
                                media_type = parts[2]
                                led_pwm = int(parts[3])
                                led_pct = int((led_pwm / 4095.0) * 100)
                                led_pct = max(0, min(100, led_pct))
                                captured_steps.append({'media_type': media_type, 'led_brightness': led_pct})
                            except Exception as e:
                                print(f"⚠️ Error parsing step: {e}")
                    elif line.startswith("$PROGRAM_END") and captured_started:
                        if captured_steps:
                            result = {
                                'steps': captured_steps,
                                'cycles_per_led_change': captured_cycles
                            }
                            print(f"✅ Loaded {len(result['steps'])} steps from Arduino with {captured_cycles} cycles per LED")
                            return result
                        else:
                            print("⚠️ Program end received but no steps captured")
                            return None
            
            await asyncio.sleep(0.05)  # Small delay to prevent busy waiting
        
        # Timeout
        print("⚠️ Timeout waiting for program from Arduino")
        return None
        
    except Exception as e:
        print(f"❌ Error loading program from Arduino: {e}")
        return None
def load_cycle_program_from_csv():
    """Load the complete cycle program from cycler.csv file (fallback method)."""
    try:
        cycler_path = os.path.join(os.getcwd(), 'cycler.csv')
        if os.path.exists(cycler_path):
            with open(cycler_path, 'r') as f:
                lines = f.readlines()
                if len(lines) < 2:  # No data lines
                    print("⚠️ No data in cycler.csv, using default program")
                    return None
                
                # Parse header to understand format
                header = lines[0].strip().split(',')
                print(f"📄 Cycler CSV header: {header}")
                
                steps = []
                cycles_per_led = 1  # Default
                
                for line in lines[1:]:
                    line = line.strip()
                    if not line:
                        continue
                    
                    parts = line.split(',')
                    if len(parts) >= 5:
                        # Expected format: stepNumber,mediaType,stimulationLEDNumber,stimulationLEDintensity,cyclesPerLEDChange
                        try:
                            step_num = int(parts[0])
                            media_type = parts[1]
                            led_number = int(parts[2])  # 0=OFF, 1=ON
                            led_intensity = int(parts[3])  # 0-255 scale
                            cycles_per_led = int(parts[4])
                            
                            # Convert LED intensity from 0-255 to 0-100 percentage
                            led_brightness = int((led_intensity / 255.0) * 100)
                            
                            step = {
                                'media_type': media_type,
                                'led_brightness': led_brightness
                            }
                            steps.append(step)
                            print(f"📝 Loaded step {step_num}: {media_type}, LED: {led_brightness}%")
                            
                        except (ValueError, IndexError) as e:
                            print(f"⚠️ Error parsing line: {line} - {e}")
                            continue
                
                if steps:
                    print(f"✅ Loaded {len(steps)} steps from cycler.csv with {cycles_per_led} cycles per LED")
                    return {'steps': steps, 'cycles_per_led_change': cycles_per_led}
                else:
                    print("⚠️ No valid steps found in cycler.csv")
                    return None
        else:
            print("⚠️ cycler.csv not found, using default program")
            return None
    except Exception as e:
        print(f"❌ Error loading cycle program: {e}")
        return None

def show_experiment_selection_dialog(experiments, filepath):
    """Show experiment selection by replacing page content temporarily."""
    print(f"🔍 show_experiment_selection_dialog called with {len(experiments)} experiments")
    print(f"🔍 Filepath: {filepath}")
    
    # No need to filter experiments - we'll fix the duration calculation to not read all lines
    print(f"✅ Processing {len(experiments)} experiment(s) for duration calculation")
    
    # Create a simple page overlay instead of dialog
    ui.run_javascript('''
        // Create overlay div
        const overlay = document.createElement('div');
        overlay.id = 'experiment-selector-overlay';
        overlay.style.cssText = `
            position: fixed;
            top: 0;
            left: 0;
            width: 100%;
            height: 100%;
            background: rgba(0,0,0,0.8);
            z-index: 99999;
            display: flex;
            justify-content: center;
            align-items: center;
        `;
        
        const content = document.createElement('div');
        content.style.cssText = `
            background: white;
            padding: 20px;
            border-radius: 8px;
            max-width: 95vw;
            width: 1400px;
            max-height: 80vh;
            overflow-y: auto;
            box-shadow: 0 4px 20px rgba(0,0,0,0.3);
        `;
        
        content.innerHTML = `
            <div style="display: flex; justify-content: space-between; align-items: center; margin-bottom: 15px; border-bottom: 2px solid #eee; padding-bottom: 10px;">
                <h2 style="margin: 0;">📊 Select Experiment to Load</h2>
                <button onclick="document.getElementById('experiment-selector-overlay').remove()" 
                        style="padding: 8px 16px; background: #f44336; color: white; border: none; border-radius: 4px; cursor: pointer; font-size: 14px; font-weight: bold;">
                    ✕ Close
                </button>
            </div>
            <p>Found ''' + str(len(experiments)) + ''' experiments in the CSV file. Click any experiment to open it in a new tab:</p>
            <div id="experiment-list" style="display: grid; grid-template-columns: repeat(auto-fit, minmax(400px, 1fr)); gap: 15px;"></div>
            <div style="margin-top: 20px; text-align: right;">
                <button onclick="document.getElementById('experiment-selector-overlay').remove()" 
                        style="padding: 10px 20px; background: #f44336; color: white; border: none; border-radius: 4px; cursor: pointer; font-size: 14px;">
                    Close Dialog
                </button>
            </div>
        `;
        
        overlay.appendChild(content);
        document.body.appendChild(overlay);
        
        console.log('Experiment selector overlay created');
    ''')
    # Add experiment buttons via JavaScript (restored working version)
    print(f"🔍 About to start duration calculations for {len(experiments)} experiments")
    
    # Import linecache for efficient line reading
    import linecache
    
    for i, exp in enumerate(experiments):
        # Calculate duration using the user's robust interval-based logic
        print(f"🔍 Starting duration calculation for experiment {i+1}")
        try:
            # DON'T read all lines - use linecache to read only what we need!
            start_line = exp['header_line']
            time_idx = exp['time_idx']
            
            # For duration calculation, we need to use unixTime (index 0) if available
            # because upTime is just seconds since experiment start, not total duration
            duration_time_idx = time_idx  # Default to the detected time column
            
            # Check if we have unixTime column (index 0) for better duration calculation
            # Use linecache to read just the header line
            header_line_text = linecache.getline(filepath, exp['header_line'])
            header = next(csv.reader([header_line_text.strip()]))
            if len(header) > 0 and 'unixTime' in header[0]:
                duration_time_idx = 0  # Use unixTime for duration calculation
                print(f"🔍 Using unixTime (index 0) for duration calculation instead of {header[time_idx]} (index {time_idx})")
            
            # Calculate duration using the TRUE first/last data rows of the experiment
            # linecache uses 1-based line numbers
            header_line_num = exp['header_line']  # This is 1-based (from CSV file line numbers)
            first_data_line_idx = header_line_num + 1  # First data row (skip header)
            last_data_line_idx = header_line_num + exp['data_rows']  # Last data row
            
            first_timestamp = None
            last_timestamp = None

            # Detect unixTime column
            unix_idx = None
            try:
                if 'unixTime' in header:
                    unix_idx = header.index('unixTime')
            except Exception:
                pass
            
            # Read first and last data timestamps if we have a unixTime column
            # Use linecache to read only the specific lines we need
            if unix_idx is not None:
                try:
                    first_line_text = linecache.getline(filepath, first_data_line_idx)
                    first_row = next(csv.reader([first_line_text.strip()]))
                    if len(first_row) > unix_idx:
                        first_timestamp = float(first_row[unix_idx])
                except Exception as e:
                    print(f"⚠️ Could not parse first timestamp for exp {i+1}: {e}")
                    first_timestamp = None
                
                try:
                    last_line_text = linecache.getline(filepath, last_data_line_idx)
                    last_row = next(csv.reader([last_line_text.strip()]))
                    if len(last_row) > unix_idx:
                        last_timestamp = float(last_row[unix_idx])
                except Exception as e:
                    print(f"⚠️ Could not parse last timestamp for exp {i+1}: {e}")
                    last_timestamp = None

            # Compute duration from unixTime with time jump detection
            total_seconds = None
            if first_timestamp is not None and last_timestamp is not None:
                print(f"🔍 Exp {i+1}: first_timestamp={first_timestamp}, last_timestamp={last_timestamp}")
                # Heuristic for seconds vs milliseconds
                is_milliseconds = first_timestamp > 1000000000000
                
                # Sample timestamps to detect time jumps (scan at intervals for speed)
                data_row_count = exp['data_rows']
                # For large files, sample more points to get accurate time jump detection
                # With 300k rows and 5000 samples, we check every ~60 rows
                sample_count = min(5000, data_row_count)
                sample_step = max(1, data_row_count // sample_count)
                
                sampled_timestamps = []
                try:
                    for sample_idx in range(0, data_row_count, sample_step):
                        line_idx = first_data_line_idx + sample_idx
                        line_text = linecache.getline(filepath, line_idx)
                        if line_text.strip():
                            row = next(csv.reader([line_text.strip()]))
                            if len(row) > unix_idx:
                                ts = float(row[unix_idx])
                                if is_milliseconds:
                                    ts = ts / 1000.0  # Convert to seconds
                                sampled_timestamps.append(ts)
                except Exception as e:
                    print(f"⚠️ Exp {i+1}: Error sampling timestamps: {e}")
                
                # Calculate duration with time jump correction
                if len(sampled_timestamps) >= 2:
                    total_seconds = 0.0
                    time_jumps = 0
                    for j in range(1, len(sampled_timestamps)):
                        diff = sampled_timestamps[j] - sampled_timestamps[j-1]
                        if diff > 0:
                            total_seconds += diff
                        else:
                            # Time jump detected (timestamp went backwards)
                            time_jumps += 1
                            print(f"🔍 Exp {i+1}: Time jump detected at sample {j}: {sampled_timestamps[j-1]:.0f} -> {sampled_timestamps[j]:.0f}")
                    
                    if time_jumps > 0:
                        print(f"🔍 Exp {i+1}: Found {time_jumps} time jump(s), corrected duration: {total_seconds:.1f}s")
                else:
                    # Fallback to simple calculation if sampling failed
                    if is_milliseconds:
                        total_seconds = (last_timestamp - first_timestamp) / 1000.0
                    else:
                        total_seconds = max(0, last_timestamp - first_timestamp)
                
                print(f"🔍 Exp {i+1}: Duration from unixTime -> {total_seconds / 3600.0:.4f}h ({total_seconds:.1f}s)")

            if total_seconds is not None and total_seconds > 0:
                duration = total_seconds / 3600.0
            else:
                # Fallback: estimate from row count and a default interval
                avg_interval_s = 10.0 # Typical OpenEvo interval
                total_duration_seconds = exp['data_rows'] * avg_interval_s
                duration = total_duration_seconds / 3600.0
                print(f"🔍 Exp {i+1}: Fallback duration calc: {avg_interval_s:.2f}s * {exp['data_rows']} rows -> {duration:.2f}h")

        except Exception as e:
            duration = 0.0
            print(f"❌ Exp {i+1}: Duration calculation failed: {e}")
            import traceback
            traceback.print_exc()

        # Format duration nicely - show minutes for short experiments
        # Note: This is an estimate based on first/last timestamp only
        # Actual duration may differ if file contains concatenated experiments
        if duration < 1.0:
            duration_str = f"~{duration * 60:.0f} min"
        else:
            duration_str = f"~{duration:.1f}h"

        cols_info = exp.get('time_col_name', 'Time') + ', ' + exp.get('od_col_name', 'OD')
        if exp.get('has_media_type'):
            cols_info += ', mediaType'
        
        ui.run_javascript(f'''
            const list = document.getElementById('experiment-list');
            if (list) {{
                const expDiv = document.createElement('div');
                expDiv.style.cssText = `
                    border: 2px solid #ddd;
                    padding: 15px;
                    border-radius: 8px;
                    background: #f9f9f9;
                    cursor: pointer;
                    transition: all 0.2s;
                `;
                
                expDiv.innerHTML = `
                    <div style="text-align: center;">
                        <div style="font-size: 18px; font-weight: bold; margin-bottom: 8px;">Experiment {i+1}</div>
                        <div style="font-size: 14px; color: #666; margin-bottom: 4px;">📍 Line {exp['header_line']}</div>
                        <div style="font-size: 14px; color: #666; margin-bottom: 4px;">📊 {exp['data_rows']} points</div>
                        <div style="font-size: 14px; color: #999; margin-bottom: 8px;" title="Estimate only - actual duration calculated after loading">⏱️ {duration_str} <span style="font-size: 10px;">(est.)</span></div>
                        <div style="font-size: 12px; color: green; margin-bottom: 10px;">📋 {cols_info}</div>
                        <button onclick="openExperiment{i}()" 
                                style="padding: 8px 16px; background: #2196F3; color: white; border: none; border-radius: 4px; cursor: pointer; font-size: 14px; width: 100%;">
                            Open in New Tab
                        </button>
                    </div>
                `;
                
                // Add hover effect
                expDiv.onmouseenter = function() {{
                    this.style.borderColor = '#2196F3';
                    this.style.backgroundColor = '#e3f2fd';
                }};
                expDiv.onmouseleave = function() {{
                    this.style.borderColor = '#ddd';
                    this.style.backgroundColor = '#f9f9f9';
                }};
                
                list.appendChild(expDiv);
                
                // Create function to open experiment in new tab with REAL DATA
                window['openExperiment{i}'] = function() {{
                    // Load real data for this experiment
                    fetch('/load_experiment_data', {{
                        method: 'POST',
                        headers: {{'Content-Type': 'application/json'}},
                        body: JSON.stringify({{
                            filepath: '{filepath}',
                            experiment_index: {i}
                        }})
                    }})
                    .then(response => {{
                        console.log('📡 Fetch response status:', response.status);
                        console.log('📡 Fetch response ok:', response.ok);
                        if (!response.ok) {{
                            throw new Error('HTTP error! status: ' + response.status);
                        }}
                        return response.json();
                    }})
                    .then(data => {{
                        console.log('📊 Received data:', data);
                        console.log('📊 Time points:', data.time_data ? data.time_data.length : 'none');
                        console.log('📊 OD points:', data.od_data ? data.od_data.length : 'none');
                        // Open synchronously to avoid popup blockers, then populate
                        const newWindow = window.open('about:blank', 'experiment_' + Date.now());
                        if (!newWindow) {{
                            console.error('❌ Failed to open new window - popup blocked?');
                            alert('❌ Failed to open experiment viewer. Please allow popups for this site.');
                            return;
                        }}
                        const timeDataJson = JSON.stringify(data.time_data || []);
                        const odDataJson = JSON.stringify(data.od_data || []);
                        const mediaDataJson = JSON.stringify(data.media_data || []);
                        const unixTimeDataJson = JSON.stringify(data.unix_time_data || []);
                        const dilutionEventDataJson = JSON.stringify(data.dilution_event_data || []);
                        const ledChannelDataJson = JSON.stringify(data.led_channel_data || []);
                        const ledPercentDataJson = JSON.stringify(data.led_percent_data || []);
                        const hasDilutionEvents = data.has_dilution_events || false;
                        const hasLedData = data.has_led_data || false;
                        const hasDuplicates = data.has_duplicate_timestamps || false;
                        const timeSpacingSeconds = data.time_spacing_seconds || 60;
                        const totalDuration = (typeof data.total_duration_hours === 'number' && isFinite(data.total_duration_hours)) ? data.total_duration_hours.toFixed(1) : '—';
                        newWindow.document.write(`
                        <html>
                        <head>
                            <title>OpenEvo Experiment {i+1} - {exp['data_rows']} points</title>
                            <script src="https://cdn.jsdelivr.net/npm/echarts@5.4.3/dist/echarts.min.js"><\\/script>
                            <style>
                                body {{ font-family: Arial, sans-serif; margin: 0; padding: 20px; background: #f5f5f5; }}
                                .container {{ max-width: 1200px; margin: 0 auto; background: white; padding: 30px; border-radius: 8px; box-shadow: 0 2px 10px rgba(0,0,0,0.1); }}
                                .header {{ text-align: center; margin-bottom: 30px; }}
                                .stats {{ display: grid; grid-template-columns: repeat(auto-fit, minmax(200px, 1fr)); gap: 20px; margin-bottom: 30px; }}
                                .stat-card {{ background: #f8f9fa; padding: 15px; border-radius: 6px; text-align: center; }}
                                .stat-value {{ font-size: 24px; font-weight: bold; color: #2196F3; }}
                                .stat-label {{ font-size: 14px; color: #666; margin-top: 5px; }}
                                .chart-container {{ 
                                    width: 100%; 
                                    height: 450px; 
                                    margin: 25px 0; 
                                    background: white;
                                    border-radius: 12px; 
                                    box-shadow: 0 4px 12px rgba(0,0,0,0.1);
                                    position: relative;
                                    padding: 10px;
                                    border: 1px solid #e0e0e0;
                                    box-sizing: border-box;
                                }}
                                .controls {{ display: flex; justify-content: space-between; align-items: center; margin-bottom: 20px; }}
                                .threshold-control {{ display: flex; align-items: center; gap: 15px; }}
                                .chart-controls {{ position: absolute; top: 15px; right: 15px; z-index: 1000; display: flex; gap: 10px; }}
                                .chart-btn {{ 
                                    background: rgba(255, 255, 255, 0.95); 
                                    border: 1px solid #e0e0e0; 
                                    border-radius: 8px; 
                                    padding: 10px 16px; 
                                    cursor: pointer; 
                                    font-size: 13px; 
                                    font-weight: 600;
                                    color: #333;
                                    box-shadow: 0 2px 8px rgba(0,0,0,0.1);
                                    transition: all 0.3s ease;
                                }}
                                .chart-btn:hover {{ 
                                    background: white; 
                                    box-shadow: 0 4px 12px rgba(0,0,0,0.15); 
                                    transform: translateY(-1px);
                                }}
                                .chart-btn.zoom {{ background: #4CAF50; color: white; border-color: #4CAF50; }}
                                .chart-btn.zoom:hover {{ background: #45a049; box-shadow: 0 4px 12px rgba(76, 175, 80, 0.3); }}
                                .chart-btn.download {{ background: #2196F3; color: white; border-color: #2196F3; }}
                                .chart-btn.download:hover {{ background: #1976D2; box-shadow: 0 4px 12px rgba(33, 150, 243, 0.3); }}
                            <\\/style>
                        <\\/head>
                        <body>
                            <div class="container">
                                <div class="header">
                                    <h1>🧬 OpenEvo Experiment {i+1}</h1>
                                </div>
                                
                                
                                <div class="stats" style="justify-content: center; gap: 40px;">
                                    <div class="stat-card">
                                        <div class="stat-value">{exp['data_rows']}</div>
                                        <div class="stat-label">Data Points</div>
                                    </div>
                                    <div class="stat-card">
                                        <div id="experiment-duration" class="stat-value">` + totalDuration + `h</div>
                                        <div class="stat-label">Duration</div>
                                    </div>
                                    <div class="stat-card">
                                        <div class="stat-value">{exp['header_line']}</div>
                                        <div class="stat-label">Header Line</div>
                                    </div>
                                </div>
                                
                                <!-- Chart Controls Section -->
                                <div style="background: #f8f9fa; padding: 20px; border-radius: 8px; margin-bottom: 20px;">
                                    <h3 style="margin: 0 0 20px 0; color: #333; text-align: center;">📊 Chart Controls</h3>
                                    
                                    <!-- Axis Controls -->
                                    <div style="background: white; padding: 15px; border-radius: 6px; margin-bottom: 15px; border-left: 4px solid #9C27B0;">
                                        <h4 style="margin: 0 0 12px 0; color: #9C27B0;">Chart Axes</h4>
                                        <div style="display: grid; grid-template-columns: 1fr 1fr; gap: 20px;">
                                            <div>
                                                <label style="font-weight: bold; display: block; margin-bottom: 8px;">OD Chart Axes:</label>
                                                <div style="display: grid; grid-template-columns: 1fr 1fr; gap: 10px;">
                                                    <div>
                                                        <label style="font-size: 12px; color: #666;">X-Min (hr):</label>
                                                        <input type="number" id="x-min-input" value="0" step="0.001" 
                                                               style="width: 100%; padding: 4px; border: 1px solid #ddd; border-radius: 3px;" 
                                                               oninput="updateAxisRanges()">
                                                    </div>
                                                    <div>
                                                        <label style="font-size: 12px; color: #666;">X-Max (hr):</label>
                                                        <input type="number" id="x-max-input" value="24" step="0.001" 
                                                               style="width: 100%; padding: 4px; border: 1px solid #ddd; border-radius: 3px;" 
                                                               oninput="updateAxisRanges()">
                                                    </div>
                                                    <div>
                                                        <label style="font-size: 12px; color: #666;">Y-Min (OD):</label>
                                                        <input type="number" id="y-min-input" value="0" step="0.001" 
                                                               style="width: 100%; padding: 4px; border: 1px solid #ddd; border-radius: 3px;" 
                                                               oninput="updateAxisRanges()">
                                                    </div>
                                                    <div>
                                                        <label style="font-size: 12px; color: #666;">Y-Max (OD):</label>
                                                        <!-- <-- CSV viewer Y-axis max input default -->
                                                        <input type="number" id="y-max-input" value="6" step="0.001" 
                                                               style="width: 100%; padding: 4px; border: 1px solid #ddd; border-radius: 3px;" 
                                                               oninput="updateAxisRanges()">
                                                    </div>
                                                </div>
                                            </div>
                                            <div>
                                                <label style="font-weight: bold; display: block; margin-bottom: 8px;">Peak Chart Axes:</label>
                                                <div style="display: grid; grid-template-columns: 1fr 1fr; gap: 10px;">
                                                    <div>
                                                        <label style="font-size: 12px; color: #666;">X-Min (Peak #):</label>
                                                        <input type="number" id="peak-x-min-input" value="0" step="1" 
                                                               style="width: 100%; padding: 4px; border: 1px solid #ddd; border-radius: 3px;" 
                                                               oninput="updateAxisRanges()">
                                                    </div>
                                                    <div>
                                                        <label style="font-size: 12px; color: #666;">X-Max (Peak #):</label>
                                                        <input type="number" id="peak-x-max-input" value="10" step="1" 
                                                               style="width: 100%; padding: 4px; border: 1px solid #ddd; border-radius: 3px;" 
                                                               oninput="updateAxisRanges()">
                                                    </div>
                                                    <div>
                                                        <label style="font-size: 12px; color: #666;">Y-Min (Interval):</label>
                                                        <input type="number" id="peak-y-min-input" value="0" step="0.1" 
                                                               style="width: 100%; padding: 4px; border: 1px solid #ddd; border-radius: 3px;" 
                                                               oninput="updateAxisRanges()">
                                                    </div>
                                                    <div>
                                                        <label style="font-size: 12px; color: #666;">Y-Max (Interval):</label>
                                                        <input type="number" id="peak-y-max-input" value="24" step="0.1" 
                                                               style="width: 100%; padding: 4px; border: 1px solid #ddd; border-radius: 3px;" 
                                                               oninput="updateAxisRanges()">
                                                    </div>
                                                </div>
                                            </div>
                                        </div>
                                    </div>
                                    
                                    <!-- Date Offset Control for RTC Correction -->
                                    <div style="background: white; padding: 15px; border-radius: 6px; margin-top: 15px; border-left: 4px solid #FF9800;">
                                        <h4 style="margin: 0 0 12px 0; color: #FF9800;">📅 Date Offset (RTC Correction)</h4>
                                        <div style="display: flex; align-items: center; gap: 15px; flex-wrap: wrap;">
                                            <label style="font-weight: bold; min-width: 120px;">Add days to dates:</label>
                                            <input type="number" id="date-offset-input" value="0" step="1" 
                                                   style="width: 100px; padding: 6px; border: 1px solid #ddd; border-radius: 3px; font-size: 14px;"
                                                   oninput="updateDateOffset(this.value)">
                                            <span id="date-offset-preview" style="color: #666; font-size: 13px;">No offset applied</span>
                                            <button onclick="document.getElementById('date-offset-input').value=0; updateDateOffset(0);" 
                                                    style="padding: 6px 12px; background: #9E9E9E; color: white; border: none; border-radius: 4px; cursor: pointer; font-size: 12px;">
                                                Reset
                                            </button>
                                        </div>
                                        <div style="margin-top: 8px; font-size: 12px; color: #666; font-style: italic;">
                                            Use this to correct dates if the Arduino RTC was set incorrectly. Positive = future, Negative = past.
                                        </div>
                                    </div>
                                </div>
                                
                                ` + (hasDuplicates ? `
                                <div style="background: #fff3cd; border: 1px solid #ffeaa7; color: #856404; padding: 12px; border-radius: 6px; margin: 15px 0; text-align: center; font-size: 13px;">
                                    ⚠️ <strong>Warning:</strong> Using sequential timestamps (` + timeSpacingSeconds + `s intervals) to avoid overlapping data. All data points preserved.
                                </div>
                                ` : '') + `
                                
                                <!-- OD Chart Section -->
                                <div style="margin: 30px 0;">
                                    <h3 style="margin: 0 0 15px 0; color: #333; font-size: 20px; font-weight: bold; text-align: center; padding: 0 20px;">📊 OD940 vs Time</h3>
                                    <div class="chart-container">
                                        <div id="od-chart" style="width: 100%; height: 100%;"><\\/div>
                                        <div class="chart-controls">
                                            <button class="chart-btn zoom" onclick="resetODZoom()" title="Reset Zoom">🔄 Reset</button>
                                            <button class="chart-btn download" onclick="downloadODChart()" title="Download Chart">💾 Save</button>
                                        </div>
                                    </div>
                                </div>
                                
                                <!-- Peak Intervals Chart Section -->
                                <div style="margin: 30px 0;">
                                    <h3 style="margin: 0 0 15px 0; color: #333; font-size: 20px; font-weight: bold; text-align: center; padding: 0 20px;">📈 Peak-to-Peak Intervals</h3>
                                    <div class="chart-container">
                                        <div id="peak-chart" style="width: 100%; height: 100%;"><\\/div>
                                        <div class="chart-controls">
                                            <button class="chart-btn zoom" onclick="resetPeakZoom()" title="Reset Zoom">🔄 Reset</button>
                                            <button class="chart-btn download" onclick="downloadPeakChart()" title="Download Chart">💾 Save</button>
                                        </div>
                                    </div>
                                </div>
                                    
                                    <!-- Peak Detection Controls -->
                                <div style="background: white; padding: 15px; border-radius: 6px; margin: 20px 0; border-left: 4px solid #2196F3;">
                                        <h4 style="margin: 0 0 12px 0; color: #2196F3;">Peak Detection</h4>
                                        <div style="display: flex; align-items: center; gap: 10px; margin-bottom: 10px;">
                                            <label style="display: flex; align-items: center; gap: 5px; cursor: pointer; font-size: 14px;">
                                                <input type="checkbox" id="use-threshold-detection" onchange="toggleDetectionMode(this.checked)" 
                                                       style="width: 16px; height: 16px; cursor: pointer;">
                                                <span id="detection-mode-label">Use Adjustable Threshold Detection</span>
                                            </label>
                                        </div>
                                        <div style="display: flex; align-items: center; gap: 15px; flex-wrap: wrap; margin-bottom: 10px;">
                                            <label style="font-weight: bold; min-width: 120px;">Threshold:</label>
                                            <input type="range" id="threshold-slider" min="0.5" max="10.0" step="0.001" value="2.0" 
                                                   style="flex: 1; min-width: 150px;" oninput="updateThreshold(this.value)">
                                            <input type="number" id="threshold-input" min="0.5" max="10.0" step="0.001" value="2.0" 
                                                   style="width: 80px; padding: 4px; border: 1px solid #ddd; border-radius: 3px;" 
                                                   oninput="updateThresholdFromInput(this.value)">
                                            <span id="threshold-value" style="font-weight: bold; color: #2196F3; min-width: 40px;">2.0</span>
                                            <button onclick="resetZoom()" 
                                                    style="padding: 8px 16px; background: #4CAF50; color: white; border: none; border-radius: 4px; cursor: pointer; font-weight: bold;">
                                                Reset Zoom
                                            </button>
                                        </div>
                                        <div style="display: flex; align-items: center; gap: 15px; flex-wrap: wrap;">
                                            <label style="font-weight: bold; min-width: 120px;">Min Peak Separation:</label>
                                            <input type="range" id="peak-separation-slider" min="0.001" max="10.0" step="0.01" value="0.5" 
                                                   style="flex: 1; min-width: 150px;" oninput="updatePeakSeparation(this.value)">
                                            <input type="number" id="peak-separation-input" min="0.001" max="10.0" step="0.01" value="0.5" 
                                                   style="width: 80px; padding: 4px; border: 1px solid #ddd; border-radius: 3px;" 
                                                   oninput="updatePeakSeparationFromInput(this.value)">
                                            <span id="peak-separation-value" style="font-weight: bold; color: #2196F3; min-width: 60px;">0.5h</span>
                                        </div>
                                    </div>
                                    
                                    <!-- Data Cleaning Controls -->
                                <div style="background: white; padding: 15px; border-radius: 6px; margin: 20px 0; border-left: 4px solid #FF5722;">
                                        <h4 style="margin: 0 0 12px 0; color: #FF5722;">Data Cleaning</h4>
                                        <div style="display: flex; align-items: center; gap: 15px; flex-wrap: wrap;">
                                            <label style="font-weight: bold; min-width: 120px;">Outlier Threshold:</label>
                                            <input type="range" id="clean-threshold-slider" min="0" max="100" step="1" value="50" 
                                                   style="flex: 1; min-width: 150px;" oninput="updateCleanThresholdFromSlider(this.value)">
                                            <input type="number" id="clean-threshold-input" min="1.000" max="10.0" step="0.001" value="2.0" 
                                                   style="width: 80px; padding: 4px; border: 1px solid #ddd; border-radius: 3px;" 
                                                   oninput="updateCleanThresholdFromInput(this.value)">
                                            <span id="clean-threshold-value" style="font-weight: bold; color: #FF5722; min-width: 50px;">2.0x</span>
                                            <button id="cleaning-toggle" onclick="toggleDataCleaning()" 
                                                    style="padding: 8px 16px; background: #9E9E9E; color: white; border: none; border-radius: 4px; cursor: pointer; font-weight: bold;">
                                                Enable Cleaning
                                            </button>
                                        </div>
                                        <div style="margin-top: 8px; font-size: 12px; color: #666; font-style: italic;">
                                            Remove data points where neighbors differ by more than the threshold ratio
                                    </div>
                                </div>
                                
                                <!-- Statistics Section -->
                                <div style="margin: 40px 0; padding: 25px; background: white; border-radius: 12px; box-shadow: 0 4px 12px rgba(0,0,0,0.1); border: 1px solid #e0e0e0;">
                                    <h3 style="margin: 0 0 20px 0; color: #333; font-size: 20px; font-weight: bold; text-align: center;">📊 Peak Analysis</h3>
                                    <div class="stats">
                                        <div class="stat-card" style="background: linear-gradient(135deg, #667eea 0%, #764ba2 100%); color: white; box-shadow: 0 4px 8px rgba(0,0,0,0.15);">
                                            <div id="avg-interval" class="stat-value" style="color: white;">--</div>
                                            <div class="stat-label" style="color: rgba(255,255,255,0.9);">Avg Interval (h)</div>
                                        </div>
                                        <div class="stat-card" style="background: linear-gradient(135deg, #f093fb 0%, #f5576c 100%); color: white; box-shadow: 0 4px 8px rgba(0,0,0,0.15);">
                                            <div id="growth-rate" class="stat-value" style="color: white;">--</div>
                                            <div class="stat-label" style="color: rgba(255,255,255,0.9);">Growth Rate (Peaks/h)</div>
                                        </div>
                                        <div class="stat-card" style="background: linear-gradient(135deg, #43cea2 0%, #185a9d 100%); color: white; box-shadow: 0 4px 8px rgba(0,0,0,0.15);">
                                            <div id="peak-count" class="stat-value" style="color: white;">--</div>
                                            <div class="stat-label" style="color: rgba(255,255,255,0.9);">Peak Count</div>
                                        </div>
                                    </div>
                                </div>
                            </div>
                            <script>
                                // Initialize charts immediately
                                const chart = echarts.init(document.getElementById('od-chart'));
                                const peakChart = echarts.init(document.getElementById('peak-chart'));
                                let currentThreshold = 2.0;
                                let cleanThreshold = 2.0;
                                let dataCleaningEnabled = false;
                                let minPeakSeparation = 0.5;  // Minimum hours between peaks
                                
                                // Use real CSV data
                                const timeData = ` + timeDataJson + `;
                                const odData = ` + odDataJson + `;
                                const mediaData = ` + mediaDataJson + `;
                                const unixTimeData = ` + unixTimeDataJson + `;
                                const dilutionEventData = ` + dilutionEventDataJson + `;
                                const ledChannelData = ` + ledChannelDataJson + `;
                                const ledPercentData = ` + ledPercentDataJson + `;
                                const hasDilutionEvents = ` + (hasDilutionEvents ? 'true' : 'false') + `;
                                const hasLedData = ` + (hasLedData ? 'true' : 'false') + `;
                                const hasDuplicateTimestamps = ` + (hasDuplicates ? 'true' : 'false') + `;
                                const timeSpacing = ` + timeSpacingSeconds + `;
                                
                                console.log('📊 CSV Viewer Data Loaded:');
                                console.log('📊 Time data points:', timeData.length);
                                console.log('📊 OD data points:', odData.length);
                                console.log('📊 Media data points:', mediaData.length);
                                console.log('📊 Sample media data:', mediaData.slice(0, 10));
                                console.log('📊 Unix time data points:', unixTimeData.length);
                                console.log('📊 Dilution event data points:', dilutionEventData.length);
                                console.log('📊 Has dilution events:', hasDilutionEvents);
                                console.log('📊 Has duplicate timestamps:', hasDuplicateTimestamps);
                                if (hasDuplicateTimestamps) {{
                                    console.warn('⚠️ WARNING: Using sequential timestamps (' + timeSpacing + 's intervals) to avoid overlapping data');
                                }}
                                
                                // Store original data
                                let originalData = odData.slice();
                                let cleanedData = odData.slice();
                                let originalTimeData = timeData.slice();
                                let cleanedTimeData = timeData.slice();
                                
                                // Date offset for RTC correction (in days)
                                let dateOffsetDays = 0;
                                
                                // Function to update date offset and refresh display
                                function updateDateOffset(days) {{
                                    dateOffsetDays = parseFloat(days) || 0;
                                    const preview = document.getElementById('date-offset-preview');
                                    if (preview) {{
                                        if (dateOffsetDays === 0) {{
                                            preview.textContent = 'No offset applied';
                                            preview.style.color = '#666';
                                        }} else {{
                                            const direction = dateOffsetDays > 0 ? 'forward' : 'backward';
                                            preview.textContent = Math.abs(dateOffsetDays) + ' day(s) ' + direction;
                                            preview.style.color = '#FF9800';
                                        }}
                                    }}
                                    // Refresh charts to update tooltips
                                    if (typeof chart !== 'undefined' && chart) {{
                                        chart.setOption(chart.getOption());
                                    }}
                                    console.log('📅 Date offset updated to: ' + dateOffsetDays + ' days');
                                }}
                                
                                // Helper function to get offset-corrected date
                                function getOffsetCorrectedDate(unixTime) {{
                                    const offsetSeconds = dateOffsetDays * 24 * 60 * 60;
                                    return new Date((unixTime + offsetSeconds) * 1000);
                                }}
                                
                                // Data cleaning function - removes outlier points where neighbors differ by threshold
                                function cleanData(data, timeData, threshold) {{
                                    if (data.length < 3) return {{ cleanedData: data.slice(), cleanedTime: timeData.slice() }}; // Need at least 3 points
                                    
                                    const cleanedDataArray = [];
                                    const cleanedTimeArray = [];
                                    const toRemove = new Set();
                                    
                                    // Check each point against its neighbors
                                    for (let i = 1; i < data.length - 1; i++) {{
                                        const current = data[i];
                                        const prev = data[i - 1];
                                        const next = data[i + 1];
                                        
                                        // Check if current point is significantly different from both neighbors
                                        const prevRatio = Math.max(current, prev) / Math.min(current, prev);
                                        const nextRatio = Math.max(current, next) / Math.min(current, next);
                                        
                                        // If both ratios exceed threshold, mark for removal
                                        if (prevRatio >= threshold && nextRatio >= threshold) {{
                                            toRemove.add(i);
                                            
                                            // Also check if neighbors should be removed (cascade effect)
                                            if (i > 1) {{
                                                const prevPrevRatio = Math.max(prev, data[i - 2]) / Math.min(prev, data[i - 2]);
                                                if (prevPrevRatio >= threshold) {{
                                                    toRemove.add(i - 1);
                                                }}
                                            }}
                                            if (i < data.length - 2) {{
                                                const nextNextRatio = Math.max(next, data[i + 2]) / Math.min(next, data[i + 2]);
                                                if (nextNextRatio >= threshold) {{
                                                    toRemove.add(i + 1);
                                                }}
                                            }}
                                        }}
                                    }}
                                    
                                    // Create cleaned arrays without marked points
                                    for (let i = 0; i < data.length; i++) {{
                                        if (!toRemove.has(i)) {{
                                            cleanedDataArray.push(data[i]);
                                            cleanedTimeArray.push(timeData[i]);
                                        }}
                                    }}
                                    
                                    return {{ cleanedData: cleanedDataArray, cleanedTime: cleanedTimeArray }};
                                }}
                                
                                // Peak detection: Use dilution events if available (and not overridden), otherwise threshold crossing
                                function detectPeaks(threshold) {{
                                    const currentData = dataCleaningEnabled ? cleanedData : originalData;
                                    const currentTimeData = dataCleaningEnabled ? cleanedTimeData : originalTimeData;
                                    
                                    if (currentData.length < 2) return [];
                                    
                                    const detectedPeaks = [];
                                    
                                    // MODE 1: Use Dilution_Event column if available AND not overridden by user
                                    if (hasDilutionEvents && dilutionEventData.length === timeData.length && !useThresholdDetection) {{
                                        console.log('🔍 Using Dilution_Event column for peak detection');
                                        for (let i = 0; i < dilutionEventData.length; i++) {{
                                            if (dilutionEventData[i] === 1) {{
                                                detectedPeaks.push({{
                                                    index: i,
                                                    time: timeData[i],
                                                    od: odData[i]
                                                }});
                                            }}
                                        }}
                                        console.log('🔍 Found ' + detectedPeaks.length + ' peaks from Dilution_Event column');
                                        return detectedPeaks;
                                    }}
                                    
                                    // MODE 2: Use threshold-based local maximum detection (fallback or user override)
                                    console.log('🔍 Using local maximum detection with threshold');
                                    
                                    const minSeparation = minPeakSeparation;  // Use slider value
                                    
                                    // Find local maxima above threshold
                                    // A point is a local maximum if it's higher than its neighbors AND above threshold
                                    for (let i = 1; i < currentData.length - 1; i++) {{
                                        const current = currentData[i];
                                        const prev = currentData[i - 1];
                                        const next = currentData[i + 1];
                                        
                                        // Check if this is a local maximum above threshold
                                        if (current >= threshold && current >= prev && current >= next) {{
                                            const peakTime = currentTimeData[i];
                                            
                                            // Check if this peak is far enough from previous peaks
                                            let isDuplicate = false;
                                            for (let j = 0; j < detectedPeaks.length; j++) {{
                                                if (Math.abs(peakTime - detectedPeaks[j].time) < minSeparation) {{
                                                    isDuplicate = true;
                                                    break;
                                                }}
                                            }}
                                            
                                            if (!isDuplicate) {{
                                                detectedPeaks.push({{
                                                    index: i,
                                                    time: peakTime,
                                                    od: current
                                                }});
                                            }}
                                        }}
                                    }}
                                    
                                    console.log('🔍 Threshold crossing found ' + detectedPeaks.length + ' peaks');
                                    return detectedPeaks;
                                }}
                                
                                // Update chart with current data and threshold
                                function updateChart() {{
                                    console.log('🔄 updateChart() called with threshold:', currentThreshold);
                                    const currentData = dataCleaningEnabled ? cleanedData : originalData;
                                    const currentTimeData = dataCleaningEnabled ? cleanedTimeData : originalTimeData;
                                    const peaks = detectPeaks(currentThreshold);
                                    console.log('🔍 Detected', peaks.length, 'peaks with threshold', currentThreshold);
                                    
                                    // Build media shading areas (POSITIVE = blue, NEGATIVE = red)
                                    // IMPORTANT: Always use ORIGINAL data for shading, never cleaned data!
                                    // The shading represents the actual media type from the SD card.
                                    const mediaAreas = [];
                                    if (Array.isArray(mediaData) && mediaData.length > 0 && Array.isArray(timeData) && timeData.length > 1) {{
                                        let lastType = null;
                                        let rangeStart = timeData[0];
                                        for (let i = 0; i < timeData.length; i++) {{
                                            const t = timeData[i];  // Use original timeData
                                            const m = (i < mediaData.length ? mediaData[i] : 'UNKNOWN');
                                            const shadeType = (m === 'POSITIVE' || m === 'NEGATIVE') ? m : null;
                                            if (shadeType !== lastType) {{
                                                if (lastType !== null) {{
                                                    const color = (lastType === 'POSITIVE') ? '#bbdefb' : '#ffcdd2';
                                                    const endX = timeData[i - 1];  // Use original timeData
                                                    if (endX != null && rangeStart != null && endX >= rangeStart) {{
                                                        mediaAreas.push([
                                                            {{ xAxis: rangeStart, itemStyle: {{ color: color, opacity: 0.3 }} }},
                                                            {{ xAxis: endX }}
                                                        ]);
                                                    }}
                                                }}
                                                lastType = shadeType;
                                                rangeStart = t;
                                            }}
                                        }}
                                        if (lastType !== null) {{
                                            const color = (lastType === 'POSITIVE') ? '#bbdefb' : '#ffcdd2';
                                            const endX = timeData[timeData.length - 1];  // Use original timeData
                                            if (endX != null && rangeStart != null && endX >= rangeStart) {{
                                                mediaAreas.push([
                                                    {{ xAxis: rangeStart, itemStyle: {{ color: color, opacity: 0.3 }} }},
                                                    {{ xAxis: endX }}
                                                ]);
                                            }}
                                        }}
                                    }}

                                    const option = {{
                                        tooltip: {{ 
                                            trigger: 'axis',
                                            formatter: function(params) {{
                                                if (params && params.length > 0) {{
                                                    const dataIndex = params[0].dataIndex;
                                                    const timeHours = params[0].data[0];
                                                    const odValue = params[0].data[1];
                                                    
                                                    // Add null checks to prevent toFixed errors
                                                    const timeStr = (timeHours != null && typeof timeHours === 'number') ? timeHours.toFixed(2) : 'N/A';
                                                    const odStr = (odValue != null && typeof odValue === 'number') ? odValue.toFixed(4) : 'N/A';
                                                    
                                                    let tooltipText = 'Runtime: ' + timeStr + ' hours<br/>';
                                                    tooltipText += 'OD940: ' + odStr + '<br/>';
                                                    
                                                    // Add NIR value if available
                                                    const chart = this;
                                                    if (chart && chart.getOption && chart.getOption().irData && dataIndex < chart.getOption().irData.length) {{
                                                        const irValue = chart.getOption().irData[dataIndex];
                                                        tooltipText += 'NIR: ' + (irValue != null ? irValue : 'N/A') + '<br/>';
                                                    }}
                                                    
                                                    // Add LED data if available (firmware V3+)
                                                    if (hasLedData && ledChannelData && ledPercentData && dataIndex < ledChannelData.length) {{
                                                        const ledChannel = ledChannelData[dataIndex];
                                                        const ledPercent = ledPercentData[dataIndex];
                                                        if (ledChannel > 0) {{
                                                            tooltipText += '<span style="color: #FF9800;">💡 LED' + ledChannel + ': ' + ledPercent + '% eff. dose</span><br/>';  // <-- % (effective dose)
                                                        }} else {{
                                                            tooltipText += '<span style="color: #9E9E9E;">💡 LED: OFF</span><br/>';
                                                        }}
                                                    }}
                                                    
                                                    // Add Unix timestamp if available
                                                    if (unixTimeData && dataIndex < unixTimeData.length && unixTimeData[dataIndex]) {{
                                                        const unixTime = unixTimeData[dataIndex];
                                                        const date = getOffsetCorrectedDate(unixTime);  // Use offset-corrected date
                                                        tooltipText += 'Unix Time: ' + unixTime + '<br/>';
                                                        tooltipText += 'Date: ' + date.toLocaleString();
                                                        if (dateOffsetDays !== 0) {{
                                                            tooltipText += ' <span style="color: #FF9800;">(+' + dateOffsetDays + 'd)</span>';
                                                        }}
                                                        if (hasDuplicateTimestamps) {{
                                                            tooltipText += '<br/><span style="color: #856404;">⚠️ Sequential time (no overlaps)</span>';
                                                        }}
                                                    }}
                                                    
                                                    return tooltipText;
                                                }}
                                                return '';
                                            }}
                                        }},
                                        legend: {{ 
                                            data: hasLedData ? ['OD940', 'LED Intensity', 'Peaks', 'Threshold', 'POSITIVE Media', 'NEGATIVE Media'] : ['OD940', 'Peaks', 'Threshold', 'POSITIVE Media', 'NEGATIVE Media'], 
                                            bottom: 0,
                                            itemGap: 15,
                                            textStyle: {{ fontSize: 11 }}
                                        }},
                                        grid: {{
                                            left: 60,
                                            right: hasLedData ? 80 : 40,  // Extra space for right Y-axis when LED data present
                                            top: 60,
                                            bottom: 80,
                                            containLabel: true
                                        }},
                                        xAxis: {{ 
                                            type: 'value',
                                            name: 'Time (hours)',
                                            nameLocation: 'middle',
                                            nameGap: 30,
                                            nameTextStyle: {{ fontSize: 14, fontWeight: 'bold' }},
                                            axisLabel: {{ fontSize: 12 }}
                                        }},
                                        yAxis: [
                                            {{ 
                                                type: 'value',
                                                name: 'OD940',
                                                min: 0,    // <-- CSV viewer Y-axis default min
                                                // max: auto-scale based on data
                                                nameLocation: 'middle',
                                                nameGap: 50,
                                                nameTextStyle: {{ fontSize: 14, fontWeight: 'bold' }},
                                                axisLabel: {{ fontSize: 12 }}
                                            }},
                                            {{
                                                type: 'value',
                                                name: 'LED %',
                                                min: 0,
                                                max: 100,
                                                position: 'right',
                                                nameLocation: 'middle',
                                                nameGap: 40,
                                                nameTextStyle: {{ fontSize: 14, fontWeight: 'bold', color: '#FF9800' }},
                                                axisLabel: {{ fontSize: 12, color: '#FF9800' }},
                                                axisLine: {{ lineStyle: {{ color: '#FF9800' }} }},
                                                splitLine: {{ show: false }}  // Don't show grid lines for LED axis
                                            }}
                                        ],
                                        dataZoom: [
                                            {{ type: 'inside', xAxisIndex: 0 }},
                                            {{ type: 'slider', xAxisIndex: 0, bottom: 80 }}
                                        ],
                                        series: (function() {{
                                            // Build series array dynamically based on available data
                                            const seriesArray = [
                                                {{
                                                    name: 'OD940',
                                                    type: 'line',
                                                    yAxisIndex: 0,
                                                    data: currentTimeData.map((time, idx) => [time, currentData[idx]]),
                                                    lineStyle: {{ color: '#000000', width: 2 }},
                                                    symbol: 'none',
                                                    markArea: {{ data: mediaAreas, silent: true }}
                                                }},
                                                {{
                                                    name: 'Peaks',
                                                    type: 'scatter',
                                                    yAxisIndex: 0,
                                                    data: peaks.map(p => [p.time, p.od]),
                                                    symbolSize: 8,
                                                    itemStyle: {{ color: '#ff4444' }}
                                                }},
                                                {{
                                                    name: 'Threshold',
                                                    type: 'line',
                                                    yAxisIndex: 0,
                                                    data: [[timeData[0] || 0, currentThreshold], [timeData[timeData.length-1] || 1, currentThreshold]],
                                                    lineStyle: {{ color: '#ff0000', type: 'dashed', width: 2 }},
                                                    symbol: 'none'
                                                }},
                                                {{
                                                    name: 'POSITIVE Media',
                                                    type: 'line',
                                                    yAxisIndex: 0,
                                                    data: [],
                                                    lineStyle: {{ width: 0 }},
                                                    symbol: 'rect',
                                                    symbolSize: [20, 14],
                                                    itemStyle: {{ color: '#bbdefb', borderColor: '#90caf9', borderWidth: 1 }}
                                                }},
                                                {{
                                                    name: 'NEGATIVE Media',
                                                    type: 'line',
                                                    yAxisIndex: 0,
                                                    data: [],
                                                    lineStyle: {{ width: 0 }},
                                                    symbol: 'rect',
                                                    symbolSize: [20, 14],
                                                    itemStyle: {{ color: '#ffcdd2', borderColor: '#ef9a9a', borderWidth: 1 }}
                                                }}
                                            ];
                                            
                                            // Add LED intensity series if data is available
                                            if (hasLedData && ledPercentData && ledPercentData.length > 0) {{
                                                // LED channel colors (6 distinct colors for LEDs 1-6)
                                                const ledColors = {{
                                                    0: '#9E9E9E',  // OFF - gray
                                                    1: '#E91E63',  // LED1 - pink
                                                    2: '#9C27B0',  // LED2 - purple  
                                                    3: '#3F51B5',  // LED3 - indigo
                                                    4: '#009688',  // LED4 - teal
                                                    5: '#8BC34A',  // LED5 - light green
                                                    6: '#FF9800'   // LED6 - orange
                                                }};
                                                
                                                // Create LED data with color based on channel
                                                // Use currentTimeData (which may be cleaned) for x-values
                                                // FIX: Build a lookup map ONCE instead of calling findIndex for each point (O(n²) -> O(n))
                                                let timeToOriginalIdx = null;
                                                if (dataCleaningEnabled) {{
                                                    timeToOriginalIdx = new Map();
                                                    originalTimeData.forEach((t, i) => {{
                                                        // Round to avoid floating point issues
                                                        const key = Math.round(t * 10000);
                                                        timeToOriginalIdx.set(key, i);
                                                    }});
                                                }}
                                                
                                                const ledData = currentTimeData.map((time, idx) => {{
                                                    // Map cleaned data index back to original data index
                                                    // Use lookup map for O(1) access instead of findIndex O(n)
                                                    let safeIdx = idx;
                                                    if (dataCleaningEnabled && timeToOriginalIdx) {{
                                                        const key = Math.round(time * 10000);
                                                        const originalIdx = timeToOriginalIdx.get(key);
                                                        if (originalIdx !== undefined && originalIdx < ledPercentData.length) {{
                                                            safeIdx = originalIdx;
                                                        }}
                                                    }}
                                                    const percent = (safeIdx < ledPercentData.length) ? ledPercentData[safeIdx] : 0;
                                                    const channel = (safeIdx < ledChannelData.length) ? ledChannelData[safeIdx] : 0;
                                                    return {{
                                                        value: [time, percent],
                                                        itemStyle: {{ color: ledColors[channel] || '#FF9800' }}
                                                    }};
                                                }});
                                                
                                                seriesArray.push({{
                                                    name: 'LED Intensity',
                                                    type: 'line',
                                                    yAxisIndex: 1,  // Right Y-axis
                                                    data: ledData,
                                                    lineStyle: {{ color: '#FF9800', width: 1.5, opacity: 0.8 }},
                                                    symbol: 'none',
                                                    areaStyle: {{ color: '#FF9800', opacity: 0.1 }}  // Light fill under LED line
                                                }});
                                            }}
                                            
                                            return seriesArray;
                                        }})()
                                    }};

                                    chart.setOption(option);

                                    // Update peak interval chart
                                    console.log('📊 Updating peak interval chart with', peaks.length, 'peaks');
                                    console.log('📊 Peak times:', peaks.map(p => p.time.toFixed(3)).join(', '));
                                    console.log('📊 useThresholdDetection:', useThresholdDetection);
                                    if (peaks.length > 1) {{
                                        const intervals = [];
                                        const intervalTimes = [];
                                        for (let i = 1; i < peaks.length; i++) {{
                                            const interval = peaks[i].time - peaks[i-1].time;
                                            if (interval > 0) {{
                                                intervals.push(interval);
                                                intervalTimes.push(peaks[i-1].time + interval/2);
                                            }}
                                        }}
                                        console.log('📊 Calculated', intervals.length, 'intervals for peak chart');

                                        const peakOption = {{
                                            title: {{}},  // Clear any previous title
                                            tooltip: {{ 
                                                trigger: 'item',
                                                formatter: function(params) {{
                                                    const point = params.data;
                                                    if (!point || !point.value) return '';
                                                    
                                                    const peakNum = point.value[0];
                                                    const interval = point.value[1];
                                                    const mediaType = point.media || 'NEUTRAL';
                                                    const peakTimeHours = point.peakTimeHours || 0;
                                                    const timestamp = point.timestamp || '';
                                                    
                                                    const mediaColor = mediaType === 'POSITIVE' ? '#2196F3' : 
                                                                      mediaType === 'NEGATIVE' ? '#f44336' : '#000000';
                                                    
                                                    let tooltipText = '<b>Interval #' + peakNum + '</b><br/>';
                                                    tooltipText += 'Duration: <b>' + interval.toFixed(2) + ' hours</b><br/>';
                                                    tooltipText += '<span style=\"color: ' + mediaColor + '; font-weight: bold;\">Media: ' + mediaType + '</span><br/>';
                                                    if (peakTimeHours > 0) {{
                                                        tooltipText += 'Peak at: <b>' + peakTimeHours.toFixed(2) + 'h</b><br/>';
                                                    }}
                                                    // Add LED data if available (firmware V3+)
                                                    const ledChannel = point.ledChannel || 0;
                                                    const ledPercent = point.ledPercent || 0;
                                                    if (ledChannel > 0) {{
                                                        tooltipText += '<span style=\"color: #FF9800;\">💡 LED' + ledChannel + ': ' + ledPercent + '% eff. dose</span><br/>';  // <-- % (effective dose)
                                                    }}
                                                    if (timestamp) {{
                                                        tooltipText += '<span style=\"color: #666; font-size: 11px;\">' + timestamp + '</span>';
                                                    }}
                                                    
                                                    return tooltipText;
                                                }}
                                            }},
                                            grid: {{
                                                left: 80,
                                                right: 60,
                                                top: 60,
                                                bottom: 80,
                                                containLabel: true
                                            }},
                                            xAxis: {{ 
                                                type: 'value',
                                                name: 'Interval Number',
                                                nameLocation: 'middle',
                                                nameGap: 30,
                                                nameTextStyle: {{ fontSize: 14, fontWeight: 'bold' }},
                                                axisLabel: {{ fontSize: 12 }}
                                            }},
                                            yAxis: {{ 
                                                type: 'value',
                                                name: 'Interval (hours)',
                                                nameLocation: 'middle',
                                                nameGap: 50,
                                                nameTextStyle: {{ fontSize: 14, fontWeight: 'bold' }},
                                                axisLabel: {{ fontSize: 12 }}
                                            }},
                                            dataZoom: [
                                                {{ type: 'inside', xAxisIndex: 0 }},
                                                {{ type: 'slider', xAxisIndex: 0, bottom: 80 }}
                                            ],
                                            series: [{{
                                                name: 'Peak Intervals',
                                                type: 'line',
                                                data: intervals.map((interval, idx) => {{
                                                    // Color = media that JUST FINISHED before the peak
                                                    let mediaTypeAtPeak = "NEUTRAL";
                                                    if (idx < peaks.length - 1 && mediaData.length > 0) {{
                                                        const endPeakIndex = peaks[idx + 1].index;
                                                        // Look 1 point BEFORE the peak
                                                        const beforePeak = Math.max(0, endPeakIndex - 1);
                                                        if (beforePeak < mediaData.length) {{
                                                            const media = mediaData[beforePeak];
                                                            if (media === 'POSITIVE' || media === 'NEGATIVE') {{
                                                                mediaTypeAtPeak = media;
                                                            }}
                                                        }}
                                                    }}
                                                    
                                                    // Get peak time and timestamp
                                                    let peakTimeHours = 0;
                                                    let timestamp = '';
                                                    let ledChannel = 0;
                                                    let ledPercent = 0;
                                                    if (idx + 1 < peaks.length) {{
                                                        peakTimeHours = peaks[idx + 1].time;
                                                        const peakIdx = peaks[idx + 1].index;
                                                        if (unixTimeData && peakIdx < unixTimeData.length) {{
                                                            const unixTime = unixTimeData[peakIdx];
                                                            if (unixTime) {{
                                                                const date = getOffsetCorrectedDate(unixTime);  // Use offset-corrected date
                                                                timestamp = date.toLocaleString();
                                                                if (dateOffsetDays !== 0) {{
                                                                    timestamp += ' (+' + dateOffsetDays + 'd)';
                                                                }}
                                                            }}
                                                        }}
                                                        // Get LED data at peak time (firmware V3+)
                                                        if (hasLedData && ledChannelData && ledPercentData && peakIdx < ledChannelData.length) {{
                                                            ledChannel = ledChannelData[peakIdx];
                                                            ledPercent = ledPercentData[peakIdx];
                                                        }}
                                                    }}
                                                    
                                                    // Determine color
                                                    const color = mediaTypeAtPeak === "POSITIVE" ? '#2196F3' : 
                                                                 mediaTypeAtPeak === "NEGATIVE" ? '#f44336' : '#000000';
                                                    
                                                    return {{
                                                        value: [idx + 1, interval],
                                                        media: mediaTypeAtPeak,
                                                        peakTimeHours: peakTimeHours,
                                                        timestamp: timestamp,
                                                        ledChannel: ledChannel,  // LED channel (1-6)
                                                        ledPercent: ledPercent,  // LED effective dose (%)  <-- %
                                                        itemStyle: {{ color: color, borderWidth: 2, borderColor: '#fff' }}
                                                    }};
                                                }}),
                                                lineStyle: {{ 
                                                    color: '#333333',
                                                    width: 2.5,
                                                    type: 'solid'
                                                }},
                                                symbol: 'circle',
                                                symbolSize: 12,
                                                showSymbol: true
                                            }}]
                                        }};
                                        
                                        peakChart.setOption(peakOption);
                                        console.log('📊 Peak chart updated successfully with', intervals.length, 'intervals');
                                        
                                        // Update stats (including Peak Count)
                                        const avgInterval = intervals.reduce((a, b) => a + b, 0) / intervals.length;
                                        const avgIntervalElement = document.getElementById('avg-interval');
                                        const growthRateElement = document.getElementById('growth-rate');
                                        const peakCountElement = document.getElementById('peak-count');
                                        if (avgIntervalElement) {{ avgIntervalElement.textContent = avgInterval.toFixed(2); }}
                                        if (growthRateElement) {{ growthRateElement.textContent = (1/avgInterval).toFixed(2) + '/h'; }}
                                        if (peakCountElement) {{ peakCountElement.textContent = String(peaks.length); }}
                                    }} else {{
                                        // Clear peak chart if not enough peaks
                                        console.log('📊 Not enough peaks for interval chart (need 2+, have', peaks.length, ')');
                                        peakChart.setOption({{
                                            title: {{}},  // No internal title - use HTML heading
                                            grid: {{
                                                left: 80,
                                                right: 60,
                                                top: 60,
                                                bottom: 80,
                                                containLabel: false
                                            }},
                                            xAxis: {{ 
                                                type: 'value',
                                                name: 'Interval Number',
                                                nameLocation: 'middle',
                                                nameGap: 30,
                                                nameTextStyle: {{ fontSize: 14, fontWeight: 'bold' }},
                                                axisLabel: {{ fontSize: 12 }}
                                            }},
                                            yAxis: {{ 
                                                type: 'value',
                                                name: 'Interval (hours)',
                                                nameLocation: 'middle',
                                                nameGap: 50,
                                                nameTextStyle: {{ fontSize: 14, fontWeight: 'bold' }},
                                                axisLabel: {{ fontSize: 12 }}
                                            }},
                                            series: []
                                        }}, true);  // notMerge=true forces complete redraw
                                        
                                        const avgIntervalElement = document.getElementById('avg-interval');
                                        const growthRateElement = document.getElementById('growth-rate');
                                        const peakCountElement = document.getElementById('peak-count');
                                        if (avgIntervalElement) {{ avgIntervalElement.textContent = '--'; }}
                                        if (growthRateElement) {{ growthRateElement.textContent = '--'; }}
                                        if (peakCountElement) {{ peakCountElement.textContent = '0'; }}
                                    }}
                                    
                                    // Duration is already calculated correctly in the backend, just display it
                                    // No need to recalculate from time data (which could be in different units)
                                }}
                                
                                // Toggle between Dilution_Event column and threshold detection
                                let useThresholdDetection = false;  // Default: use Dilution_Event if available
                                
                                function toggleDetectionMode(checked) {{
                                    useThresholdDetection = checked;
                                    console.log('🔄 Detection mode changed:', useThresholdDetection ? 'Threshold' : 'Dilution_Event');
                                    updateChart();  // Redetect peaks with new mode
                                }}
                                
                                // Threshold update function (from slider)
                                function updateThreshold(value) {{
                                    console.log('🎚️ updateThreshold() called with value:', value);
                                    currentThreshold = parseFloat(value);
                                    document.getElementById('threshold-value').textContent = value;
                                    document.getElementById('threshold-input').value = value;
                                    updateChart();
                                }}
                                
                                // Threshold update function (from text input)
                                function updateThresholdFromInput(value) {{
                                    currentThreshold = parseFloat(value);
                                    document.getElementById('threshold-value').textContent = value;
                                    document.getElementById('threshold-slider').value = value;
                                    updateChart();
                                }}
                                
                                // Peak separation update function (from slider)
                                function updatePeakSeparation(value) {{
                                    minPeakSeparation = parseFloat(value);
                                    document.getElementById('peak-separation-value').textContent = value + 'h';
                                    document.getElementById('peak-separation-input').value = value;
                                    updateChart();
                                }}
                                
                                // Peak separation update function (from text input)
                                function updatePeakSeparationFromInput(value) {{
                                    minPeakSeparation = parseFloat(value);
                                    document.getElementById('peak-separation-value').textContent = value + 'h';
                                    document.getElementById('peak-separation-slider').value = value;
                                    updateChart();
                                }}
                                
                                // Logarithmic scale conversion for outlier threshold
                                // Maps slider (0-100) to threshold (1.000-10.0) with log scale
                                // More precision at low end (1.000-1.1) where it matters most
                                function sliderToThreshold(sliderValue) {{
                                    // Map 0-100 to log scale from 1.0 to 10.0
                                    // log(1) = 0, log(10) = 1
                                    const minLog = Math.log10(1.0);
                                    const maxLog = Math.log10(10.0);
                                    const logValue = minLog + (sliderValue / 100.0) * (maxLog - minLog);
                                    return Math.pow(10, logValue);
                                }}
                                
                                function thresholdToSlider(thresholdValue) {{
                                    // Inverse: map threshold (1.0-10.0) back to slider (0-100)
                                    const minLog = Math.log10(1.0);
                                    const maxLog = Math.log10(10.0);
                                    const logValue = Math.log10(thresholdValue);
                                    return ((logValue - minLog) / (maxLog - minLog)) * 100.0;
                                }}
                                
                                // Data cleaning threshold update function (from slider with log scale)
                                function updateCleanThresholdFromSlider(sliderValue) {{
                                    cleanThreshold = sliderToThreshold(parseFloat(sliderValue));
                                    document.getElementById('clean-threshold-value').textContent = cleanThreshold.toFixed(3) + 'x';
                                    document.getElementById('clean-threshold-input').value = cleanThreshold.toFixed(3);
                                    if (dataCleaningEnabled) {{
                                        const result = cleanData(originalData, originalTimeData, cleanThreshold);
                                        cleanedData = result.cleanedData;
                                        cleanedTimeData = result.cleanedTime;
                                        updateChart();
                                    }}
                                }}
                                
                                // Data cleaning threshold update function (from text input)
                                function updateCleanThresholdFromInput(value) {{
                                    cleanThreshold = parseFloat(value);
                                    document.getElementById('clean-threshold-value').textContent = cleanThreshold.toFixed(3) + 'x';
                                    const sliderValue = thresholdToSlider(cleanThreshold);
                                    document.getElementById('clean-threshold-slider').value = sliderValue.toFixed(0);
                                    if (dataCleaningEnabled) {{
                                        const result = cleanData(originalData, originalTimeData, cleanThreshold);
                                        cleanedData = result.cleanedData;
                                        cleanedTimeData = result.cleanedTime;
                                        updateChart();
                                    }}
                                }}
                                
                                // Toggle data cleaning on/off
                                function toggleDataCleaning() {{
                                    dataCleaningEnabled = !dataCleaningEnabled;
                                    const button = document.getElementById('cleaning-toggle');
                                    
                                    if (dataCleaningEnabled) {{
                                        const result = cleanData(originalData, originalTimeData, cleanThreshold);
                                        cleanedData = result.cleanedData;
                                        cleanedTimeData = result.cleanedTime;
                                        button.textContent = 'Disable Cleaning';
                                        button.style.background = '#4CAF50';
                                        console.log('Data cleaning enabled. Removed ' + (originalData.length - cleanedData.length) + ' outlier points.');
                                    }} else {{
                                        button.textContent = 'Enable Cleaning';
                                        button.style.background = '#9E9E9E';
                                        console.log('Data cleaning disabled. Using original data.');
                                    }}
                                    
                                    updateChart();
                                }}
                                
                                // Update axis ranges function
                                function updateAxisRanges() {{
                                    console.log('🎛️ updateAxisRanges() called');
                                    const xMin = parseFloat(document.getElementById('x-min-input').value);
                                    const xMax = parseFloat(document.getElementById('x-max-input').value);
                                    const yMin = parseFloat(document.getElementById('y-min-input').value);
                                    const yMax = parseFloat(document.getElementById('y-max-input').value);
                                    
                                    const peakXMin = parseFloat(document.getElementById('peak-x-min-input').value);
                                    const peakXMax = parseFloat(document.getElementById('peak-x-max-input').value);
                                    const peakYMin = parseFloat(document.getElementById('peak-y-min-input').value);
                                    const peakYMax = parseFloat(document.getElementById('peak-y-max-input').value);
                                    
                                    console.log('🎛️ Peak axis values:', peakXMin, peakXMax, peakYMin, peakYMax);
                                    
                                    // Update OD chart axes
                                    if (typeof chart !== 'undefined' && chart) {{
                                        chart.setOption({{
                                            xAxis: {{
                                                min: xMin,
                                                max: xMax
                                            }},
                                            yAxis: {{
                                                min: yMin,
                                                max: yMax
                                            }}
                                        }});
                                        console.log('✅ OD chart axes updated');
                                    }} else {{
                                        console.error('❌ OD chart not found');
                                    }}
                                    
                                    // Update peak chart axes
                                    if (typeof peakChart !== 'undefined' && peakChart) {{
                                        peakChart.setOption({{
                                            xAxis: {{
                                                min: peakXMin,
                                                max: peakXMax
                                            }},
                                            yAxis: {{
                                                min: peakYMin,
                                                max: peakYMax
                                            }}
                                        }});
                                        console.log('✅ Peak chart axes updated');
                                    }} else {{
                                        console.error('❌ Peak chart not found or not initialized');
                                    }}
                                }}
                                
                                // Reset zoom functions for individual charts
                                function resetODZoom() {{
                                    chart.dispatchAction({{ type: 'dataZoom', start: 0, end: 100 }});
                                    
                                    // Reset axis inputs to auto-calculated values
                                    const timeExtent = [Math.min(...timeData), Math.max(...timeData)];
                                    const odExtent = [Math.min(...odData), Math.max(...odData)];
                                    
                                    document.getElementById('x-min-input').value = Math.max(0, timeExtent[0] - 0.5);
                                    document.getElementById('x-max-input').value = timeExtent[1] + 0.5;
                                    document.getElementById('y-min-input').value = Math.max(0, odExtent[0] - 0.2);
                                    document.getElementById('y-max-input').value = odExtent[1] + 0.5;
                                    
                                    updateAxisRanges();
                                }}
                                
                                function resetPeakZoom() {{
                                    peakChart.dispatchAction({{ type: 'dataZoom', start: 0, end: 100 }});
                                    
                                    const timeExtent = [Math.min(...timeData), Math.max(...timeData)];
                                    document.getElementById('peak-x-min-input').value = Math.max(0, timeExtent[0] - 0.5);
                                    document.getElementById('peak-x-max-input').value = timeExtent[1] + 0.5;
                                    document.getElementById('peak-y-min-input').value = 0;
                                    document.getElementById('peak-y-max-input').value = 24;
                                    
                                    updateAxisRanges();
                                }}
                                
                                // Legacy function for backward compatibility
                                function resetZoom() {{
                                    resetODZoom();
                                    resetPeakZoom();
                                }}
                                
                                // Download chart functions
                                function downloadODChart() {{
                                    const url = chart.getDataURL({{
                                        type: 'png',
                                        pixelRatio: 2,
                                        backgroundColor: '#fff'
                                    }});
                                    const link = document.createElement('a');
                                    link.href = url;
                                    link.download = 'od_chart.png';
                                    link.click();
                                }}
                                
                                function downloadPeakChart() {{
                                    const url = peakChart.getDataURL({{
                                        type: 'png',
                                        pixelRatio: 2,
                                        backgroundColor: '#fff'
                                    }});
                                    const link = document.createElement('a');
                                    link.href = url;
                                    link.download = 'peak_intervals_chart.png';
                                    link.click();
                                }}
                                // Initial chart render
                                updateChart();
                            <\\/script>
                        <\\/body>
                        <\\/html>
                        `);
                        newWindow.document.close();
                    }})
                    .catch(error => {{
                        console.error('❌ Error loading experiment data:', error);
                        console.error('❌ Error details:', error.stack);
                        console.error('❌ Fetch URL was: /load_experiment_data');
                        alert('Failed to load experiment data: ' + error.message + '\\n\\nCheck browser console (F12) for details.');
                    }});
                }};
            }}
        ''')

def show_file_picker():
    """Show a native file picker to select CSV files from anywhere on your computer."""
    with ui.dialog() as file_dialog, ui.card().style('min-width: 500px'):
        ui.label('Load CSV File').classes('text-h6 q-mb-md')
        ui.label('Select a CSV file from your computer:').classes('q-mb-md')
        
        # Add progress indicator (hidden by default)
        progress_label = ui.label('').classes('q-mb-sm')
        progress_label.visible = False
        
        async def handle_upload(e):
            """Handle the uploaded CSV file."""
            try:
                # Get the uploaded file content.
                # NiceGUI changed the upload API across versions: older builds
                # exposed e.content (a sync file-like object), while 3.4.1+ uses
                # e.file (a FileUpload with an async .read()). Support both so
                # this works on every machine's NiceGUI version.
                if hasattr(e, 'file'):
                    content = await e.file.read()
                else:
                    content = e.content.read()
                
                # Create a permanent file in the current directory
                import datetime
                timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
                permanent_file_path = f'uploaded_csv_{timestamp}.csv'
                
                # Write the uploaded copy as RAW BYTES to preserve the original line
                # endings. Writing in text mode on Windows translates '\n' -> '\r\n',
                # which turns the file's existing '\r\n' into '\r\r\n' and inserts a
                # blank line between every data row (parser then sees "1 column" rows
                # and reports "No experiments found").
                data_bytes = content if isinstance(content, (bytes, bytearray)) else str(content).encode('utf-8')
                with open(permanent_file_path, 'wb') as perm_file:
                    perm_file.write(data_bytes)
                
                print(f"📁 Saved uploaded CSV to: {permanent_file_path}")

                # V1.9: prune old uploaded_csv_*.csv copies so they don't pile up
                # (these can be 100+ MB each). The just-saved file is <1h old and
                # the cleanup only removes files older than 1 hour, so the current
                # upload and any file being viewed in a sibling tab are preserved.
                try:
                    cleanup_old_uploaded_files()
                except Exception as _prune_err:
                    print(f"⚠️ Upload prune skipped: {_prune_err}")
                
                # NOTE: Do NOT clear global live view data arrays here!
                # The CSV viewer opens in a separate browser window with its own data.
                # Clearing these would corrupt the live experiment view.
                # global time_data, od_data, peak_times, peak_intervals, media_areas  # REMOVED
                
                # For CSV viewer, we need to show experiment selection, not load into live view
                print(f"📊 Redirecting to CSV viewer for file: {permanent_file_path}")
                
                # Call the experiment scanning function instead
                try:
                    experiments = []
                    
                    # Show progress
                    progress_label.visible = True
                    progress_label.text = '📊 Reading file...'
                    await asyncio.sleep(0.1)  # Let UI update
                    
                    with open(permanent_file_path, 'r') as f:
                        all_lines = f.readlines()
                    
                    total_lines = len(all_lines)
                    print(f"📄 File has {total_lines} total lines")
                    
                    progress_label.text = f'🔍 Scanning {total_lines:,} lines for experiments...'
                    await asyncio.sleep(0.1)  # Let UI update
                    
                    # FAST but COMPLETE header detection - find ALL headers efficiently
                    print(f"🚀 Using targeted header detection for upload")
                    
                    # Use grep-like approach: only check lines that contain header keywords
                    for line_num, line in enumerate(all_lines):
                        # Yield to UI every 1000 lines to prevent freezing
                        if line_num % 1000 == 0 and line_num > 0:
                            progress_label.text = f'🔍 Scanning... {line_num:,} / {total_lines:,} lines ({int(100*line_num/total_lines)}%)'
                            await asyncio.sleep(0)  # Yield to UI thread
                        # Only check lines that look like headers (contain both time and OD keywords)
                        if ('uptime' in line.lower() or 'unixtime' in line.lower()) and 'od940' in line.lower():
                            print(f"📄 Found header candidate at line {line_num+1}: {line.strip()[:150]}...")
                            print(f"📄 Full header columns: {line.strip().split(',')}")
                            print(f"📄 Header column count: {len(line.strip().split(','))}")
                            
                            # Check if data rows have more columns than header.
                            # Find the first NON-BLANK line after the header (blank lines
                            # can appear in CRLF-doubled copies).
                            if line_num + 1 < len(all_lines):
                                sample_data = ""
                                for j in range(line_num + 1, len(all_lines)):
                                    if all_lines[j].strip():
                                        sample_data = all_lines[j].strip()
                                        break
                                data_cols = sample_data.split(',')
                                print(f"📄 Sample data row: {sample_data[:150]}...")
                                print(f"📄 Data column count: {len(data_cols)}")
                                if len(data_cols) > len(line.strip().split(',')):
                                    print(f"⚠️ DATA HAS MORE COLUMNS THAN HEADER! Missing columns in header.")
                                    print(f"📄 Extra data columns: {data_cols[len(line.strip().split(',')):]}")
                                    # Add mediaType to header if missing
                                    header_cols = line.strip().split(',')
                                    if len(data_cols) == len(header_cols) + 1:
                                        header_cols.append('mediaType')
                                        print(f"🔧 FIXED: Added 'mediaType' to header: {header_cols}")
                                        line = ','.join(header_cols)  # Update the line for processing
                            
                            # Verify it's actually a header
                            has_time = any(col in line.lower() for col in ['uptime', 'time', 'timestamp', 'unix'])
                            has_od = any(col in line.lower() for col in ['od940', 'od', 'optical', 'density'])
                            
                            if has_time and has_od:
                                header = next(csv.reader([line.strip()]))
                                
                                # Count data rows. Skip blank lines (CRLF doubling) instead
                                # of terminating on them; only stop at the next experiment's
                                # header line.
                                data_count = 0
                                for i in range(line_num + 1, len(all_lines)):
                                    data_line = all_lines[i].strip()
                                    if not data_line:
                                        continue  # blank line - skip, don't end the experiment
                                    if ('uptime' in data_line.lower() or 'unixtime' in data_line.lower()) and 'od940' in data_line.lower():
                                        break  # start of another experiment header
                                    if ',' in data_line:
                                        data_count += 1
                                
                                if data_count >= 3:
                                    # Find column indices (same as in the endpoint)
                                    time_idx = None
                                    od_idx = None
                                    media_idx = None
                                    
                                    for i, col in enumerate(header):
                                        col_l = col.lower()
                                        if any(time_col in col_l for time_col in ['uptime', 'time', 'timestamp', 'unix']):
                                            time_idx = i
                                        elif any(od_col in col_l for od_col in ['od940', 'od', 'optical', 'density']):
                                            od_idx = i
                                    # Prefer mediaType explicitly; fallback to other media-like columns
                                    for i, col in enumerate(header):
                                        col_l = col.lower()
                                        if 'mediatype' in col_l or 'media_type' in col_l or 'media type' in col_l:
                                            media_idx = i
                                            break
                                    if media_idx is None:
                                        for i, col in enumerate(header):
                                            col_l = col.lower()
                                            # Avoid choosing mediaTemp if mediaType exists; but allow as last resort
                                            if 'mediatemp' in col_l:
                                                media_idx = i
                                                break
                                    if media_idx is None:
                                        for i, col in enumerate(header):
                                            col_l = col.lower()
                                            if 'media' in col_l:
                                                media_idx = i
                                                break
                                    
                                    experiments.append({
                                        'header_line': line_num + 1,
                                        'data_rows': data_count,
                                        'time_idx': time_idx,
                                        'od_idx': od_idx,
                                        'media_idx': media_idx,
                                        'has_media_type': media_idx is not None,
                                        'start_line': line_num + 1,
                                        'header_columns': header
                                    })
                                    print(f"🔍 Found experiment {len(experiments)}: Line {line_num + 1}, {data_count} points")
                    
                    if experiments:
                        progress_label.text = f'✅ Found {len(experiments)} experiment(s)!'
                        await asyncio.sleep(0.3)  # Show success briefly
                        print(f"✅ Found {len(experiments)} experiments, showing selection dialog")
                        # Show the experiment selection dialog with the new file
                        show_experiment_selection_dialog(experiments, permanent_file_path)
                    else:
                        progress_label.visible = False
                        ui.notify('❌ No valid experiments found in this CSV file', type='negative')
                        print(f"❌ No experiments found in {permanent_file_path}")
                        
                except Exception as scan_error:
                    progress_label.visible = False
                    print(f"❌ Error scanning CSV: {scan_error}")
                    ui.notify(f'❌ Error reading CSV file: {scan_error}', type='negative')
                
                progress_label.visible = False
                file_dialog.close()
                
            except Exception as error:
                ui.notify(f'❌ Failed to load CSV file: {error}', type='negative')
                print(f"❌ Upload error: {error}")
                import traceback
                traceback.print_exc()
        
        # File upload component
        ui.upload(
            on_upload=handle_upload,
            multiple=False,
            auto_upload=True,
            label='Click to select CSV file...'
        ).props('accept=".csv"').classes('w-full')
        
        ui.separator().classes('q-my-md')
        
        with ui.row().classes('w-full justify-end q-mt-md'):
            ui.button('Close', on_click=file_dialog.close)
    
    file_dialog.open()
        
@app.post("/upload_csv")
async def upload_csv(request: Request):
    """Save uploaded CSV content to a temporary file."""
    global current_session_csv
    
    data = await request.json()
    filename = data.get("filename", "uploaded_file.csv")
    content = data.get("content", "")
    
    # Clean up previous session's CSV file when a new one is uploaded
    if current_session_csv and os.path.exists(current_session_csv):
        try:
            os.remove(current_session_csv)
            print(f"🗑️ Cleaned up previous session CSV: {current_session_csv}")
        except Exception as cleanup_error:
            print(f"⚠️ Failed to clean up previous CSV: {cleanup_error}")
    
    # Sanitize filename
    safe_filename = "".join(c for c in filename if c.isalnum() or c in (' ', '.', '_')).rstrip()
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    temp_filename = f"uploaded_csv_{timestamp}.csv"
    
    temp_filepath = os.path.join(temp_dir, temp_filename)
    
    try:
        # newline="" prevents Windows newline translation that would otherwise
        # double '\r\n' -> '\r\r\n' and insert blank lines between data rows.
        with open(temp_filepath, "w", newline="") as f:
            f.write(content)
        print(f"📁 Saved uploaded CSV to: {temp_filepath}")
        
        # Track this as the current session's CSV file
        current_session_csv = temp_filepath
        
        return JSONResponse({"filepath": temp_filepath})
    except Exception as e:
        print(f"❌ Error saving uploaded CSV: {e}")
        return JSONResponse({"error": str(e)}, status_code=500)

# Helper function to dynamically detect media_type column
def _detect_media_type_index(all_lines, start_line, header_columns):
    media_idx = None
    
    # 1. Try to find mediaType in header (case-insensitive)
    for i, col in enumerate(header_columns):
        if col.lower() == 'mediatype':
            media_idx = i
            print(f"📊 Found mediaType in header at index {media_idx} (column: '{col}')")
            return media_idx

    # 2. If not in header, check multiple data rows for media type values
    print(f"📊 mediaType not found in header, checking data rows for media types...")
    
    # Check up to 100 data rows to find media types (they may appear later in the experiment)
    rows_to_check = min(100, len(all_lines) - start_line)
    
    for row_offset in range(rows_to_check):
        line_idx = start_line + row_offset
        if line_idx >= len(all_lines):
            break
            
        line = all_lines[line_idx].strip()
        if not line or ('upTime' in line and 'OD940' in line):  # Skip empty lines or headers
            continue
            
        try:
            row = next(csv.reader([line]))
            for i, value in enumerate(row):
                normalized_value = value.upper().strip()
                if normalized_value in ['POSITIVE', 'NEGATIVE', 'NEUTRAL']:
                    media_idx = i
                    print(f"📊 Found mediaType in data row {line_idx} at index {media_idx} (value: '{normalized_value}')")
                    return media_idx
        except csv.Error as e:
            continue
    
    print(f"📊 No mediaType column found in header or first {rows_to_check} data rows.")
    return media_idx


@app.post('/load_experiment_data')
async def load_experiment_data(request: Request):
    print("🚀 ENDPOINT CALLED: /load_experiment_data")
    data = await request.json()
    filepath = data.get('filepath')
    experiment_index = data.get('experiment_index')
    print(f"🔍 Loading experiment data: filepath={filepath}, index={experiment_index}")
    print(f"📋 Request data: {data}")

    if not filepath or experiment_index is None:
        return JSONResponse({'error': 'Filepath and experiment index are required'}, status_code=400)

    # Use a shared file path if running in a cloud environment
    if os.environ.get('NICEGUI_ENVIRONMENT') == 'web':
        filepath = f"/tmp/{filepath}"

    print(f"📁 File exists: {os.path.exists(filepath)}")
    if not os.path.exists(filepath):
        return JSONResponse({'error': f'File not found at {filepath}'}, status_code=404)

    try:
        with open(filepath, 'r') as f:
            all_lines = f.readlines()

        experiments = scan_for_experiments(filepath)
        if not experiments or experiment_index >= len(experiments):
            return JSONResponse({'error': 'Experiment index out of bounds'}, status_code=400)

        exp = experiments[experiment_index]
        
        # Simple, direct media type detection for the selected experiment
        header = exp.get('header_columns', [])
        media_idx = None
        
        # 1. Check header
        for i, col in enumerate(header):
            if col.lower() == 'mediatype':
                media_idx = i
                print(f"📊 Found mediaType in header at index {media_idx}")
                break
        
        # 2. If not in header, check last data row for extra column
        if media_idx is None:
            last_data_idx = exp['start_line'] + exp['data_rows'] - 1
            if last_data_idx < len(all_lines):
                last_line = all_lines[last_data_idx].strip()
                try:
                    row = next(csv.reader([last_line]))
                    if len(row) > len(header):
                        extra_col_idx = len(header)
                        if row[extra_col_idx].strip().upper() in ['POSITIVE', 'NEGATIVE', 'NEUTRAL']:
                            media_idx = extra_col_idx
                            header.append('mediaType') # Virtually add header
                            exp['header_columns'] = header # Update experiment info
                            print(f"📊 Found mediaType in extra data column {media_idx}. Virtually added to header.")
                except Exception:
                    pass # Ignore parsing errors on a single line

        # Proceed with data loading
        time_idx = exp.get('time_idx')
        od_idx = exp.get('od_idx')
        
        if time_idx is None or od_idx is None:
            return JSONResponse({'error': 'Time or OD column not found in experiment header'}, status_code=400)

        time_data_local = []
        od_data_local = []
        media_data_local = []
        unix_time_data_local = []
        dilution_event_data_local = []  # NEW: Track dilution events from CSV
        led_channel_data_local = []  # NEW: Track LED channel (1-6)
        led_percent_data_local = []  # NEW: Track LED effective dose (%)  <-- %
        
        # Check if Dilution_Event column exists
        dilution_event_idx = None
        if 'Dilution_Event' in header:
            dilution_event_idx = header.index('Dilution_Event')
            print(f"✅ Found Dilution_Event column at index {dilution_event_idx}")
        else:
            print(f"ℹ️  No Dilution_Event column found - will use threshold detection")
        
        # Check if LED_Channel and LED_Percent columns exist (firmware V3+)
        led_channel_idx = None
        led_percent_idx = None
        if 'LED_Channel' in header:
            led_channel_idx = header.index('LED_Channel')
            print(f"✅ Found LED_Channel column at index {led_channel_idx}")
        if 'LED_Percent' in header:
            led_percent_idx = header.index('LED_Percent')
            print(f"✅ Found LED_Percent column at index {led_percent_idx}")
        
        start_line = exp['start_line']
        num_rows = exp['data_rows']
        # OPTIMIZATION: Target ~5,000 points for viewer to prevent browser freeze
        # 50,000 was causing "Pages Unresponsive" errors with large JSON payloads
        stride = max(1, num_rows // 5000) 
        
        # PRE-SCAN: Find all dilution event rows so we don't skip them with stride
        # This ensures peak detection works correctly even with large files
        dilution_event_rows = set()
        if dilution_event_idx is not None:
            print(f"🔍 Pre-scanning for Dilution_Event=1 rows...")
            for i in range(num_rows):
                line_idx = start_line + i
                if line_idx >= len(all_lines):
                    break
                line = all_lines[line_idx].strip()
                if not line:
                    continue
                try:
                    row = next(csv.reader([line]))
                    if dilution_event_idx < len(row) and row[dilution_event_idx].strip() == '1':
                        dilution_event_rows.add(i)
                except:
                    pass
            print(f"✅ Found {len(dilution_event_rows)} dilution events in CSV")
        
        # Initialize duplicate detection variables
        has_duplicates = False
        time_spacing = 10  # Default 10 second intervals (EEVO records every 10 seconds)

        # Create set of row indices to process (stride + dilution events)
        rows_to_process = set(range(0, num_rows, stride))
        rows_to_process.update(dilution_event_rows)  # Always include dilution events
        rows_to_process = sorted(rows_to_process)
        
        # Track original row indices for proper sequential time calculation
        original_row_indices = []
        
        for i in rows_to_process:
            line_idx = start_line + i
            if line_idx >= len(all_lines):
                break
            line = all_lines[line_idx].strip()
            if not line:
                continue
            
            try:
                row = next(csv.reader([line]))
                
                # Time and OD data
                # Keep time in original units (don't convert to hours yet)
                # The relative time conversion logic below will handle the conversion
                time_val = float(row[time_idx])
                od_val = float(row[od_idx])
                
                time_data_local.append(time_val)
                od_data_local.append(od_val)
                original_row_indices.append(i)  # Track original row position
                
                # Unix timestamp data (preserve original for tooltips - from 9-22 working logic)
                unix_time_idx = exp.get('unix_time_idx')
                unix_timestamp = None
                if unix_time_idx is not None and unix_time_idx < len(row):
                    try:
                        # Get the original Unix timestamp from CSV for tooltips
                        original_unix = int(float(row[unix_time_idx]))
                        unix_timestamp = original_unix
                    except (ValueError, TypeError):
                        # Generate sequential Unix timestamp as fallback for tooltip
                        if len(unix_time_data_local) == 0 and unix_time_idx is not None:
                            # Get first timestamp as baseline
                            try:
                                first_data_line = all_lines[exp['start_line']].strip()
                                first_row = next(csv.reader([first_data_line]))
                                if len(first_row) > unix_time_idx:
                                    first_unix = int(float(first_row[unix_time_idx]))
                                    unix_timestamp = first_unix + (len(unix_time_data_local) * 1)  # 1 second intervals
                            except:
                                pass
                
                unix_time_data_local.append(unix_timestamp)
                
                # Media Type Data
                if media_idx is not None and media_idx < len(row):
                    media_type = row[media_idx].strip().upper()
                    if media_type in ['POSITIVE', 'NEGATIVE', 'NEUTRAL']:
                        media_data_local.append(media_type)
                    else:
                        media_data_local.append('NEUTRAL') # Default empty/invalid to NEUTRAL
                else:
                    media_data_local.append('NEUTRAL') # Default missing column to NEUTRAL
                
                # Dilution Event Data (NEW)
                if dilution_event_idx is not None and dilution_event_idx < len(row):
                    try:
                        dilution_event = int(row[dilution_event_idx])
                        dilution_event_data_local.append(dilution_event)
                    except (ValueError, TypeError):
                        dilution_event_data_local.append(0)  # Default to no event
                else:
                    dilution_event_data_local.append(0)  # No column = no events
                
                # LED Channel Data (firmware V3+)
                if led_channel_idx is not None and led_channel_idx < len(row):
                    try:
                        led_channel = int(row[led_channel_idx])
                        led_channel_data_local.append(led_channel)
                    except (ValueError, TypeError):
                        led_channel_data_local.append(0)  # Default to 0 (unknown)
                else:
                    led_channel_data_local.append(0)  # No column = unknown
                
                # LED Percent Data (firmware V3+ - effective dose in %)  <-- %
                if led_percent_idx is not None and led_percent_idx < len(row):
                    try:
                        led_percent = int(row[led_percent_idx])
                        led_percent_data_local.append(led_percent)
                    except (ValueError, TypeError):
                        led_percent_data_local.append(0)  # Default to 0%
                else:
                    led_percent_data_local.append(0)  # No column = 0%

            except (ValueError, IndexError):
                continue
        
        # Calculate actual time spacing from Unix timestamps (if available)
        if len(unix_time_data_local) >= 3:
            # Get the most common interval between consecutive timestamps
            intervals = []
            valid_unix = [t for t in unix_time_data_local[:20] if t is not None]  # Sample first 20 points
            for i in range(1, len(valid_unix)):
                interval = valid_unix[i] - valid_unix[i-1]
                if 1 <= interval <= 120:  # Reasonable range: 1 second to 2 minutes
                    intervals.append(interval)
            if intervals:
                # Use the most common interval (mode)
                from collections import Counter
                most_common = Counter(intervals).most_common(1)
                if most_common:
                    time_spacing = most_common[0][0]
                    print(f"✅ Detected time spacing from Unix timestamps: {time_spacing} seconds")
        
        # ROBUST TIME VALIDATION - detect corrupted time data
        use_sequential_time = False
        sequential_reason = ""
        
        if len(time_data_local) > 1:
            # Check 1: Duplicates
            seen_times = set()
            for t in time_data_local:
                if t in seen_times:
                    use_sequential_time = True
                    sequential_reason = "duplicate timestamps"
                    break
                seen_times.add(t)
            
            # Check 2: Time going backwards
            if not use_sequential_time:
                for i in range(1, len(time_data_local)):
                    if time_data_local[i] < time_data_local[i-1]:
                        use_sequential_time = True
                        sequential_reason = "time going backwards"
                        break
            
            # Check 3: Unreasonably large time values (> 10 years)
            if not use_sequential_time:
                max_time = max(time_data_local)
                if max_time > 87600:  # 10 years in hours
                    use_sequential_time = True
                    sequential_reason = f"unreasonably large time ({max_time:.0f}h)"
            
            # Check 4: Large gaps between points (> 24 hours suggests corrupted data)
            if not use_sequential_time:
                for i in range(1, len(time_data_local)):
                    gap = abs(time_data_local[i] - time_data_local[i-1])
                    if gap > 24:
                        use_sequential_time = True
                        sequential_reason = f"large time gap ({gap:.1f}h)"
                        break
            
            # Check 5: Row count vs time span mismatch (KEY FIX for your issue!)
            # If we have Unix timestamps, verify row count makes sense for the duration
            if not use_sequential_time and len(unix_time_data_local) >= 2:
                if unix_time_data_local[0] is not None and unix_time_data_local[-1] is not None:
                    actual_duration_seconds = unix_time_data_local[-1] - unix_time_data_local[0]
                    expected_rows = actual_duration_seconds / time_spacing
                    actual_rows = len(time_data_local)
                    
                    # If we have 3x more rows than expected, time data is corrupted
                    if actual_rows > expected_rows * 3:
                        use_sequential_time = True
                        sequential_reason = f"row count mismatch (have {actual_rows}, expect ~{int(expected_rows)} for {actual_duration_seconds/3600:.1f}h)"
        
        # Apply time fix if needed - use Unix timestamps with offset correction for time jumps
        if use_sequential_time:
            print(f"⚠️  {sequential_reason} - using Unix timestamps with jump correction")
            
            # BETTER APPROACH: Use actual Unix timestamps but handle backwards jumps
            # by adding offset when time goes backwards significantly
            valid_unix = [t for t in unix_time_data_local if t is not None]
            
            if len(valid_unix) >= 2:
                # Detect and correct time jumps (when experiments are concatenated)
                corrected_times = []
                time_offset = 0
                prev_unix = valid_unix[0]
                
                for i, unix_t in enumerate(unix_time_data_local):
                    if unix_t is None:
                        # Use interpolated time for missing timestamps
                        if corrected_times:
                            corrected_times.append(corrected_times[-1] + time_spacing)
                        else:
                            corrected_times.append(0)
                    else:
                        # Check for backwards time jump (> 1 hour backwards = likely new experiment)
                        if unix_t < prev_unix - 3600:
                            # Time jumped backwards - add offset to continue from previous time
                            jump_size = prev_unix - unix_t
                            time_offset += jump_size + 3600  # Add the jump size plus 1 hour gap
                            print(f"⚠️  Detected time jump at index {i}: {jump_size/3600:.1f}h backwards, adding offset")
                        
                        corrected_time = unix_t + time_offset
                        corrected_times.append(corrected_time)
                        prev_unix = unix_t
                
                # Convert to hours relative to start
                if corrected_times:
                    start_time = corrected_times[0]
                    time_data_local = [(t - start_time) / 3600.0 for t in corrected_times]
                    # ALSO update unix_time_data_local with corrected timestamps for tooltip display
                    unix_time_data_local = corrected_times.copy()
                    total_duration = (corrected_times[-1] - corrected_times[0]) / 3600.0
                    print(f"✅ Using Unix timestamps with jump correction: {total_duration:.2f}h total duration")
                    print(f"✅ Corrected Unix timestamps for tooltip display (first: {unix_time_data_local[0]}, last: {unix_time_data_local[-1]})")
                    has_duplicates = True
            else:
                # Fallback: use default interval spacing based on original row positions
                if original_row_indices and num_rows > 0:
                    time_data_local = [row_idx * time_spacing / 3600.0 for row_idx in original_row_indices]
                    print(f"✅ Fallback: Using sequential time with {time_spacing}s intervals")
                else:
                    time_data_local = [i * time_spacing / 3600.0 for i in range(len(time_data_local))]
                    print(f"✅ Fallback: Using sequential time with {time_spacing}s intervals")
                has_duplicates = True
        
        has_media_type = len(set(media_data_local)) > 1 or (len(set(media_data_local)) == 1 and 'NEUTRAL' not in set(media_data_local))
        has_dilution_events = dilution_event_idx is not None and sum(dilution_event_data_local) > 0
        has_led_data = led_channel_idx is not None and led_percent_idx is not None

        # Calculate total duration from the processed time data (which includes jump corrections)
        total_duration_hours = 0.0
        if time_data_local and len(time_data_local) >= 2:
            total_duration_hours = max(time_data_local) - min(time_data_local)
            print(f"✅ Calculated duration: {total_duration_hours:.4f}h from processed time data")

        # ===== CRITICAL FIX: Convert time_data to hours =====
        # Use Unix timestamps to calculate the ACTUAL duration and scale time_data accordingly
        # This handles any unit (ms, seconds, etc.) automatically
        if not use_sequential_time and time_data_local:
            max_time_val = max(time_data_local) if time_data_local else 0
            min_time_val = min(time_data_local) if time_data_local else 0
            uptime_range = max_time_val - min_time_val
            
            # Use Unix timestamps to get the REAL duration in seconds
            valid_unix = [t for t in unix_time_data_local if t is not None]
            print(f"📊 DEBUG: {len(valid_unix)} valid Unix timestamps, {len(time_data_local)} upTime values")
            print(f"📊 DEBUG: upTime range: {min_time_val:.0f} to {max_time_val:.0f} (range: {uptime_range:.0f})")
            if len(valid_unix) >= 2:
                print(f"📊 DEBUG: Unix range: {valid_unix[0]} to {valid_unix[-1]}")
                unix_duration_seconds = valid_unix[-1] - valid_unix[0]
                unix_duration_hours = unix_duration_seconds / 3600.0
                scale_factor = unix_duration_hours / uptime_range if uptime_range > 0 else 0
                print(f"📊 DEBUG: Unix duration: {unix_duration_seconds:.0f}s = {unix_duration_hours:.4f}h, scale factor: {scale_factor:.10f}")
                
                if uptime_range > 0 and unix_duration_hours > 0:
                    # Scale upTime to match Unix timestamp duration
                    time_data_local = [(t - min_time_val) * unix_duration_hours / uptime_range for t in time_data_local]
                    print(f"✅ Scaled time using Unix timestamps: upTime range {uptime_range:.0f} → {unix_duration_hours:.4f}h")
                else:
                    # Fallback: assume upTime is in seconds
                    time_data_local = [(t - min_time_val) / 3600.0 for t in time_data_local]
                    print(f"✅ Converted time from seconds to hours (fallback, max was {max_time_val}s)")
            else:
                # No valid Unix timestamps - guess based on value magnitude
                if max_time_val > 100000:  # Likely milliseconds
                    time_data_local = [(t - min_time_val) / 3600000.0 for t in time_data_local]
                    print(f"✅ Converted time from milliseconds to hours (max was {max_time_val}ms)")
                elif max_time_val > 100:  # Likely seconds
                    time_data_local = [(t - min_time_val) / 3600.0 for t in time_data_local]
                    print(f"✅ Converted time from seconds to hours (max was {max_time_val}s)")
        
        # Log LED data stats for debugging
        if has_led_data:
            unique_channels = set(led_channel_data_local)
            print(f"📊 LED data: {len(led_channel_data_local)} points, channels used: {sorted(unique_channels)}")
            print(f"📊 LED percent range: {min(led_percent_data_local)}-{max(led_percent_data_local)}%")

        response_data = {
            'time_data': time_data_local,
            'od_data': od_data_local,
            'media_data': media_data_local,
            'unix_time_data': unix_time_data_local,
            'dilution_event_data': dilution_event_data_local,  # Dilution events from CSV
            'has_dilution_events': has_dilution_events,  # Flag if dilution events exist
            'led_channel_data': led_channel_data_local,  # LED channel (1-6)
            'led_percent_data': led_percent_data_local,  # LED effective dose (%)  <-- %
            'has_led_data': has_led_data,  # Flag if LED data exists
            'has_duplicate_timestamps': has_duplicates,
            'time_spacing_seconds': time_spacing,
            'has_media_type': has_media_type,
            'experiment_info': exp,
            'total_duration_hours': total_duration_hours,  # Duration from backend
        }
        
        print(f"📊 Response: {len(time_data_local)} points, duration={total_duration_hours:.4f}h, has_led={has_led_data}, has_dilution={has_dilution_events}")
        return JSONResponse(response_data)

    except Exception as e:
        print(f"❌ ERROR in load_experiment_data: {e}")
        import traceback
        traceback.print_exc()
        return JSONResponse({'error': str(e)}, status_code=500)

def cleanup_current_session_csv():
    """Clean up the current session's CSV file."""
    global current_session_csv
    
    if current_session_csv and os.path.exists(current_session_csv):
        try:
            os.remove(current_session_csv)
            print(f"🗑️ Cleaned up current session CSV: {current_session_csv}")
            current_session_csv = None
        except Exception as e:
            print(f"⚠️ Failed to clean up current session CSV: {e}")

def cleanup_old_uploaded_files(max_age_seconds=3600):
    """Clean up old uploaded CSV files to prevent accumulation.

    V2.0: scans EVERY directory an upload can land in — the current working
    directory, the directory this script actually lives in, and the per-session
    temp_dir. Previously only '.' (cwd) was scanned, so when the app was launched
    from a different working directory (e.g. double-clicked) the uploaded_csv_*.csv
    files were never found and piled up. The current session file is always
    skipped, and only files older than max_age_seconds (default 1 hour) are
    removed so a file being viewed in another tab is never deleted out from under
    the user.
    """
    try:
        current_time = time.time()
        cleanup_count = 0

        # Collect all candidate directories (deduped, absolute).
        dirs_to_scan = set()
        try:
            dirs_to_scan.add(os.path.abspath('.'))
        except Exception:
            pass
        try:
            dirs_to_scan.add(os.path.dirname(os.path.abspath(__file__)))
        except Exception:
            pass
        try:
            if temp_dir:
                dirs_to_scan.add(os.path.abspath(temp_dir))
        except Exception:
            pass

        for scan_dir in dirs_to_scan:
            if not os.path.isdir(scan_dir):
                continue
            for filename in os.listdir(scan_dir):
                if filename.startswith('uploaded_csv_') and filename.endswith('.csv'):
                    filepath = os.path.join(scan_dir, filename)

                    # Skip current session file
                    if current_session_csv and os.path.abspath(filepath) == os.path.abspath(current_session_csv):
                        continue

                    try:
                        file_age = current_time - os.path.getmtime(filepath)
                        if file_age > max_age_seconds:
                            os.remove(filepath)
                            cleanup_count += 1
                            print(f"🗑️ Cleaned up old uploaded file: {filepath}")
                    except Exception as e:
                        print(f"⚠️ Failed to clean up {filename}: {e}")

        if cleanup_count > 0:
            print(f"🧹 Cleaned up {cleanup_count} old uploaded CSV files")

    except Exception as e:
        print(f"⚠️ Error during uploaded file cleanup: {e}")

async def show_file_viewer():
    """Dialog to browse and load CSV files from anywhere on your computer."""
    await show_file_picker()

if __name__ in {"__main__", "__mp_main__"}:
    # On Windows, Python 3.8+ defaults to ProactorEventLoop which can silently
    # drop WebSocket frames sent to remote clients through tunnels like
    # Cloudflare, causing the "page loads but nothing updates" symptom.
    # Switching to SelectorEventLoop before uvicorn/NiceGUI starts fixes this.
    if platform.system() == "Windows":
        import asyncio
        asyncio.set_event_loop_policy(asyncio.WindowsSelectorEventLoopPolicy())
        print("🪟 Windows detected: using SelectorEventLoop for WebSocket reliability")

    print("🚀 Starting OpenEvo Interface 2026-05-26...")
    
    # Register cleanup functions to run at exit (in reverse order of registration)
    atexit.register(cleanup_current_session_csv)
    
    # CRITICAL: Register Arduino shutdown to release COM port on exit
    def cleanup_arduino_on_exit():
        """Ensure COM port is released when program exits"""
        try:
            if arduino:
                arduino.shutdown()
        except Exception as e:
            print(f"⚠️ Error during Arduino cleanup: {e}")
    atexit.register(cleanup_arduino_on_exit)

    # Make sure any tunnel process is terminated on exit so it doesn't get orphaned
    def cleanup_tunnel_on_exit():
        """Stop any active tunnel process when the program exits."""
        try:
            if is_global or cloudflared_process or tunnelmole_process:
                stop_tunnel()
        except Exception as e:
            print(f"⚠️ Error during tunnel cleanup: {e}")
    atexit.register(cleanup_tunnel_on_exit)
    
    # Clean up old files in background (non-blocking)
    def background_cleanup():
        try:
            cleanup_old_uploaded_files()
        except:
            pass
    threading.Thread(target=background_cleanup, daemon=True).start()
    
    # DON'T load from local JSON file on startup - use hard-coded defaults!
    # Config will be loaded from Arduino's SD card when resuming experiments
    # or from hard-coded defaults when starting new experiments

    
    # Read total steps from cycler.csv at startup
    read_total_steps_from_cycler()
    
    # Periodic memory cleanup function for month-long stability
    async def periodic_memory_cleanup():
        """Run garbage collection periodically to prevent memory leaks."""
        if not hasattr(periodic_memory_cleanup, 'counter'):
            periodic_memory_cleanup.counter = 0
        
        periodic_memory_cleanup.counter += 1
        
        # Every 5 minutes (150 * 2 seconds)
        if periodic_memory_cleanup.counter % 150 == 0:
            collected = gc.collect()
            print(f"🧹 Periodic memory cleanup: collected {collected} objects")
            print(f"📊 Current data points in memory: {len(time_data)}")
            print(f"📊 Peak times tracked: {len(peak_times)}")
            print(f"📊 Peak intervals tracked: {len(peak_intervals)}")
    
    # Diagnostic: log every client that connects/disconnects so we can see
    # in the terminal whether a remote phone is reaching NiceGUI at all.
    # NOTE (V10.1): NiceGUI 3.4.1 changed app.clients into a method -> len() on it
    # crashed on every connect (and the socketio disconnect handler now passes an
    # extra arg). Accept *args and never call len(app.clients) so these handlers
    # can never raise and break a client/Cloudflare connection.
    @app.on_connect
    def _on_client_connect(*args):
        import datetime
        cid = getattr(args[0], 'id', '?') if args else '?'
        print(f"🔌 [CLIENT CONNECTED]  id={cid}  @ {datetime.datetime.now().strftime('%H:%M:%S')}")

    @app.on_disconnect
    def _on_client_disconnect(*args):
        import datetime
        cid = getattr(args[0], 'id', '?') if args else '?'
        print(f"🔌 [CLIENT DISCONNECTED] id={cid}  @ {datetime.datetime.now().strftime('%H:%M:%S')}")

    # Initialize the UI
    main_page()
    
    # Start timers
    # ------------------------------------------------------------------
    # V1.4 ARDUINO FIX: run the serial-read/update loop UNCONDITIONALLY.
    # ------------------------------------------------------------------
    # In the proven non-mirroring version (2026-05-26 V1) update_data ran on a
    # plain ui.timer that was always alive, so the serial port was drained every
    # 2s, last_data_time stayed fresh, and the USB watchdog never reset a healthy
    # Arduino. The mirroring hacks (pinned script_client + has_socket_connection
    # override) disrupt that client's socket lifecycle, which can stall a
    # client-gated ui.timer. When the timer stalls nothing reads the serial port,
    # last_data_time goes stale, and the watchdog kills a perfectly good
    # connection (the "Connection DEAD" / program-timeout symptom).
    #
    # Fix: drive update_data from a background asyncio task that is NOT gated by
    # browser/websocket connection state. We capture the module-level script
    # client and run each iteration inside its context so UI updates still emit
    # to every connected browser (local + remote mirror).
    from nicegui import context as _ng_context
    from nicegui import background_tasks as _ng_bg

    try:
        _update_client = _ng_context.client  # the module-level script_client
    except Exception:
        _update_client = None

    async def _serial_update_loop():
        import asyncio as _asyncio
        while True:
            try:
                if _update_client is not None:
                    with _update_client:
                        await update_data()
                else:
                    await update_data()
            except Exception as _e:
                print(f"⚠️ update loop error: {_e}")
            await _asyncio.sleep(2.0)

    async def _memory_cleanup_loop():
        import asyncio as _asyncio
        while True:
            try:
                if _update_client is not None:
                    with _update_client:
                        await periodic_memory_cleanup()
                else:
                    await periodic_memory_cleanup()
            except Exception:
                pass
            await _asyncio.sleep(2.0)

    @app.on_startup
    async def _start_background_loops():
        _ng_bg.create(_serial_update_loop(), name='serial_update_loop')
        _ng_bg.create(_memory_cleanup_loop(), name='memory_cleanup_loop')
        print("🔄 Background serial-update loop started (client-independent)")
    
    # NOTE: USB Watchdog will start when user clicks Connect (not at startup)
    # This prevents unnecessary port scanning before user is ready

    print("🚀 Starting OpenEvo Interface 2026-05-26...")
    print("🧠 Memory optimization enabled for long-term runs:")
    print("   📊 Display window: 288 hours (12 days)")
    print("   📉 Data resolution: 10-second intervals (keeps every 5th point)")
    print("   💾 Stored points: ~103,680 max (~450-650 MB)")
    print("   ⏱️  Chart update rate: Every 10 seconds")
    print("   💿 Arduino SD card: 10-second resolution (averaged data)")
    print("   🔄 Garbage collection: Every 5 minutes")
    # ================================================================
    # V10.3 SOCKETIO COMPATIBILITY SHIM
    # ================================================================
    # nicegui 3.4.1 requires python-socketio>=5.14, but socketio 5.15.x calls
    # disconnect handlers with (sid, reason) while NiceGUI's internal handler is
    # def _on_disconnect(sid) — only one arg. Result: EVERY client disconnect
    # raised "TypeError: _on_disconnect() takes 1 positional argument but 2 were
    # given", so NiceGUI's own client cleanup never ran (stale clients pile up,
    # error spam floods the console, and remote/Cloudflare reconnects get messy).
    # Re-register a tolerant wrapper that drops the extra arg(s) and calls the
    # original cleanup. Verified against socketio 5.15.1 / nicegui 3.4.1.
    try:
        import importlib as _importlib
        _ng_app_module = _importlib.import_module('nicegui.nicegui')
        from nicegui import core as _ng_core_sio
        _orig_on_disconnect = _ng_app_module._on_disconnect

        def _on_disconnect_compat(sid, *args, **kwargs):
            return _orig_on_disconnect(sid)
        _ng_core_sio.sio.on('disconnect', _on_disconnect_compat)
        print("🔧 socketio disconnect compatibility shim installed (5.15 fix)")
    except Exception as _sio_err:
        print(f"⚠️  Could not install socketio disconnect shim: {_sio_err}")

    # ================================================================
    # V10.2 MIRRORING FIX:  block NiceGUI 3.x script-mode re-execution.
    # ================================================================
    # In script mode (UI built at module scope, no @ui.page), NiceGUI's 404
    # handler (nicegui.py:_exception_handler_404) serves "/" by calling
    # page('')._wrap(run_script)(...), which re-executes THIS ENTIRE script via
    # runpy on every "/" request — and on every junk 404 like /favicon.ico and
    # every Cloudflare tunnel reachability probe.  Each re-execution builds a
    # brand-new Client with its own arduino=None and its own timers, so the
    # actually-connected browser lands in a different client room than the one
    # the live experiment is running in — remote pages load but never mirror.
    #
    # FIX: pin the original module's script_client so it is never deleted, and
    # replace the 404 handler so EVERY browser (local + remote) is served THAT
    # one pinned client.  They all join the same socket.io room and receive the
    # same emits from the same module-level ui.timer(update_data) — i.e. real
    # mirroring — without ever re-executing the script.
    #
    # NOTE: this does NOT touch the serial/connect code (unchanged from the
    # proven V10.1), and the prune timeout is extended so slow Cloudflare
    # WebSocket handshakes (which can take 60+ s) are not pruned mid-connect.
    try:
        from nicegui.client import Client as _NiceGUIClient
        _NiceGUIClient.prune_instances.__func__.__kwdefaults__ = {'client_age_threshold': 300.0}
        print("🌐 Cloudflare fix: client prune timeout extended to 300s")
    except Exception as _patch_err:
        print(f"⚠️  Could not patch client prune timeout: {_patch_err}")

    try:
        from nicegui import core as _ng_core
        from nicegui.client import Client as _NgClient
        from fastapi.responses import Response as _FastResponse

        # Resolve the pinned script client defensively across NiceGUI versions.
        # Most 3.x builds expose it as nicegui.core.script_client, but some
        # builds raise AttributeError there (seen on a Mac where mirroring
        # silently disabled itself). In that case fall back to the same client
        # we already captured via nicegui.context.client for the update loop —
        # that path works even when core.script_client is missing.
        _pinned_sc = None
        try:
            _pinned_sc = getattr(_ng_core, 'script_client', None)
        except Exception:
            _pinned_sc = None
        if _pinned_sc is None:
            _pinned_sc = _update_client
        if _pinned_sc is None:
            print("⚠️  script_client is None — cannot pin (mirroring fix skipped)")
        else:
            _orig_client_delete = _NgClient.delete

            def _delete_skipping_pinned(self):
                # Never delete the pinned script_client; delete everything else normally.
                if self is _pinned_sc:
                    return
                return _orig_client_delete(self)
            _NgClient.delete = _delete_skipping_pinned

            # CRITICAL (V1.2): Correctly track multiple sockets for shared client
            # When the remote (Cloudflare) browser disconnects, NiceGUI's handler sets
            # self.tab_id = None on the shared script_client, which makes its
            # has_socket_connection property return False. When that happens:
            # 1. button clicks from the local browser are ignored (UI freezes).
            # 2. ui.timer (update_data) hangs waiting for a new socket handshake,
            #    starving the Arduino connection.
            # Fix: Track the actual number of connected sockets via _socket_to_document_id
            # so the client is only considered disconnected when ALL browsers close.
            _orig_has_socket = _NgClient.has_socket_connection.fget
            def _has_socket_compat(self):
                if self is _pinned_sc:
                    return len(getattr(self, '_socket_to_document_id', {})) > 0
                return _orig_has_socket(self)
            _NgClient.has_socket_connection = property(_has_socket_compat)

            # The pinned script_client was created at module scope with no request
            # object. NiceGUI's socket.io handshake does request_contextvar.set(self.request),
            # which raises "Request is not set" for a request-less client and aborts
            # the handshake (browser connects but never mirrors). An HTTP page load
            # always happens before the socket handshake, so capture that first real
            # request here and attach it to the pinned client.
            async def _serve_pinned_client(request, exception):
                try:
                    # CRITICAL (V1.0): only rebuild/serve the full OpenEvo page for
                    # the index path. Every OTHER 404 — favicon.ico, robots.txt,
                    # browser prefetches, and especially Cloudflare's frequent tunnel
                    # reachability probes — must get a CHEAP 404, NOT a full
                    # build_response(). Rebuilding the entire page on every probe
                    # floods the event loop and starves the serial Arduino read loop,
                    # which is exactly why connect "did everything slower" and then
                    # timed out in the pinned-client builds (V10.2/V10.3).
                    if request.url.path not in ('/', ''):
                        return _FastResponse('Not Found', status_code=404)
                    if _pinned_sc._request is None:
                        _pinned_sc._request = request
                    return _pinned_sc.build_response(request)
                except Exception as _resp_err:
                    print(f"⚠️  serve_pinned_client error: {_resp_err}")
                    return _FastResponse('Not Found', status_code=404)
            app.add_exception_handler(404, _serve_pinned_client)

            print(f"✅ Pinned script_client {_pinned_sc.id} — re-execution blocked (mirroring on)")
    except Exception as _exec_fix_err:
        print(f"⚠️  Could not install mirroring fix: {_exec_fix_err}")

    # show=False (V10.1): the launcher script already opens the browser. Letting
    # ui.run also open one produced a second/duplicate window. The launcher
    # handles opening http://localhost:8080.
    #
    # storage_secret intentionally NOT set (V10.2): OpenEvo never uses ui.storage /
    # app.storage. Passing a secret makes NiceGUI schedule prune_user_storage(),
    # which iterates every client's request.session every 10s and crashes on the
    # request-less pinned client ("Request is not set"). Dropping it removes that
    # pruner entirely with no functional downside.
    # V1.5: wrap ui.run so Ctrl+C exits cleanly instead of dumping a
    # KeyboardInterrupt/CancelledError traceback (purely cosmetic - shutdown
    # and Arduino cleanup already happen correctly either way).
    try:
        ui.run(host='0.0.0.0', port=8080, title='OpenEvo', reload=False, show=False)
    except KeyboardInterrupt:
        print("👋 OpenEvo stopped.")