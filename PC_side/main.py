

import PySimpleGUI as sg
import time
import serial as ser
import os
import matplotlib.pyplot as plt
import matplotlib.cm as cm
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import math
import numpy as np
import traceback
import struct
import pyperclip
from scipy.signal import find_peaks, peak_widths, medfilt
global adc2dist_func_1, adc2dist_func_2, sum2dist_func
from statistics import mode, StatisticsError




s = None

light_epsilon = 0.3
object_light_epsilon = 0.3
calibration_map = []
calibration_map_ldr1 = []
calibration_map_ldr2 = []
packet_buffer = b''


# At the top of the file, after all imports

# Create the global variables and initialize them to None
adc2dist_func_1 = None
adc2dist_func_2 = None
sum2dist_func = None

# הגדרות עיצוב גלובליות
THEME_BG = '#2C3E50'
TEXT_COLOR = '#ECF0F1'
INPUT_BG = '#34495E'
ACCENT_COLOR = '#3498DB'
ACCENT_COLOR_LIGHT = '#5DADE2'
SECONDARY_BUTTON_COLOR = '#566573'

PI = 3.14159

# ======================= CONSTANTS  =======================
# Cross-talk (wide)
CROSSTALK_WIDE_ENABLE = True
CROSSTALK_G_MAX       = 0.99
CROSSTALK_KAPPA       = 1.0
CROSSTALK_POWER       = 1.7

# Sum→Distance (wide)
WIDE_USE_DELTA        = False
SUMW_CENTER_W         = 0.80
SUMW_OFFAXIS_W        = 0.60
K_DELTA_TO_SUM        = 0.18

# angle shaping (wide)
OFF_DEADBAND_DEG      = 12.0
OFF_MAX_BOOST         = 0.80
CENTER_SHRINK_K       = 0.18
CENTER_SIGMA_DEG      = 11.0

WIDE_GATE_CM          = 3.0

# Snap/robust
SNAP_BY_DIRECT_CM        = True
SNAP_CM_EPS              = 1.2
SNAP_MAX_SENSOR_DISAGREE = 2.5

# Peaks / windows / filters
PEAK_PROMINENCE       = 25
MERGE_TOLERANCE_DEG   = 10
WIDE_VALLEY_THRESH    = 20.0
MEDFILT_K             = 7
MIN_PEAK_WIDTH_DEG    = 6.0
MAX_LOCAL_STD         = 120.0
PAIR_MAX_ANGLE_DIFF   = 25.0
ALLOW_ANCHOR_SINGLE   = True

# ADC⇄V, weights
ADC_PER_V             = 1023.0 / 3.54
WEIGHT_GAMMA          = 1.0
FAR_WEIGHT            = 3.0
ANGLE_MID             = 90.0
ANGLE_BAND            = 1.0

# S-min windows / refine
SPLIT_IF_GAP_OVER_DEG = 90.0
S_MIN_PLATEAU_TOL     = 4.0
GLOBAL_HALF_WIN       = 8
ANGLE_REFINE_WIN      = 6

# edges
EDGE_LEFT_MAX_DEG     = 20.0
EDGE_RIGHT_MIN_DEG    = 160.0
EDGE_NEAR_TOL_DEG     = 3.0
EDGE_MONO_MIN_DROP    = 8.0
EDGE_MONO_MIN_POINTS  = 6

# Δ-fit
FIT_W_DELTA           = 0.70
FIT_W_SYMM            = 1.00
FIT_W_ERR             = 2.20
DELTA_ACCEPT_MAX_ERR  = 140.0
SAT_ADC               = 1018

PRIMARY_REL_EPS       = 0.12
PRIMARY_ABS_EPS       = 8.0
LOCAL_S_WEIGHT        = 0.02

# תחום VALLEY-IN לממוזג
IN_MIN, IN_MAX        = 75.0, 105.0


EDGE_SHORT_RUN_MAX_LEN = 2


#########################################
################ STATE 1 ################
#########################################
"""
Handles the GUI and logic for the Object Detector System (Mode 1).
Manages the scan process and calls processing/plotting functions.
"""
def objects_detector():
    # --- הגדרת סגנונות ---
    button_style_primary = {'button_color': ('white', ACCENT_COLOR), 'font': ('Segoe UI', 12), 'border_width': 0,
                            'mouseover_colors': ('white', ACCENT_COLOR_LIGHT)}
    button_style_secondary = {'button_color': ('white', SECONDARY_BUTTON_COLOR), 'font': ('Segoe UI', 12),
                              'border_width': 0, 'mouseover_colors': ('white', '#707B7C')}

    # --- מבנה מסגרת בקרה ---
    controls_frame_layout = [
        [sg.Text("Masking Distance [cm]:", background_color=THEME_BG, text_color=TEXT_COLOR, font=('Segoe UI', 11)),
         sg.InputText(key="-DISTANCE-", size=(20, 1), default_text="200", background_color=INPUT_BG,
                      text_color=TEXT_COLOR, border_width=0)],
        [sg.Push(background_color=THEME_BG),
         sg.Button("Start Objects Scan", key="-SCAN-", **button_style_primary),
         sg.Button("Copy Log", key="-COPY_LOG-", **button_style_secondary),
         sg.Button("Back", key="-BACK-", **button_style_secondary),
         sg.Push(background_color=THEME_BG)]
    ]

    # --- מבנה מסגרת פלט ---
    output_frame_layout = [
        [sg.Output(key="-OUTPUT-", size=(80, 15), background_color=INPUT_BG, text_color=TEXT_COLOR)]
    ]

    # --- מבנה ראשי המשלב הכל ---
    layout = [
        [sg.Text("Objects Detector Control", font=('Segoe UI', 18, 'bold'), background_color=THEME_BG,
                 text_color=TEXT_COLOR)],
        [sg.Frame("Controls", controls_frame_layout, background_color=THEME_BG, border_width=0,
                  title_color=TEXT_COLOR)],
        [sg.Frame("Log Output", output_frame_layout, background_color=THEME_BG, border_width=0, title_color=TEXT_COLOR)]
    ]

    window = sg.Window("Objects Detector System", layout, background_color=THEME_BG, resizable=True, finalize=True)

    while True:
        event, values = window.read()
        if event in (sg.WINDOW_CLOSED, "-BACK-", "-ESCAPE-"):
            send_command('0')
            break
        elif event == "-COPY_LOG-":
            log_content = window["-OUTPUT-"].get()

            pyperclip.copy(log_content)

            sg.popup_quick("Log copied to clipboard!")
        elif event == "-SCAN-" or event == '\r':
            window["-SCAN-"].update(disabled=True)
            window["-BACK-"].update(disabled=True)
            window['-OUTPUT-'].update("")

            s.reset_input_buffer()
            time.sleep(0.1)
            send_command('S')

            scan_data = []  # This will hold the final, masked data for plotting
            try:
                masking_distance = int(values["-DISTANCE-"])
            except ValueError:
                masking_distance = 200  # Default value on error

            # --- Real-time data receiving and processing loop ---
            while True:
                window.refresh()
                data_bytes = s.read(3)

                if len(data_bytes) < 3:
                    print("Timeout: Finished receiving data.")
                    break

                distance = (data_bytes[1] << 8) | data_bytes[0]
                angle = data_bytes[2]

                if distance == 0xFFFF:
                    print("--- End of Scan Signal Received ---")
                    break

                    # Apply masking AND update the screen in real-time
                if distance > masking_distance:
                    scan_data.append((0, angle))  # Add masked data for the plot
                    window['-OUTPUT-'].update(f"Angle: {angle:>3}°, Dist: {'{:>3}'.format(distance)} [cm]", append=True)
                    window['-OUTPUT-'].Widget.tag_configure("red_text", foreground="red")
                    window['-OUTPUT-'].Widget.insert("end", " - MASKED\n", "red_text")
                else:
                    scan_data.append((distance, angle))  # Add real data for the plot
                    window['-OUTPUT-'].update(f"Angle: {angle:>3}°, Dist: {'{:>3}'.format(distance)} [cm]\n",
                                              append=True)

            if not scan_data:
                window['-OUTPUT-'].update("Scan finished with no data points.")
            else:
                # --- הוספת קוד סינון רעשים ---
                # 1. הפרדת המרחקים והזוויות למערכים נפרדים
                distances = np.array([item[0] for item in scan_data])
                angles = [item[1] for item in scan_data]

                # 2. החלת מסנן חציון על המרחקים
                # kernel_size חייב להיות אי-זוגי. 5 הוא ערך טוב שמתקן רעש של עד 2 נקודות.
                filtered_distances = medfilt(distances, kernel_size=5)

                # 3. הרכבת רשימת הנתונים מחדש עם המרחקים המסוננים
                clean_scan_data = list(zip(filtered_distances, angles))
                # ---------------------------------

                # זיהוי אובייקטים בשיטה: First/Last Same Distance + Edge Plateau
                objs = find_objects_firstlast_equal(
                    clean_scan_data,  # <--- שינוי חשוב: השתמש במידע המסונן
                    distance_threshold_cm=masking_distance,
                    edge_short_run_max_len=2
                )

                window['-OUTPUT-'].update(f"Scan complete! Found {len(objs)} object(s).\n{'-' * 30}\n",
                                          append=True)

                # פורמט הפלט זהה לגרסת הבדיקות
                window['-OUTPUT-'].update(f"Found {len(objs)} object(s):\n", append=True)
                for o in objs:
                    line = (
                        f"Obj#{o['id']}: (ρ, φ, l) = "
                        f"({o.get('rho_display', o['rho']):.0f} cm, "
                        f"{o.get('phi_display', o['phi']):.0f}°, "
                        f"{o.get('width_display', o['width']):.0f} cm) | "
                        f"D*={o['target_distance_cm']} cm | span {o['start_angle']:.0f}–{o['end_angle']:.0f}°, "
                        f"first/last {o['angle_first_target']:.0f}/{o['angle_last_target']:.0f}°, "
                        f"pts={o['n_points']}"
                    )
                    window['-OUTPUT-'].update(line + "\n", append=True)

                # שרטוט כמו בבודק: נקודה לכל אובייקט וה-hover מציג רק (ρ, φ, l)
                draw_scanner_map(objs)

            window["-SCAN-"].update(disabled=False)
            window["-BACK-"].update(disabled=False)

    window.close()



def find_objects_from_scan(scan_data, min_points_for_object=3, distance_threshold_cm=5):
    """
    Processes a list of (distance, angle) points, groups them into objects,
    and calculates properties for each object.
    Now splits objects if there's a large jump in distance.
    """
    detected_objects = []
    current_object_points = []

    for distance, angle in scan_data:
        if distance > 0:
            # Check for large jump in distance from the previous point
            if current_object_points and abs(distance - current_object_points[-1]['distance']) > distance_threshold_cm:
                # This is a new object. Finalize the previous one.
                if len(current_object_points) >= min_points_for_object:
                    detected_objects.append(current_object_points)
                current_object_points = [] # Start a new object

            current_object_points.append({'distance': distance, 'angle': angle})
        else:
            # Gap detected, finalize the previous object
            if len(current_object_points) >= min_points_for_object:
                detected_objects.append(current_object_points)
            current_object_points = []

    if len(current_object_points) >= min_points_for_object:
        detected_objects.append(current_object_points)


    object_summary = []
    for i, obj_points in enumerate(detected_objects):
        start_point = obj_points[0]
        end_point = obj_points[-1]
        avg_distance = sum(p['distance'] for p in obj_points) / len(obj_points)
        r1, r2 = start_point['distance'], end_point['distance']
        theta1_rad, theta2_rad = math.radians(start_point['angle']), math.radians(end_point['angle'])
        delta_theta = abs(theta2_rad - theta1_rad)
        width = math.sqrt(r1**2 + r2**2 - 2 * r1 * r2 * math.cos(delta_theta))

        object_summary.append({
            'id': i + 1,
            'start_angle': start_point['angle'],
            'end_angle': end_point['angle'],
            'avg_distance': avg_distance,
            'width': width,
            'points': obj_points # Store all points for hover feature
        })
    return object_summary

"""
Generates and displays a polar plot for detected objects.
Represents each object as a single point and provides details on hover.
"""

def _rle_runs(values):
    """Run-length encoding -> [(value, i0, i1), ...] עבור רצפים שווים רציפים."""
    if not values:
        return []
    runs, v_prev, i0 = [], values[0], 0
    for i in range(1, len(values)):
        if values[i] != v_prev:
            runs.append((v_prev, i0, i - 1))
            v_prev = values[i]
            i0 = i
    runs.append((v_prev, i0, len(values) - 1))
    return runs

def _cluster_objects_by_threshold(scan_data, threshold_cm):
    """פיצול לקלאסטרים צמודים עם 0<distance<threshold; distance==0 יוצר רווח (gap)."""
    clusters, cur = [], []
    for d, a in scan_data:
        if 0.0 < d < threshold_cm:
            cur.append((d, a))
        else:
            if cur:
                clusters.append(cur)
                cur = []
    if cur:
        clusters.append(cur)
    return clusters

def find_objects_firstlast_equal(scan_data,distance_threshold_cm=400.0,edge_short_run_max_len=2):
    """
    זיהוי אובייקטים לפי 'first/last same rounded distance' עם תיקון Edge-Plateau.
    קלט: scan_data כ-(distance, angle) עם 0 כ-MASKED/רווח.
    פלט: רשימת מילונים עם שדות זהים לבודק (phi/rho/width, תצוגה מעוגלת, וכו').
    """
    if not scan_data:
        return []

    all_angles = [a for _d, a in scan_data]
    global_min_angle = min(all_angles) if all_angles else 0.0
    global_max_angle = max(all_angles) if all_angles else 180.0

    clusters = _cluster_objects_by_threshold(scan_data, distance_threshold_cm)
    objects, obj_id = [], 1

    for cl in clusters:
        cl = sorted(cl, key=lambda x: x[1])
        dists = np.array([p[0] for p in cl], dtype=float)
        angs  = np.array([p[1] for p in cl], dtype=float)
        R     = np.rint(dists).astype(int).tolist()

        cl_start, cl_end = float(angs[0]), float(angs[-1])
        runs = _rle_runs(R)
        D_target = runs[0][0] if runs else int(round(dists[0]))

        try:
            # מצא את המרחק השכיח ביותר (mode) ברצף המדידות
            D_target = mode(R)
        except StatisticsError:
            # במקרה הנדיר שיש שני מרחקים באותה שכיחות, השתמש בחציון
            D_target = int(np.median(R))

        # first/last של ה-D_target בתוך הקלאסטר
        idx_first = next((i for i, v in enumerate(R) if v == D_target), 0)
        idx_last  = len(R) - 1 - next((i for i, v in enumerate(reversed(R)) if v == D_target), 0)
        if idx_last < idx_first:
            idx_first, idx_last = 0, len(R) - 1  # fallback נדיר

        angle_first, angle_last = float(angs[idx_first]), float(angs[idx_last])

        # φ, ρ מהפלטו הנבחר
        phi = 0.5 * (angle_first + angle_last)
        rho = float(np.mean(dists[idx_first:idx_last + 1]))

        # Δφ בסיס = כל המניפה של הקלאסטר; בקצה מוסיפים פיצוי סימטרי רק אם ה-target שווה לערך הקצה
        width_deg_used = max(0.0, cl_end - cl_start)
        K_CORRECTION_FACTOR = 0.4  # התחל עם ערך זה, ושנה אותו לפי הצורך
        center_angle = (cl_start + cl_end) / 2.0
        correction_ratio = 1.0 - K_CORRECTION_FACTOR * (abs(center_angle - 90.0) / 90.0)

        # זהו הרוחב הזוויתי המתוקן
        width_deg_used = width_deg_used


        if cl_start == global_min_angle and len(R) > 0 and D_target == R[0]:
            j_last_D0 = max(i for i, v in enumerate(R) if v == R[0])
            width_deg_used += max(0.0, cl_end - float(angs[j_last_D0]))

        if cl_end == global_max_angle and len(R) > 0 and D_target == R[-1]:
            i_first_Dend = min(i for i, v in enumerate(R) if v == R[-1])
            width_deg_used += max(0.0, float(angs[i_first_Dend]) - cl_start)

        width_deg_used = min(180.0, width_deg_used)

        # רוחב לינארי: l = 2ρ sin(Δφ/2) עם Δφ במעלות -> רדיאנים
        delta_phi_rad = width_deg_used * (PI / 180.0)
        width = float(2.0 * rho * math.sin(delta_phi_rad / 2.0))

        # --- Half-up rounding להצגה בלבד ---
        phi_display   = float(math.floor(phi   + 0.5))
        rho_display   = float(math.floor(rho   + 0.5))
        width_display = float(math.floor(width + 0.5))
        delta_phi_deg_display = float(math.floor(width_deg_used + 0.5))

        objects.append({
            "id": obj_id,
            "start_angle": cl_start, "end_angle": cl_end,
            "target_distance_cm": int(D_target),
            "angle_first_target": angle_first, "angle_last_target": angle_last,
            "phi": float(phi), "rho": rho, "width": width,
            "delta_phi_deg": width_deg_used, "delta_phi_rad": float(delta_phi_rad),
            # שדות הצגה (מעוגלים half-up)
            "phi_display": phi_display,
            "rho_display": rho_display,
            "width_display": width_display,
            "delta_phi_deg_display": delta_phi_deg_display,
            "n_points": int(len(cl)),
        })
        obj_id += 1

    return objects


def draw_scanner_map(objects):
    if not objects:
        sg.popup("No objects to plot.")
        return

    pts = [(math.radians(o["phi"]), o["rho"]) for o in objects]
    labels = [
        f"ρ={o.get('rho_display', o['rho']):.0f} cm\n"
        f"φ={o.get('phi_display', o['phi']):.0f}°\n"
        f"l={o.get('width_display', o['width']):.0f} cm"
        for o in objects
    ]
    max_r = max(r for _, r in pts) if pts else 50

    layout = [[sg.Canvas(key="-CANVAS-", size=(800, 800))]]
    win = sg.Window("Objects Map", layout, finalize=True, resizable=True)

    fig = plt.figure(figsize=(8, 8))
    ax = fig.add_subplot(111, polar=True)
    scat = ax.scatter([t for t, _ in pts], [r for _, r in pts], s=80, alpha=0.9)

    annot = ax.annotate("", xy=(0, 0), xytext=(20, 20), textcoords="offset points",
                        bbox=dict(boxstyle="round", fc="lightblue", alpha=0.9),
                        arrowprops=dict(arrowstyle="->"))
    annot.set_visible(False)

    def update_annot(ind):
        i = ind["ind"][0]
        annot.xy = scat.get_offsets()[i]
        annot.set_text(labels[i])

    def hover(event):
        vis = annot.get_visible()
        if event.inaxes == ax:
            cont, ind = scat.contains(event)
            if cont:
                update_annot(ind)
                annot.set_visible(True)
                fig.canvas.draw_idle()
            elif vis:
                annot.set_visible(False)
                fig.canvas.draw_idle()

    fig.canvas.mpl_connect("motion_notify_event", hover)
    ax.set_thetamin(0)
    ax.set_thetamax(180)
    ax.set_ylim(0, max_r * 1.15)
    ax.set_title("Detected Objects (ρ, φ, l)", va='bottom')
    ax.grid(True)

    canvas = FigureCanvasTkAgg(fig, master=win["-CANVAS-"].TKCanvas)
    canvas.draw()
    canvas.get_tk_widget().pack(side="top", fill="both", expand=True)

    while True:
        ev, _ = win.read()
        if ev == sg.WINDOW_CLOSED:
            break

    canvas.get_tk_widget().destroy()
    plt.close(fig)
    win.close()


#########################################
################ STATE 2 ################
#########################################

"""
Handles the GUI and logic for the Telemeter mode (Mode 2).
Allows the user to point the sensor to a specific angle and get
real-time distance measurements.
"""
def telemeter():
    dynamic_flag = 0

    button_style_primary = {'button_color': ('white', ACCENT_COLOR), 'font': ('Segoe UI', 12), 'border_width': 0,
                            'mouseover_colors': ('white', ACCENT_COLOR_LIGHT)}
    button_style_secondary = {'button_color': ('white', SECONDARY_BUTTON_COLOR), 'font': ('Segoe UI', 12),
                              'border_width': 0, 'mouseover_colors': ('white', '#707B7C')}

    # --- מבנה מסגרת בקרה ---
    controls_frame_layout = [
        [sg.Text("Angle [0°-180°]:", background_color=THEME_BG, text_color=TEXT_COLOR),
         sg.InputText(key="-ANGLE-", size=(10, 1), background_color=INPUT_BG, text_color=TEXT_COLOR, border_width=0,
                      justification='center')],
        [sg.Button("Start Measure", key="-START-", **button_style_primary, size=(15, 1)),
         sg.Button("Stop Measure", key="-STOP-", **button_style_secondary, size=(15, 1), disabled=True)]
    ]

    # --- מבנה מסגרת תצוגת נתונים ---
    output_frame_layout = [
        [sg.Text("Waiting for data...", key="-OUTPUT-", font=('Courier New', 16, 'bold'), size=(30, 1),
                 justification='center', background_color=THEME_BG, text_color=TEXT_COLOR)]
    ]

    # --- מבנה ראשי המשלב הכל ---
    layout = [
        [sg.Text("Telemeter", font=('Segoe UI', 18, 'bold'), background_color=THEME_BG, text_color=TEXT_COLOR)],
        [sg.Frame("Controls", controls_frame_layout, element_justification='center', background_color=THEME_BG,
                  border_width=0, title_color=TEXT_COLOR)],
        [sg.Sizer(0, 10)],
        [sg.Frame("Live Data", output_frame_layout, element_justification='center', background_color=THEME_BG,
                  border_width=0, title_color=TEXT_COLOR)],
        [sg.Sizer(0, 10)],
        [sg.Button("Back", key="-BACK-", **button_style_secondary)]
    ]

    window = sg.Window("Telemeter", layout, background_color=THEME_BG, element_justification='center', finalize=True)

    while True:
        event, values = window.read(timeout=100)

        if event in (sg.WINDOW_CLOSED, "-BACK-", "-ESCAPE-"):
            if dynamic_flag: send_command('M')
            send_command('0')
            break

        elif event == "-START-" or event == '\r':
            try:
                angle_to_send = int(values["-ANGLE-"])
                if not 0 <= angle_to_send <= 180:
                    sg.popup_error("Angle must be between 0 and 180.")
                    continue

                window["-START-"].update(disabled=True)
                window["-BACK-"].update(disabled=True)
                window["-STOP-"].update(disabled=False)
                window['-OUTPUT-'].update("")

                s.reset_input_buffer()
                # --- New Binary Protocol ---
                send_command('T')  # Send the command character
                time.sleep(0.05)  # Small delay to ensure MCU processes the command
                # Send the angle as a single byte
                s.write(angle_to_send.to_bytes(1, 'big'))
                dynamic_flag = 1

            except ValueError:
                sg.popup_error("Invalid angle. Please enter a number.")

        elif event == "-STOP-":
            send_command('M')
            dynamic_flag = 0
            window["-STOP-"].update(disabled=True)
            window["-START-"].update(disabled=False)
            window["-BACK-"].update(disabled=False)

        elif event == "__TIMEOUT__":
            if dynamic_flag and s.in_waiting >= 3:
                data_bytes = s.read(3)
                distance = (data_bytes[1] << 8) | data_bytes[0]
                angle = data_bytes[2]
                window['-OUTPUT-'].update(f"Distance: {distance:>3} cm | Angle: {angle:>3}°")

    window.close()


#########################################
################ STATE 3 ################
#########################################

"""
Handles the GUI and logic for the Light Detector System (Mode 3).
Manages the interactive calibration process and the light scanning process.
"""
def lights_detector():
    global calibration_map  # Use the global map variable
    calibration_active = False
    calibration_sample_count = 0
    LDR_calibrate_arr = []
    temp_ldr1_str = None
    global adc2dist_func_1, adc2dist_func_2, sum2dist_func

    # --- הגדרת סגנונות ---
    button_style_primary = {'button_color': ('white', ACCENT_COLOR), 'font': ('Segoe UI', 12), 'border_width': 0,
                            'mouseover_colors': ('white', ACCENT_COLOR_LIGHT)}
    button_style_secondary = {'button_color': ('white', SECONDARY_BUTTON_COLOR), 'font': ('Segoe UI', 12),
                              'border_width': 0, 'mouseover_colors': ('white', '#707B7C')}

    # --- מבנה מסגרת סריקה ---
    scan_frame_layout = [
        [sg.Button("Start Light Sources Scan", key="-SCAN-", **button_style_primary, size=(40, 2))]
    ]

    # --- מבנה מסגרת פלט ---
    output_frame_layout = [
        [sg.Output(key="-OUTPUT-", size=(80, 15), background_color=INPUT_BG, text_color=TEXT_COLOR)]
    ]

    # --- מבנה ראשי המשלב הכל ---
    layout = [
        [sg.Text("Light Sources Detector", font=('Segoe UI', 18, 'bold'), background_color=THEME_BG,
                 text_color=TEXT_COLOR)],
        [sg.Frame("Scan", scan_frame_layout, element_justification='center', background_color=THEME_BG, border_width=0,
                  title_color=TEXT_COLOR)],
        [sg.Frame("Log Output", output_frame_layout, background_color=THEME_BG, border_width=0,
                  title_color=TEXT_COLOR)],
        [sg.Button("Copy Log", key="-COPY_LOG-", **button_style_secondary), sg.Push(background_color=THEME_BG),
         sg.Button("Back", key="-BACK-", **button_style_secondary)]
    ]

    window = sg.Window("Light Sources Detector System", layout, background_color=THEME_BG, finalize=True)

    while True:
        event, values = window.read(timeout=100)

        if event in (sg.WINDOW_CLOSED, "-BACK-", "-ESCAPE-"):
            send_command('0')
            break
        elif event == "-COPY_LOG-":
            log_content = window["-OUTPUT-"].get()

            pyperclip.copy(log_content)

            sg.popup_quick("Log copied to clipboard!")

        elif event == "-SCAN-" or event == '\r':
            # This logic now correctly uses the in-memory calibration_map
            if not adc2dist_func_1 or not adc2dist_func_2:
                sg.popup_error("Calibration maps are empty. Please run a full calibration from Script Mode first.")
                continue

            window["-BACK-"].update(disabled=True)
            window["-SCAN-"].update(disabled=True)
            window["-OUTPUT-"].update("")

            all_scan_readings = []
            s.reset_input_buffer()
            send_command('K')

            # --- This entire while loop replaces the old one in lights_detector ---
            while True:
                window.refresh()
                data_bytes = s.read(5)  # Read the new 5-byte packet
                if len(data_bytes) < 5:
                    print("Timeout: Finished receiving data.")
                    break

                # Unpack two 16-bit integers (H) and one 8-bit integer (B)
                ldr1_val, ldr2_val, angle = struct.unpack('<HHB', data_bytes)

                if ldr1_val == 0xFFFF:  # Use one of the values to check for end-of-scan
                    print("--- End of Scan Signal Received ---")
                    break

                # Store the raw data from both sensors
                all_scan_readings.append((ldr1_val, ldr2_val, angle))
                window['-OUTPUT-'].update(f"Angle: {angle:>3}°, LDR1: {ldr1_val:<4}, LDR2: {ldr2_val:<4}\n",
                                          append=True)

            if not all_scan_readings:
                sg.popup("Scan complete. No data was received.")
            else:
                # Prepare raw data: list of (ldr1, ldr2, angle)
                scan_data_for_processing = [((item[0]), (item[1]), item[2]) for item in all_scan_readings]

                # Create the sum-to-distance helper function needed by the new algorithm
                sum2dist_func = make_sum2dist_func(adc2dist_func_1, adc2dist_func_2)

                # Call the new algorithm correctly and receive all its return values
                light_sources, atype, peaks1, peaks2, plats1, plats2 = process_light_scan_data(
                    scan_data_for_processing,
                    adc2dist_func_1,
                    adc2dist_func_2,
                    sum2dist_func
                )

                window['-OUTPUT-'].update(f"\n--- ANALYSIS COMPLETE ---\n", append=True)
                window['-OUTPUT-'].update(f"Analysis Type: ** {atype} **\n", append=True)

                if light_sources:
                    window['-OUTPUT-'].update(f"SUCCESS: Calculated {len(light_sources)} final light source(s).\n",
                                              append=True)
                    # Sort sources by angle for clean display
                    light_sources_sorted = sorted(light_sources, key=lambda x: x['angle'])
                    for i, ssrc in enumerate(light_sources_sorted):
                        window['-OUTPUT-'].update(f"--- Source #{i + 1} ---\n", append=True)
                        window['-OUTPUT-'].update(f"  > Angle:    {ssrc['angle']:.2f}°\n", append=True)
                        window['-OUTPUT-'].update(f"  > Distance: {ssrc['distance']:.2f} cm\n", append=True)

                    # Prepare data for the plot: a list of (distance, angle)
                    plot_data = [(ssrc['distance'], ssrc['angle']) for ssrc in light_sources_sorted]
                    draw_scanner_map_lights(plot_data)
                else:
                    sg.popup("Scan complete. No significant light sources detected.")
            window["-BACK-"].update(disabled=False)
            window["-SCAN-"].update(disabled=False)

    window.close()



def process_light_scan_data(scan_data, adc2d_1, adc2d_2, sum2d=None):
    if not scan_data:
        return [], "N/A", [], [], [], []

    ang = np.array([t[2] for t in scan_data], float)
    r1  = np.array([t[0] for t in scan_data], float)
    r2  = np.array([t[1] for t in scan_data], float)

    I1s, I2s = medfilt(1023.0 - r1, MEDFILT_K), medfilt(1023.0 - r2, MEDFILT_K)

    p1o,_ = find_peaks(I1s, prominence=PEAK_PROMINENCE)
    p2o,_ = find_peaks(I2s, prominence=PEAK_PROMINENCE)
    p1o   = merge_adjacent_peaks(_filter_peaks_by_width_and_stability(p1o, I1s, ang, r1), ang, r1)
    p2o   = merge_adjacent_peaks(_filter_peaks_by_width_and_stability(p2o, I2s, ang, r2), ang, r2)

    pl1 = _detect_plateaus(r1, ang)
    pl2 = _detect_plateaus(r2, ang)

    def is_edge_idx(i): return (ang[i] <= EDGE_LEFT_MAX_DEG) or (ang[i] >= EDGE_RIGHT_MIN_DEG)
    left_cands  = [i for i in list(pl2) if not is_edge_idx(i)] + [i for i in p2o if not is_edge_idx(i)]
    right_cands = [i for i in p1o if not is_edge_idx(i)] + [i for i in list(pl1) if not is_edge_idx(i)]

    found, atype = [], ""
    if left_cands and right_cands:
        iL = min(left_cands,  key=lambda i: ang[i])
        iR = max(right_cands, key=lambda i: ang[i])
        if (ang[iR] - ang[iL]) > SPLIT_IF_GAP_OVER_DEG:
            S = r1 + r2
            zoneL = _s_min_zone_indices_window(ang, S, 0, iL, tol=S_MIN_PLATEAU_TOL)
            aL = _refine_angle_near(
                float(_choose_angle_in_zone(ang, r1, r2, zoneL, adc2d_1, adc2d_2, S=S, prefer='LDR2')),
                ang, r1, r2, adc2d_1, adc2d_2, ANGLE_REFINE_WIN
            )
            zoneR = _s_min_zone_indices_window(ang, S, iR, len(S)-1, tol=S_MIN_PLATEAU_TOL)
            aR = _refine_angle_near(
                float(_choose_angle_in_zone(ang, r1, r2, zoneR, adc2d_1, adc2d_2, S=S, prefer='LDR1')),
                ang, r1, r2, adc2d_1, adc2d_2, ANGLE_REFINE_WIN
            )
            found += [{'angle': float(aL), 'mode':'delta'}, {'angle': float(aR), 'mode':'delta'}]
            atype = "Two separate sources (>90° apart); half-range S-min + Δ-fit (plateau-pref left)"

    if not found:
        p1 = _augment_with_edge_seed(p1o, r1, ang)
        p2 = _augment_with_edge_seed(p2o, r2, ang)
        n1, n2 = len(p1), len(p2)

        if n1 > 0 and n1 == n2 and n1 >= 2:
            atype = f"{n1} Separate Sources Detected (Indexwise Pairing + Δ-fit local refine)"
            p1s = np.array(sorted(p1, key=lambda i: ang[i]), int)
            p2s = np.array(sorted(p2, key=lambda i: ang[i]), int)
            for i in range(n1):
                a_avg = 0.5*(float(ang[p1s[i]]) + float(ang[p2s[i]]))
                a_ref = _refine_angle_near(a_avg, ang, r1, r2, adc2d_1, adc2d_2, ANGLE_REFINE_WIN)
                found.append({'angle': a_ref, 'mode':'delta'})

        elif n1 == 1 and n2 == 1:
            v1, v2 = analyze_valley(r1, ang, p1[0]), analyze_valley(r2, ang, p2[0])
            atype = f"Analysis (LDR1 width: {v1['width']:.1f}°, LDR2 width: {v2['width']:.1f}°): "
            if v1 and v2 and v1['is_wide'] and v2['is_wide']:
                def choose_edge_by_zone(valley, peak_deg):
                    inner = valley['exit_angle'] if peak_deg < 90.0 else valley['entry_angle']
                    outer = valley['entry_angle'] if peak_deg < 90.0 else valley['exit_angle']
                    return inner if (peak_deg <= IN_MAX) else outer
                peak1_deg = float(ang[int(p1[0])]); peak2_deg = float(ang[int(p2[0])])
                a1 = float(choose_edge_by_zone(v1, peak1_deg))
                a2 = float(choose_edge_by_zone(v2, peak2_deg))
                atype += "Single Wide Blob Detected (valley-edge by zone [75..105]°)"
                found += [{'angle': a1, 'mode': 'wide'}, {'angle': a2, 'mode': 'wide'}]
            else:
                atype += "Single Narrow Source Detected (Global S-min + Δ-fit)"
                S = r1 + r2; i0 = int(np.argmin(S))
                zone = _s_min_zone_indices_window(ang, S, max(0, i0 - GLOBAL_HALF_WIN), min(len(S)-1, i0 + GLOBAL_HALF_WIN), tol=S_MIN_PLATEAU_TOL)
                a_ref = _refine_angle_near(
                    float(_choose_angle_in_zone(ang, r1, r2, zone, adc2d_1, adc2d_2, S=S, tie_center=float(ang[i0]))),
                    ang, r1, r2, adc2d_1, adc2d_2, ANGLE_REFINE_WIN
                )
                found.append({'angle': float(a_ref), 'mode':'delta'})

        elif n1 > 0 and n2 > 0:
            atype = "Pairing after noise filtering"
            used2 = set()
            for a in p1:
                diffs = [abs(ang[b]-ang[a]) if j not in used2 else 1e9 for j, b in enumerate(p2)]
                if not diffs: break
                j = int(np.argmin(diffs))
                if diffs[j] <= PAIR_MAX_ANGLE_DIFF:
                    used2.add(j); b = p2[j]
                    a_avg = 0.5*(float(ang[a]) + float(ang[b]))
                    a_ref = _refine_angle_near(a_avg, ang, r1, r2, adc2d_1, adc2d_2, ANGLE_REFINE_WIN)
                    found.append({'angle': a_ref, 'mode':'delta'})
            if not found and ALLOW_ANCHOR_SINGLE:
                if n1 >= 1 and (n2 == 0 or n2 > 1):
                    k = p1[int(np.argmax(I1s[p1]))]; atype = "Single-sensor anchor (LDR1) after noise rejection"
                    found.append({'angle': float(ang[k]), 'mode':'avg'})
                elif n2 >= 1 and (n1 == 0 or n1 > 1):
                    k = p2[int(np.argmax(I2s[p2]))]; atype = "Single-sensor anchor (LDR2) after noise rejection"
                    found.append({'angle': float(ang[k]), 'mode':'avg'})
        else:
            seeds = []
            if len(p1o) >= 1: seeds.append(p1o[int(np.argmax(I1s[p1o]))])
            elif len(pl1) >= 1: seeds.append(int(pl1[-1]))
            if len(p2o) >= 1: seeds.append(p2o[int(np.argmax(I2s[p2o]))])
            elif len(pl2) >= 1: seeds.append(int(pl2[0]))
            if len(seeds) >= 2 and abs(ang[min(seeds)] - ang[max(seeds)]) > SPLIT_IF_GAP_OVER_DEG:
                atype = "Two separate sources (>90° apart); half-range S-min + Δ-fit"
                i_left, i_right = (min(seeds), max(seeds)); S = r1 + r2
                zoneL = _s_min_zone_indices_window(ang, S, 0, i_left, tol=S_MIN_PLATEAU_TOL)
                aL = _refine_angle_near(
                    float(_choose_angle_in_zone(ang, r1, r2, zoneL, adc2d_1, adc2d_2, S=S, prefer='LDR2')),
                    ang, r1, r2, adc2d_1, adc2d_2, ANGLE_REFINE_WIN
                )
                zoneR = _s_min_zone_indices_window(ang, S, i_right, len(S) - 1, tol=S_MIN_PLATEAU_TOL)
                aR = _refine_angle_near(
                    float(_choose_angle_in_zone(ang, r1, r2, zoneR, adc2d_1, adc2d_2, S=S, prefer='LDR1')),
                    ang, r1, r2, adc2d_1, adc2d_2, ANGLE_REFINE_WIN
                )
                found += [{'angle': float(aL), 'mode': 'delta'}, {'angle': float(aR), 'mode': 'delta'}]
            else:
                atype = "Noisy or unmatched peaks; using S-min window + Δ-fit"
                S = r1 + r2; i0 = int(np.argmin(S))
                zone = _s_min_zone_indices_window(ang, S, max(0, i0 - GLOBAL_HALF_WIN), min(len(S)-1, i0 + GLOBAL_HALF_WIN), tol=S_MIN_PLATEAU_TOL)
                a_ref = _refine_angle_near(
                    float(_choose_angle_in_zone(ang, r1, r2, zone, adc2d_1, adc2d_2, S=S, tie_center=float(ang[i0]))),
                    ang, r1, r2, adc2d_1, adc2d_2, ANGLE_REFINE_WIN
                )
                found.append({'angle': float(a_ref), 'mode':'delta'})

    out, seen = [], set()
    angles_found = [float(s['angle']) for s in found]

    def _offaxis_boost(a_deg, d_base):
        off = abs(a_deg - ANGLE_MID)
        if off <= OFF_DEADBAND_DEG: return d_base
        t = min(1.0, (off - OFF_DEADBAND_DEG) / (90.0 - OFF_DEADBAND_DEG))
        return d_base * (1.0 + OFF_MAX_BOOST * t)

    for src in found:
        a = float(src['angle']); ak = round(a)
        if ak in seen: continue
        seen.add(ak)

        r1_at = float(np.interp(a, ang, r1))
        r2_at = float(np.interp(a, ang, r2))
        d1_direct = float(adc2d_1(r1_at))
        d2_direct = float(adc2d_2(r2_at))

        if src.get('mode','avg') == 'delta':
            d_est, ok = distance_from_delta(r1_at, r2_at, adc2d_1, adc2d_2)
            dist = (d1_direct + d2_direct)/2.0 if not ok else d_est
        elif src['mode'] == 'wide':
            if CROSSTALK_WIDE_ENABLE and len(angles_found) >= 2:
                if len(angles_found) == 2:
                    a1, a2 = sorted(angles_found)
                    (J1a, J2a), (J1b, J2b) = _demix_pair_intensities(a1, a2, r1, r2, ang)
                    if abs(a - a1) < abs(a - a2): i1_clean, i2_clean = J1a, J2a
                    else:                           i1_clean, i2_clean = J1b, J2b
                    r1_at_clean = 1023.0 - i1_clean
                    r2_at_clean = 1023.0 - i2_clean
                else:
                    r1_at_clean, r2_at_clean, _, _ = _clean_crosstalk_for_angle(a, angles_found, r1, r2, ang)
                d1_direct = float(adc2d_1(r1_at_clean))
                d2_direct = float(adc2d_2(r2_at_clean))
                i1w = (1023.0 - float(r1_at_clean)) + 1e-3
                i2w = (1023.0 - float(r2_at_clean)) + 1e-3
                w1, w2 = (i1w ** WEIGHT_GAMMA), (i2w ** WEIGHT_GAMMA)
                if a < (ANGLE_MID - ANGLE_BAND):   w2 *= FAR_WEIGHT
                elif a > (ANGLE_MID + ANGLE_BAND): w1 *= FAR_WEIGHT
                d_direct_clean = (w1 * d1_direct + w2 * d2_direct) / (w1 + w2)
                if sum2d is not None:
                    S_clean = float(r1_at_clean + r2_at_clean)
                    D_clean = float(abs(r1_at_clean - r2_at_clean))
                    S_eff   = S_clean + K_DELTA_TO_SUM * D_clean
                    d_sum   = float(sum2d(S_eff))
                else:
                    d_sum = d_direct_clean
                off = abs(a - ANGLE_MID)
                wsum = SUMW_CENTER_W if off <= OFF_DEADBAND_DEG else SUMW_OFFAXIS_W
                dist0 = wsum * d_sum + (1.0 - wsum) * d_direct_clean
                shrink = CENTER_SHRINK_K * math.exp(-0.5 * ((a - ANGLE_MID)/CENTER_SIGMA_DEG)**2)
                dist   = _offaxis_boost(a, dist0 * (1.0 - shrink))
                if WIDE_USE_DELTA:
                    d_delta_clean, ok_delta = distance_from_delta(r1_at_clean, r2_at_clean, adc2d_1, adc2d_2)
                    if ok_delta and abs(d_delta_clean - dist) <= WIDE_GATE_CM:
                        dist = 0.5 * (dist + d_delta_clean)
            else:
                dist = (d1_direct + d2_direct)/2.0
        else:
            dist = (d1_direct + d2_direct)/2.0

        if SNAP_BY_DIRECT_CM:
            d_avg = 0.5*(d1_direct + d2_direct)
            if (abs(d1_direct - d2_direct) <= SNAP_MAX_SENSOR_DISAGREE and abs(dist - d_avg) <= SNAP_CM_EPS):
                dist = round(d_avg)

        out.append({'angle': a, 'distance': float(dist)})

    peaks1 = [{'angle': ang[p], 'adc': r1[p]} for p in p1o]
    peaks2 = [{'angle': ang[p], 'adc': r2[p]} for p in p2o]
    plats1 = [{'angle': ang[i], 'adc': r1[i]} for i in pl1]
    plats2 = [{'angle': ang[i], 'adc': r2[i]} for i in pl2]
    return out, atype, peaks1, peaks2, plats1, plats2
"""
Generates and displays a polar plot for detected light sources.
Represents each source as a single point and provides details on hover.
"""
def draw_scanner_map_lights(scan_data):
    if not scan_data:
        sg.popup("Scan complete. No light sources detected."); return
    distances = [item[0] for item in scan_data]
    angles    = [item[1] for item in scan_data]
    angles_rad = [math.radians(a) for a in angles]

    layout = [[sg.Canvas(key="-CANVAS-", size=(800, 800))]]
    window = sg.Window("Light Sources Map", layout, finalize=True, resizable=True)

    fig = plt.figure(figsize=(8, 8))
    ax = fig.add_subplot(111, polar=True)
    scatter = ax.scatter(angles_rad, distances, s=100, edgecolors='black', alpha=0.75, label='Light Source', color='yellow' )

    ax.set_thetamin(0); ax.set_thetamax(180); ax.set_ylim(0, 60)
    ax.set_title("Detected Light Sources", va='bottom'); ax.legend()

    annot = ax.annotate("", xy=(0,0), xytext=(20,20), textcoords="offset points",
                        bbox=dict(boxstyle="round", fc="orange", alpha=0.8),
                        arrowprops=dict(arrowstyle="->"))
    annot.set_visible(False)
    def update_annot(ind):
        pos = scatter.get_offsets()[ind["ind"][0]]; annot.xy = pos
        i = ind["ind"][0]; annot.set_text(f"Angle: {angles[i]:.1f}°\nDist: {distances[i]:.1f} cm")
    def hover(event):
        vis = annot.get_visible()
        if event.inaxes == ax:
            cont, ind = scatter.contains(event)
            if cont: update_annot(ind); annot.set_visible(True); fig.canvas.draw_idle()
            elif vis: annot.set_visible(False); fig.canvas.draw_idle()
    fig.canvas.mpl_connect("motion_notify_event", hover)

    canvas = FigureCanvasTkAgg(fig, master=window["-CANVAS-"].TKCanvas)
    canvas.draw(); canvas.get_tk_widget().pack(side="top", fill="both", expand=True)

    while True:
        ev, _ = window.read()
        if ev == sg.WINDOW_CLOSED: break

    canvas.get_tk_widget().destroy(); plt.close(fig); window.close()


# ======================= HELPERS funcs for state 3 =======================
def _g_leak(delta_deg: float) -> float:
    c = math.cos(math.radians(float(delta_deg)))
    if c <= 0.0: return 0.0
    g = CROSSTALK_KAPPA * (c ** CROSSTALK_POWER)
    return float(min(CROSSTALK_G_MAX, max(0.0, g)))

def _clean_crosstalk_for_angle(a_k, angles_all, r1, r2, ang):
    I1_k = 1023.0 - float(np.interp(a_k, ang, r1))
    I2_k = 1023.0 - float(np.interp(a_k, ang, r2))
    leak1 = leak2 = 0.0
    for a_j in angles_all:
        if abs(a_j - a_k) < 1e-9: continue
        g = _g_leak(abs(a_k - a_j))
        I1_j = 1023.0 - float(np.interp(a_j, ang, r1))
        I2_j = 1023.0 - float(np.interp(a_j, ang, r2))
        leak1 += g * I1_j; leak2 += g * I2_j
    I1_clean = max(0.0, I1_k - leak1)
    I2_clean = max(0.0, I2_k - leak2)
    return 1023.0 - I1_clean, 1023.0 - I2_clean, I1_clean, I2_clean

def _demix_pair_intensities(a1, a2, r1, r2, ang):
    g = _g_leak(abs(a1 - a2))
    denom = max(1e-6, 1.0 - g*g)
    I1a = 1023.0 - float(np.interp(a1, ang, r1))
    I1b = 1023.0 - float(np.interp(a2, ang, r1))
    J1a = max(0.0, (I1a - g*I1b) / denom)
    J1b = max(0.0, (I1b - g*I1a) / denom)
    I2a = 1023.0 - float(np.interp(a1, ang, r2))
    I2b = 1023.0 - float(np.interp(a2, ang, r2))
    J2a = max(0.0, (I2a - g*I2b) / denom)
    J2b = max(0.0, (I2b - g*I2a) / denom)
    return (J1a, J2a), (J1b, J2b)

def _fmt_plats(plats):
    return ', '.join([f"{q['angle']:.1f}° (ADC={q['adc']})" for q in plats]) or 'None'

def make_adc2dist_func(calib_volts):
    if len(calib_volts) != 10:
        sg.popup_error("Calibration must have exactly 10 numbers (for 5..50 cm).")
        return None
    d_full   = np.arange(5, 51, 5, float)
    v_full   = np.maximum.accumulate(np.array(calib_volts, float))
    adc_full = np.maximum.accumulate(v_full * ADC_PER_V)
    mask_unsat = adc_full < (SAT_ADC + 1)
    sat_end_cm = float(d_full[np.where(mask_unsat)[0][-1]]) if np.any(mask_unsat) else 25.0
    d_curve   = np.arange(5.0, sat_end_cm + 0.001, 1.0)
    adc_curve = np.interp(d_curve, d_full, adc_full)
    adc_eps   = adc_full + np.linspace(0.0, 1e-6, len(adc_full))
    def f(adc_in):
        return np.interp(np.asarray(adc_in, float), adc_eps, d_full, left=5.0, right=50.0)
    f.d_curve, f.adc_curve = d_curve, adc_curve
    return f

def make_sum2dist_func(f1, f2):
    d = f1.d_curve
    d_max = min(f1.d_curve[-1], f2.d_curve[-1])
    d = d[d <= d_max]
    a1 = np.interp(d, f1.d_curve, f1.adc_curve)
    a2 = np.interp(d, f2.d_curve, f2.adc_curve)
    S  = np.maximum.accumulate(a1 + a2)
    def s2d(S_in):
        return float(np.interp(float(S_in), S, d, left=d[0], right=d[-1]))
    s2d.S_curve, s2d.d_curve = S, d
    return s2d

def _adc_at_distance(f, d):
    return float(np.interp(float(d), f.d_curve, f.adc_curve))

def merge_adjacent_peaks(ids, ang, raw):
    ids = list(ids)
    if len(ids) <= 1: return np.array(ids, int)
    out, grp = [], [ids[0]]
    for i in ids[1:]:
        if (ang[i] - ang[grp[-1]]) < MERGE_TOLERANCE_DEG: grp.append(i)
        else:
            s, e = grp[0], grp[-1]
            out.append(s + np.argmin(raw[s:e+1])); grp = [i]
    s, e = grp[0], grp[-1]
    out.append(s + np.argmin(raw[s:e+1]))
    return np.array(out, int)

def analyze_valley(raw, ang, p):
    if p is None or p < 0 or p >= len(raw): return None
    thr = raw[p] + (25.5 - raw[p]/34.0)
    ei = next((i+1 for i in range(p, -1, -1) if raw[i] > thr), 0)
    xo = next((i-1 for i in range(p, len(raw)) if raw[i] > thr), len(raw)-1)
    ei, xo = max(0, min(p, ei)), min(len(raw)-1, max(p, xo))
    w = ang[xo] - ang[ei]
    return {'entry_angle': ang[ei], 'exit_angle': ang[xo], 'width': w, 'is_wide': w >= WIDE_VALLEY_THRESH}

def _filter_peaks_by_width_and_stability(peaks, intens_sm, ang, raw):
    if len(peaks) == 0: return peaks
    widths, _, _, _ = peak_widths(intens_sm, peaks, rel_height=0.5)
    keep, step = [], (np.mean(np.diff(ang)) if len(ang) > 1 else 1.0)
    for p, w in zip(peaks, widths):
        if w * step < MIN_PEAK_WIDTH_DEG: continue
        i0, i1 = max(0, p-3), min(len(raw), p+4)
        if (i1-i0) >= 3 and np.std(raw[i0:i1]) > MAX_LOCAL_STD: continue
        keep.append(int(p))
    return np.array(keep, int)

def _detect_plateaus(raw, ang, min_len=5, max_range=6, max_step=1):
    raw = np.asarray(raw, float); ang = np.asarray(ang, float)
    idxs = []; n = len(raw); i = 0
    while i < n-1:
        if raw[i] >= SAT_ADC: i += 1; continue
        j = i + 1
        while j < n and raw[j] < SAT_ADC and abs(raw[j]-raw[j-1]) <= max_step:
            j += 1
        seg = raw[i:j]
        if len(seg) >= min_len and (seg.max()-seg.min()) <= max_range:
            idxs.append(int((i+j-1)//2))
        i = j
    return np.array(sorted(set(idxs)), int)

def _delta_table(f1, f2):
    d = f1.d_curve
    d_common_end = min(f1.d_curve[-1], f2.d_curve[-1])
    d = d[d <= d_common_end]
    a1 = np.interp(d, f1.d_curve, f1.adc_curve)
    a2 = np.interp(d, f2.d_curve, f2.adc_curve)
    return d, np.abs(a1 - a2), a1, a2

def _fit_distance_from_pair(r1_meas, r2_meas, f1, f2):
    d_vec, delta_vec, a1_vec, a2_vec = _delta_table(f1, f2)
    d_best, score_best, err1_best, err2_best = d_vec[0], 1e18, None, None
    d_meas = abs(r1_meas - r2_meas)
    for d, dlt, a1, a2 in zip(d_vec, delta_vec, a1_vec, a2_vec):
        e1, e2 = abs(r1_meas - a1), abs(r2_meas - a2)
        score = (FIT_W_DELTA*abs(d_meas - dlt) + FIT_W_SYMM*abs(e1 - e2) + FIT_W_ERR*(e1 + e2))
        if score < score_best:
            score_best, d_best, err1_best, err2_best = score, d, e1, e2
    ok = (max(err1_best, err2_best) <= DELTA_ACCEPT_MAX_ERR)
    a1e = _adc_at_distance(f1, d_best); a2e = _adc_at_distance(f2, d_best)
    return float(d_best), ok, float(err1_best), float(err2_best), float(abs(a1e - a2e))

def distance_from_delta(r1_meas, r2_meas, f1, f2):
    d_est, ok, *_ = _fit_distance_from_pair(r1_meas, r2_meas, f1, f2)
    return d_est, ok

def _s_min_zone_indices_window(angles, S, start, end, tol=S_MIN_PLATEAU_TOL):
    start = max(0, int(start)); end = min(len(S)-1, int(end))
    if end < start: start, end = end, start
    i0 = start + int(np.argmin(S[start:end+1])); v = S[i0]
    L = i0
    while L-1 >= start and S[L-1] <= v + tol: L -= 1
    R = i0
    while R+1 <= end   and S[R+1] <= v + tol: R += 1
    return np.arange(L, R+1)

def _edge_monotone_ok(raw, ang, side):
    if side == 'left': idxs = np.where(ang <= EDGE_LEFT_MAX_DEG)[0]
    else:               idxs = np.where(ang >= EDGE_RIGHT_MIN_DEG)[0]
    if len(idxs) < EDGE_MONO_MIN_POINTS: return False
    y = raw[idxs]; a = ang[idxs]; slope = np.polyfit(a, y, 1)[0]
    if side == 'right': return (slope < -0.1) and (y[0] - y[-1] >= EDGE_MONO_MIN_DROP)
    else:               return (slope > 0.1) and (y[-1] - y[0] >= EDGE_MONO_MIN_DROP)

def _augment_with_edge_seed(peaks, raw, ang):
    ids = list(peaks) if isinstance(peaks, (list, np.ndarray)) else ([int(peaks)] if peaks is not None else [])
    last = len(ang) - 1
    if _edge_monotone_ok(raw, ang, 'right') and not any(abs(ang[p]-ang[last]) <= EDGE_NEAR_TOL_DEG for p in ids):
        ids.append(last)
    if _edge_monotone_ok(raw, ang, 'left')  and not any(abs(ang[p]-ang[0])   <= EDGE_NEAR_TOL_DEG for p in ids):
        ids.append(0)
    return np.array(sorted(set(ids)), int)

def _refine_angle_near(a0, ang, r1, r2, f1, f2, half_win=ANGLE_REFINE_WIN):
    i0 = int(round(a0)); L = max(0, i0-half_win); R = min(len(ang)-1, i0+half_win)
    best_i, best_score = i0, 1e18
    for i in range(L, R+1):
        d_est, _, e1, e2, dlt_pred = _fit_distance_from_pair(r1[i], r2[i], f1, f2)
        d_meas = abs(r1[i] - r2[i])
        score = FIT_W_DELTA*abs(d_meas - dlt_pred) + FIT_W_SYMM*abs(e1 - e2) + FIT_W_ERR*(e1 + e2)
        if score < best_score: best_score, best_i = score, i
    return float(ang[best_i])

def _choose_angle_in_zone(angles, r1, r2, zone, f1, f2, S=None, tie_center=None, prefer=None):
    a = np.asarray(angles, float)
    w1 = FIT_W_ERR * (2.0 if prefer == 'LDR1' else 1.0)
    w2 = FIT_W_ERR * (2.0 if prefer == 'LDR2' else 1.0)
    primary = []
    for i in zone:
        d_est, _, e1, e2, dlt_pred = _fit_distance_from_pair(r1[i], r2[i], f1, f2)
        d_meas = abs(r1[i] - r2[i])
        scoreA = FIT_W_DELTA*abs(d_meas - dlt_pred) + FIT_W_SYMM*abs(e1 - e2) + (w1*e1 + w2*e2)
        score  = scoreA + (LOCAL_S_WEIGHT * (S[i] if S is not None else 0.0))
        primary.append((int(i), score))
    smin = min(primary, key=lambda t: t[1])[1]
    eps  = smin*PRIMARY_REL_EPS + PRIMARY_ABS_EPS
    cand = [t for t in primary if (t[1] - smin) <= eps]
    if S is not None:
        cand.sort(key=lambda t: (S[t[0]], abs((a[t[0]] if tie_center is None else a[t[0]]-tie_center))))
    else:
        cand.sort(key=lambda t: t[1])
    return float(angles[cand[0][0]])


#########################################
################ STATE 4 ################
#########################################

"""
Handles the GUI and logic for the combined Light & Object Detector (Mode 4).
Receives a 5-byte data packet and orchestrates the processing and plotting.
"""
def light_objects_detector():
    global adc2dist_func_1, adc2dist_func_2, sum2dist_func
    # --- הגדרת סגנונות ---
    button_style_primary = {'button_color': ('white', ACCENT_COLOR), 'font': ('Segoe UI', 12), 'border_width': 0,
                            'mouseover_colors': ('white', ACCENT_COLOR_LIGHT)}
    button_style_secondary = {'button_color': ('white', SECONDARY_BUTTON_COLOR), 'font': ('Segoe UI', 12),
                              'border_width': 0, 'mouseover_colors': ('white', '#707B7C')}

    # --- מבנה מסגרת בקרה ---
    controls_frame_layout = [
        [sg.Text("Masking Distance [cm]:", background_color=THEME_BG, text_color=TEXT_COLOR, font=('Segoe UI', 11)),
         sg.InputText(key="-DISTANCE-", size=(20, 1), default_text="200", background_color=INPUT_BG,
                      text_color=TEXT_COLOR, border_width=0)],
        [sg.Push(background_color=THEME_BG),
         sg.Button("Start Combined Scan", key="-SCAN-", **button_style_primary),
         sg.Button("Copy Log", key="-COPY_LOG-", **button_style_secondary),
         sg.Button("Back", key="-BACK-", **button_style_secondary),
         sg.Push(background_color=THEME_BG)]
    ]

    # --- מבנה מסגרת פלט ---
    output_frame_layout = [
        [sg.Output(key="-OUTPUT-", size=(80, 15), background_color=INPUT_BG, text_color=TEXT_COLOR)]
    ]

    # --- מבנה ראשי המשלב הכל ---
    layout = [
        [sg.Text("Light & Object Detector (Bonus)", font=('Segoe UI', 18, 'bold'), background_color=THEME_BG,
                 text_color=TEXT_COLOR)],
        [sg.Frame("Controls", controls_frame_layout, background_color=THEME_BG, border_width=0,
                  title_color=TEXT_COLOR)],
        [sg.Frame("Log Output", output_frame_layout, background_color=THEME_BG, border_width=0, title_color=TEXT_COLOR)]
    ]

    window = sg.Window("Light & Object Detector", layout, background_color=THEME_BG, resizable=True, finalize=True)

    while True:
        event, values = window.read()
        if event in (sg.WINDOW_CLOSED, "-BACK-", "-ESCAPE-"):
            send_command('0')
            break

        elif event == "-COPY_LOG-":
            pyperclip.copy(window["-OUTPUT-"].get())
            sg.popup_quick("Log copied to clipboard!")

        elif event == "-SCAN-" or event == '\r':
            if not adc2dist_func_1 or not adc2dist_func_2:
                sg.popup_error("Calibration maps are empty. Please run a full calibration first.")
                continue

            window["-SCAN-"].update(disabled=True)
            window["-BACK-"].update(disabled=True)
            window["-OUTPUT-"].update("")

            # --- קליטת דאטה מהסיריאל ---
            raw_scan_data = []  # (distance, ldr1, ldr2, angle)
            s.reset_input_buffer()
            send_command('X')  # Combined scan

            while True:
                window.refresh()
                data_bytes = s.read(7)  # 3*uint16 + 1*uint8
                if len(data_bytes) < 7:
                    print("Timeout: Finished receiving data.")
                    break

                distance, ldr1_val, ldr2_val, angle = struct.unpack('<HHHB', data_bytes)

                if distance == 0xFFFF:
                    print("--- End of Scan Signal Received ---")
                    break

                raw_scan_data.append((distance, ldr1_val, ldr2_val, angle))
                window['-OUTPUT-'].update(
                    f"Angle: {angle:>3}°, Dist: {distance:<4}, LDR1: {ldr1_val:<4}, LDR2: {ldr2_val:<4}\n",
                    append=True
                )

            # --- עיבוד ---
            if not raw_scan_data:
                sg.popup("Scan complete. No data was received.")
            else:
                try:
                    masking_distance = float(values["-DISTANCE-"])
                except ValueError:
                    masking_distance = 200.0



                log = window['-OUTPUT-'].print
                object_scan_data = []
                for dist, _l1, _l2, ang in raw_scan_data:
                    d = 0.0 if dist > masking_distance else float(dist)
                    object_scan_data.append((d, float(ang)))

                objects = find_objects_firstlast_equal(
                    object_scan_data,
                    distance_threshold_cm=masking_distance,
                    edge_short_run_max_len=EDGE_SHORT_RUN_MAX_LEN
                )

                log(f"\n[OBJECTS] Found {len(objects)} object(s).")
                for o in objects:
                    log(
                        f"  Obj#{o['id']}: (ρ, φ, l) = "
                        f"({o.get('rho_display', o['rho']):.0f} cm, "
                        f"{o.get('phi_display', o['phi']):.0f}°, "
                        f"{o.get('width_display', o['width']):.0f} cm)"
                    )

                # ===== Lights Processing (using the correct global calibration models) =====
                light_scan_for_processing = [(item[1], item[2], float(item[3])) for item in raw_scan_data]

                # This is the main fix: We are now using the global variables directly
                light_sources, atype, peaks1, peaks2, plats1, plats2 = process_light_scan_data(
                    light_scan_for_processing,
                    adc2dist_func_1,
                    adc2dist_func_2,
                    sum2dist_func
                )

                log(f"\n[LIGHTS] Analysis Type: {atype}")
                if light_sources:
                    log(f"SUCCESS: Calculated {len(light_sources)} light source(s).")
                    for i, ssrc in enumerate(sorted(light_sources, key=lambda x: x['angle'])):
                        log(f"  Light#{i + 1}: Angle={ssrc['angle']:.2f}°, Distance={ssrc['distance']:.2f} cm")
                else:
                    log("No light sources identified.")

                # ===== Combined Plot =====
                draw_combined_map(objects, light_sources)

            window["-SCAN-"].update(disabled=False)
            window["-BACK-"].update(disabled=False)

    window.close()

    """
    Combined polar plot: Objects (מצב 1) + Lights (מצב 3).
    - Objects מצויירים כנקודות ב-(φ,ρ) עם תווית: ρ/φ/l (מעוגל half-up)
    - Lights מצויירים ב-(angle, distance)
    """

def draw_combined_map(objects, light_sources):
    if not objects and not light_sources:
        sg.popup("No objects or light sources to plot.")
        return

    layout = [[sg.Canvas(key="-CANVAS-", size=(900, 900))]]
    window = sg.Window("Combined Scanner Map", layout, finalize=True, resizable=True)

    fig = plt.figure(figsize=(9, 9))
    ax = fig.add_subplot(111, polar=True)

    # Objects
    obj_scat = None
    obj_labels = []
    if objects:
        obj_thetas = [math.radians(o['phi']) for o in objects]
        obj_rs = [o['rho'] for o in objects]
        obj_scat = ax.scatter(obj_thetas, obj_rs, s=90, alpha=0.85, label='Object', c='tab:blue')
        for o in objects:
            obj_labels.append(
                f"Object #{o['id']}\n"
                f"ρ={o.get('rho_display', o['rho']):.0f} cm\n"
                f"φ={o.get('phi_display', o['phi']):.0f}°\n"
                f"l={o.get('width_display', o['width']):.0f} cm"
            )

    # Lights
    light_scat = None
    light_labels = []
    if light_sources:
        L_thetas = [math.radians(s['angle']) for s in light_sources]
        L_rs = [s['distance'] for s in light_sources]
        light_scat = ax.scatter(L_thetas, L_rs, s=120, alpha=0.9, label='Light', edgecolors='black', c='yellow')
        for i, s in enumerate(light_sources, 1):
            light_labels.append(f"Light #{i}\nAngle={s['angle']:.1f}°\nDist={s['distance']:.1f} cm")

    annot = ax.annotate("", xy=(0, 0), xytext=(20, 20), textcoords="offset points",
                        bbox=dict(boxstyle="round", fc="orange", alpha=0.9),
                        arrowprops=dict(arrowstyle="->"))
    annot.set_visible(False)

    def hover(event):
        vis = annot.get_visible()
        if event.inaxes != ax:
            if vis: annot.set_visible(False); fig.canvas.draw_idle()
            return
        # objects first
        if obj_scat is not None:
            cont, ind = obj_scat.contains(event)
            if cont:
                i = ind['ind'][0]
                annot.xy = obj_scat.get_offsets()[i]
                annot.set_text(obj_labels[i])
                annot.set_visible(True);
                fig.canvas.draw_idle();
                return
        if light_scat is not None:
            cont, ind = light_scat.contains(event)
            if cont:
                i = ind['ind'][0]
                annot.xy = light_scat.get_offsets()[i]
                annot.set_text(light_labels[i])
                annot.set_visible(True);
                fig.canvas.draw_idle();
                return
        if vis: annot.set_visible(False); fig.canvas.draw_idle()

    fig.canvas.mpl_connect("motion_notify_event", hover)

    # Limits
    all_r = []
    if objects: all_r += [o['rho'] for o in objects]
    if light_sources: all_r += [s['distance'] for s in light_sources]
    rmax = max(all_r) if all_r else 50.0
    ax.set_thetamin(0);
    ax.set_thetamax(180);
    ax.set_ylim(0, rmax * 1.15)
    ax.set_title("Detected Objects (blue) and Light Sources (yellow)")
    ax.legend(loc='upper right')

    canvas = FigureCanvasTkAgg(fig, master=window["-CANVAS-"].TKCanvas)
    canvas.draw();
    canvas.get_tk_widget().pack(side="top", fill="both", expand=True)

    while True:
        ev, _ = window.read()
        if ev == sg.WINDOW_CLOSED:
            break

    canvas.get_tk_widget().destroy();
    plt.close(fig);
    window.close()



#########################################
################ STATE 5 ################
#########################################
"""
Handles the GUI and logic for Script Mode (Mode 5).
Allows uploading and executing pre-written script files on the MCU.
"""


def refresh_file_list(window):
    """
    Asks the MCU for the current list of files and updates the GUI.
    Returns the list of filenames.
    """
    global s
    if not window:
        return []

    print("Requesting file list from MCU...")
    window['-FILE_LIST-'].update([])  # Clear the listbox
    window.refresh()

    s.reset_input_buffer()
    s.write(b'L')  # Send 'List' command

    mcu_files = []
    # Loop to receive filenames until an End-of-Transmission (EOT) char is received
    while True:
        try:
            # Set a timeout for readline to avoid getting stuck
            s.timeout = 1
            line = s.readline()
            if not line:  # Timeout occurred
                print("Timeout waiting for file list.")
                break

            # Check for End of Transmission character (ASCII 4)
            if line.strip() == b'\x04':
                print("End of file list received.")
                break

            filename = line.decode('ascii').strip()
            if filename:
                mcu_files.append(filename)

        except Exception as e:
            print(f"Error reading file list: {e}")
            break

    window['-FILE_LIST-'].update(mcu_files)  # Update the GUI
    print(f"Found {len(mcu_files)} file(s).")
    return mcu_files


def file_mode():
    """
    Manages the File System and Calibration mode of the device.
    """
    global s
    global calibration_map_ldr1, calibration_map_ldr2
    working_directory = os.getcwd()  # Or your specific scripts directory
    mcu_file_list = []  # To store the list of files for mapping index

    # --- הגדרת סגנונות ---
    button_style_primary = {'button_color': ('white', ACCENT_COLOR), 'font': ('Segoe UI', 11), 'border_width': 0,
                            'mouseover_colors': ('white', ACCENT_COLOR_LIGHT)}
    button_style_secondary = {'button_color': ('white', SECONDARY_BUTTON_COLOR), 'font': ('Segoe UI', 11),
                              'border_width': 0, 'mouseover_colors': ('white', '#707B7C')}
    browse_button_style = {'button_color': ('white', SECONDARY_BUTTON_COLOR), 'font': ('Segoe UI', 11)}

    # --- מבנה מסגרת העלאת קבצים ---
    upload_frame_layout = [
        [sg.Text("File Path:", background_color=THEME_BG, text_color=TEXT_COLOR),
         sg.Input(key="-FILE_PATH-", size=(45, 1), background_color=INPUT_BG, text_color=TEXT_COLOR, border_width=0),
         sg.FileBrowse(button_text="Browse", **browse_button_style)],
        [sg.Button("Validate & Upload File", key="-UPLOAD-", **button_style_primary, size=(20, 1))]
    ]

    # --- מבנה מסגרת ניהול קבצים בבקר ---
    mcu_files_frame_layout = [
        [sg.Listbox(values=[], size=(45, 8), key="-FILE_LIST-", background_color=INPUT_BG, text_color=TEXT_COLOR,
                    highlight_background_color=ACCENT_COLOR, enable_events=True),
         sg.Column([
             [sg.Button("Play Script", key="-PLAY-", **button_style_secondary, size=(15, 1), disabled=True)],
             [sg.Button("View on LCD", key="-VIEW_LCD-", **button_style_secondary, size=(15, 1))],
             # --- התיקון: הוספת כפתור מחיקה בחזרה ---
             [sg.Button("Delete All Files", key="-DELETE-", **button_style_secondary, size=(15, 1))],
             [sg.Button("Refresh List", key="-REFRESH-", **button_style_secondary, size=(15, 1))],
         ], background_color=THEME_BG)]
    ]

    # --- מבנה מסגרת כיול ---
    calibration_layout = [
        [sg.Button("Start LDR Calibration", key="-CALIBRATE-", **button_style_secondary, size=(25, 1))],
        [sg.ProgressBar(20, orientation='h', size=(30, 20), key='-PBAR-', bar_color=(ACCENT_COLOR, INPUT_BG))]
    ]

    # --- מבנה מסגרת לוג וסטטוס ---
    status_layout = [
        [sg.Text("Status:", size=(8, 1), background_color=THEME_BG, text_color=TEXT_COLOR),
         sg.Text("Ready", key="-STATUS-", size=(60, 1), background_color=THEME_BG, text_color=TEXT_COLOR)],
        [sg.Output(key="-OUTPUT-", size=(80, 8), background_color=INPUT_BG, text_color=TEXT_COLOR)]
    ]

    # --- מבנה ראשי המשלב הכל ---
    layout = [
        [sg.Text("File Management & Calibration", font=('Segoe UI', 18, 'bold'), background_color=THEME_BG,
                 text_color=TEXT_COLOR)],
        [sg.Frame("File Upload", upload_frame_layout, background_color=THEME_BG, border_width=0,
                  title_color=TEXT_COLOR)],
        [sg.Frame("MCU File System", mcu_files_frame_layout, background_color=THEME_BG, border_width=0,
                  title_color=TEXT_COLOR)],
        [sg.Frame("LDR Calibration", calibration_layout, element_justification='center', background_color=THEME_BG,
                  border_width=0, title_color=TEXT_COLOR)],
        [sg.Frame("Log & Status", status_layout, background_color=THEME_BG, border_width=0, title_color=TEXT_COLOR)],
        # --- התיקון: הוספת כפתור העתקת לוג ---
        [sg.Button("Copy Log", key="-COPY_LOG-", **button_style_secondary), sg.Push(background_color=THEME_BG),
         sg.Button("Back", key="-BACK-", **button_style_secondary)]
    ]

    window = sg.Window("File Mode", layout, background_color=THEME_BG, finalize=True)

    buttons_to_disable = ["-UPLOAD-", "-REFRESH-", "-CALIBRATE-", "-PLAY-", "-DELETE-", "-VIEW_LCD-"]
    STATE_WAITING_FOR_HEADER = 0
    STATE_WAITING_FOR_PAYLOAD = 1
    current_state = STATE_WAITING_FOR_HEADER
    # Initial population of the file list
    mcu_file_list = refresh_file_list(window)

    calibration_active = False
    script_is_running = False  # משתנה לניהול מצב ריצת הסקריפט
    calibration_sample_count = 0
    LDR1_calibrate_arr, LDR2_calibrate_arr = [], []

    while True:
        event, values = window.read(timeout=20)

        while s.in_waiting > 0:

            if calibration_active:
                try:
                    # קרא שורה שלמה, פענח, ונקה רווחים
                    line = s.readline().decode('ascii').strip()
                    if line:
                        adc_val = int(line)
                        voltage = adc_val / 287.0
                        calibration_sample_count += 1

                        if calibration_sample_count <= 10:
                            LDR1_calibrate_arr.append(voltage)
                            print(f"LDR1 Sample {calibration_sample_count} Received. Voltage: {voltage:.2f} [V]")
                        else:
                            LDR2_calibrate_arr.append(voltage)
                            print(f"LDR2 Sample {calibration_sample_count - 10} Received. Voltage: {voltage:.2f} [V]")

                        window['-PBAR-'].update(current_count=calibration_sample_count)

                        if calibration_sample_count >= 20:
                            print("--- Calibration Complete! ---")
                            calibration_active = False
                            window["-CALIBRATE-"].update("Start LDR Calibration")
                            # כאן ניתן להפעיל את הפונקציה expand_calibration_array
                except (ValueError, IndexError) as e:
                    print(f"Calibration data error: {e}")

                # דלג על שאר הלולאה כדי להמשיך לקרוא שורות כיול
                continue

            # --- מצב 2: תקשורת בינארית רגילה ---
            # קרא בייט אחד בכל פעם ופעל בהתאם לערך שלו
            first_byte = s.read(1)

            # מקרה א': התקבלה חבילת נתוני מדידה
            if first_byte == b'\xfe':
                print("---------------------------------")
                payload = s.read(3) # קרא את 3 בתים הנתונים
                if len(payload) == 3:
                    distance = (payload[1] << 8) | payload[0]
                    angle = payload[2]
                    print(f"MCU SCRIPT RESPONSE -> Angle: {angle} deg, Distance: {distance} cm")
                else:
                    print("Error: Incomplete measurement packet received.")

            # מקרה ב': התקבל אות התחלת סקריפט
            elif first_byte == b'\xcc':
                print("MCU SIGNAL: Script execution started.")
                script_is_running = True
                for key in buttons_to_disable:
                    window[key].update(disabled=True)

            # מקרה ג': התקבל אות סיום סקריפט
            elif first_byte == b'\x7f':
                print("MCU SIGNAL: Script execution finished.")
                script_is_running = False
                for key in buttons_to_disable:
                    window[key].update(disabled=False)



        # ---------------------------------------------------------------------
        # חלק ב': טיפול באירועים מהמשתמש (לחיצות כפתורים)
        # ---------------------------------------------------------------------
        if event == sg.WIN_CLOSED or event == "-BACK-":
            if script_is_running:
                send_command('Q')  # אם סקריפט רץ, שלח בקשת עצירה
            else:
                send_command('0')  # אם לא, פשוט בקש לחזור לתפריט
            break

        elif event == "-COPY_LOG-":
            log_content = window["-OUTPUT-"].get()

            pyperclip.copy(log_content)

            sg.popup_quick("Log copied to clipboard!")

        # (החזרתי את שאר הכפתורים שהיו חסרים בקוד שלך)
        elif event == "-PLAY-":
            if values['-FILE_LIST-']:
                selected_filename = values['-FILE_LIST-'][0]
                s.reset_input_buffer()
                s.write(b'R')
                time.sleep(0.05)
                s.write(selected_filename.encode('ascii') + b'\n')
            else:

                sg.popup_warn("Please select a file.")

        elif event == '-FILE_LIST-':
            # This event fires whenever the user clicks an item in the listbox
            if values['-FILE_LIST-']:  # This checks if something is selected
                # An item is selected, so enable the buttons
                window['-PLAY-'].update(disabled=False)
            else:
                # Nothing is selected, so disable the buttons
                window['-PLAY-'].update(disabled=True)

        elif event == "-REFRESH-":
            mcu_file_list = refresh_file_list(window)

        elif event == "-UPLOAD-":
            # בלוק הקוד שלך להעלאת קבצים היה תקין והשארתי אותו כאן
            filepath = values["-FILE_PATH-"]
            if not filepath or not os.path.exists(filepath):
                sg.popup_error("File path is empty or file does not exist.")
                break

            window.disable()
            window["-STATUS-"].update(f"Validating file: {os.path.basename(filepath)}...")
            file_type = determine_file_type(filepath)
            is_valid, error_message = validate_script_syntax(filepath) if file_type == 2 else (True, "OK")

            if not is_valid:
                sg.popup_error(f"Validation Failed!\n\n{error_message}")
                window.enable()
                break

            window["-STATUS-"].update("Validation successful. Starting upload...")
            success = upload_file_to_mcu(filepath, file_type)

            if success:
                window["-STATUS-"].update("Upload complete. Refreshing list...")
                mcu_file_list = refresh_file_list(window)
            else:
                window["-STATUS-"].update("Upload failed.")
            window.enable()



        elif event == "-REFRESH-":
            mcu_file_list = refresh_file_list(window)


        elif event == "-VIEW_LCD-":
            window["-STATUS-"].update("MCU is now in LCD browsing mode. Use pushbuttons on the device.")
            send_command('V')

        elif event == "-DELETE-":
            send_command('D')
            mcu_file_list = refresh_file_list(window)


        elif event == "-CALIBRATE-":
            if not calibration_active:
                s.reset_input_buffer()

                send_command('J')

                calibration_active = True
                calibration_sample_count = 0
                LDR1_calibrate_arr = []  # Temporary list for LDR1
                LDR2_calibrate_arr = []  # Temporary list for LDR2

                window["-CALIBRATE-"].update("Abort Calibration")
                # Disable other buttons...
                window["-OUTPUT-"].update("Calibration started for LDR1. Press PB0 on device for sample 1...\n")
                window['-PBAR-'].update(current_count=0)
            else:
                # Abort logic
                calibration_active = False
                send_command('R')  # You need a command to reset the MCU state
                window["-CALIBRATE-"].update("Start LDR Calibration")
                # Re-enable buttons...
                window['-OUTPUT-'].update("Calibration aborted by user.\n", append=True)


    window.close()


# In main.py (add this function before the file_mode function)

def refresh_file_list(window):
    """
    Asks the MCU for the current list of files and updates the GUI Listbox.
    Returns the Python list of filenames received.
    """
    global s
    if not window:
        return []

    window["-STATUS-"].update("Requesting file list from MCU...")
    window['-FILE_LIST-'].update([])  # Clear the listbox before populating
    window.refresh()

    s.reset_input_buffer()
    s.write(b'L')  # Send the 'List' command

    mcu_files = []
    # Loop to receive filenames until an End-of-Transmission (EOT) char is received
    while True:
        try:
            s.timeout = 1.5  # Set a timeout to prevent getting stuck
            line = s.readline()

            if not line:  # Timeout occurred
                print("Timeout waiting for file list response.")
                break

            # The EOT character might be received alone or with other characters.
            # So we check if it's contained in the received bytes.
            if b'\x04' in line:
                print("End of file list marker received.")
                # It's possible the last filename came with the EOT, so we clean it.
                cleaned_line = line.replace(b'\x04', b'').strip()
                if cleaned_line:
                    mcu_files.append(cleaned_line.decode('ascii', errors='ignore'))
                break  # Exit the loop

            filename = line.decode('ascii', errors='ignore').strip()
            if filename:
                mcu_files.append(filename)



        except Exception as e:
            print(f"An error occurred while reading the file list: {e}")
            break

    window['-FILE_LIST-'].update(mcu_files)  # Update the GUI with the new list
    window["-STATUS-"].update(f"Found {len(mcu_files)} file(s) on device.")
    return mcu_files


# main.py

def determine_file_type(filepath):
    """
    Determines file type by finding the *first non-empty line*
    and analyzing its syntax. Ignores blank lines.
    """
    global command_dict
    try:
        with open(filepath, 'r') as f:
            # קוראים את הקובץ שורה אחר שורה
            for line in f:
                # 1. מנקים רווחים ותווי "אנטר" מתחילת וסוף השורה
                stripped_line = line.strip()

                # 2. אם השורה לא ריקה אחרי הניקוי, זו השורה שמעניינת אותנו
                if stripped_line:
                    # מצאנו את השורה הראשונה עם תוכן, ננתח אותה
                    parts = stripped_line.split()
                    command = parts[0].lower()

                    if command in command_dict:
                        _opcode, expected_args_count = command_dict[command]
                        args_part = parts[1:]

                        if not args_part:
                            actual_args_count = 0
                        else:
                            actual_args_count = len("".join(args_part).split(','))

                        if actual_args_count == expected_args_count:
                            # התחביר תואם לפקודת סקריפט
                            return 2  # SCRIPT

                    # אם הגענו לכאן, השורה הראשונה אינה פקודה חוקית -> זהו קובץ טקסט
                    return 1  # TEXT

    except Exception as e:
        print(f"An error occurred analyzing '{os.path.basename(filepath)}': {e}")
        return 1  # TEXT in case of any error

    # אם הקובץ ריק לחלוטין, הוא ייחשב כקובץ טקסט
    return 1  # TEXT


# In main.py

def validate_script_syntax(filepath):
    """
    Validates the syntax of a script file line by line.
    Returns (True, "OK") if valid, or (False, "Error message") if invalid.
    """
    global command_dict
    with open(filepath, 'r') as f:
        lines = f.readlines()

    for i, line in enumerate(lines):
        line_num = i + 1
        line = line.strip()
        if not line:  # Skip empty lines
            continue

        parts = line.split()
        command = parts[0]
        args = parts[1].split(',') if len(parts) > 1 else []

        # 1. Check if command is valid
        if command not in command_dict:
            return (False, f"Error on line {line_num}: Unknown command '{command}'")

        expected_args_count = command_dict[command][1]

        # 2. Check number of arguments
        if len(args) != expected_args_count:
            return (False,
                    f"Error on line {line_num}: Command '{command}' expects {expected_args_count} argument(s), but got {len(args)}")

        # 3. Check if arguments are valid integers
        for arg in args:
            try:
                int(arg)
            except ValueError:
                return (False, f"Error on line {line_num}: Argument '{arg}' is not a valid integer.")

    return (True, "Script syntax is valid.")


# main.py - הוסף את הפונקציה הזו לקובץ

def translate_script_to_binary(filepath):
    """
    Reads a text script file and translates it into its binary representation
    according to the project's ISA.
    Returns a bytearray of the encoded script, or None on error.
    """
    global command_dict
    encoded_bytes = bytearray()

    try:
        with open(filepath, 'r') as f:
            lines = f.readlines()

        for i, line in enumerate(lines):
            line = line.strip()
            if not line or line.startswith('#'):  # Skip empty lines or comments
                continue

            parts = line.split()
            command = parts[0].lower() # case-insensitive command
            # Arguments are separated by commas after the command
            args_str = ''.join(parts[1:]).split(',')

            if command not in command_dict:
                print(f"Error on line {i+1}: Unknown command '{command}'")
                return None

            opcode, expected_args_count = command_dict[command]

            # Handle cases with no arguments properly
            if expected_args_count == 0:
                args = []
            else:
                # Filter out any empty strings that might result from splitting
                args = [arg for arg in args_str if arg]

            if len(args) != expected_args_count:
                print(f"Error on line {i+1}: Command '{command}' expects {expected_args_count} args, got {len(args)}")
                return None

            # Add the opcode to our byte array
            encoded_bytes.append(opcode)

            # Add arguments
            for arg in args:
                try:
                    val = int(arg)
                    if not 0 <= val <= 255:
                       print(f"Error on line {i+1}: Argument '{arg}' is out of the valid range (0-255).")
                       return None
                    encoded_bytes.append(val)
                except ValueError:
                    print(f"Error on line {i+1}: Argument '{arg}' is not a valid integer.")
                    return None

        return encoded_bytes

    except Exception as e:
        print(f"An error occurred during script translation: {e}")
        return None




def upload_file_to_mcu(filepath, file_type):
    global s
    # ... (the start of the function, reading the file, etc. remains the same)
    content = None

    # אם הקובץ הוא סקריפט, תרגם אותו לבינארי
    if file_type == 2:  # 2 is the type for SCRIPT
        print("File identified as a script. Translating to binary...")
        content = translate_script_to_binary(filepath)
        if content is None:
            sg.popup_error("Script translation failed. Please check the syntax and see the console for errors.")
            return False
    # אחרת, קרא אותו כקובץ טקסט רגיל
    else:
        try:
            with open(filepath, 'rb') as f:
                content = f.read()
        except FileNotFoundError:
            sg.popup_error(f"File not found: {filepath}")
            return False
    

    filename_bytes = os.path.basename(filepath).encode('ascii')
    file_size = len(content)

    # --- Start Protocol (Handshakes 1 and 2 are the same) ---
    print(f"Starting upload for '{filename_bytes.decode()}' ({file_size} bytes)")
    s.reset_input_buffer()
    s.write(b'U')
    response = s.read(1)
    if response != b'A':
        sg.popup_error(f"MCU did not acknowledge upload start. Response: {response}")
        return False
    print("MCU ACK 1: Ready for metadata.")

    # Prepare metadata payload and calculate its checksum
    metadata_payload = struct.pack('<17sBH', filename_bytes, file_type, file_size)
    metadata_checksum = calculate_checksum(metadata_payload)

    # Create the packet (payload + checksum) and send it
    packet_to_send = bytearray(metadata_payload)
    packet_to_send.append(metadata_checksum)
    s.write(packet_to_send)

    response = s.read(1)

    if response == b'B':
        # Success! The MCU is ready for the file content.
        print("MCU ACK 2: Ready for content.")
    elif response == b'F':
        # Specific error: Not enough space on the device.
        sg.popup_error("Upload Failed: Not enough space on the MCU's flash memory.", title="Storage Error")
        return False
    elif response == b'N':
        # Specific error: Checksum on metadata failed.
        sg.popup_error("Upload Failed: MCU rejected metadata (Checksum failed). Please try again.",
                       title="Protocol Error")
        return False
    else:
        # Generic/unknown error.
        sg.popup_error(f"Upload Failed: Unknown response from MCU after metadata: {response}",
                       title="Communication Error")
        return False

    # --- NEW CHUNKED SENDING LOGIC ---
    # --- CHUNKED SENDING LOGIC WITH RETRY LIMIT ---
    CHUNK_SIZE = 64
    bytes_sent = 0
    upload_successful = False

    while bytes_sent < file_size:
        # Prepare the current chunk to be sent
        chunk_data = content[bytes_sent: bytes_sent + CHUNK_SIZE]

        # --- NEW: Retry loop for each chunk ---
        chunk_sent_successfully = False
        for attempt in range(3):  # This will try a maximum of 3 times
            print(f"Sending chunk at byte {bytes_sent} (Attempt {attempt + 1}/3)...")

            # Prepare and send the packet with checksum
            checksum = calculate_checksum(chunk_data)
            packet_to_send = bytearray(chunk_data)
            packet_to_send.append(checksum)
            s.write(packet_to_send)

            # Wait for a 1-byte response
            response = s.read(1)

            # Act based on the response
            if response in [b'K', b'S']:
                # ACK received: Success! Advance to the next chunk.
                bytes_sent += len(chunk_data)
                chunk_sent_successfully = True
                if response == b'S':
                    print("Final ACK received. Upload complete.")
                    upload_successful = True
                break  # Success, so break the inner 'for' loop and move to the next chunk
            else:
                # NACK or Timeout received.
                print(f"-> NACK or Timeout (Response: {response}). Retrying...")
                time.sleep(0.2)  # Small delay before the next attempt

        # After the retry loop, check if the chunk was sent successfully
        if not chunk_sent_successfully:
            sg.popup_error("Failed to send a data chunk after 3 attempts. Aborting upload.")
            return False  # Abort the entire upload

        # If the entire file has been sent
        if upload_successful:
            break  # Break the main 'while' loop

    # If the loop finished successfully
    sg.popup("Success!", "File uploaded successfully.")
    return True



"""
Translates a human-readable script string (e.g., 'servo_deg 90')
into the MCU's expected hex format (e.g., '065A').
"""
def command_encoder(data_str):
    translated_string = ""
    lines = data_str.split('\n')
    for line in lines:
        line = line.strip()
        if line:
            parts = line.split(' ', 1)
            command = parts[0]
            args = parts[1] if len(parts) > 1 else ""
            hex_value = command_dict.get(command)
            if hex_value is not None:
                opcode = hex(hex_value)[2:].zfill(2)  # Get the opcode in hex format
                hex_args = ''
                if args:
                    # Split the arguments by comma and convert each argument to hex format
                    hex_args_list = [hex(int(arg))[2:].zfill(2).upper() for arg in args.split(',')]
                    hex_args = ''.join(hex_args_list)  # Concatenate the hex arguments
                translated_string += opcode + hex_args + '\n'
    return translated_string



#########################################
#### GENERAL AND COMMUNICTION FUNCS #####
#########################################
"""
Initializes and opens the serial port connection to the MCU.
It resets the input and output buffers to ensure a clean communication session.
Returns True if the connection is successful, False otherwise.
"""
def init_uart():
    global s
    try:
        # פותחים את החיבור
        s = ser.Serial('COM17', baudrate=9600, timeout=1)

        s.write(b'\x00' * 50)
        s.flush()
        time.sleep(2)


        # מנקים את הבאפרים מיד לאחר הפתיחה המוצלחת
        s.reset_input_buffer()
        s.reset_output_buffer()

        print("Serial connection to COM17 established successfully.")
        return True

    except ser.SerialException as e:
        print(f"FATAL ERROR: Could not open serial port COM17. {e}")
        print("Please check the device connection and COM port number.")
        return False


"""
Sends a single command character to the MCU.
This is used for sending simple, one-byte commands like state change requests.
A small delay is added to ensure the MCU has time to process it.
"""
def send_command(char):
    global s
    s.write(bytes(char, 'ascii'))
    time.sleep(0.05)  # delay for accurate read/write operations on both ends



"""
Reads a single line of ASCII data from the serial port.
It waits for a newline character ('\n') or for the timeout to occur.
Returns the decoded and stripped string, or an empty string if no data is received.
"""
def receive_data():
    try:
        line = s.readline()
        return line.decode('ascii').strip()
    except Exception as e:
        print(f"שגיאה אירעה בזמן קבלת מידע: {e}")
        return ""



"""
Sends a multi-character data payload to the MCU.
It iterates through the string, sends each character, and then sends a '$' character
to signify the end of the transmission. Primarily used for uploading script data.
"""
def send_data(data_str):
    global s
    for char in data_str:
        a = len(data_str)
        send_command(char)
    time.sleep(0.05)
    s.write(bytes('$', 'ascii'))




def calculate_checksum(data_payload):
    """
    Calculates the Checksum 256 for a given data payload.
    """
    current_sum = sum(data_payload) % 256
    checksum = (256 - current_sum) % 256
    return checksum



"""
The main entry point of the PC-side application.
Initializes the GUI, establishes serial communication, performs the
initial handshake with the MCU, and runs the main event loop.
"""


def main():
    global s
    global adc2dist_func_1, adc2dist_func_2, sum2dist_func

    # =========================================================================
    #      שלב 1: אימות חיבור (לפני הצגת החלון)
    # =========================================================================
    try:
        if not init_uart():
            sg.popup_error("Failed to open serial port. Is the device connected?", title="Init Error")
            return

        print("Letting serial port stabilize...")
        time.sleep(2)
        s.reset_input_buffer()
        s.reset_output_buffer()
        print("Port is stable. Starting handshake.")

        handshake_successful = False
        for i in range(5):
            print(f"Pinging MCU... (Attempt {i + 1}/5)")
            send_command('P')
            response = s.read(1)
            if response == b'A':
                handshake_successful = True
                print("MCU Acknowledged! Connection established.")
                break
            time.sleep(0.5)

        if not handshake_successful:
            sg.popup_error("Failed to connect to MCU. Please check connection and reset the board.",
                           title="Connection Error")
            if 's' in globals() and s.is_open:
                s.close()
            return

    except Exception as e:
        sg.popup_error(f"A critical error occurred during connection:\n\n{e}", title="Fatal Error")
        return

    # =========================================================================
    #      שלב 2: הצגת החלון הראשי (רק אחרי חיבור מוצלח)
    # =========================================================================
    print("Connection verified. Launching GUI...")

    # --- הגדרות עיצוב ---
    THEME_BACKGROUND_COLOR = '#2C3E50'
    sg.theme_background_color(THEME_BACKGROUND_COLOR)
    IMAGE_BUTTON_STYLE = {'button_color': (THEME_BACKGROUND_COLOR, THEME_BACKGROUND_COLOR), 'border_width': 0,
                          'pad': (15, 15)}
    image_folder = 'Menu Buttons'

    main_column = [
        [sg.Push(), sg.Text("DCS Final Project", font=('Segoe UI', 24, 'bold'), text_color='#ECF0F1'), sg.Push()],
        [sg.Sizer(0, 20)],
        [sg.Push(),
         sg.Button('', image_filename=os.path.join(image_folder, 'button1_objects.png'), key='Object Detector System',
                   **IMAGE_BUTTON_STYLE),
         sg.Button('', image_filename=os.path.join(image_folder, 'button2_telemeter.png'), key='Telemeter',
                   **IMAGE_BUTTON_STYLE),
         sg.Button('', image_filename=os.path.join(image_folder, 'button3_lights.png'),
                   key='Light Sources Detector System', **IMAGE_BUTTON_STYLE),
         sg.Push()],
        [sg.Push(),
         sg.Button('', image_filename=os.path.join(image_folder, 'button4_lights_objects.png'),
                   key='Light Sources and Objects Detector System', **IMAGE_BUTTON_STYLE),
         sg.Button('', image_filename=os.path.join(image_folder, 'button5_file_mode.png'), key='File Mode',
                   **IMAGE_BUTTON_STYLE),
         sg.Button('', image_filename=os.path.join(image_folder, 'button6_quit.png'), key='Exit', **IMAGE_BUTTON_STYLE),
         sg.Push()],
        [sg.Sizer(0, 30)]
    ]
    layout = [[sg.Column(main_column, element_justification='center')]]
    window = sg.Window("DCS - Digital Computer Structure Project", layout, size=(800, 450),
                       element_justification='center', finalize=True)

    # --- לוגיקה לאחר יצירת החלון ---
    try:
        print("Requesting initial calibration data from MCU Flash...")
        s.reset_input_buffer()
        send_command('Z')
        time.sleep(0.5)
        calibration_bytes = s.read(20)

        if len(calibration_bytes) < 20:
            print("Warning: Did not receive all 20 calibration bytes. Maps will be empty.")
            calibration_map_ldr1 = []
            calibration_map_ldr2 = []
        else:
            # המרת הנתונים הגולמיים לוולטים
            msp_calib_arr1 = [(byte_value << 3) / 287.0 for byte_value in calibration_bytes[:10]]
            msp_calib_arr2 = [(byte_value << 3) / 287.0 for byte_value in calibration_bytes[10:]]
            print(f"LDR1 Initial data (as voltages): {msp_calib_arr1}")
            print(f"LDR2 Initial data (as voltages): {msp_calib_arr2}")

            # יצירת מודלי הכיול והשמה למשתנים הגלובליים
            adc2dist_func_1 = make_adc2dist_func(msp_calib_arr1)
            adc2dist_func_2 = make_adc2dist_func(msp_calib_arr2)

            # יצירת פונקציית העזר הנוספת
            if adc2dist_func_1 and adc2dist_func_2:
                sum2dist_func = make_sum2dist_func(adc2dist_func_1, adc2dist_func_2)
                print("Global calibration models are ready.")
            else:
                sum2dist_func = None
                print("Warning: Failed to create one or more calibration models.")

        # --- לולאת אירועים ראשית ---
        while True:
            event, values = window.read()

            if event in (sg.WIN_CLOSED, 'Exit'):
                send_command('Q')
                break

            menu_actions = {
                'Object Detector System': ('1', objects_detector),
                'Telemeter': ('2', telemeter),
                'Light Sources Detector System': ('3', lights_detector),
                'Light Sources and Objects Detector System': ('4', light_objects_detector),
                'File Mode': ('5', file_mode)
            }

            if event in menu_actions:
                command, function_to_call = menu_actions[event]
                send_command(command)
                time.sleep(0.1)
                window.hide()
                try:
                    function_to_call()
                finally:
                    window.un_hide()

    except Exception as e:
        print("==========================================")
        print("      AN UNEXPECTED ERROR OCCURRED!       ")
        print("==========================================")
        traceback.print_exc()
        sg.popup_error(f"A critical error occurred:\n\n{e}\n\nSee console for details.", title="Fatal Error")
    finally:
        if 's' in globals() and s.is_open:
            s.close()
        window.close()

if __name__ == '__main__':
    command_dict = {
        "inc_lcd":    (0x01, 1),
        "dec_lcd":    (0x02, 1),
        "rra_lcd":    (0x03, 1),
        "set_delay":  (0x04, 1),
        "clear_lcd":  (0x05, 0),
        "servo_deg":  (0x06, 1),
        "servo_scan": (0x07, 2),
        "sleep":      (0x08, 0)
    }
    main()

