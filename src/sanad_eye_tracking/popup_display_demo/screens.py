#!/usr/bin/env python3
"""
screens.py – Screen definitions for the SANAD smart wheelchair pop-up UI demo.

Each screen is a class that knows how to:
  • draw(canvas)  –  render itself onto a BGR canvas
  • get_popup_buttons()  –  return the 8 contextual popup actions for this screen
"""

import cv2
import numpy as np
import math
import time


# ═══════════════════════════════════════════════════════════════════════
#  Colour Palette
# ═══════════════════════════════════════════════════════════════════════
class C:
    """BGR colour constants for Ubuntu Yaru Dark UI."""
    BG_DARK       = (30, 30, 30)         # Very dark grey
    BG_CARD       = (45, 45, 45)         # Dark grey
    BG_CARD_ALT   = (61, 61, 61)         # Medium dark grey
    ACCENT_TEAL   = (32, 84, 233)        # Ubuntu Orange (override for primary accent)
    ACCENT_GREEN  = (50, 205, 50)        # Green
    ACCENT_ORANGE = (32, 84, 233)        # Ubuntu Orange
    ACCENT_RED    = (60, 60, 220)        # Red
    ACCENT_BLUE   = (210, 115, 20)       # Blue
    ACCENT_PURPLE = (111, 33, 119)       # Aubergine
    WHITE         = (240, 240, 240)      # Off-white
    GRAY          = (170, 170, 170)      # Light grey
    DIM           = (100, 100, 100)      # Dim grey
    HEADER_BG     = (35, 35, 35)         # Header
    DIVIDER       = (80, 80, 80)         # Divider


# ═══════════════════════════════════════════════════════════════════════
#  Drawing helpers
# ═══════════════════════════════════════════════════════════════════════
def draw_rounded_rect(img, pt1, pt2, color, radius=12, thickness=-1):
    """Draw a rectangle with rounded corners."""
    x1, y1 = pt1
    x2, y2 = pt2
    r = min(radius, (x2-x1)//2, (y2-y1)//2)
    # Fill
    if thickness == -1:
        cv2.rectangle(img, (x1+r, y1), (x2-r, y2), color, -1)
        cv2.rectangle(img, (x1, y1+r), (x2, y2-r), color, -1)
        cv2.circle(img, (x1+r, y1+r), r, color, -1)
        cv2.circle(img, (x2-r, y1+r), r, color, -1)
        cv2.circle(img, (x1+r, y2-r), r, color, -1)
        cv2.circle(img, (x2-r, y2-r), r, color, -1)
    else:
        pts = np.array([
            [x1+r, y1], [x2-r, y1],
            [x2, y1+r], [x2, y2-r],
            [x2-r, y2], [x1+r, y2],
            [x1, y2-r], [x1, y1+r]
        ], np.int32)
        cv2.polylines(img, [pts], True, color, thickness, cv2.LINE_AA)


def draw_icon_circle(img, cx, cy, radius, color, icon_char=None):
    """Draw a filled circle icon with an optional text character inside."""
    cv2.circle(img, (cx, cy), radius, color, -1, cv2.LINE_AA)
    # dark border
    cv2.circle(img, (cx, cy), radius, (0,0,0), 1, cv2.LINE_AA)
    if icon_char:
        font = cv2.FONT_HERSHEY_SIMPLEX
        scale = radius / 20.0
        (tw, th), _ = cv2.getTextSize(icon_char, font, scale, 2)
        cv2.putText(img, icon_char, (cx - tw//2, cy + th//2),
                    font, scale, C.WHITE, 2, cv2.LINE_AA)


def draw_header(img, title, subtitle=None, accent_color=C.ACCENT_TEAL):
    """Draw a top header bar."""
    h, w = img.shape[:2]
    bar_h = 64
    cv2.rectangle(img, (0, 0), (w, bar_h), C.HEADER_BG, -1)
    # Accent line
    cv2.rectangle(img, (0, bar_h-3), (w, bar_h), accent_color, -1)
    # Title
    cv2.putText(img, title, (24, 42), cv2.FONT_HERSHEY_SIMPLEX,
                1.0, C.WHITE, 2, cv2.LINE_AA)
    if subtitle:
        (tw, _), _ = cv2.getTextSize(title, cv2.FONT_HERSHEY_SIMPLEX, 1.0, 2)
        cv2.putText(img, subtitle, (24 + tw + 20, 42),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, C.GRAY, 1, cv2.LINE_AA)


def draw_status_bar(img, battery=85, speed="2.5 km/h", mode="AUTO"):
    """Draw a bottom status bar."""
    h, w = img.shape[:2]
    bar_h = 36
    y0 = h - bar_h
    cv2.rectangle(img, (0, y0), (w, h), C.HEADER_BG, -1)
    cv2.rectangle(img, (0, y0), (w, y0+2), C.DIVIDER, -1)

    # Battery
    bat_color = C.ACCENT_GREEN if battery > 30 else C.ACCENT_ORANGE if battery > 15 else C.ACCENT_RED
    cv2.putText(img, f"BAT: {battery}%", (16, y0+24),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, bat_color, 1, cv2.LINE_AA)
    # Speed
    cv2.putText(img, f"SPD: {speed}", (w//4, y0+24),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, C.GRAY, 1, cv2.LINE_AA)
    # Mode
    cv2.putText(img, f"MODE: {mode}", (w//2, y0+24),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, C.ACCENT_TEAL, 1, cv2.LINE_AA)
    # Time
    t_str = time.strftime("%H:%M")
    (tw, _), _ = cv2.getTextSize(t_str, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
    cv2.putText(img, t_str, (w - tw - 16, y0+24),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, C.WHITE, 1, cv2.LINE_AA)


def draw_card(img, x, y, w_card, h_card, title, lines, accent=C.ACCENT_TEAL, icon_char=None):
    """Draw a rounded card with a title and content lines."""
    draw_rounded_rect(img, (x, y), (x+w_card, y+h_card), C.BG_CARD, 14)
    # Accent top strip
    cv2.rectangle(img, (x, y), (x+w_card, y+4), accent, -1)
    # Icon
    if icon_char:
        draw_icon_circle(img, x+28, y+32, 16, accent, icon_char)
        title_x = x + 54
    else:
        title_x = x + 16
    cv2.putText(img, title, (title_x, y+38),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, C.WHITE, 1, cv2.LINE_AA)
    for i, line in enumerate(lines):
        cv2.putText(img, line, (x+16, y+66 + i*24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, C.GRAY, 1, cv2.LINE_AA)


def draw_big_icon_card(img, x, y, w_card, h_card, label, accent=C.ACCENT_TEAL, icon_char="?"):
    """A larger card with a centred icon and label below."""
    draw_rounded_rect(img, (x, y), (x+w_card, y+h_card), C.BG_CARD, 14)
    cx = x + w_card // 2
    cy = y + h_card // 2 - 12
    draw_icon_circle(img, cx, cy, 28, accent, icon_char)
    (tw, _), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.48, 1)
    cv2.putText(img, label, (cx - tw//2, cy + 50),
                cv2.FONT_HERSHEY_SIMPLEX, 0.48, C.WHITE, 1, cv2.LINE_AA)


def draw_progress_bar(img, x, y, w_bar, h_bar, progress, color=C.ACCENT_TEAL, bg=C.BG_CARD_ALT):
    """Draw a horizontal progress bar (progress in 0.0-1.0)."""
    draw_rounded_rect(img, (x, y), (x+w_bar, y+h_bar), bg, h_bar//2)
    fill_w = int(w_bar * max(0, min(1, progress)))
    if fill_w > 4:
        draw_rounded_rect(img, (x, y), (x+fill_w, y+h_bar), color, h_bar//2)


# ═══════════════════════════════════════════════════════════════════════
#  Screen: Home
# ═══════════════════════════════════════════════════════════════════════
class HomeScreen:
    NAME = "home"
    TITLE = "SANAD Home"

    def draw(self, canvas):
        h, w = canvas.shape[:2]
        canvas[:] = C.BG_DARK
        draw_header(canvas, "SANAD", "Smart Wheelchair System", C.ACCENT_TEAL)
        draw_status_bar(canvas)

        content_y = 90
        pad = 20
        card_w = (w - pad*3) // 2
        card_h = 130

        # Row 1: Navigation + Communication
        draw_big_icon_card(canvas, pad, content_y, card_w, card_h,
                           "Navigation", C.ACCENT_BLUE, "N")
        draw_big_icon_card(canvas, pad*2+card_w, content_y, card_w, card_h,
                           "Communication", C.ACCENT_GREEN, "C")

        # Row 2: Health + Settings
        draw_big_icon_card(canvas, pad, content_y+card_h+pad, card_w, card_h,
                           "Health Monitor", C.ACCENT_RED, "H")
        draw_big_icon_card(canvas, pad*2+card_w, content_y+card_h+pad, card_w, card_h,
                           "Settings", C.ACCENT_PURPLE, "S")

        # Quick status cards
        y3 = content_y + (card_h+pad)*2 + 10
        small_w = (w - pad*4) // 3
        small_h = 90

        draw_card(canvas, pad, y3, small_w, small_h,
                  "Battery", ["85%  |  Est. 3.2 hrs"], C.ACCENT_GREEN, "B")
        draw_card(canvas, pad*2+small_w, y3, small_w, small_h,
                  "Speed", ["2.5 km/h  |  Auto"], C.ACCENT_BLUE, ">")
        draw_card(canvas, pad*3+small_w*2, y3, small_w, small_h,
                  "Connection", ["WiFi OK  |  BT OK"], C.ACCENT_TEAL, "W")

    def get_popup_buttons(self):
        return [
            ("Navigate",     "nav"),
            ("Speed Up",     "speed_up"),
            ("Communicate",  "comm"),
            ("Emergency",    "emergency"),
            ("Call Help",    "call"),
            ("Health",       "health"),
            ("Stop",         "stop"),
            ("Settings",     "settings"),
        ]


# ═══════════════════════════════════════════════════════════════════════
#  Screen: Navigation
# ═══════════════════════════════════════════════════════════════════════
class NavigationScreen:
    NAME = "nav"
    TITLE = "Navigation"

    def __init__(self):
        self._anim_t = 0

    def draw(self, canvas):
        h, w = canvas.shape[:2]
        canvas[:] = C.BG_DARK
        draw_header(canvas, "Navigation", "Route & Movement Control", C.ACCENT_BLUE)
        draw_status_bar(canvas, speed="2.5 km/h", mode="MANUAL")

        self._anim_t += 1

        # Simulated map area
        map_x, map_y = 20, 80
        map_w, map_h = w - 40, (h - 140) // 2
        draw_rounded_rect(canvas, (map_x, map_y), (map_x+map_w, map_y+map_h),
                          (35, 40, 35), 12)
        # Grid lines
        for gx in range(map_x+30, map_x+map_w-10, 40):
            cv2.line(canvas, (gx, map_y+10), (gx, map_y+map_h-10), (50,55,50), 1)
        for gy in range(map_y+30, map_y+map_h-10, 40):
            cv2.line(canvas, (map_x+10, gy), (map_x+map_w-10, gy), (50,55,50), 1)

        # Simulated route path (animated)
        route_pts = [
            (map_x + 60, map_y + map_h - 40),
            (map_x + map_w//3, map_y + map_h//2),
            (map_x + map_w//2, map_y + map_h//3),
            (map_x + map_w*2//3, map_y + 50),
        ]
        for i in range(len(route_pts)-1):
            cv2.line(canvas, route_pts[i], route_pts[i+1],
                     C.ACCENT_BLUE, 3, cv2.LINE_AA)
        # Animated dot along path
        seg = (self._anim_t // 20) % (len(route_pts)-1)
        t = (self._anim_t % 20) / 20.0
        px = int(route_pts[seg][0] * (1-t) + route_pts[seg+1][0] * t)
        py = int(route_pts[seg][1] * (1-t) + route_pts[seg+1][1] * t)
        cv2.circle(canvas, (px, py), 8, C.ACCENT_TEAL, -1, cv2.LINE_AA)
        cv2.circle(canvas, (px, py), 12, C.ACCENT_TEAL, 2, cv2.LINE_AA)

        # Destination marker
        cv2.drawMarker(canvas, route_pts[-1], C.ACCENT_GREEN,
                       cv2.MARKER_DIAMOND, 16, 2)
        cv2.putText(canvas, "DEST", (route_pts[-1][0]+12, route_pts[-1][1]+5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, C.ACCENT_GREEN, 1, cv2.LINE_AA)

        # Current position marker
        cv2.drawMarker(canvas, route_pts[0], C.ACCENT_ORANGE,
                       cv2.MARKER_STAR, 14, 2)
        cv2.putText(canvas, "YOU", (route_pts[0][0]+12, route_pts[0][1]+5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, C.ACCENT_ORANGE, 1, cv2.LINE_AA)

        # Info cards below map
        cy = map_y + map_h + 16
        card_w3 = (w - 80) // 3
        draw_card(canvas, 20, cy, card_w3, 85,
                  "Distance", ["350m remaining", "ETA: 4 min"], C.ACCENT_BLUE)
        draw_card(canvas, 40+card_w3, cy, card_w3, 85,
                  "Obstacles", ["None detected", "Path clear"], C.ACCENT_GREEN)
        draw_card(canvas, 60+card_w3*2, cy, card_w3, 85,
                  "Terrain", ["Indoor  |  Flat", "Grip: Good"], C.ACCENT_TEAL)

        # Direction indicator
        cy2 = cy + 100
        draw_card(canvas, 20, cy2, w-40, 60,
                  "Next Turn", ["Turn RIGHT in 15 meters  >>"], C.ACCENT_ORANGE, ">")

    def get_popup_buttons(self):
        return [
            ("Go Home",    "home"),
            ("Speed Up",   "speed_up"),
            ("Turn Right", "turn_right"),
            ("Turn Left",  "turn_left"),
            ("Reverse",    "reverse"),
            ("Stop",       "stop"),
            ("Reroute",    "reroute"),
            ("Emergency",  "emergency"),
        ]


# ═══════════════════════════════════════════════════════════════════════
#  Screen: Communication
# ═══════════════════════════════════════════════════════════════════════
class CommunicationScreen:
    NAME = "comm"
    TITLE = "Communication"

    def draw(self, canvas):
        h, w = canvas.shape[:2]
        canvas[:] = C.BG_DARK
        draw_header(canvas, "Communication", "Quick Messages & Alerts", C.ACCENT_GREEN)
        draw_status_bar(canvas)

        pad = 20
        y0 = 85

        # Phrase cards — 2 columns, 4 rows
        phrases = [
            ("Yes / Agree",      "Y"),  ("No / Disagree",     "X"),
            ("I need water",     "W"),  ("I need help",       "!"),
            ("Take me outside",  ">"),  ("I feel pain",       "+"),
            ("Thank you",        "T"),  ("Call my family",    "F"),
        ]
        card_w = (w - pad*3) // 2
        card_h = 58
        colors = [C.ACCENT_GREEN, C.ACCENT_RED, C.ACCENT_BLUE, C.ACCENT_ORANGE,
                  C.ACCENT_TEAL, C.ACCENT_RED, C.ACCENT_GREEN, C.ACCENT_PURPLE]

        for i, (phrase, icon) in enumerate(phrases):
            col = i % 2
            row = i // 2
            x = pad + col * (card_w + pad)
            y = y0 + row * (card_h + 10)
            color = colors[i % len(colors)]
            draw_rounded_rect(canvas, (x, y), (x+card_w, y+card_h), C.BG_CARD, 10)
            cv2.rectangle(canvas, (x, y), (x+5, y+card_h), color, -1)
            draw_icon_circle(canvas, x+30, y+card_h//2, 14, color, icon)
            cv2.putText(canvas, phrase, (x+54, y+card_h//2+6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.48, C.WHITE, 1, cv2.LINE_AA)

        # Recent message log
        log_y = y0 + 4*(card_h+10) + 20
        draw_card(canvas, pad, log_y, w - pad*2, 100,
                  "Recent Messages", [
                      '> "I need water"    -  2 min ago',
                      '> "Thank you"       -  8 min ago',
                      '> "Yes / Agree"     - 15 min ago',
                  ], C.ACCENT_GREEN)

    def get_popup_buttons(self):
        return [
            ("Go Home",      "home"),
            ("Say Yes",      "say_yes"),
            ("Say No",       "say_no"),
            ("Need Water",   "water"),
            ("Need Help",    "help"),
            ("Feel Pain",    "pain"),
            ("Call Family",  "call_family"),
            ("Emergency",    "emergency"),
        ]


# ═══════════════════════════════════════════════════════════════════════
#  Screen: Health Monitor
# ═══════════════════════════════════════════════════════════════════════
class HealthScreen:
    NAME = "health"
    TITLE = "Health Monitor"

    def __init__(self):
        self._t = 0

    def draw(self, canvas):
        h, w = canvas.shape[:2]
        canvas[:] = C.BG_DARK
        draw_header(canvas, "Health Monitor", "Vital Signs & Alerts", C.ACCENT_RED)
        draw_status_bar(canvas)

        self._t += 1
        pad = 20
        y0 = 85

        # Heart rate card with animated ECG wave
        ecg_card_w = w - pad*2
        ecg_card_h = 130
        draw_rounded_rect(canvas, (pad, y0), (pad+ecg_card_w, y0+ecg_card_h), C.BG_CARD, 14)
        cv2.rectangle(canvas, (pad, y0), (pad+ecg_card_w, y0+4), C.ACCENT_RED, -1)
        cv2.putText(canvas, "Heart Rate", (pad+16, y0+28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, C.WHITE, 1, cv2.LINE_AA)

        # Simulated BPM value (animated slight variation)
        bpm = 72 + int(3 * math.sin(self._t * 0.1))
        bpm_str = f"{bpm} BPM"
        cv2.putText(canvas, bpm_str, (pad+ecg_card_w-130, y0+30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, C.ACCENT_RED, 2, cv2.LINE_AA)

        # ECG waveform
        ecg_y_center = y0 + 80
        ecg_x_start = pad + 20
        ecg_w = ecg_card_w - 40
        pts = []
        for i in range(ecg_w):
            phase = (i + self._t * 3) % 80
            if 20 < phase < 25:
                val = -30
            elif 25 <= phase < 30:
                val = 40
            elif 30 <= phase < 35:
                val = -15
            else:
                val = int(3 * math.sin(i * 0.08))
            pts.append((ecg_x_start + i, ecg_y_center + val))
        if len(pts) > 1:
            cv2.polylines(canvas, [np.array(pts, np.int32)], False,
                          C.ACCENT_RED, 2, cv2.LINE_AA)

        # Vitals grid
        vy = y0 + ecg_card_h + 16
        v_card_w = (w - pad*3) // 2
        v_card_h = 85

        # SpO2
        draw_card(canvas, pad, vy, v_card_w, v_card_h,
                  "SpO2", ["98%", "Normal range"], C.ACCENT_BLUE, "O")
        # Temperature
        draw_card(canvas, pad*2+v_card_w, vy, v_card_w, v_card_h,
                  "Temperature", ["36.8 C", "Normal range"], C.ACCENT_ORANGE, "T")

        # Row 2
        vy2 = vy + v_card_h + 12
        # Blood Pressure
        draw_card(canvas, pad, vy2, v_card_w, v_card_h,
                  "Blood Pressure", ["120/80 mmHg", "Optimal"], C.ACCENT_GREEN, "P")
        # Stress Level
        draw_card(canvas, pad*2+v_card_w, vy2, v_card_w, v_card_h,
                  "Stress Level", ["Low", "Relaxed state"], C.ACCENT_TEAL, "~")

        # Activity summary bar
        ay = vy2 + v_card_h + 16
        draw_card(canvas, pad, ay, w-pad*2, 55,
                  "Activity Today", ["Moved: 1.2 km  |  Sitting: 4.5 hrs"], C.ACCENT_PURPLE)
        draw_progress_bar(canvas, pad+16, ay+42, w-pad*2-32, 8, 0.45, C.ACCENT_PURPLE)

    def get_popup_buttons(self):
        return [
            ("Go Home",      "home"),
            ("Call Doctor",  "call_doctor"),
            ("Alert Nurse",  "alert_nurse"),
            ("Log Symptom",  "log_symptom"),
            ("Take Meds",   "take_meds"),
            ("Pain Level",   "pain_level"),
            ("View History", "view_history"),
            ("Emergency",    "emergency"),
        ]


# ═══════════════════════════════════════════════════════════════════════
#  Screen: Settings
# ═══════════════════════════════════════════════════════════════════════
class SettingsScreen:
    NAME = "settings"
    TITLE = "Settings"

    def draw(self, canvas):
        h, w = canvas.shape[:2]
        canvas[:] = C.BG_DARK
        draw_header(canvas, "Settings", "System Preferences", C.ACCENT_PURPLE)
        draw_status_bar(canvas)

        pad = 20
        y0 = 85
        full_w = w - pad*2
        row_h = 55
        gap = 8

        settings_items = [
            ("Speed Limit",        "Max 5 km/h",        0.5,  C.ACCENT_BLUE),
            ("Volume",             "80%",                0.8,  C.ACCENT_GREEN),
            ("Screen Brightness",  "70%",                0.7,  C.ACCENT_ORANGE),
            ("Gaze Sensitivity",   "Medium",             0.5,  C.ACCENT_TEAL),
            ("Dwell Click Time",   "1.5 seconds",        0.6,  C.ACCENT_PURPLE),
            ("Obstacle Avoidance", "Enabled",            1.0,  C.ACCENT_GREEN),
            ("Auto-Brake",         "Enabled",            1.0,  C.ACCENT_RED),
            ("WiFi",               "Connected",          0.9,  C.ACCENT_BLUE),
        ]

        for i, (name, value, bar_val, color) in enumerate(settings_items):
            y = y0 + i * (row_h + gap)
            draw_rounded_rect(canvas, (pad, y), (pad+full_w, y+row_h), C.BG_CARD, 10)
            # Left accent
            cv2.rectangle(canvas, (pad, y), (pad+4, y+row_h), color, -1)
            # Name
            cv2.putText(canvas, name, (pad+16, y+24),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.48, C.WHITE, 1, cv2.LINE_AA)
            # Value
            cv2.putText(canvas, value, (pad+16, y+44),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.38, C.GRAY, 1, cv2.LINE_AA)
            # Progress bar on the right side
            bar_x = pad + full_w - 160
            bar_w = 140
            draw_progress_bar(canvas, bar_x, y+20, bar_w, 10, bar_val, color)

    def get_popup_buttons(self):
        return [
            ("Go Home",       "home"),
            ("Speed +",       "speed_up"),
            ("Speed -",       "speed_down"),
            ("Volume +",      "vol_up"),
            ("Volume -",      "vol_down"),
            ("Brightness +",  "bright_up"),
            ("Brightness -",  "bright_down"),
            ("Reset All",     "reset_settings"),
        ]


# ── Screen registry ─────────────────────────────────────────────────
ALL_SCREENS = {
    "home":     HomeScreen,
    "nav":      NavigationScreen,
    "comm":     CommunicationScreen,
    "health":   HealthScreen,
    "settings": SettingsScreen,
}
