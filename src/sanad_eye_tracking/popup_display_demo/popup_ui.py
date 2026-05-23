#!/usr/bin/env python3
"""
popup_ui.py – Edge-triggered pop-up button system with gaze dwell-click.

8 buttons arranged around the window edges:
  top-left / top-center / top-right
  mid-left /             / mid-right
  bottom-left / bottom-center / bottom-right

Buttons are hidden unless the gaze approaches the corresponding edge region.
A 1.5-second dwell on a button triggers a click, with a visible loading
arc that fills during the dwell.
"""

import cv2
import numpy as np
import time
import math


# ═══════════════════════════════════════════════════════════════════════
#  Constants
# ═══════════════════════════════════════════════════════════════════════
DWELL_DURATION = 3.0        # seconds to activate
EDGE_TRIGGER   = 0.18       # normalised distance from edge to show buttons
BUTTON_RATIO   = (0.15, 0.10)  # (width, height) as fraction of window
FADE_SPEED     = 0.12       # opacity approach speed per frame
COOLDOWN       = 0.6        # seconds after a click before another can fire

# Positions for 8 buttons (row, col) – used for layout computation
#   row: 0=top  1=mid  2=bottom
#   col: 0=left 1=center 2=right
LAYOUT = [
    (0, 0),  # top-left
    (0, 1),  # top-center
    (0, 2),  # top-right
    (1, 0),  # mid-left
    (1, 2),  # mid-right
    (2, 0),  # bottom-left
    (2, 1),  # bottom-center
    (2, 2),  # bottom-right
]

# Colours (BGR) - Ubuntu Yaru Dark aesthetic
COL_BG           = (30,  30,  30)
COL_BTN_BG       = (45,  45,  45)
COL_BTN_HOVER    = (61,  61,  61)
COL_BTN_TEXT     = (240, 240, 240)
COL_ARC_BG       = (80,  80,  80)
COL_ARC_FILL     = (32,  84, 233)   # Ubuntu Orange
COL_ACTIVATED    = (32,  84, 233)   # Ubuntu Orange
COL_EMERGENCY    = (60,  60, 220)   # Red


class PopupButton:
    """A single popup button with dwell-click state."""

    def __init__(self, label, action_id, row, col):
        self.label = label
        self.action_id = action_id
        self.row = row
        self.col = col
        # Computed per-frame
        self.rect = (0, 0, 0, 0)  # (x1, y1, x2, y2)
        self.visible = False
        self.opacity = 0.0
        self.dwell_start = None
        self.dwell_progress = 0.0
        self.activated = False
        self._activation_time = 0

    def compute_rect(self, win_w, win_h):
        """Compute the button rectangle based on window size."""
        bw = int(win_w * BUTTON_RATIO[0])
        bh = int(win_h * BUTTON_RATIO[1])

        # Horizontal
        if self.col == 0:
            x = 0
        elif self.col == 1:
            x = (win_w - bw) // 2
        else:
            x = win_w - bw

        # Vertical
        if self.row == 0:
            y = 0
        elif self.row == 1:
            y = (win_h - bh) // 2
        else:
            y = win_h - bh

        self.rect = (x, y, x + bw, y + bh)

    def edge_region_active(self, gx, gy):
        """Return True if gaze is near ANY edge of the screen."""
        t = EDGE_TRIGGER
        top    = gy < t
        bottom = gy > (1.0 - t)
        left   = gx < t
        right  = gx > (1.0 - t)

        return top or bottom or left or right

    def gaze_inside(self, gx_px, gy_px):
        """Return True if gaze pixel coords are inside this button."""
        x1, y1, x2, y2 = self.rect
        # Add a generous hover zone around the button for easier targeting
        pad = max(12, (x2 - x1) // 6)
        return (x1 - pad) <= gx_px <= (x2 + pad) and (y1 - pad) <= gy_px <= (y2 + pad)


class PopupSystem:
    """
    Manages 8 popup buttons, handles visibility, dwell-click, and rendering.
    """

    def __init__(self):
        self.buttons = []
        self._last_click_time = 0

    def set_buttons(self, button_defs):
        """
        button_defs: list of 8 (label, action_id) tuples.
        """
        self.buttons = []
        for i, (label, action_id) in enumerate(button_defs):
            row, col = LAYOUT[i]
            self.buttons.append(PopupButton(label, action_id, row, col))

    def update(self, gaze_x, gaze_y, win_w, win_h):
        """
        Update visibility and dwell state.  Returns action_id if a click fires.
        gaze_x, gaze_y are normalised [0,1].
        """
        gx_px = gaze_x * win_w
        gy_px = gaze_y * win_h
        now = time.time()
        fired_action = None

        for btn in self.buttons:
            btn.compute_rect(win_w, win_h)

            # Visibility
            should_show = btn.edge_region_active(gaze_x, gaze_y)
            if should_show:
                btn.opacity = min(1.0, btn.opacity + FADE_SPEED)
            else:
                btn.opacity = max(0.0, btn.opacity - FADE_SPEED)
            btn.visible = btn.opacity > 0.02

            # Activation flash decay
            if btn.activated and (now - btn._activation_time) > 0.4:
                btn.activated = False

            # Dwell
            if btn.visible and btn.gaze_inside(gx_px, gy_px):
                if btn.dwell_start is None:
                    btn.dwell_start = now
                elapsed = now - btn.dwell_start
                btn.dwell_progress = min(1.0, elapsed / DWELL_DURATION)

                if btn.dwell_progress >= 1.0 and (now - self._last_click_time) > COOLDOWN:
                    fired_action = btn.action_id
                    btn.activated = True
                    btn._activation_time = now
                    btn.dwell_start = None
                    btn.dwell_progress = 0.0
                    self._last_click_time = now
            else:
                btn.dwell_start = None
                btn.dwell_progress = 0.0

        return fired_action

    def draw(self, canvas):
        """Overlay popup buttons onto canvas."""
        for btn in self.buttons:
            if not btn.visible:
                continue
            self._draw_button(canvas, btn)

    def _draw_button(self, canvas, btn):
        """Render a single button with fade, hover highlight and dwell arc."""
        x1, y1, x2, y2 = btn.rect
        bw = x2 - x1
        bh = y2 - y1
        alpha = btn.opacity

        # Choose background colour
        if btn.activated:
            bg = COL_ACTIVATED
        elif btn.dwell_progress > 0:
            bg = COL_BTN_HOVER
        else:
            bg = COL_BTN_BG

        # Special colour for emergency
        if "emergency" in btn.action_id.lower():
            if btn.dwell_progress > 0:
                bg = (40, 40, 180)
            elif not btn.activated:
                bg = COL_EMERGENCY

        # Scale factors based on button size
        scale_f = bw / 170.0  # reference: ~170px at 1920w

        # Draw with alpha blending (create overlay)
        overlay = canvas.copy()
        # Flat rectangle since they are on the absolute borders
        cv2.rectangle(overlay, (x1, y1), (x2, y2), bg, -1)

        # Border glow when dwelling
        if btn.dwell_progress > 0:
            glow_color = COL_ARC_FILL
            if btn.activated:
                glow_color = COL_ACTIVATED
            cv2.rectangle(overlay, (x1, y1), (x2, y2), glow_color, 2, cv2.LINE_AA)

        cv2.addWeighted(overlay, alpha, canvas, 1.0 - alpha, 0, canvas)

        # Label text (scale with button size)
        font = cv2.FONT_HERSHEY_SIMPLEX
        scale = max(0.35, 0.55 * scale_f)
        thickness = max(1, int(1.5 * scale_f))
        (tw, th), _ = cv2.getTextSize(btn.label, font, scale, thickness)
        tx = x1 + (bw - tw) // 2
        ty = y1 + (bh + th) // 2

        # If dwelling, shift text right to make room for arc indicator
        arc_area = int(30 * scale_f)
        if btn.dwell_progress > 0:
            tx = x1 + arc_area + (bw - arc_area - tw) // 2

        text_color = tuple(int(c * alpha) for c in COL_BTN_TEXT)
        if alpha > 0.5:
            cv2.putText(canvas, btn.label, (tx, ty), font, scale,
                        text_color, thickness, cv2.LINE_AA)

        # Dwell progress arc (small circle on the left of the button)
        if btn.dwell_progress > 0 and not btn.activated:
            arc_cx = x1 + int(16 * scale_f)
            arc_cy = y1 + bh // 2
            arc_r = max(6, int(12 * scale_f))
            # Background circle
            cv2.circle(canvas, (arc_cx, arc_cy), arc_r, COL_ARC_BG, 2, cv2.LINE_AA)
            # Progress arc
            angle = int(360 * btn.dwell_progress)
            if angle > 0:
                cv2.ellipse(canvas, (arc_cx, arc_cy), (arc_r, arc_r),
                            -90, 0, angle, COL_ARC_FILL, max(2, int(3 * scale_f)), cv2.LINE_AA)

        # Activation check mark flash
        if btn.activated:
            cx = x1 + int(16 * scale_f)
            cy = y1 + bh // 2
            cv2.putText(canvas, "OK", (cx - int(8*scale_f), cy + int(5*scale_f)),
                        cv2.FONT_HERSHEY_SIMPLEX, max(0.3, 0.5 * scale_f),
                        C.WHITE if alpha > 0.5 else (0,0,0), 1, cv2.LINE_AA)


# Quick import for the colours used in draw
class C:
    WHITE = (255, 255, 255)
