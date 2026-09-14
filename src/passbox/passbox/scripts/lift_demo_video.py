#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Render video mô phỏng thang passbox 3 tầng: chuyển giao hàng AGV1 -> AGV2,
người dùng ở T1 là ngoại cảnh chen ngang làm sai chu trình.

Kịch bản:
  - Chu trình đúng: AGV1 (T2) đặt hàng vào kệ trong thang -> AGV2 (T3) nhận hàng.
  - AGV1 gọi thang lên T2 để đặt hàng vào kệ.
  - Trong lúc AGV1 đang đặt hàng, người dùng ở T1 nhấn gọi tầng (ngoại cảnh).
  - AGV1 hạ hàng xong -> AGV2 (T3) gọi tầng để nhận hàng.
  - Vì yêu cầu T1 vào hàng đợi trước, thang buộc phải xuống T1 (điểm dừng vô ích,
    hàng vẫn nằm trên kệ) rồi mới lên T3 -> chu trình bị trễ.

Xuất: demo/lift_demo.mp4 (thêm --gif để xuất kèm GIF)
"""

import argparse
import math
import os
import subprocess
import sys

from PIL import Image, ImageDraw, ImageFont

W, H = 1280, 720
FPS = 30

# ---------------------------------------------------------------- palette
BG          = (15, 23, 42)
PANEL       = (23, 33, 56)
PANEL_LINE  = (51, 65, 85)
SLAB        = (51, 65, 85)
SLAB_TOP    = (71, 85, 105)
CORRIDOR    = (28, 38, 58)
SHAFT       = (7, 12, 26)
SHAFT_WALL  = (60, 74, 96)
CAR_FILL    = (232, 238, 246)
CAR_LINE    = (120, 136, 158)
CAR_IN      = (208, 218, 232)
SHELF       = (146, 104, 58)
SHELF_DARK  = (110, 76, 40)
DOOR        = (108, 126, 152)
DOOR_LINE   = (150, 166, 190)
AGV1_C      = (34, 197, 214)
AGV2_C      = (167, 139, 250)
USER_C      = (250, 191, 66)
CARGO       = (249, 115, 22)
LIT         = (250, 204, 21)
OFF         = (68, 82, 106)
TXT         = (228, 235, 245)
MUTED       = (148, 163, 184)
GOOD        = (52, 211, 153)
WARN        = (251, 146, 60)
BAD         = (248, 113, 113)

# ---------------------------------------------------------------- geometry
BLD_X0, BLD_X1 = 40, 830
BLD_Y0, BLD_Y1 = 86, 642
FLOOR_H = 180
GROUND_Y = 632
SLAB_T = 12

WALL_X0, WALL_X1 = 505, 527          # vách ngăn hành lang / giếng thang
SHAFT_X0, SHAFT_X1 = 527, 795
CAR_W, CAR_H = 252, 160
DOOR_H = 148

PANEL_X0, PANEL_X1 = 850, 1252
PANEL_Y0, PANEL_Y1 = 86, 642
CAP_Y0, CAP_Y1 = 654, 708

AGV_W, AGV_H = 104, 44
BOX_W, BOX_H = 56, 42

AGV_HOME = 205.0
AGV_DOCK = 428.0
AGV_AWAY = 120.0
USER_HOME = 150.0
USER_BTN = 432.0
USER_DOOR = 452.0


def floor_bottom(f):
    return GROUND_Y - (f - 1) * FLOOR_H


def car_bottom(pos):
    """pos = vị trí tầng hiện tại (float)."""
    return GROUND_Y - (pos - 1) * FLOOR_H - SLAB_T


def ease(p):
    p = max(0.0, min(1.0, p))
    return p * p * (3.0 - 2.0 * p)


def lerp(a, b, p):
    return a + (b - a) * p


# ---------------------------------------------------------------- fonts
def font(size, bold=False):
    path = "C:/Windows/Fonts/%s" % ("arialbd.ttf" if bold else "arial.ttf")
    try:
        return ImageFont.truetype(path, size)
    except OSError:
        return ImageFont.load_default()


F_TITLE = font(23, True)
F_SUB   = font(14)
F_SEC   = font(16, True)
F_BODY  = font(15)
F_BOLD  = font(15, True)
F_LOG   = font(13)
F_CAP   = font(20, True)
F_TAG   = font(13, True)
F_FLOOR = font(19, True)
F_SMALL = font(11, True)


# ---------------------------------------------------------------- timeline
Q_AGV1 = ("T2", "AGV1 — giao hàng", AGV1_C, "agv")
Q_AGV2 = ("T3", "AGV2 — nhận hàng", AGV2_C, "agv")
Q_USER = ("T1", "Người dùng", USER_C, "ext")

STEPS = [
    dict(k="init",        d=2.4, cap="Chu trình đúng: AGV1 (T2) đặt hàng vào kệ trong thang → AGV2 (T3) nhận hàng",
         logs=["Chu trình: chuyển giao hàng AGV1 → AGV2 qua thang", "Thang IDLE tại T1, cửa đóng"]),
    dict(k="agv1_call",   d=1.8, cap="AGV1 gọi thang lên T2 để đặt hàng vào kệ trong thang",
         logs=["AGV1 (T2): CALL → tầng 2", "Hàng đợi: [T2/AGV1]"]),
    dict(k="move_1_2",    d=2.6, cap="Thang di chuyển T1 → T2 (đáp ứng yêu cầu của AGV1)",
         logs=["Thang: T1 → T2 (hướng LÊN)"]),
    dict(k="open_2",      d=1.0, cap="Thang đến T2, mở cửa",
         logs=["Thang: đến T2, MỞ CỬA"]),
    dict(k="agv1_dock",   d=2.0, cap="AGV1 tiến vào vị trí dock trước cửa thang",
         logs=["AGV1: DOCKING tại T2"]),
    dict(k="user_press",  d=1.8, cap="NGOẠI CẢNH: người dùng ở T1 nhấn gọi tầng, chen vào giữa chu trình AGV",
         logs=["Người dùng (T1): CALL → tầng 1  [ngoại cảnh]", "Hàng đợi: [T1/Người dùng]"]),
    dict(k="agv1_load",   d=2.4, cap="AGV1 đặt kiện hàng vào kệ (slot A) — đích đến là AGV2 ở T3",
         logs=["AGV1: chuyển hàng lên kệ slot A", "Kiện hàng: đích T3 / AGV2"]),
    dict(k="agv1_undock", d=1.8, cap="AGV1 hạ hàng xong và rút ra khỏi cửa thang",
         logs=["AGV1: hạ hàng HOÀN TẤT", "AGV1: UNDOCK, giải phóng thang"]),
    dict(k="agv2_call",   d=1.9, cap="AGV2 ở T3 gọi tầng để nhận hàng — nhưng T1 đã nằm trước trong hàng đợi",
         logs=["AGV2 (T3): CALL → tầng 3", "Hàng đợi: [T1/Người dùng, T3/AGV2]"]),
    dict(k="close_2",     d=1.0, cap="Thang đóng cửa tại T2",
         logs=["Thang: ĐÓNG CỬA tại T2"]),
    dict(k="move_2_1",    d=2.6, cap="SAI CHU TRÌNH: thang phải xuống T1 (yêu cầu đến trước) thay vì lên T3",
         logs=["Thang: T2 → T1 (hướng XUỐNG)", "Chu trình AGV1→AGV2 bị chen ngang"]),
    dict(k="open_1",      d=1.0, cap="Mở cửa tại T1 theo lệnh gọi tầng của người dùng",
         logs=["Thang: đến T1, MỞ CỬA", "Xóa yêu cầu T1"]),
    dict(k="user_look",   d=2.4, cap="Kiện hàng trên kệ thuộc chu trình AGV → người dùng không lấy. Điểm dừng vô ích.",
         logs=["T1: không có tác vụ — hàng vẫn ở slot A"]),
    dict(k="close_1",     d=1.1, cap="Đóng cửa tại T1, kiện hàng vẫn nằm nguyên trên kệ",
         logs=["Thang: ĐÓNG CỬA tại T1"]),
    dict(k="move_1_3",    d=3.6, cap="Thang đi ngược lại lên T3 — quãng đường phát sinh do bị chen ngang",
         logs=["Thang: T1 → T3 (hướng LÊN)"]),
    dict(k="open_3",      d=1.0, cap="Thang đến T3, mở cửa cho AGV2",
         logs=["Thang: đến T3, MỞ CỬA", "Xóa yêu cầu T3"]),
    dict(k="agv2_pick",   d=3.0, cap="AGV2 dock và nhận kiện hàng từ kệ (slot A)",
         logs=["AGV2: DOCKING tại T3", "AGV2: nhận hàng từ kệ slot A"]),
    dict(k="agv2_leave",  d=1.6, cap="AGV2 rời đi cùng kiện hàng — chuyển giao AGV1 → AGV2 hoàn tất",
         logs=["AGV2: UNDOCK, mang hàng đi", "Chuyển giao AGV1 → AGV2: HOÀN TẤT"]),
    dict(k="done",        d=2.6, cap="Chu trình xong nhưng bị trễ vì lệnh gọi tầng của người dùng ở T1",
         logs=["Hàng đợi rỗng — thang IDLE tại T3"]),
]

_t = 0.0
for _i, _s in enumerate(STEPS):
    _s["idx"] = _i
    _s["t0"] = _t
    _t += _s["d"]
TOTAL = _t

BY_KEY = {s["k"]: s for s in STEPS}
# cửa sổ đi vòng: từ lúc đóng cửa ở T2 đến khi mở cửa xong ở T3
DETOUR_T0 = BY_KEY["close_2"]["t0"]
DETOUR_T1 = BY_KEY["open_3"]["t0"] + BY_KEY["open_3"]["d"]
# nếu đi thẳng T2 -> T3: đóng cửa + 1 tầng + mở cửa
DIRECT_D = BY_KEY["close_2"]["d"] + BY_KEY["move_2_1"]["d"] + BY_KEY["open_3"]["d"]
DELAY_MAX = (DETOUR_T1 - DETOUR_T0) - DIRECT_D


def delay_at(t):
    """Thời gian trễ phát sinh do phải đi vòng xuống T1."""
    if t <= DETOUR_T0:
        return 0.0
    return max(0.0, min(t, DETOUR_T1) - DETOUR_T0 - DIRECT_D)


def base_state():
    return dict(
        car=1.0, door=0.0, agv1=AGV_HOME, agv2=AGV_HOME, user=USER_HOME,
        buttons={}, queue=[], cargo=("agv1", None, 0.0),
        moving=0, dirn="", cap="", step=0, logs=[], arm=0.0, waste=0,
    )


def state_at(t):
    """Trả về trạng thái tại thời điểm t (giây)."""
    st = base_state()
    logs = []
    for s in STEPS:
        s0, s1 = s["t0"], s["t0"] + s["d"]
        active = t < s1
        p = 1.0 if not active else max(0.0, min(1.0, (t - s0) / s["d"]))
        e = ease(p)
        k = s["k"]
        st["buttons"], st["queue"] = {}, []
        st["moving"], st["dirn"], st["arm"], st["waste"] = 0, "", 0.0, 0

        if t >= s0:
            for ln in s["logs"]:
                logs.append((s0, ln))

        if k == "agv1_call":
            st["buttons"][2] = "AGV1"
            st["queue"] = [Q_AGV1]
        elif k == "move_1_2":
            st["buttons"][2] = "AGV1"
            st["queue"] = [Q_AGV1]
            st["car"] = lerp(1.0, 2.0, e)
            st["moving"] = 1
            st["dirn"] = "LÊN"
        elif k == "open_2":
            st["car"] = 2.0
            st["door"] = e
        elif k == "agv1_dock":
            st["car"], st["door"] = 2.0, 1.0
            st["agv1"] = lerp(AGV_HOME, AGV_DOCK, e)
        elif k == "user_press":
            st["car"], st["door"], st["agv1"] = 2.0, 1.0, AGV_DOCK
            if p < 0.32:
                st["user"] = lerp(USER_HOME, USER_BTN, ease(p / 0.32))
            else:
                st["user"] = USER_BTN
                st["arm"] = 1.0
            if p > 0.3:
                st["buttons"][1] = "USER"
                st["queue"] = [Q_USER]
        elif k == "agv1_load":
            st["car"], st["door"], st["agv1"] = 2.0, 1.0, AGV_DOCK
            st["user"] = USER_BTN
            st["buttons"][1] = "USER"
            st["queue"] = [Q_USER]
            st["cargo"] = ("agv1", "shelf_a", e)
        elif k == "agv1_undock":
            st["car"], st["door"] = 2.0, 1.0
            st["agv1"] = lerp(AGV_DOCK, AGV_HOME, e)
            st["user"] = lerp(USER_BTN, USER_HOME, e)
            st["buttons"][1] = "USER"
            st["queue"] = [Q_USER]
            st["cargo"] = ("shelf_a", None, 0.0)
        elif k in ("agv2_call", "close_2", "move_2_1"):
            st["car"], st["door"] = 2.0, 1.0
            st["cargo"] = ("shelf_a", None, 0.0)
            st["buttons"][1] = "USER"
            st["buttons"][3] = "AGV2"
            st["queue"] = [Q_USER, Q_AGV2]
            if k == "close_2":
                st["door"] = 1.0 - e
            elif k == "move_2_1":
                st["door"] = 0.0
                st["car"] = lerp(2.0, 1.0, e)
                st["moving"] = 1
                st["dirn"] = "XUỐNG"
        elif k == "open_1":
            st["car"], st["door"] = 1.0, e
            st["cargo"] = ("shelf_a", None, 0.0)
            st["buttons"][3] = "AGV2"
            st["queue"] = [Q_AGV2]
            st["waste"] = 1
        elif k == "user_look":
            st["car"], st["door"] = 1.0, 1.0
            st["cargo"] = ("shelf_a", None, 0.0)
            st["buttons"][3] = "AGV2"
            st["queue"] = [Q_AGV2]
            st["waste"] = 1
            if p < 0.35:
                st["user"] = lerp(USER_HOME, USER_DOOR, ease(p / 0.35))
            elif p < 0.72:
                st["user"] = USER_DOOR
                st["arm"] = 1.0
            else:
                st["user"] = lerp(USER_DOOR, USER_HOME, ease((p - 0.72) / 0.28))
        elif k == "close_1":
            st["car"], st["door"] = 1.0, 1.0 - e
            st["user"] = USER_HOME
            st["cargo"] = ("shelf_a", None, 0.0)
            st["buttons"][3] = "AGV2"
            st["queue"] = [Q_AGV2]
            st["waste"] = 1
        elif k == "move_1_3":
            st["car"] = lerp(1.0, 3.0, e)
            st["door"] = 0.0
            st["moving"] = 1
            st["dirn"] = "LÊN"
            st["cargo"] = ("shelf_a", None, 0.0)
            st["buttons"][3] = "AGV2"
            st["queue"] = [Q_AGV2]
        elif k == "open_3":
            st["car"], st["door"] = 3.0, e
            st["cargo"] = ("shelf_a", None, 0.0)
        elif k == "agv2_pick":
            st["car"], st["door"] = 3.0, 1.0
            if p < 0.45:
                st["agv2"] = lerp(AGV_HOME, AGV_DOCK, ease(p / 0.45))
                st["cargo"] = ("shelf_a", None, 0.0)
            else:
                st["agv2"] = AGV_DOCK
                st["cargo"] = ("shelf_a", "agv2", ease((p - 0.45) / 0.55))
        elif k == "agv2_leave":
            st["car"], st["door"] = 3.0, 1.0
            st["agv2"] = lerp(AGV_DOCK, AGV_HOME, e)
            st["cargo"] = ("agv2", None, 0.0)
        elif k == "done":
            st["car"], st["door"] = 3.0, 1.0
            st["agv2"] = lerp(AGV_HOME, AGV_AWAY, ease(min(1.0, p * 1.4)))
            st["cargo"] = ("agv2", None, 0.0)

        if active:
            st["cap"] = s["cap"]
            st["step"] = s["idx"]
            break

    st["logs"] = logs[-9:]
    return st


# ---------------------------------------------------------------- draw utils
def rrect(d, box, r, fill=None, outline=None, w=1):
    d.rounded_rectangle(box, radius=r, fill=fill, outline=outline, width=w)


def ctext(d, xy, s, f, fill):
    d.text(xy, s, font=f, fill=fill, anchor="mm")


def cargo_holder(st):
    src, dst, p = st["cargo"]
    return dst if (dst and p > 0.5) else src


def cargo_anchor(st, ref):
    """Toạ độ tâm kiện hàng theo điểm neo."""
    cb = car_bottom(st["car"])
    shelf_y = cb - 62
    if ref == "shelf_a":
        return (SHAFT_X0 + 84, shelf_y - BOX_H / 2 - 3)
    if ref == "agv1":
        return (st["agv1"], floor_bottom(2) - SLAB_T - AGV_H - BOX_H / 2 - 4)
    if ref == "agv2":
        return (st["agv2"], floor_bottom(3) - SLAB_T - AGV_H - BOX_H / 2 - 4)
    return (0, 0)


def cargo_pos(st):
    src, dst, p = st["cargo"]
    a = cargo_anchor(st, src)
    if dst is None or p <= 0.0:
        return a
    b = cargo_anchor(st, dst)
    x = lerp(a[0], b[0], p)
    y = lerp(a[1], b[1], p) - 34 * math.sin(math.pi * p)
    return (x, y)


def draw_cargo(d, st):
    cx, cy = cargo_pos(st)
    x0, y0 = cx - BOX_W / 2, cy - BOX_H / 2
    x1, y1 = cx + BOX_W / 2, cy + BOX_H / 2
    rrect(d, (x0, y0, x1, y1), 5, fill=CARGO, outline=(255, 255, 255), w=2)
    d.line((x0 + 4, cy, x1 - 4, cy), fill=(255, 255, 255), width=2)
    tag = {"shelf_a": "KIỆN HÀNG → T3 / AGV2"}.get(cargo_holder(st), "KIỆN HÀNG")
    ctext(d, (cx, y0 - 10), tag, F_SMALL, CARGO)


def draw_agv(d, cx, floor, col, name, badge):
    by = floor_bottom(floor) - SLAB_T
    x0, x1 = cx - AGV_W / 2, cx + AGV_W / 2
    y0, y1 = by - AGV_H, by - 7
    rrect(d, (x0, y0, x1, y1), 8, fill=col, outline=(240, 250, 255), w=2)
    d.rectangle((x0 + 8, y0 - 5, x1 - 8, y0 + 1), fill=(226, 240, 248))
    for wx in (x0 + 20, cx, x1 - 20):
        d.ellipse((wx - 9, by - 15, wx + 9, by + 3), fill=(24, 32, 48), outline=(120, 138, 162))
    ctext(d, (cx, (y0 + y1) / 2 + 3), name, F_TAG, (10, 20, 35))
    if badge:
        d.ellipse((x1 - 10, y0 - 28, x1 + 14, y0 - 4), fill=LIT, outline=(255, 255, 255))
        ctext(d, (x1 + 2, y0 - 16), badge, F_SMALL, (40, 30, 0))


def draw_user(d, st):
    cx = st["user"]
    by = floor_bottom(1) - SLAB_T - 6
    hy = by - 74
    d.ellipse((cx - 11, hy - 11, cx + 11, hy + 11), fill=USER_C, outline=(255, 255, 255), width=2)
    d.line((cx, hy + 11, cx, by - 26), fill=USER_C, width=6)
    d.line((cx, by - 26, cx - 13, by), fill=USER_C, width=5)
    d.line((cx, by - 26, cx + 13, by), fill=USER_C, width=5)
    ay = by - 46
    if st["arm"] > 0.5:
        d.line((cx, ay, cx + 32, ay - 14), fill=USER_C, width=5)
    else:
        d.line((cx, ay, cx + 18, ay + 14), fill=USER_C, width=5)
    d.line((cx, ay, cx - 18, ay + 14), fill=USER_C, width=5)
    ctext(d, (cx, hy - 42), "NGƯỜI DÙNG", F_SMALL, USER_C)
    ctext(d, (cx, hy - 27), "(NGOẠI CẢNH)", F_SMALL, BAD)


def draw_building(d, st):
    rrect(d, (BLD_X0, BLD_Y0, BLD_X1, BLD_Y1), 10, fill=CORRIDOR, outline=PANEL_LINE, w=2)
    d.rectangle((SHAFT_X0, BLD_Y0 + 2, SHAFT_X1, GROUND_Y), fill=SHAFT)
    for f in (1, 2, 3):
        fb = floor_bottom(f)
        ft = fb - FLOOR_H
        d.rectangle((BLD_X0 + 2, fb - SLAB_T, BLD_X1 - 2, fb), fill=SLAB)
        d.line((BLD_X0 + 2, fb - SLAB_T, BLD_X1 - 2, fb - SLAB_T), fill=SLAB_TOP, width=2)
        d.text((BLD_X0 + 14, ft + 12), "T%d" % f, font=F_FLOOR, fill=MUTED)
        d.rectangle((WALL_X0, ft, WALL_X1, fb - SLAB_T), fill=SHAFT_WALL)
        d.rectangle((SHAFT_X1, ft, SHAFT_X1 + 18, fb - SLAB_T), fill=SHAFT_WALL)
        dy1 = fb - SLAB_T - 6
        dy0 = dy1 - DOOR_H
        d.rectangle((WALL_X0, dy0, WALL_X1, dy1), fill=SHAFT)
        who = st["buttons"].get(f)
        bx, by = WALL_X0 - 26, dy0 + 34
        rrect(d, (bx - 13, by - 17, bx + 13, by + 31), 4, fill=(38, 50, 72), outline=PANEL_LINE, w=1)
        col = LIT if who else OFF
        d.ellipse((bx - 8, by - 12, bx + 8, by + 4), fill=col, outline=(210, 220, 235))
        ctext(d, (bx, by + 19), "T%d" % f, F_SMALL, LIT if who else MUTED)
        if who:
            d.ellipse((bx - 13, by - 17, bx + 13, by + 9), outline=LIT, width=2)


def draw_car(d, st):
    cb = car_bottom(st["car"])
    x0, x1 = SHAFT_X0 + 8, SHAFT_X0 + 8 + CAR_W
    y0, y1 = cb - CAR_H, cb
    d.line(((x0 + x1) / 2, BLD_Y0 + 4, (x0 + x1) / 2, y0), fill=(90, 104, 128), width=2)
    rrect(d, (x0, y0, x1, y1), 6, fill=CAR_FILL, outline=CAR_LINE, w=3)
    d.rectangle((x0 + 6, y0 + 6, x1 - 6, y1 - 6), fill=CAR_IN)
    shelf_y = cb - 62
    d.rectangle((x0 + 14, shelf_y, x1 - 14, shelf_y + 9), fill=SHELF, outline=SHELF_DARK)
    d.rectangle((x0 + 14, y0 + 26, x1 - 14, y0 + 34), fill=SHELF, outline=SHELF_DARK)
    for sx in (x0 + 14, (x0 + x1) / 2 - 3, x1 - 20):
        d.rectangle((sx, y0 + 26, sx + 6, shelf_y + 9), fill=SHELF_DARK)
    ctext(d, (x0 + 76, shelf_y + 22), "SLOT A", F_SMALL, (120, 100, 80))
    ctext(d, (x0 + 190, shelf_y + 22), "SLOT B", F_SMALL, (120, 100, 80))
    ctext(d, (x0 + CAR_W / 2, y0 + 15), "KỆ HÀNG TRONG THANG", F_SMALL, (90, 104, 128))


def draw_doors(d, st):
    """Cửa tầng: tầng cân bằng cabin thì mở/đóng theo st["door"], các tầng khác luôn đóng."""
    pos = st["car"]
    aligned = int(round(pos)) if abs(pos - round(pos)) <= 0.02 else None
    for f in (1, 2, 3):
        opened = st["door"] if f == aligned else 0.0
        fb = floor_bottom(f)
        dy1 = fb - SLAB_T - 6
        dy0 = dy1 - DOOR_H
        half = DOOR_H / 2.0
        slide = half * opened
        if half - slide <= 0.5:
            continue
        for y0, y1 in ((dy0 - slide, dy0 + half - slide), (dy1 - half + slide, dy1 + slide)):
            d.rectangle((WALL_X0, y0, WALL_X1, y1), fill=DOOR, outline=DOOR_LINE)
            d.line((WALL_X0 + 10, y0 + 4, WALL_X0 + 10, y1 - 4), fill=(78, 94, 118), width=2)


def draw_waste_tag(d, st):
    """Nhãn cảnh báo điểm dừng ngoài chu trình tại T1."""
    if not st["waste"]:
        return
    y = floor_bottom(1) - FLOOR_H + 28
    x0, x1 = 58, 326
    rrect(d, (x0, y - 14, x1, y + 14), 4, fill=(58, 28, 28), outline=BAD, w=2)
    ctext(d, ((x0 + x1) / 2, y + 1), "ĐIỂM DỪNG NGOÀI CHU TRÌNH", F_TAG, BAD)


def draw_panel(d, st, t):
    rrect(d, (PANEL_X0, PANEL_Y0, PANEL_X1, PANEL_Y1), 10, fill=PANEL, outline=PANEL_LINE, w=2)
    x = PANEL_X0 + 18
    y = PANEL_Y0 + 16

    d.text((x, y), "TRẠNG THÁI THANG", font=F_SEC, fill=TXT)
    y += 28
    pos = st["car"]
    if st["moving"]:
        cur, col = "%.1f  (%s)" % (pos, st["dirn"]), WARN
    else:
        cur, col = "T%d  (DỪNG)" % int(round(pos)), GOOD
    door_txt = "MỞ" if st["door"] > 0.95 else ("ĐÓNG" if st["door"] < 0.05 else "ĐANG CHẠY")
    door_col = GOOD if st["door"] > 0.95 else (MUTED if st["door"] < 0.05 else WARN)
    held = cargo_holder(st)
    shelf_txt = {"agv1": "TRỐNG (hàng ở AGV1)", "shelf_a": "KIỆN HÀNG → T3 / AGV2",
                 "agv2": "TRỐNG (AGV2 đã nhận)"}.get(held, "TRỐNG")
    dl = delay_at(t)
    rows = [
        ("Chu trình", "AGV1 → AGV2", AGV2_C),
        ("Vị trí", cur, col),
        ("Cửa", door_txt, door_col),
        ("Kệ slot A", shelf_txt, CARGO if held == "shelf_a" else MUTED),
        ("Trễ phát sinh", ("+%.1f s" % dl).replace(".", ","), BAD if dl > 0.05 else MUTED),
    ]
    for lab, val, c in rows:
        d.text((x, y), lab, font=F_BODY, fill=MUTED)
        d.text((x + 122, y), val, font=F_BOLD, fill=c)
        y += 24

    y += 12
    d.line((x, y, PANEL_X1 - 18, y), fill=PANEL_LINE, width=1)
    y += 14
    d.text((x, y), "HÀNG ĐỢI YÊU CẦU (FIFO)", font=F_SEC, fill=TXT)
    y += 28
    if not st["queue"]:
        d.text((x, y), "(rỗng)", font=F_BODY, fill=MUTED)
        y += 26
    else:
        for i, (fl, who, c, kind) in enumerate(st["queue"]):
            rrect(d, (x, y - 4, PANEL_X1 - 18, y + 38), 5,
                  fill=(32, 45, 70) if i else (44, 62, 96),
                  outline=BAD if kind == "ext" else c, w=1)
            d.text((x + 10, y), "%d. %s" % (i + 1, fl), font=F_BOLD, fill=c)
            d.text((x + 74, y), who, font=F_BODY, fill=TXT)
            if kind == "ext":
                d.text((x + 10, y + 19), "NGOẠI CẢNH — chen ngang chu trình", font=F_SMALL, fill=BAD)
            elif i == 0:
                d.text((x + 10, y + 19), "ĐANG XỬ LÝ", font=F_SMALL, fill=GOOD)
            else:
                d.text((x + 10, y + 19), "CHỜ", font=F_SMALL, fill=MUTED)
            y += 48
    y += 6
    d.line((x, y, PANEL_X1 - 18, y), fill=PANEL_LINE, width=1)
    y += 14
    d.text((x, y), "NHẬT KÝ SỰ KIỆN", font=F_SEC, fill=TXT)
    y += 26
    for ts, ln in st["logs"][-7:]:
        d.text((x, y), "[%05.1fs]" % ts, font=F_LOG, fill=(100, 116, 139))
        d.text((x + 62, y), ln, font=F_LOG, fill=MUTED if ts < t - 2.5 else TXT)
        y += 19

    by = PANEL_Y1 - 30
    d.text((x, by - 22), "Bước %d/%d" % (st["step"] + 1, len(STEPS)), font=F_LOG, fill=MUTED)
    d.rectangle((x, by, PANEL_X1 - 18, by + 8), fill=(38, 50, 72))
    w = (PANEL_X1 - 18 - x) * min(1.0, t / TOTAL)
    d.rectangle((x, by, x + w, by + 8), fill=(56, 189, 248))


def draw_legend(d):
    y = 62
    items = [("AGV 1 — giao hàng", AGV1_C), ("AGV 2 — nhận hàng", AGV2_C),
             ("Người dùng (ngoại cảnh)", USER_C), ("Kiện hàng", CARGO),
             ("Nút gọi tầng sáng", LIT)]
    x = BLD_X0 + 2
    for lab, c in items:
        d.ellipse((x, y + 3, x + 12, y + 15), fill=c)
        d.text((x + 18, y), lab, font=F_LOG, fill=MUTED)
        x += 26 + int(d.textlength(lab, font=F_LOG))


def render(t):
    img = Image.new("RGB", (W, H), BG)
    d = ImageDraw.Draw(img)
    st = state_at(t)

    d.rectangle((0, 0, W, 58), fill=(20, 30, 52))
    d.text((40, 7), "THANG PASSBOX 3 TẦNG — CHUYỂN GIAO HÀNG AGV1 → AGV2, NGƯỜI DÙNG CHEN NGANG",
           font=F_TITLE, fill=TXT)
    d.text((42, 35), "Chu trình đúng: AGV1 (T2) đặt hàng vào kệ → AGV2 (T3) nhận. Lệnh gọi tầng của "
                     "người dùng ở T1 chen vào giữa, buộc thang đi vòng xuống T1 rồi mới lên T3.",
           font=F_SUB, fill=MUTED)

    draw_legend(d)
    draw_building(d, st)
    draw_car(d, st)
    draw_doors(d, st)
    draw_waste_tag(d, st)

    draw_agv(d, st["agv1"], 2, AGV1_C, "AGV 1", "T2" if st["buttons"].get(2) else None)
    draw_agv(d, st["agv2"], 3, AGV2_C, "AGV 2", "T3" if st["buttons"].get(3) else None)
    draw_user(d, st)
    draw_cargo(d, st)

    draw_panel(d, st, t)

    rrect(d, (40, CAP_Y0, PANEL_X1, CAP_Y1), 8, fill=(20, 30, 52), outline=PANEL_LINE, w=2)
    ctext(d, ((40 + PANEL_X1) / 2, (CAP_Y0 + CAP_Y1) / 2 + 1), st["cap"], F_CAP, TXT)
    return img


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-o", "--out", default=None)
    ap.add_argument("--fps", type=int, default=FPS)
    ap.add_argument("--gif", action="store_true", help="xuất kèm file GIF")
    ap.add_argument("--png", type=float, default=None, help="chỉ xuất 1 khung tại giây t")
    a = ap.parse_args()

    here = os.path.dirname(os.path.abspath(__file__))
    outdir = os.path.join(os.path.dirname(here), "demo")
    os.makedirs(outdir, exist_ok=True)
    out = a.out or os.path.join(outdir, "lift_demo.mp4")

    if a.png is not None:
        p = os.path.join(outdir, "frame_%.1f.png" % a.png)
        render(a.png).save(p)
        print("PNG:", p)
        return

    import imageio_ffmpeg
    ff = imageio_ffmpeg.get_ffmpeg_exe()
    n = int(TOTAL * a.fps)
    cmd = [ff, "-y", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", "%dx%d" % (W, H),
           "-r", str(a.fps), "-i", "-", "-an", "-c:v", "libx264", "-preset", "medium",
           "-crf", "20", "-pix_fmt", "yuv420p", "-movflags", "+faststart", out]
    proc = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.DEVNULL,
                            stderr=subprocess.PIPE)
    gif_frames = []
    for i in range(n):
        t = i / float(a.fps)
        img = render(t)
        proc.stdin.write(img.tobytes())
        if a.gif and i % 3 == 0:
            gif_frames.append(img.resize((W // 2, H // 2), Image.LANCZOS)
                                 .convert("P", palette=Image.ADAPTIVE, colors=128))
        if i % 60 == 0:
            sys.stdout.write("\r  render %d/%d (%.1fs/%.1fs)" % (i, n, t, TOTAL))
            sys.stdout.flush()
    proc.stdin.close()
    err = proc.stderr.read().decode("utf-8", "ignore")
    if proc.wait() != 0:
        print("\nffmpeg lỗi:\n", err[-2000:])
        sys.exit(1)
    print("\nMP4: %s  (%.1fs, %d frames, %.2f MB)" % (out, TOTAL, n, os.path.getsize(out) / 1e6))

    if a.gif:
        g = os.path.splitext(out)[0] + ".gif"
        gif_frames[0].save(g, save_all=True, append_images=gif_frames[1:],
                           duration=int(1000 * 3 / a.fps), loop=0, optimize=True)
        print("GIF: %s (%.2f MB)" % (g, os.path.getsize(g) / 1e6))


if __name__ == "__main__":
    main()
