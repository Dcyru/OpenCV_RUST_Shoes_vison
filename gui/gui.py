#!/usr/bin/env python3
# =============================================================================
#  gui.py v14 — Delta Robot Vision Dashboard
#  Nhận dữ liệu + JPEG frame từ main.rs (hoặc sim_server) qua IPC TCP :5556
#  Protocol: "DATA:<json_len>:<jpeg_len>\n<json_text><jpeg_bytes>"
#
#  Chạy: python gui.py
#  Yêu cầu: main.rs đang chạy, hoặc sim_server, hoặc plc_simulator.py
#
#  ── THEO ĐÚNG MODBUS_TCPIP.txt + bảng PLC thật ──────────────────────────
#  D100-D107/D108-D115/D116-D123... mỗi giá trị CHIẾM VÙNG 8 byte (4 word)
#  nhưng PLC chỉ ghi giá trị thật vào 4 byte ĐẦU (2 word, D[n],D[n+1]).
#  4 byte SAU (D[n+2],D[n+3]) là vùng đệm/không dùng — bỏ qua khi tính.
#  Giá trị là số 32-bit CÓ DẤU (có thể âm khi servo quay ngược).
#  D100-D103=pulse1(*), D104-D107=pulse2, D108-D111=pulse3
#  D112-D115=speed1, D116-D119=speed2, D120-D123=speed3
#  (*) giá trị thật pulse1 chỉ nằm ở D100,D101 — D102,D103 là vùng đệm.
#  M109 (PLC M1024) — bit lên 1 = đếm 1 lần = 1 CHU TRÌNH 3 PHÔI (3 lót/chu trình)
#  M110 (PLC M1025) — đã về home
#  M111 (PLC M1026) — đã gắp vật xong (KHÔNG thành công)
#  M112             — VISION_READY (PC báo có dX/dY)
#  M113 (PLC M2002) — PLC ghi: LỆCH TRỤC X/Y NGOÀI VÙNG LÀM VIỆC => NGỪNG ROBOT
#  M114 (PLC M2003) — PC ghi: PHÁT HIỆN PHÔI=1, KHÔNG CÓ=0 (PLC lên lấy)
#  M2000            — COMM_OK (trạng thái kết nối truyền thông)
#
#  ── ĐỘNG HỌC ROBOT DELTA (Python port từ ikm.m / dkm.m Matlab) ──────────
#  Thông số cơ khí [R, r, L, l] = [130, 50, 180, 430] mm
#  dkm(theta) : 3 góc động cơ (độ) -> (x,y,z) mm   — dùng để vẽ quỹ đạo thực
#  ikm(xyz)   : (x,y,z) mm -> 3 góc động cơ (độ)   — dùng để kiểm tra ngược
# =============================================================================

import tkinter as tk
from tkinter import ttk, font as tkfont
import socket, threading, json, time, queue, struct, io, math
import subprocess, sys, os
from datetime import datetime
from collections import deque
from PIL import Image, ImageTk   # pip install pillow

# ── Cấu hình ──────────────────────────────────────────────────────
IPC_HOST    = "127.0.0.1"
IPC_PORT    = 5556
RECONNECT_S = 2.0
MAX_LOG     = 300
SPARK_LEN   = 120

# ── Tên file thực thi vision engine (main.rs đã build) để GUI tự
#   khởi động khi chưa thấy nó chạy — đóng gói cùng thư mục với GUI
#   thì chạy như "1 phần mềm" duy nhất, không cần mở tay 2 chương trình.
MAIN_EXE_NAMES = ["vision_main.exe"] if os.name == "nt" else ["vision_main"]

# ── Tên M coil (phải khớp với main.rs M_IO_NAMES + MODBUS_TCPIP.txt) ──────
# Index 0..14 = M100..M114, index 15 = M2000 (COMM_OK)
M_NAMES = [
    "AUTO", "MANUAL", "E_STOP", "S1_ANGLE_SENSOR",
    "S2_ANGLE_SENSOR", "S3_ANGLE_SENSOR", "LIFT_UP", "LIFT_DOWN",
    "WORK_DETECT", "CYCLE_3P", "HOME_DONE", "GRIP_DONE_NG",
    "VISION_READY", "AXIS_DEV_STOP", "WORKPIECE_PC",
]
M_ADDRS = list(range(100, 115)) + [2000]
M_LABELS = {100 + i: name for i, name in enumerate(M_NAMES[:15])}
M_LABELS[2000] = "COMM_OK"

# Index trong mảng pm phẳng tương ứng M109/M110/M111/M112/M113/M114 (offset từ M100)
IDX_CYCLE_3P     = 9    # M109 — 1 chu trình 3 phôi
IDX_HOME_DONE    = 10   # M110
IDX_GRIP_DONE_NG = 11   # M111
IDX_VISION_READY = 12   # M112
IDX_AXIS_DEV     = 13   # M113 — PLC ghi: lệch trục X/Y ngoài vùng làm việc => NGỪNG ROBOT
IDX_WORKPIECE_PC = 14   # M114 — PC ghi: phát hiện phôi=1, không có=0 (PLC lên lấy)

# ── D register robot status (D100..D123, PLC ghi qua FC16) ───────────────
# Mỗi giá trị là số 64-bit, chiếm 4 word liên tiếp (D[n]=LSB...D[n+3]=MSB)
# D100-D103=pulse1, D104-D107=pulse2, D108-D111=pulse3
# D112-D115=speed1, D116-D119=speed2, D120-D123=speed3
ROBOT_D64 = [
    ("pulse1", 100, "Servo 1 — số xung thực tế"),
    ("pulse2", 104, "Servo 2 — số xung thực tế"),
    ("pulse3", 108, "Servo 3 — số xung thực tế"),
    ("speed1", 112, "Servo 1 — tốc độ phát xung"),
    ("speed2", 116, "Servo 2 — tốc độ phát xung"),
    ("speed3", 120, "Servo 3 — tốc độ phát xung"),
]

# ── Tỉ lệ xung → góc động cơ (độ) ──────────────────────────────────────
# !!! CHỈNH LẠI CHO ĐÚNG THÔNG SỐ SERVO THỰC TẾ CỦA ANH (xung/vòng, hộp số...) !!!
# Giá trị 100 xung/độ là MẶC ĐỊNH TẠM — cần xác nhận lại với driver servo thật.
PULSE_PER_DEG = 100.0

# ── Thông số cơ khí Robot Delta [R, r, L, l] (mm) ─────────────────────────
# R = bán kính đế cố định, r = bán kính mâm gắp,
# L = chiều dài tay đòn trên, l = chiều dài tay đòn dưới
ROBOT_R = 135.0
ROBOT_r = 47.0
ROBOT_L = 183.37
ROBOT_l = 429.0
ALPHA_DEG = (-90.0, 30.0, 150.0)   # góc bố trí 3 cánh tay quanh tâm đế

TRAJ_LEN = 400   # số điểm quỹ đạo lưu lại để vẽ

STACK_NAMES = {0:"SINGLE",1:"2-KHÍT",2:"2-LỆCH",3:"2-XOAY",4:"2-PHỨC"}

# ── Màu sắc ───────────────────────────────────────────────────────
C = {
    "bg":     "#0d1117",
    "bg2":    "#161b22",
    "bg3":    "#21262d",
    "border": "#30363d",
    "cyan":   "#58a6ff",
    "green":  "#3fb950",
    "red":    "#f85149",
    "yellow": "#d29922",
    "orange": "#e3b341",
    "white":  "#c9d1d9",
    "gray":   "#484f58",
    "purple": "#bc8cff",
    "teal":   "#39d353",
    "lime":   "#7ee787",
    "blue":   "#1f6feb",
}


# =============================================================================
#  ĐỘNG HỌC ROBOT DELTA — port trực tiếp từ ikm.m / dkm.m (Matlab)
# =============================================================================
_ALPHA = [math.radians(a) for a in ALPHA_DEG]


def dkm(theta_deg, R=ROBOT_R, r=ROBOT_r, L=ROBOT_L, l=ROBOT_l):
    """Động học THUẬN: 3 góc động cơ (độ) -> (x,y,z) mm.
    Trả về None nếu điểm nằm ngoài không gian làm việc (vô nghiệm)."""
    p = R - r
    th = [math.radians(t) for t in theta_deg]

    def joint(i):
        a = _ALPHA[i]
        x = (p + L * math.cos(th[i])) * math.cos(a)
        y = (p + L * math.cos(th[i])) * math.sin(a)
        z = -L * math.sin(th[i])
        return x, y, z

    x1, y1, z1 = joint(0)
    x2, y2, z2 = joint(1)
    x3, y3, z3 = joint(2)

    w1 = x1**2 + y1**2 + z1**2
    w2 = x2**2 + y2**2 + z2**2
    w3 = x3**2 + y3**2 + z3**2

    d1, d2, d3 = x2 - x1, y2 - y1, z2 - z1
    e1, e2, e3 = x3 - x1, y3 - y1, z3 - z1

    A1, B1, C1, D1 = 2*d1, 2*d2, 2*d3, w2 - w1
    A2, B2, C2, D2 = 2*e1, 2*e2, 2*e3, w3 - w1

    det2D = A1*B2 - A2*B1
    if abs(det2D) < 1e-6:
        return None

    a1 = (C2*B1 - C1*B2) / det2D
    b1 = (D1*B2 - D2*B1) / det2D
    a2 = (C1*A2 - C2*A1) / det2D
    b2 = (D2*A1 - D1*A2) / det2D

    c1 = b1 - x1
    c2 = b2 - y1

    QA = a1**2 + a2**2 + 1
    QB = 2*a1*c1 + 2*a2*c2 - 2*z1
    QC = c1**2 + c2**2 + z1**2 - l**2

    delta = QB**2 - 4*QA*QC
    if delta < 0:
        return None

    z_sol = (-QB - math.sqrt(delta)) / (2*QA)
    x_sol = a1*z_sol + b1
    y_sol = a2*z_sol + b2
    return (x_sol, y_sol, z_sol)


def ikm(xyz, R=ROBOT_R, r=ROBOT_r, L=ROBOT_L, l=ROBOT_l):
    """Động học NGHỊCH: (x,y,z) mm -> 3 góc động cơ (độ).
    Trả về None cho trục nào vô nghiệm (ngoài không gian làm việc)."""
    x, y, z = xyz
    p = R - r
    theta = [None, None, None]
    for i in range(3):
        a = _ALPHA[i]
        A = 2*L*(p - x*math.cos(a) - y*math.sin(a))
        B = 2*z*L
        Cc = x**2 + y**2 + z**2 + p**2 + L**2 - l**2 - 2*p*(x*math.cos(a) + y*math.sin(a))
        delta = 4*B**2 - 4*(Cc - A)*(A + Cc)
        if delta < 0:
            theta[i] = None
        else:
            t = (-2*B - math.sqrt(delta)) / (2*(Cc - A))
            theta[i] = 2*math.atan(t) * 180 / math.pi
    return theta


# =============================================================================
#  IPC RECEIVER
# =============================================================================
class IpcReceiver(threading.Thread):
    """Nhận data+JPEG từ main.rs, đồng thời có thể GỬI LỆNH ngược lại
    (CMD:s / CMD:d / CMD:r / CMD:p) qua CHÍNH socket đang kết nối, để
    toàn bộ thao tác (chụp mẫu 45 khung, chọn ROI, load lại, bật/tắt
    TCP) làm được 100% từ GUI — không cần vào cửa sổ OpenCV của
    main.rs nữa."""
    def __init__(self, q: queue.Queue):
        super().__init__(daemon=True)
        self.q = q
        self.running = True
        self._sock = None
        self._sock_lock = threading.Lock()

    def send_cmd(self, ch: str):
        """Gửi 1 lệnh 1-ký-tự cho main.rs, ví dụ send_cmd('s') để bắt
        đầu chụp 45 khung ảnh mẫu. Trả về True nếu gửi thành công
        (tức đang có kết nối tới main.rs)."""
        with self._sock_lock:
            s = self._sock
        if s is None:
            return False
        try:
            s.sendall(f"CMD:{ch}\n".encode("utf-8"))
            return True
        except Exception:
            return False

    def run(self):
        while self.running:
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.settimeout(4.0)
                sock.connect((IPC_HOST, IPC_PORT))
                sock.settimeout(2.0)
                with self._sock_lock:
                    self._sock = sock
                self.q.put({"_status": "connected"})
                buf = b""
                while self.running:
                    try:
                        chunk = sock.recv(65536)
                        if not chunk: break
                        buf += chunk
                        while True:
                            nl = buf.find(b"\n")
                            if nl < 0: break
                            header = buf[:nl].decode("utf-8", errors="ignore")
                            if not header.startswith("DATA:"):
                                buf = buf[nl+1:]; continue
                            try:
                                _, jl, il = header.split(":")
                                jl, il = int(jl), int(il)
                            except:
                                buf = buf[nl+1:]; continue
                            total = nl + 1 + jl + il
                            if len(buf) < total: break
                            json_bytes = buf[nl+1: nl+1+jl]
                            jpeg_bytes = buf[nl+1+jl: total]
                            buf = buf[total:]
                            try:
                                data = json.loads(json_bytes)
                                if jpeg_bytes:
                                    data["_jpeg"] = jpeg_bytes
                                self.q.put(data)
                            except: pass
                    except socket.timeout: continue
                    except Exception: break
            except Exception: pass
            finally:
                with self._sock_lock:
                    self._sock = None
                self.q.put({"_status": "disconnected"})
                try: sock.close()
                except: pass
            if self.running: time.sleep(RECONNECT_S)

    def stop(self): self.running = False


# =============================================================================
#  SPARKLINE
# =============================================================================
class Sparkline(tk.Canvas):
    def __init__(self, master, w=320, h=56, color="#58a6ff", ymin=-35, ymax=35, **kw):
        super().__init__(master, width=w, height=h, bg=C["bg2"],
                         highlightthickness=1, highlightbackground=C["border"], **kw)
        self.w=w; self.h=h; self.color=color
        self.ymin=ymin; self.ymax=ymax
        self.data = deque([0.0]*SPARK_LEN, maxlen=SPARK_LEN)

    def push(self, v):
        self.data.append(v)
        self.delete("all")
        y0 = self.h * self.ymax / (self.ymax - self.ymin)
        self.create_line(0, y0, self.w, y0, fill=C["gray"], dash=(2,4))
        pts = list(self.data); n = len(pts)
        if n < 2: return
        coords = []
        for i,val in enumerate(pts):
            x = i * self.w / (n-1)
            val = max(self.ymin, min(self.ymax, val))
            y = self.h - (val - self.ymin) / (self.ymax - self.ymin) * self.h
            coords.extend([x, y])
        if len(coords) >= 4:
            self.create_line(*coords, fill=self.color, width=2, smooth=True)


_MONO_FONT_CACHE = None

def get_mono_font():
    """Trả về tên font monospace hỗ trợ tiếng Việt có sẵn trên máy đang chạy
    (cache lại sau lần gọi đầu). Dùng chung cho mọi nơi vẽ chữ trực tiếp lên
    Canvas (vd: create_text) để tránh 'mất chữ' khi Consolas không tồn tại
    trên Linux/macOS."""
    global _MONO_FONT_CACHE
    if _MONO_FONT_CACHE is not None:
        return _MONO_FONT_CACHE
    candidates = [
        "Consolas", "Cascadia Mono", "DejaVu Sans Mono",
        "Noto Sans Mono", "Menlo", "Courier New",
    ]
    try:
        available = set(tkfont.families())
    except Exception:
        available = set()
    for name in candidates:
        if name in available:
            _MONO_FONT_CACHE = name
            return name
    _MONO_FONT_CACHE = "TkFixedFont"
    return _MONO_FONT_CACHE


# =============================================================================
#  TRAJECTORY VIEW — quỹ đạo TCP robot tính từ động học thuận (dkm)
#  Chiếu (x,y) nhìn từ trên xuống, mm -> pixel theo workspace robot
# =============================================================================
class TrajectoryView(tk.Canvas):
    def __init__(self, master, w=420, h=320, **kw):
        super().__init__(master, width=w, height=h, bg=C["bg"],
                          highlightthickness=1, highlightbackground=C["border"], **kw)
        self.w = w; self.h = h
        self.points = deque(maxlen=TRAJ_LEN)
        # Phạm vi hiển thị (mm) quanh tâm robot — auto theo workspace ước lượng
        self.range_mm = max(ROBOT_R, ROBOT_l - ROBOT_L) * 1.0 + 80.0

    def _to_px(self, x, y):
        cx, cy = self.w/2, self.h/2
        scale = (min(self.w, self.h)/2 - 16) / self.range_mm
        return cx + x*scale, cy - y*scale   # y+ robot = lên trên màn hình

    def push(self, xyz):
        if xyz is None:
            return
        self.points.append(xyz)
        self.redraw()

    def redraw(self):
        self.delete("all")
        cx, cy = self.w/2, self.h/2
        # Lưới + vòng tròn workspace tham chiếu
        for frac in (0.33, 0.66, 1.0):
            rr = (min(self.w, self.h)/2 - 16) * frac
            self.create_oval(cx-rr, cy-rr, cx+rr, cy+rr,
                              outline=C["border"], dash=(2,3))
        self.create_line(cx-self.w/2+8, cy, cx+self.w/2-8, cy, fill=C["border"])
        self.create_line(cx, cy-self.h/2+8, cx, cy+self.h/2-8, fill=C["border"])
        self.create_text(cx, 10, text="QUỸ ĐẠO TCP (x,y) — nhìn từ trên",
                          fill=C["gray"], font=(get_mono_font(), 10))

        pts = list(self.points)
        if len(pts) >= 2:
            coords = []
            for (x, y, z) in pts:
                px, py = self._to_px(x, y)
                coords.extend([px, py])
            self.create_line(*coords, fill=C["cyan"], width=2, smooth=True)
        if pts:
            x, y, z = pts[-1]
            px, py = self._to_px(x, y)
            self.create_oval(px-6, py-6, px+6, py+6, fill=C["green"], outline="")
            self.create_text(px, py-16, text=f"({x:.0f},{y:.0f},{z:.0f})mm",
                              fill=C["green"], font=(get_mono_font(), 10, "bold"))

    def clear(self):
        self.points.clear()
        self.redraw()


# =============================================================================
#  TỰ KHỞI ĐỘNG VISION ENGINE (main.rs đã build) — để GUI chạy như
#  MỘT phần mềm duy nhất: double-click gui(.exe) là xong, không cần
#  mở tay cửa sổ dòng lệnh main.rs riêng.
#  Đặt file build của main.rs (đổi tên thành "vision_main.exe" trên
#  Windows hoặc "vision_main" trên Linux) CÙNG THƯ MỤC với gui.py /
#  bản GUI đã đóng gói (PyInstaller).
#  LƯU Ý: main.rs bind cổng Modbus TCP :502 (<1024) nên cần quyền
#  Admin (Windows) / root hoặc setcap (Linux) — xem ghi chú cuối file.
# =============================================================================
def _app_base_dir():
    """Thư mục chứa gui.py, hoặc thư mục chứa file .exe nếu đã đóng
    gói bằng PyInstaller (--onefile) — dùng để tìm vision_main cạnh nó."""
    if getattr(sys, "frozen", False):
        return os.path.dirname(sys.executable)
    return os.path.dirname(os.path.abspath(__file__))


def _find_main_exe():
    for d in (_app_base_dir(), os.getcwd()):
        for name in MAIN_EXE_NAMES:
            p = os.path.join(d, name)
            if os.path.isfile(p):
                return p
    return None


def _ipc_port_open(timeout=0.4):
    try:
        with socket.create_connection((IPC_HOST, IPC_PORT), timeout=timeout):
            return True
    except Exception:
        return False


def try_autostart_main_engine():
    """Nếu chưa có gì lắng nghe ở IPC_PORT, thử tự chạy vision_main
    cạnh gui.py. Không báo lỗi cứng nếu thất bại — vòng kết nối lại
    (RECONNECT_S) của IpcReceiver sẽ tự bắt khi engine lên sau đó."""
    if _ipc_port_open():
        return  # đã có engine chạy sẵn (có thể do người dùng tự mở)
    exe = _find_main_exe()
    if not exe:
        print(f"[AUTOSTART] Không tìm thấy {MAIN_EXE_NAMES[0]} cạnh gui.py — "
              f"bạn cần tự chạy vision engine, hoặc đặt file build đúng tên/thư mục.")
        return
    try:
        if os.name == "nt":
            subprocess.Popen([exe], cwd=os.path.dirname(exe),
                              creationflags=subprocess.CREATE_NEW_CONSOLE)
        else:
            subprocess.Popen([exe], cwd=os.path.dirname(exe))
        print(f"[AUTOSTART] Đã khởi động: {exe}")
    except Exception as e:
        print(f"[AUTOSTART] Không khởi động được {exe}: {e}\n"
              f"  Nếu lỗi liên quan quyền/cổng 502: chạy GUI dưới quyền "
              f"Admin (Windows) hoặc dùng setcap trên Linux (xem README).")


# =============================================================================
#  MAIN APP
# =============================================================================
class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Delta Robot Vision v14")
        self.configure(bg=C["bg"])
        self.geometry("1820x1000")
        self.minsize(1400, 800)
        # Cho phép phóng to toàn màn hình ngay khi mở (giao diện không bị "mất")
        self.state("zoomed") if self._is_windows() else None

        self.q        = queue.Queue()
        self.recv     = IpcReceiver(self.q)
        self.last_d   = {}
        self.cycle_last  = False     # cạnh lên của M109 (CYCLE_3P)
        self.cycle_count = 0         # số CHU TRÌNH (mỗi chu trình = 3 lót)
        self.estop_last  = False     # cạnh lên của M102 (E_STOP) — để reset bộ đếm gắp phôi
        self.auto_last   = False     # trạng thái AUTO (M100) frame trước — để phát hiện cạnh XUỐNG (bật rồi tắt)
        self.vals     = {}
        self.frame_ct = 0
        self.fps_t    = time.time()
        self.fps_v    = 0.0

        # ROI drag-select state
        self.roi_rect      = None
        self._roi_drag     = {}
        self._cam_offset   = (0, 0)
        self._cam_img_size = (1, 1)

        self._fonts()
        self._build()
        self.recv.start()
        self.after(33, self._poll)
        self.protocol("WM_DELETE_WINDOW", self._close)

        # ── Phím tắt điều khiển main.rs từ xa (giống phím trên cửa sổ
        # OpenCV cũ, nhưng bấm ngay trên GUI này — không cần focus vào
        # cửa sổ nào khác) ──────────────────────────────────────────
        for k in ("s", "S"): self.bind_all(f"<KeyPress-{k}>", lambda e: self._cmd_capture())
        for k in ("d", "D"): self.bind_all(f"<KeyPress-{k}>", lambda e: self._cmd_roi_confirm())
        for k in ("r", "R"): self.bind_all(f"<KeyPress-{k}>", lambda e: self._cmd_reload())
        for k in ("p", "P"): self.bind_all(f"<KeyPress-{k}>", lambda e: self._cmd_toggle_tcp())

    @staticmethod
    def _is_windows():
        import platform
        return platform.system() == "Windows"

    def _fonts(self):
        fam = get_mono_font()
        self.F = {
            "big":   tkfont.Font(family=fam, size=52, weight="bold"),
            "mid":   tkfont.Font(family=fam, size=17, weight="bold"),
            "sm":    tkfont.Font(family=fam, size=15),
            "xs":    tkfont.Font(family=fam, size=14),
            "xxs":   tkfont.Font(family=fam, size=12),
            "title": tkfont.Font(family=fam, size=14, weight="bold"),
            "led":   tkfont.Font(family=fam, size=11, weight="bold"),
            "led_sm":tkfont.Font(family=fam, size=8,  weight="bold"),
        }

    # ─────────────────────────────────────────────────────────────
    def _build(self):
        # ── Top bar ───────────────────────────────────────────────
        top = tk.Frame(self, bg=C["bg3"], pady=6)
        top.pack(fill="x")
        tk.Label(top, text="⬡ DELTA ROBOT VISION v14", bg=C["bg3"],
                 fg=C["cyan"], font=self.F["mid"]).pack(side="left", padx=14)
        self.lbl_conn = tk.Label(top, text="● MẤT KẾT NỐI", bg=C["bg3"],
                                  fg=C["red"], font=self.F["sm"])
        self.lbl_conn.pack(side="left", padx=18)
        self.lbl_src  = tk.Label(top, text="[main.rs / plc_simulator.py]", bg=C["bg3"],
                                  fg=C["gray"], font=self.F["xs"])
        self.lbl_src.pack(side="left", padx=4)
        self.lbl_fps  = tk.Label(top, text="0.0 fps", bg=C["bg3"],
                                  fg=C["gray"], font=self.F["xs"])
        self.lbl_fps.pack(side="right", padx=12)
        self.lbl_time = tk.Label(top, text="--:--:--", bg=C["bg3"],
                                  fg=C["gray"], font=self.F["xs"])
        self.lbl_time.pack(side="right", padx=12)

        # ── 3-column paned layout ────────────────────────────────
        panes = tk.PanedWindow(self, orient="horizontal", bg=C["bg"],
                                sashwidth=5, sashrelief="flat")
        panes.pack(fill="both", expand=True, padx=4, pady=4)

        left  = tk.Frame(panes, bg=C["bg"])
        mid   = tk.Frame(panes, bg=C["bg"])
        right = tk.Frame(panes, bg=C["bg"])
        panes.add(left,  minsize=680)
        panes.add(mid,   minsize=360)
        panes.add(right, minsize=480)

        self._build_mid(left)
        self._build_left(mid)
        self._build_right(right)

    # ── LEFT (cột giữa khung): Góc + Tọa độ + Spine + Stack ───────
    def _build_left(self, parent):
        c = self._card(parent, "GÓC LỆCH  (dAngle)")
        self.lbl_card_angle = c.master.winfo_children()[0]
        self.lbl_da    = tk.Label(c, text="+0.0°", bg=C["bg2"],
                                   fg=C["green"], font=self.F["big"])
        self.lbl_da.pack(pady=(4,0))
        self.lbl_a360  = tk.Label(c, text="abs: 0.0°", bg=C["bg2"],
                                   fg=C["gray"], font=self.F["sm"])
        self.lbl_a360.pack()
        self.lbl_dir   = tk.Label(c, text="✓ OK", bg=C["bg2"],
                                   fg=C["green"], font=self.F["mid"])
        self.lbl_dir.pack(pady=(2,6))

        c2 = self._card(parent, "TỌA ĐỘ  (pixel)")
        g = tk.Frame(c2, bg=C["bg2"]); g.pack(pady=3)
        self.vals = {}
        for i, (lbl, key) in enumerate([
            ("cx","cx"),("cy","cy"),("dX","dx"),("dY","dy"),("Off","off"),
        ]):
            tk.Label(g, text=lbl, bg=C["bg2"], fg=C["gray"],
                     font=self.F["xxs"], width=7).grid(row=0, column=i, padx=3)
            v = tk.Label(g, text="---", bg=C["bg2"], fg=C["white"],
                         font=self.F["xs"], width=7)
            v.grid(row=1, column=i, padx=3)
            self.vals[key] = v

        c3 = self._card(parent, "SPINE / HÌNH HỌC")
        g3 = tk.Frame(c3, bg=C["bg2"]); g3.pack(pady=3)
        for i, (lbl, key) in enumerate([
            ("Tip X","tx"),("Tip Y","ty"),("Heel X","hx"),
            ("Heel Y","hy"),("Len","len"),
        ]):
            tk.Label(g3, text=lbl, bg=C["bg2"], fg=C["gray"],
                     font=self.F["xxs"], width=8).grid(row=0, column=i, padx=3)
            v = tk.Label(g3, text="---", bg=C["bg2"], fg=C["cyan"],
                         font=self.F["xs"], width=8)
            v.grid(row=1, column=i, padx=3)
            self.vals[key] = v

        g3b = tk.Frame(c3, bg=C["bg2"]); g3b.pack(pady=(0,3))
        for i, (lbl, key) in enumerate([("Area","area"),("Solid","sol")]):
            tk.Label(g3b, text=lbl, bg=C["bg2"], fg=C["gray"],
                     font=self.F["xxs"], width=11).grid(row=0, column=i, padx=5)
            v = tk.Label(g3b, text="---", bg=C["bg2"], fg=C["purple"],
                         font=self.F["xs"], width=11)
            v.grid(row=1, column=i, padx=5)
            self.vals[key] = v

        c4 = self._card(parent, "LÓT TRÊN  (khi chồng)")
        g4 = tk.Frame(c4, bg=C["bg2"]); g4.pack(pady=3)
        for i, (lbl, key) in enumerate([
            ("Stack","ss"),("TopA","ta"),("dAngle","tda"),
            ("tcx","tcx"),("tcy","tcy"),("off","toff"),
        ]):
            tk.Label(g4, text=lbl, bg=C["bg2"], fg=C["gray"],
                     font=self.F["xxs"], width=7).grid(row=0, column=i, padx=2)
            v = tk.Label(g4, text="---", bg=C["bg2"], fg=C["purple"],
                         font=self.F["xs"], width=7)
            v.grid(row=1, column=i, padx=2)
            self.vals[key] = v

        cl = self._card(parent, "LOG SỰ KIỆN")
        cl.pack(fill="both", expand=True)
        self.log_txt = tk.Text(cl, bg=C["bg"], fg=C["gray"],
                                font=self.F["xxs"], height=7,
                                state="disabled", highlightthickness=0, bd=0)
        sb = tk.Scrollbar(cl, command=self.log_txt.yview)
        self.log_txt.configure(yscrollcommand=sb.set)
        sb.pack(side="right", fill="y")
        self.log_txt.pack(fill="both", expand=True)

    # ── MID (cột trái khung): Camera + Cảnh báo + Flags ───────────
    def _build_mid(self, parent):
        cam_outer = tk.Frame(parent, bg=C["bg3"], bd=0)
        cam_outer.pack(fill="both", expand=True, pady=(0,4))

        cam_title = tk.Frame(cam_outer, bg=C["bg3"])
        cam_title.pack(fill="x")
        tk.Label(cam_title, text=" CAMERA FEED ", bg=C["bg3"],
                 fg=C["cyan"], font=self.F["title"]).pack(side="left", padx=6, pady=(4,0))
        self.lbl_proc = tk.Label(cam_title, text="[Chờ kết nối]", bg=C["bg3"],
                                  fg=C["gray"], font=self.F["xs"])
        self.lbl_proc.pack(side="right", padx=10, pady=(4,0))

        roi_bar = tk.Frame(cam_outer, bg=C["bg3"])
        roi_bar.pack(fill="x", padx=6, pady=(0,3))
        tk.Label(roi_bar, text="ROI:", bg=C["bg3"], fg=C["gray"],
                 font=self.F["xs"]).pack(side="left")
        self.lbl_roi = tk.Label(roi_bar, text="[Toàn khung]", bg=C["bg3"],
                                 fg=C["yellow"], font=self.F["xs"])
        self.lbl_roi.pack(side="left", padx=8)
        tk.Button(roi_bar, text="✕ Xóa ROI", bg=C["bg3"], fg=C["red"],
                  font=self.F["xs"], bd=0, cursor="hand2",
                  activebackground=C["bg3"], activeforeground=C["orange"],
                  command=self._roi_clear).pack(side="left", padx=6)
        tk.Label(roi_bar, text="← Kéo chuột trên ảnh để chọn vùng xử lý",
                 bg=C["bg3"], fg=C["gray"], font=self.F["xs"]).pack(side="left", padx=10)

        # ── Thanh điều khiển từ xa — thay thế hoàn toàn thao tác trên
        # cửa sổ OpenCV của main.rs. Gửi lệnh qua IPC :5556 (CMD:x).
        ctrl_bar = tk.Frame(cam_outer, bg=C["bg3"])
        ctrl_bar.pack(fill="x", padx=6, pady=(0,4))

        def _mk(txt, color, cmd, tip):
            b = tk.Button(ctrl_bar, text=txt, bg=C["bg2"], fg=color,
                          font=self.F["xs"], bd=1, relief="solid",
                          activebackground=C["bg3"], activeforeground=color,
                          cursor="hand2", command=cmd)
            b.pack(side="left", padx=3)
            return b

        _mk("▣ Xác nhận ROI Full-frame [D]", C["cyan"],  self._cmd_roi_confirm,
            "Xác nhận toàn khung hình làm vùng xử lý")
        _mk("📷 Chụp mẫu 45 khung [S]",      C["yellow"], self._cmd_capture,
            "Chụp trung bình 45 khung làm ảnh mẫu tham chiếu")
        _mk("↺ Load lại mẫu [R]",           C["teal"],   self._cmd_reload,
            "Load lại ảnh mẫu đã lưu từ file")
        self.btn_tcp = _mk("⏻ Bật/Tắt TCP [P]", C["orange"], self._cmd_toggle_tcp,
            "Bật/tắt gửi dữ liệu Modbus TCP cho PLC")

        self.cam_canvas = tk.Canvas(cam_outer, bg="#0d1117",
                                    highlightthickness=0, cursor="crosshair")
        self.cam_canvas.pack(fill="both", expand=True, padx=4, pady=(0,4))
        self.cam_canvas.create_text(10, 10, anchor="nw", text="Chờ kết nối...",
                                    fill=C["gray"], font=self.F["sm"], tags="placeholder")
        self._cam_img      = None
        self._cam_img_size = (1, 1)

        self.cam_canvas.bind("<ButtonPress-1>",   self._roi_press)
        self.cam_canvas.bind("<B1-Motion>",        self._roi_motion)
        self.cam_canvas.bind("<ButtonRelease-1>",  self._roi_release)

        cw = self._card(parent, "CẢNH BÁO")
        self.lbl_warn = tk.Label(cw, text="● BÌNH THƯỜNG", bg=C["bg2"],
                                  fg=C["green"], font=self.F["sm"],
                                  wraplength=600, justify="center")
        self.lbl_warn.pack(pady=4, fill="x")

        cf = self._card(parent, "TRẠNG THÁI AN TOÀN / PHÔI")
        fg_grid = tk.Frame(cf, bg=C["bg2"]); fg_grid.pack(pady=3)
        self.flag_w = {}
        # key khớp trực tiếp với tên trong M_NAMES (đọc từ m_values thật, không phải cờ ảo)
        flag_defs = [
            ("E_STOP","E_STOP"), ("LỆCH TRỤC\n(M113)","AXIS_DEV_STOP"),
            ("CÓ PHÔI\n(M114)","WORKPIECE_PC"), ("VISION\nREADY","VISION_READY"),
        ]
        warn_clr = {"E_STOP":C["red"], "AXIS_DEV_STOP":C["red"],
                    "WORKPIECE_PC":C["green"], "VISION_READY":C["teal"]}
        for i,(label,key) in enumerate(flag_defs):
            r,c2 = divmod(i,4)
            fr = tk.Frame(fg_grid, bg=C["gray"], padx=4, pady=3)
            fr.grid(row=r, column=c2, padx=3, pady=3)
            lb = tk.Label(fr, text=label, bg=C["gray"], fg=C["bg"],
                          font=self.F["xxs"], width=9, justify="center")
            lb.pack()
            self.flag_w[key] = (fr, lb, warn_clr.get(key, C["teal"]))

        cp = self._card(parent, "KẾT NỐI")
        self.lbl_plc_status = tk.Label(cp, text="○ PLC CHƯA KẾT NỐI", bg=C["bg2"],
                                        fg=C["gray"], font=self.F["sm"])
        self.lbl_plc_status.pack(pady=(0,4), fill="x")
        pg = tk.Frame(cp, bg=C["bg2"]); pg.pack(pady=3)
        for i,(lbl,key) in enumerate([("TCP CLI","cli"),("DATA OK","ok")]):
            tk.Label(pg, text=lbl, bg=C["bg2"], fg=C["gray"],
                     font=self.F["xxs"], width=12).grid(row=0, column=i, padx=6)
            v = tk.Label(pg, text="---", bg=C["bg2"], fg=C["white"],
                         font=self.F["xs"], width=12)
            v.grid(row=1, column=i, padx=6)
            self.vals[f"_p_{key}"] = v

    # ── RIGHT: scrollable container ───────────────────────────────
    def _build_right(self, parent):
        # Bọc toàn bộ cột phải trong Canvas+Scrollbar để cuộn dọc
        _outer = tk.Frame(parent, bg=C["bg"])
        _outer.pack(fill="both", expand=True)
        _vbar = tk.Scrollbar(_outer, orient="vertical", bg=C["bg3"],
                             troughcolor=C["bg"], activebackground=C["cyan"])
        _vbar.pack(side="right", fill="y")
        _cv = tk.Canvas(_outer, bg=C["bg"], highlightthickness=0,
                        yscrollcommand=_vbar.set)
        _cv.pack(side="left", fill="both", expand=True)
        _vbar.config(command=_cv.yview)
        _inner = tk.Frame(_cv, bg=C["bg"])
        _win = _cv.create_window((0, 0), window=_inner, anchor="nw")

        def _on_frame_configure(e):
            _cv.configure(scrollregion=_cv.bbox("all"))
        def _on_canvas_configure(e):
            _cv.itemconfig(_win, width=e.width)
        _inner.bind("<Configure>", _on_frame_configure)
        _cv.bind("<Configure>", _on_canvas_configure)

        # Mouse wheel scroll
        def _on_mousewheel(e):
            _cv.yview_scroll(int(-1*(e.delta/120)), "units")
        def _on_mousewheel_linux(e):
            if e.num == 4: _cv.yview_scroll(-1, "units")
            elif e.num == 5: _cv.yview_scroll(1, "units")
        _cv.bind_all("<MouseWheel>", _on_mousewheel)
        _cv.bind_all("<Button-4>",   _on_mousewheel_linux)
        _cv.bind_all("<Button-5>",   _on_mousewheel_linux)

        parent = _inner   # tất cả card pack vào _inner

        # Card: M COIL LED panel
        cm = self._card(parent, "M COIL — TRẠNG THÁI CẢM BIẾN")
        self.m_leds = []
        m_grid = tk.Frame(cm, bg=C["bg2"]); m_grid.pack(pady=3)
        for i, addr in enumerate(M_ADDRS):
            r, c2 = divmod(i, 4)
            cell = tk.Frame(m_grid, bg=C["bg2"], padx=2, pady=2)
            cell.grid(row=r, column=c2, padx=3, pady=3)
            led = tk.Canvas(cell, width=18, height=18, bg=C["bg2"],
                            highlightthickness=0)
            led.pack()
            oval = led.create_oval(2, 2, 16, 16, fill=C["gray"], outline="")
            name = M_LABELS.get(addr, f"M{addr}")
            lbl  = tk.Label(cell, text=f"M{addr}\n{name}", bg=C["bg2"],
                            fg=C["gray"], font=self.F["led_sm"], justify="center")
            lbl.pack()
            self.m_leds.append((led, oval, lbl, name))

        # Card: SỐ LẦN GẮP LÓT
        ccy = self._card(parent, "SỐ LẦN GẮP LÓT  (M109=1 c.trình=3 phôi)")
        cyg = tk.Frame(ccy, bg=C["bg2"]); cyg.pack(pady=3, fill="x")
        self.lbl_cycle_count = tk.Label(cyg, text="0", bg=C["bg2"],
                                         fg=C["teal"], font=self.F["mid"])
        self.lbl_cycle_count.grid(row=0, column=0, padx=14)
        tk.Label(cyg, text="chu trình", bg=C["bg2"], fg=C["gray"],
                 font=self.F["xxs"]).grid(row=1, column=0)
        self.lbl_pickup_total = tk.Label(cyg, text="0", bg=C["bg2"],
                                          fg=C["lime"], font=self.F["mid"])
        self.lbl_pickup_total.grid(row=0, column=1, padx=14)
        tk.Label(cyg, text="lót đã gắp (×3)", bg=C["bg2"], fg=C["gray"],
                 font=self.F["xxs"]).grid(row=1, column=1)

        # Card: Robot Status (D100..D120 từ PLC — đúng theo MODBUS_TCPIP.md)
        cr = self._card(parent, "ROBOT STATUS — XUNG/TỐC ĐỘ SERVO")
        rg = tk.Frame(cr, bg=C["bg2"]); rg.pack(pady=2)
        self.robot_vals = {}
        robot_fields = [
            ("D100 Xung S1","pulse1"),("D104 Xung S2","pulse2"),("D108 Xung S3","pulse3"),
            ("D112 TĐ S1","speed1"),("D116 TĐ S2","speed2"),("D120 TĐ S3","speed3"),
        ]
        for i,(lbl,key) in enumerate(robot_fields):
            r, c2 = divmod(i, 3)
            tk.Label(rg, text=lbl, bg=C["bg2"], fg=C["gray"],
                     font=self.F["xxs"], width=12).grid(row=r*2, column=c2, padx=3, pady=(2,0))
            v = tk.Label(rg, text="---", bg=C["bg2"], fg=C["cyan"],
                         font=self.F["xs"], width=12)
            v.grid(row=r*2+1, column=c2, padx=3, pady=(0,2))
            self.robot_vals[key] = v

        rb = tk.Frame(cr, bg=C["bg2"]); rb.pack(pady=(3,3))
        self.lbl_robot_state = tk.Label(rb, text="● ---", bg=C["bg2"],
                                         fg=C["gray"], font=self.F["sm"])
        self.lbl_robot_state.pack()

        # Card: Động học — góc theta tính từ xung
        ck = self._card(parent, "ĐỘNG HỌC — GÓC THETA (ikm/dkm)")
        kg = tk.Frame(ck, bg=C["bg2"]); kg.pack(pady=3)
        self.theta_vals = {}
        for i, lbl in enumerate(["θ1", "θ2", "θ3"]):
            tk.Label(kg, text=lbl, bg=C["bg2"], fg=C["gray"],
                     font=self.F["xxs"], width=10).grid(row=0, column=i, padx=6)
            v = tk.Label(kg, text="---", bg=C["bg2"], fg=C["yellow"],
                         font=self.F["xs"], width=10)
            v.grid(row=1, column=i, padx=6)
            self.theta_vals[f"th{i}"] = v
        self.lbl_tcp_xyz = tk.Label(ck, text="TCP (x,y,z) = ---, ---, --- mm",
                                     bg=C["bg2"], fg=C["white"], font=self.F["xs"])
        self.lbl_tcp_xyz.pack(pady=(2,2))
        tk.Label(ck, text=f"R={ROBOT_R:.0f} r={ROBOT_r:.0f} L={ROBOT_L:.0f} "
                          f"l={ROBOT_l:.0f} mm | {PULSE_PER_DEG:.0f} xung/°",
                 bg=C["bg2"], fg=C["gray"], font=self.F["led"]).pack(pady=(0,3))

        # Card: Digital twin (cánh tay) — thu nhỏ chiều cao
        twin = self._card(parent, "DIGITAL TWIN — CÁNH TAY ROBOT")
        self.twin_canvas = tk.Canvas(twin, width=420, height=220, bg=C["bg"],
                                     highlightthickness=1, highlightbackground=C["border"])
        self.twin_canvas.pack(fill="x", padx=3, pady=3)

        # Card: Quỹ đạo TCP — thu nhỏ
        ctj = self._card(parent, "QUỸ ĐẠO DI CHUYỂN  (D112→D120)")
        self.traj_view = TrajectoryView(ctj, w=420, h=240)
        self.traj_view.pack(padx=3, pady=3)
        tk.Button(ctj, text="🗑 Xóa quỹ đạo", bg=C["bg3"], fg=C["yellow"],
                  font=self.F["xs"], bd=0, cursor="hand2",
                  activebackground=C["bg3"], activeforeground=C["orange"],
                  command=self.traj_view.clear).pack(pady=(0,3))

        # Card: D LOT (D0..D10) — dữ liệu PC gửi cho PLC đọc, nổi bật D0/D4/D5
        cdl = self._card(parent, "D LOT (D0..D10) — PC → PLC")
        dl_grid = tk.Frame(cdl, bg=C["bg2"]); dl_grid.pack(pady=2, fill="x")
        self.dlot_vals = {}
        dlot_fields = [
            ("D0\ndelta_ang", 0), ("D1\ndir", 1), ("D2\ndx", 2), ("D3\ndy", 3),
            ("D4\n(=D2)", 4), ("D5\n(=D3)", 5), ("D6\nflags", 6), ("D7\noffset", 7),
            ("D8\nangle360", 8), ("D9\nstack", 9), ("D10\ndata_ok", 10),
        ]
        highlight_addrs = {0, 4, 5}   # D0, D4, D5 — nổi bật theo yêu cầu
        for i, (lbl, addr) in enumerate(dlot_fields):
            r, c2 = divmod(i, 6)
            is_hl = addr in highlight_addrs
            fg_lbl = C["yellow"] if is_hl else C["gray"]
            fg_val = C["cyan"] if is_hl else C["white"]
            tk.Label(dl_grid, text=lbl, bg=C["bg2"], fg=fg_lbl,
                     font=self.F["xxs"], width=9, justify="center").grid(row=r*2, column=c2, padx=3, pady=(2,0))
            v = tk.Label(dl_grid, text="---", bg=C["bg2"], fg=fg_val,
                         font=self.F["xs"] if is_hl else self.F["xxs"], width=9)
            v.grid(row=r*2+1, column=c2, padx=3, pady=(0,4))
            self.dlot_vals[addr] = v

        # Card: PLC D Register (raw) — có Scrollbar để không mất dữ liệu khi dài
        cd = self._card(parent, "PLC D REGISTER  (raw)")
        cd_inner = tk.Frame(cd, bg=C["bg"])
        cd_inner.pack(fill="both", expand=True, padx=3, pady=3)
        plc_sb = tk.Scrollbar(cd_inner, orient="vertical")
        plc_sb.pack(side="right", fill="y")
        self.plc_d_text = tk.Text(cd_inner, bg=C["bg"], fg=C["white"],
                                   font=self.F["led"], height=6,
                                   state="disabled", highlightthickness=0, bd=0,
                                   wrap="none", yscrollcommand=plc_sb.set)
        self.plc_d_text.pack(side="left", fill="both", expand=True)
        plc_sb.config(command=self.plc_d_text.yview)

        # Card: Xử lý ảnh — thống kê
        cv_card = self._card(parent, "XỬ LÝ ẢNH — THỐNG KÊ")
        cv_grid = tk.Frame(cv_card, bg=C["bg2"]); cv_grid.pack(pady=2)
        self.cv_vals = {}
        cv_fields = [
            ("FPS","fps"),("Area","area_cv"),("Solid","sol_cv"),
            ("Stack","ss_cv"),("Flipped","flip_cv"),
        ]
        for i,(lbl,key) in enumerate(cv_fields):
            tk.Label(cv_grid, text=lbl, bg=C["bg2"], fg=C["gray"],
                     font=self.F["led"], width=8).grid(row=0, column=i, padx=3)
            v = tk.Label(cv_grid, text="---", bg=C["bg2"], fg=C["lime"],
                         font=self.F["xs"], width=8)
            v.grid(row=1, column=i, padx=3)
            self.cv_vals[key] = v

        self.lbl_pipeline = tk.Label(cv_card, text="── Chờ frame ──",
                                      bg=C["bg2"], fg=C["gray"], font=self.F["xxs"])
        self.lbl_pipeline.pack(pady=(0,3))

    def _card(self, parent, title):
        outer = tk.Frame(parent, bg=C["bg3"], bd=0)
        outer.pack(fill="x", pady=2, padx=2)
        tk.Label(outer, text=f" {title} ", bg=C["bg3"],
                 fg=C["cyan"], font=self.F["title"]).pack(anchor="w", padx=4, pady=(3,0))
        inner = tk.Frame(outer, bg=C["bg2"], padx=6, pady=4)
        inner.pack(fill="x", padx=4, pady=(0,4))
        return inner

    # ── Poll queue ─────────────────────────────────────────────────
    def _poll(self):
        updated = False
        try:
            while True:
                item = self.q.get_nowait()
                if "_status" in item:
                    st = item["_status"]
                    if st == "connected":
                        self.lbl_conn.config(text="● CONNECTED :5556", fg=C["green"])
                        self._log("IPC kết nối :5556", "ok")
                    else:
                        self.lbl_conn.config(text="● MẤT KẾT NỐI — thử lại...", fg=C["red"])
                        self._log("IPC mất kết nối", "err")
                else:
                    self.last_d = item
                    updated = True
                    self.frame_ct += 1
        except queue.Empty:
            pass

        if updated:
            self._update(self.last_d)

        now = time.time()
        if now - self.fps_t >= 1.0:
            self.fps_v = self.frame_ct / (now - self.fps_t)
            self.frame_ct = 0; self.fps_t = now
            self.cv_vals.get("fps", tk.Label()).config(text=f"{self.fps_v:.1f}")
        self.lbl_fps.config(text=f"{self.fps_v:.1f} fps")
        self.lbl_time.config(text=datetime.now().strftime("%H:%M:%S"))
        self.after(33, self._poll)

    # ── Cập nhật toàn bộ UI ────────────────────────────────────────
    def _update(self, d: dict):
        frz     = d.get("frz",  False)
        stacked = d.get("stk",  False)
        data_ok = d.get("ok",   False)
        ss      = d.get("ss",   0)
        dmg     = d.get("dmg",  0)
        flags   = d.get("fl",   0)

        # ── Góc lệch ──────────────────────────────────────────────
        top_valid = d.get("tv", False)
        da_raw  = d.get("da",  0.0)
        tda_raw = d.get("tda", 0.0)
        da   = tda_raw if (stacked and top_valid) else da_raw
        a360 = d.get("ta", 0.0) if (stacked and top_valid) else d.get("ang", 0.0)

        # "flip" (bottom, tính trên toàn bộ silhouette khi stacked) và "tflip"
        # (top, tính riêng cho lớp trên qua registration) là 2 tín hiệu khác
        # nhau. Phải chọn đúng cái tương ứng với "da" đang hiển thị, nếu
        # không sẽ có tình trạng góc lệch rất nhỏ nhưng vẫn báo "LÓT NGƯỢC"
        # (vì bottom bị lật trong khi lớp đang xem là top thì không).
        flipped = d.get("tflip", False) if (stacked and top_valid) else d.get("flip", False)

        src_label = "LÓT TRÊN" if (stacked and top_valid) else ("LÓT ĐƠN" if not stacked else "LÓT DƯỚI")
        self.lbl_card_angle.config(text=f"GÓC LỆCH  ({src_label})")

        ang_col = C["red"] if flipped else (C["yellow"] if abs(da)>5 else C["green"])
        self.lbl_da.config(text=f"{da:+.1f}°", fg=ang_col)
        self.lbl_a360.config(text=f"abs: {a360:.1f}°")
        if flipped:
            self.lbl_dir.config(text="⚠ LÓT NGƯỢC!", fg=C["red"])
        elif abs(da) < 5:
            self.lbl_dir.config(text="✓ GÓC OK", fg=C["green"])
        elif da > 0:
            self.lbl_dir.config(text=f"↻ XOAY PHẢI {da:.1f}°", fg=C["yellow"])
        else:
            self.lbl_dir.config(text=f"↺ XOAY TRÁI {abs(da):.1f}°", fg=C["yellow"])

        # ── Tọa độ ────────────────────────────────────────────────
        dx = d.get("dx", 0.0); dy = d.get("dy", 0.0)
        off = (dx**2 + dy**2)**0.5
        def sv(key, val, fmt="{:.1f}", col=C["white"]):
            w = self.vals.get(key)
            if w: w.config(text=fmt.format(val), fg=col)
        sv("cx",  d.get("cx",0))
        sv("cy",  d.get("cy",0))
        sv("dx",  dx,  col=C["yellow"] if abs(dx)>15 else C["white"])
        sv("dy",  dy,  col=C["yellow"] if abs(dy)>15 else C["white"])
        sv("off", off, col=C["yellow"] if off>15 else C["white"])
        sv("tx",  d.get("tx",0))
        sv("ty",  d.get("ty",0))
        sv("hx",  d.get("hx",0))
        sv("hy",  d.get("hy",0))
        sv("len", d.get("len",0), col=C["cyan"])
        sv("area",f"{d.get('area',0):.0f}", "{}", C["purple"])
        sv("sol", f"{d.get('sol',0):.3f}", "{}", C["purple"])

        sv("ss",   STACK_NAMES.get(ss, str(ss)), "{}",
           C["yellow"] if stacked else C["white"])
        sv("ta",   d.get("ta",0))
        sv("tda",  d.get("tda",0), col=C["yellow"] if abs(d.get("tda",0))>2 else C["white"])
        sv("tcx",  d.get("tcx",0))
        sv("tcy",  d.get("tcy",0))
        sv("toff", d.get("toff",0), col=C["yellow"] if d.get("toff",0)>5 else C["white"])

        # ── Kết nối ───────────────────────────────────────────────
        cli = d.get("cli", 0)
        sv("_p_cli",       str(cli), "{}", C["green"] if cli>0 else C["gray"])
        sv("_p_ok",        "✓ OK" if data_ok else "✗ NO", "{}",
           C["green"] if data_ok else C["red"])
        if cli > 0:
            self.lbl_plc_status.config(text=f"● PLC ĐÃ KẾT NỐI ({cli})", fg=C["green"])
        else:
            self.lbl_plc_status.config(text="○ PLC CHƯA KẾT NỐI", fg=C["gray"])

        # ── M coil quan trọng (đọc trước để dùng cho cảnh báo) ─────
        pm = d.get("pm", [])
        m_values = self._flatten_m_coils(pm)
        axis_dev = bool(m_values.get(IDX_AXIS_DEV, bool(d.get("axis_dev", 0))))
        e_stop   = bool(m_values.get(2, False))   # M102 E_STOP

        # ── TRẠNG THÁI AN TOÀN / PHÔI ──────────────────────────────
        flag_state = {
            "E_STOP":        e_stop,
            "AXIS_DEV_STOP": axis_dev,
            "WORKPIECE_PC":  bool(m_values.get(IDX_WORKPIECE_PC, bool(d.get("workpiece_pc", 0)))),
            "VISION_READY":  bool(m_values.get(IDX_VISION_READY, bool(d.get("vision_ready", 0)))),
        }
        for key,(fr,lb,ac) in self.flag_w.items():
            if flag_state.get(key, False):
                fr.config(bg=ac); lb.config(bg=ac, fg=C["bg"])
            else:
                fr.config(bg=C["gray"]); lb.config(bg=C["gray"], fg=C["bg"])

        # ── Cảnh báo tổng hợp ─────────────────────────────────────
        if axis_dev:
            self.lbl_warn.config(text="⛔ LỆCH TRỤC X/Y NGOÀI VÙNG LÀM VIỆC — DỪNG ROBOT", fg=C["red"])
        elif e_stop:
            self.lbl_warn.config(text="⛔ E-STOP", fg=C["red"])
        elif frz:
            self.lbl_warn.config(text="⚠ VẬT LẠ — PLC NHẬN 0", fg=C["red"])
        elif flipped:
            self.lbl_warn.config(text="⚠ LÓT NGƯỢC — ĐẶT LẠI!", fg=C["red"])
        elif stacked:
            self.lbl_warn.config(text=f"⚠ {STACK_NAMES.get(ss,'CHỒNG')}", fg=C["yellow"])
        elif (dmg&0b01) or (dmg&0b10):
            self.lbl_warn.config(text="⚠ LÓT BỊ HỎNG", fg=C["orange"])
        elif abs(da)>5 or off>15:
            self.lbl_warn.config(text=f"↻ LỆCH — góc {da:+.1f}° off {off:.0f}px",
                                 fg=C["yellow"])
        else:
            self.lbl_warn.config(text="✓ BÌNH THƯỜNG", fg=C["green"])

        # ── Camera frame ──────────────────────────────────────────
        if "_jpeg" in d and d["_jpeg"]:
            try:
                img = Image.open(io.BytesIO(d["_jpeg"]))
                cw  = max(self.cam_canvas.winfo_width(),  200)
                ch  = max(self.cam_canvas.winfo_height(), 120)
                img.thumbnail((cw, ch), Image.LANCZOS)
                iw, ih = img.size
                self._cam_img_size = (iw, ih)
                self._cam_img = ImageTk.PhotoImage(img)
                ox = (cw - iw) // 2
                oy = (ch - ih) // 2
                self._cam_offset = (ox, oy)
                self.cam_canvas.delete("all")
                self.cam_canvas.create_image(ox, oy, anchor="nw",
                                             image=self._cam_img, tags="frame")
                self._draw_roi_overlay()
            except Exception:
                pass

        # ── Pipeline status label ─────────────────────────────────
        proc_parts = []
        if data_ok: proc_parts.append("✓ Tracking")
        if stacked:  proc_parts.append(f"Stacked={STACK_NAMES.get(ss,'?')}")
        if flipped:  proc_parts.append("Flipped")
        if frz:      proc_parts.append("FROZEN")
        if dmg:      proc_parts.append(f"DMG={dmg:02b}")
        proc_str = "  ".join(proc_parts) if proc_parts else "No data"
        self.lbl_proc.config(text=proc_str,
            fg=C["red"] if (frz or flipped) else
               C["yellow"] if (stacked or dmg) else
               C["green"] if data_ok else C["gray"])

        # ── CV thống kê ───────────────────────────────────────────
        def scv(key, val, col=C["lime"]):
            w = self.cv_vals.get(key)
            if w: w.config(text=str(val), fg=col)
        scv("area_cv", f"{d.get('area',0):.0f}")
        scv("sol_cv",  f"{d.get('sol',0):.3f}")
        scv("ss_cv",   STACK_NAMES.get(ss,"---"),
            col=C["yellow"] if stacked else C["lime"])
        scv("flip_cv", "YES" if flipped else "no",
            col=C["red"] if flipped else C["lime"])

        pipeline_txt = "→ Seg → PCA → Spine"
        if stacked: pipeline_txt += " → Stack"
        if data_ok: pipeline_txt += " → ✓ OK"
        else: pipeline_txt = "── Không có lót ──"
        self.lbl_pipeline.config(text=pipeline_txt,
            fg=C["green"] if data_ok else C["gray"])

        # ── M COIL LEDs (đọc từ "pm" — danh sách phẳng M100..M114,M2000) ──
        for idx, (led, oval, lbl, name) in enumerate(self.m_leds):
            active = m_values.get(idx, False)
            if active:
                if name in ("E_STOP", "AXIS_DEV_STOP"):
                    color = C["red"]
                elif name in ("AUTO", "HOME_DONE", "VISION_READY", "COMM_OK", "WORKPIECE_PC"):
                    color = C["green"]
                elif name == "GRIP_DONE_NG":
                    color = C["orange"]
                else:
                    color = C["teal"]
                led.itemconfig(oval, fill=color)
                lbl.config(fg=color)
            else:
                led.itemconfig(oval, fill=C["gray"])
                lbl.config(fg=C["gray"])

        # ── RESET BỘ ĐẾM GẮP PHÔI khi có cạnh lên của E_STOP (M102) ─
        # PLC nhấn nút DỪNG → M102 lên 1 → GUI reset lại số chu trình / số lót đã gắp
        if e_stop and not self.estop_last:
            self.cycle_count = 0
            self.cycle_last  = False
            self._log("⛔ E-STOP từ PLC — đã reset số lần gắp phôi về 0", "warn")
        self.estop_last = e_stop

        # ── RESET BỘ ĐẾM GẮP PHÔI khi AUTO (M100) chuyển từ BẬT -> TẮT ──
        # Nghĩa là 1 phiên AUTO vừa kết thúc (tắt chế độ tự động) -> tự
        # động đưa số lần gắp lót về 0 để chuẩn bị đếm lại từ đầu cho
        # phiên AUTO kế tiếp.
        auto_now = bool(m_values.get(0, bool(d.get("auto", 0))))    # M100
        if self.auto_last and not auto_now:
            self.cycle_count = 0
            self.cycle_last  = False
            self._log("AUTO đã tắt — đã reset số lần gắp lót về 0", "warn")
        self.auto_last = auto_now

        # ── SỐ LẦN GẮP LÓT — đếm cạnh lên của M109 (CYCLE_3P) ──────
        # Theo MODBUS_TCPIP.txt: M1024(PLC)→M109(PC) lên 1 = 1 chu trình = 3 phôi
        cycle_now = bool(m_values.get(IDX_CYCLE_3P, False)) or bool(d.get("m109", 0))
        if cycle_now and not self.cycle_last:
            self.cycle_count += 1
            self._log(f"Chu trình gắp #{self.cycle_count} hoàn tất (+3 lót)", "ok")
        self.cycle_last = cycle_now
        self.lbl_cycle_count.config(text=str(self.cycle_count))
        self.lbl_pickup_total.config(text=str(self.cycle_count * 3))

        # ── Robot Status (dựa trên M coil THẬT: AUTO/MANUAL/E_STOP) ────
        auto    = auto_now
        manual  = bool(m_values.get(1, bool(d.get("manual", 0))))  # M101
        home    = bool(m_values.get(IDX_HOME_DONE, bool(d.get("home", 0))))  # M110

        if axis_dev:
            rname, rcol = "DỪNG (LỆCH TRỤC)", C["red"]
        elif e_stop:
            rname, rcol = "E-STOP", C["red"]
        elif auto:
            rname, rcol = "AUTO", C["green"]
        elif manual:
            rname, rcol = "MANUAL", C["yellow"]
        else:
            rname, rcol = "---", C["gray"]

        # ── PLC D raw text ────────────────────────────────────────
        pd = d.get("pd", [])
        self._update_plc_d_text(pd)
        robot64 = self._robot_d64_from_pd(pd)

        # ── Đảo dấu số xung (pulse) về dương TRƯỚC khi hiển thị lên GUI
        #    và đưa vào thuật toán động học (nhân -1) ────────────────────
        for pk in ("pulse1", "pulse2", "pulse3"):
            if pk in robot64:
                robot64[pk] = -robot64[pk]
            if pk in d:
                d[pk] = -d[pk]

        for key, _, _ in ROBOT_D64:
            val = int(d.get(key, robot64.get(key, 0)))
            if key in self.robot_vals:
                self.robot_vals[key].config(text=f"{val:,}", fg=C["cyan"])

        # ── D LOT (D0..D10) — cập nhật, giữ giá trị cũ nếu thiếu dữ liệu ──
        pl = d.get("pl", [])
        for entry in pl:
            addr = entry.get("a")
            raw  = entry.get("v", 0)
            sig  = entry.get("i", raw)
            w = self.dlot_vals.get(addr)
            if w is not None:
                w.config(text=f"{sig}")

        pulse1 = int(d.get("pulse1", robot64.get("pulse1", 0)))
        pulse2 = int(d.get("pulse2", robot64.get("pulse2", 0)))
        pulse3 = int(d.get("pulse3", robot64.get("pulse3", 0)))
        speed1 = int(d.get("speed1", robot64.get("speed1", 0)))
        speed2 = int(d.get("speed2", robot64.get("speed2", 0)))
        speed3 = int(d.get("speed3", robot64.get("speed3", 0)))
        pulses = [pulse1, pulse2, pulse3]

        # ── ĐỘNG HỌC — xung → theta → (x,y,z) qua dkm() (port Matlab) ──
        theta = [p / PULSE_PER_DEG for p in pulses]
        for i, key in enumerate(("th0","th1","th2")):
            self.theta_vals[key].config(text=f"{theta[i]:+.2f}°")

        xyz = dkm(theta)
        if xyz is not None:
            self.lbl_tcp_xyz.config(
                text=f"TCP (x,y,z) = {xyz[0]:+.1f}, {xyz[1]:+.1f}, {xyz[2]:+.1f} mm",
                fg=C["white"])
            self.traj_view.push(xyz)
        else:
            self.lbl_tcp_xyz.config(text="TCP — NGOÀI KHÔNG GIAN LÀM VIỆC (vô nghiệm)",
                                     fg=C["red"])

        self.lbl_robot_state.config(
            text=f"● {rname}   tốc độ=({speed1},{speed2},{speed3})", fg=rcol)

        self._draw_digital_twin(theta, pulses)

    # ── Gộp pm (list dict) thành dict {index_offset_M100: bool} ───────
    def _flatten_m_coils(self, pm):
        """pm có thể là:
           - list các {"a":addr,"n":name,"v":0/1}  (main.rs hiện tại)
           - list các {"n":name,"v":[0/1,...]}      (sim cũ)
        Chuẩn hoá về dict index (0..12 = M100..M112, 13 = M2000)."""
        m_values = {}
        has_addr_form = any("a" in e for e in pm if isinstance(e, dict))
        if has_addr_form:
            for entry in pm:
                addr = entry.get("a")
                v = entry.get("v", 0)
                if addr is None:
                    continue
                if addr == 2000:
                    m_values[15] = bool(v)
                elif 100 <= addr <= 114:
                    m_values[addr - 100] = bool(v)
            return m_values

        # Dạng cũ: list of {"n":name,"v":[...]} hoặc {"n":name,"v":scalar}
        if pm and isinstance(pm[0].get("v"), list):
            offset = 0
            for entry in pm:
                vlist = entry.get("v", [])
                for i, v in enumerate(vlist):
                    m_values[offset + i] = bool(v)
                offset += len(vlist)
        else:
            for entry in pm:
                name = entry.get("n", "")
                v = entry.get("v", 0)
                for idx, n in enumerate(M_NAMES):
                    if n == name:
                        m_values[idx] = bool(v)
                        break
        return m_values

    def _robot_d64_from_pd(self, pd):
        """Đọc số 32-bit có dấu từ 2 word ĐẦU trong mỗi khối 4 word (theo
        bảng PLC thật: mỗi giá trị chiếm 8 byte/4 word, nhưng chỉ 4 byte
        đầu (2 word) chứa giá trị thật, 4 byte sau là vùng đệm/rác).
        word tại [i]=LSB, [i+1]=MSB của số 32-bit có dấu."""
        words = []
        for entry in pd:
            val = entry.get("v", 0)
            if isinstance(val, int):
                words.append(val & 0xFFFF)
        out = {}
        for key, start, _ in ROBOT_D64:
            i = start - 100
            if i + 1 < len(words):
                w0, w1 = words[i], words[i+1]   # bỏ qua words[i+2], words[i+3] (rác)
                raw = w0 | (w1 << 16)
                # signed i32
                if raw & (1 << 31):
                    raw -= 1 << 32
                out[key] = raw
        return out

    def _draw_digital_twin(self, theta, pulses):
        c = getattr(self, "twin_canvas", None)
        if not c:
            return
        c.delete("all")
        w = max(c.winfo_width(), 420)
        h = max(c.winfo_height(), 220)
        cx, cy = w / 2, h / 2 + 10

        ARM_BASE_RADIUS = 90
        ARM_LINK_1 = 80

        c.create_oval(cx-ARM_BASE_RADIUS, cy-ARM_BASE_RADIUS,
                      cx+ARM_BASE_RADIUS, cy+ARM_BASE_RADIUS,
                      outline=C["border"], width=1)
        c.create_oval(cx-6, cy-6, cx+6, cy+6, fill=C["white"], outline="")
        colors = [C["cyan"], C["yellow"], C["purple"]]
        wrist_pts = []
        for i, base in enumerate(ALPHA_DEG):
            b = math.radians(base)
            bx = cx + ARM_BASE_RADIUS * 0.62 * math.cos(b)
            by = cy - ARM_BASE_RADIUS * 0.62 * math.sin(b)   # y+ lên trên
            a = math.radians(base + theta[i])
            ex = bx + ARM_LINK_1 * math.cos(a)
            ey = by - ARM_LINK_1 * math.sin(a)               # y+ lên trên
            wrist_pts.append((ex, ey))
            c.create_line(bx, by, ex, ey, fill=colors[i], width=5)
            c.create_oval(bx-6, by-6, bx+6, by+6, fill=colors[i], outline="")
            c.create_oval(ex-5, ey-5, ex+5, ey+5, fill=C["white"], outline=colors[i])
            c.create_text(bx, by-18, text=f"S{i+1}", fill=colors[i], font=self.F["xxs"])
        if wrist_pts:
            tx = sum(p[0] for p in wrist_pts) / len(wrist_pts)
            ty = sum(p[1] for p in wrist_pts) / len(wrist_pts)
            for i, (ex, ey) in enumerate(wrist_pts):
                c.create_line(ex, ey, tx, ty, fill=colors[i], width=2, dash=(4, 3))
            c.create_oval(tx-12, ty-12, tx+12, ty+12, fill=C["green"], outline="")
            c.create_text(tx, ty+22, text="TCP", fill=C["green"], font=self.F["xxs"])
        c.create_text(10, 10, anchor="nw",
                      text=f"pulse: {pulses[0]} | {pulses[1]} | {pulses[2]}",
                      fill=C["gray"], font=self.F["xxs"])

    def _update_plc_d_text(self, pd):
        if not pd:
            return  # giữ nguyên nội dung cũ, không xóa khi chưa có dữ liệu mới
        self.plc_d_text.config(state="normal")
        self.plc_d_text.delete("1.0", "end")
        robot64 = self._robot_d64_from_pd(pd)
        if robot64:
            self.plc_d_text.insert("end", "  64-bit robot data\n")
            for key, addr, label in ROBOT_D64:
                self.plc_d_text.insert("end", f"    D{addr}: {label} = {robot64.get(key, 0)}\n")
            self.plc_d_text.insert("end", "\n  raw words\n")
        for entry in pd:
            name = entry.get("n", "?")
            val  = entry.get("v", 0)
            if isinstance(val, int):
                signed = struct.unpack(">h", struct.pack(">H", val & 0xFFFF))[0]
                self.plc_d_text.insert("end", f"  {name}: {val}  (i16={signed:+d})\n")
            elif isinstance(val, list):
                line = f"  [{name}]\n"
                for i, v in enumerate(val):
                    signed = struct.unpack(">h", struct.pack(">H", v & 0xFFFF))[0]
                    line += f"    D{100+i}: {v} ({signed:+d})\n"
                self.plc_d_text.insert("end", line)
        self.plc_d_text.config(state="disabled")

    # ── ROI drag-select helpers ────────────────────────────────────
    def _canvas_to_img(self, cx, cy):
        iw, ih = self._cam_img_size
        ox, oy = getattr(self, "_cam_offset", (0, 0))
        x = max(0, min(iw, cx - ox))
        y = max(0, min(ih, cy - oy))
        return x, y

    def _roi_press(self, event):
        x, y = self._canvas_to_img(event.x, event.y)
        self._roi_drag = {"x0": x, "y0": y, "x1": x, "y1": y,
                          "cx0": event.x, "cy0": event.y}

    def _roi_motion(self, event):
        if not self._roi_drag:
            return
        x1, y1 = self._canvas_to_img(event.x, event.y)
        self._roi_drag["x1"] = x1
        self._roi_drag["y1"] = y1
        self.cam_canvas.delete("roi_rubber")
        self.cam_canvas.create_rectangle(
            self._roi_drag["cx0"], self._roi_drag["cy0"],
            event.x, event.y,
            outline=C["yellow"], width=2, dash=(6, 3), tags="roi_rubber"
        )

    def _roi_release(self, event):
        if not self._roi_drag:
            return
        x0 = self._roi_drag["x0"]; y0 = self._roi_drag["y0"]
        x1, y1 = self._canvas_to_img(event.x, event.y)
        rx0, rx1 = sorted([x0, x1])
        ry0, ry1 = sorted([y0, y1])
        iw, ih = self._cam_img_size
        if iw > 0 and ih > 0 and (rx1 - rx0) > 8 and (ry1 - ry0) > 8:
            self.roi_rect = (rx0/iw, ry0/ih, rx1/iw, ry1/ih)
            self.lbl_roi.config(
                text=f"({rx0},{ry0})→({rx1},{ry1})  [{rx1-rx0}×{ry1-ry0}px]",
                fg=C["teal"])
            self._log(f"ROI set: ({rx0},{ry0})→({rx1},{ry1})", "ok")
        self._roi_drag = {}
        self.cam_canvas.delete("roi_rubber")
        self._draw_roi_overlay()

    # ── Điều khiển từ xa main.rs (thay cho phím trên cửa sổ OpenCV) ──
    def _cmd_capture(self):
        if self.recv.send_cmd("s"):
            self._log("→ main.rs: bắt đầu CHỤP MẪU 45 khung ảnh", "ok")
        else:
            self._log("Chưa kết nối main.rs — không gửi được lệnh chụp mẫu", "err")

    def _cmd_roi_confirm(self):
        if self.recv.send_cmd("d"):
            self._log("→ main.rs: xác nhận ROI = full-frame", "ok")
        else:
            self._log("Chưa kết nối main.rs — không gửi được lệnh ROI", "err")

    def _cmd_reload(self):
        if self.recv.send_cmd("r"):
            self._log("→ main.rs: load lại ảnh mẫu từ file", "ok")
        else:
            self._log("Chưa kết nối main.rs — không gửi được lệnh load", "err")

    def _cmd_toggle_tcp(self):
        if self.recv.send_cmd("p"):
            self._log("→ main.rs: bật/tắt Modbus TCP", "ok")
        else:
            self._log("Chưa kết nối main.rs — không gửi được lệnh TCP", "err")

    def _roi_clear(self):
        self.roi_rect = None
        self.lbl_roi.config(text="[Toàn khung]", fg=C["yellow"])
        self.cam_canvas.delete("roi_overlay")
        self._log("ROI cleared — dùng toàn khung hình", "warn")

    def _draw_roi_overlay(self):
        self.cam_canvas.delete("roi_overlay")
        if self.roi_rect is None:
            return
        iw, ih = self._cam_img_size
        ox, oy = getattr(self, "_cam_offset", (0, 0))
        nx0, ny0, nx1, ny1 = self.roi_rect
        cx0 = ox + nx0 * iw;  cy0 = oy + ny0 * ih
        cx1 = ox + nx1 * iw;  cy1 = oy + ny1 * ih
        self.cam_canvas.create_rectangle(ox, oy, ox+iw, cy0,
            fill="#000000", stipple="gray50", outline="", tags="roi_overlay")
        self.cam_canvas.create_rectangle(ox, cy1, ox+iw, oy+ih,
            fill="#000000", stipple="gray50", outline="", tags="roi_overlay")
        self.cam_canvas.create_rectangle(ox, cy0, cx0, cy1,
            fill="#000000", stipple="gray50", outline="", tags="roi_overlay")
        self.cam_canvas.create_rectangle(cx1, cy0, ox+iw, cy1,
            fill="#000000", stipple="gray50", outline="", tags="roi_overlay")
        self.cam_canvas.create_rectangle(cx0, cy0, cx1, cy1,
            outline=C["teal"], width=2, tags="roi_overlay")
        self.cam_canvas.create_text(cx0+4, cy0+4, anchor="nw",
            text="▣ ROI", fill=C["teal"],
            font=self.F["xs"], tags="roi_overlay")

    def get_roi_pixels(self, frame_w, frame_h):
        if self.roi_rect is None:
            return None
        nx0, ny0, nx1, ny1 = self.roi_rect
        return (int(nx0*frame_w), int(ny0*frame_h),
                int(nx1*frame_w), int(ny1*frame_h))

    def _log(self, msg, level="info"):
        ts  = datetime.now().strftime("%H:%M:%S")
        col = {"ok":C["green"],"err":C["red"],"warn":C["yellow"]}.get(level,C["gray"])
        self.log_txt.config(state="normal")
        self.log_txt.insert("end", f"[{ts}] {msg}\n", level)
        self.log_txt.tag_config(level, foreground=col)
        self.log_txt.see("end")
        lines = int(self.log_txt.index("end-1c").split(".")[0])
        if lines > MAX_LOG: self.log_txt.delete("1.0", "2.0")
        self.log_txt.config(state="disabled")

    def _close(self):
        self.recv.stop()
        self.destroy()

# =============================================================================
if __name__ == "__main__":
    try:
        import PIL
    except ImportError:
        print("Cần cài Pillow: pip install pillow")
        exit(1)
    try_autostart_main_engine()
    app = App()
    app.mainloop()