#!/usr/bin/env python3
"""
plc_simulator.py — Giả lập PLC thật (Modbus TCP MASTER)
=====================================================================

  PC (main.rs) là SLAVE thuần — chỉ lắng nghe, không chủ động gì cả.
  PLC mới là bên CHỦ ĐỘNG mọi giao tiếp:

    FC03 — PLC đọc  D0..D10     (tọa độ lót từ vision, PC ghi)
    FC16 — PLC ghi  D100..D123  (robot status: xung/tốc độ servo 1/2/3)
    FC05 — PLC ghi  M100..M111  (trạng thái AUTO/MANUAL/E-STOP/cảm biến...)
    FC05 — PLC ghi  M113        (AXIS_DEVIATION_STOP — PLC ghi)
    FC05 — PLC ghi  M2000       (trạng thái kết nối truyền thông OK)
    FC01 — PLC đọc  M112 VISION_READY / M114 WORKPIECE_DETECT_PC (PC ghi, chỉ đọc)

  Script này CHỦ ĐỘNG kết nối tới main.rs (giống PLC thật) và lặp lại
  đúng chu trình PLC sẽ làm, theo đúng địa chỉ + cách đóng gói dữ liệu
  trong main.rs (xem build_slave_regs / d_words_to_i32_lsb / handle_plc_client).

  ── QUAN TRỌNG: ĐỊNH DẠNG D100..D123 (robot status) ──────────────────
  main.rs (ROBOT_VALUE_MODE = 3) xử lý mỗi giá trị robot (pulse1/2/3,
  speed1/2/3) là số nguyên CÓ DẤU 64-BIT THẬT, đóng trong 1 khối 4 word
  (8 byte) — CẢ 4 word đều là data thật, không có word đệm:
      D[n]   = word thấp nhất  (bit 0-15,  LSB)
      D[n+1] = word thấp giữa  (bit 16-31)
      D[n+2] = word cao giữa   (bit 32-47)
      D[n+3] = word cao nhất   (bit 48-63, MSB, mang bit dấu)

  Script này GIẢ LẬP PLC gửi CẢ KHỐI 24 word (6 giá trị x 4 word) trong
  CÙNG 1 LẦN qua FC16 (multi-register write) — khớp giả thuyết ưu tiên
  hiện tại: PLC ghi nguyên 64-bit 1 lúc, không rời rạc từng word qua
  FC06. Nếu PLC Xinje thật của bạn lại ghi rời từng word (FC06), xem
  lại hàm `write_robot_words_fc06()` (để sẵn, không gọi mặc định) và
  đổi lời gọi trong vòng lặp `run()`.

  Giá trị có thể âm hoặc dương — script mô phỏng pulse/speed dao động
  qua lại CẢ ÂM và DƯƠNG (pulse theo sin, speed theo cos lệch pha,
  không lấy abs()) để main.rs thấy đúng số âm/dương khi servo đảo chiều.

  Chạy:
    python plc_simulator.py                  # nối 127.0.0.1:502
    python plc_simulator.py 127.0.0.1 5020   # nối port khác (vd. không có quyền Admin)

  Trước đó cần chạy main.rs (hoặc sim_server.py) để có Slave đang lắng nghe.

  ── MỚI: KIỂM TRA BỘ LỌC "NHẢY GÓC" (ANGLE_DEADBAND_DEG trong main.rs) ──
  Script này giờ tự động theo dõi 2 giá trị góc PC gửi qua D0 (delta_angle)
  và D8 (angle360) ở mỗi chu kỳ đọc, so sánh với lần đọc trước để phân loại:
    - "giữ nguyên"  : 2 lần đọc y hệt nhau (vật đứng yên hoặc nhiễu đã bị
                       bộ lọc deadband 1° trong main.rs chặn lại)
    - "đổi thật"    : lệch >= 1° (vật thực sự xoay, PC cập nhật ngay,
                       KHÔNG chờ thêm frame nào)
    - "NHẢY VẶT"    : lệch > 0 nhưng < 1° — nếu thấy cái này xuất hiện,
                       nghĩa là bộ lọc trong main.rs CHƯA hoạt động đúng.

  Cách test thực tế:
    1) Chạy main.rs với camera đang nhìn 1 vật ĐỨNG YÊN hoàn toàn.
    2) Chạy script này, để chạy ít nhất 10-15 giây (100+ chu kỳ).
    3) Xem dòng "[PLC][JITTER STATS]" in định kỳ + tổng kết cuối (Ctrl+C):
       jitter_count phải = 0 khi vật đứng yên. Nếu > 0 nghĩa là vẫn còn
       nhiễu lọt qua bộ lọc, cần xem lại ANGLE_DEADBAND_DEG trong main.rs.
    4) Sau đó thử xoay vật thật > 1-2° và quan sát: giá trị phải đổi ngay
       ở đúng chu kỳ đó (không delay), để xác nhận bộ lọc không gây trễ.
"""

import socket, struct, sys, time, math

# ──────────────────────────────────────────────────────────────────
# ĐỊA CHỈ — PHẢI KHỚP 100% VỚI main.rs
# ──────────────────────────────────────────────────────────────────
UNIT_ID = 0x01

# D register — PLC đọc vùng lót (PC ghi, vision)
D_LOT_START   = 0      # D0
D_LOT_COUNT   = 11     # D0..D10

# D register — PLC ghi vùng robot status (đúng layout cố định trong main.rs)
# 6 giá trị (pulse1/2/3, speed1/2/3), MỖI giá trị chiếm 1 khối 4 word (8 byte),
# CẢ 4 word đều là data thật của số i64 có dấu (ROBOT_VALUE_MODE=3 trong main.rs):
#   D[n]   = w0 = bit 0-15   (LSB nhất)
#   D[n+1] = w1 = bit 16-31
#   D[n+2] = w2 = bit 32-47
#   D[n+3] = w3 = bit 48-63  (MSB nhất, mang bit dấu)
# D100..D123 = 24 word = 6 giá trị x 4 word, khớp đúng D_ROBOT_NAMES trong main.rs.
D_ROBOT_START = 100    # D100
D_ROBOT_COUNT = 24     # D100..D123

# M coil — PLC ghi trạng thái I/O (FC05, 1 coil mỗi lần — đúng như main.rs xử lý)
M_AUTO            = 100   # TRẠNG THÁI AUTO
M_MANUAL          = 101   # TRẠNG THÁI MANUAL
M_ESTOP           = 102   # E-STOP (X16)
M_S1_ANGLE_SENSOR = 103   # CẢM BIẾN GÓC SERVO 1 (X0)
M_S2_ANGLE_SENSOR = 104   # CẢM BIẾN GÓC SERVO 2 (X1)
M_S3_ANGLE_SENSOR = 105   # CẢM BIẾN GÓC SERVO 3 (X2)
M_LIFT_UP         = 106   # HÀNH TRÌNH BỆ NÂNG - TRÊN (X3)
M_LIFT_DOWN       = 107   # HÀNH TRÌNH BỆ NÂNG - DƯỚI (X4)
M_WORK_DETECT     = 108   # CẢM BIẾN PHÁT HIỆN PHÔI (X5)
M_CYCLE_3P        = 109   # 1 chu trình 3 phôi (M1024)
M_HOME_DONE       = 110   # ĐÃ VỀ HOME (M1025)
M_GRIP_DONE_NG    = 111   # ĐÃ GẮP VẬT XONG, NOT thành công (M1026)
M_VISION_READY    = 112   # PC báo có dX/dY sẵn sàng (CHỈ ĐỌC — PC ghi, PLC chỉ FC01)
M_AXIS_DEVIATION  = 113   # PLC ghi: lệch trục X/Y ngoài vùng làm việc => NGỪNG ROBOT
M_WORKPIECE_DETECT_PC = 114  # PC ghi: phát hiện phôi (CHỈ ĐỌC — PC ghi, PLC chỉ FC01)

M_CONN_OK = 2000          # TRẠNG THÁI KẾT NỐI TRUYỀN THÔNG OK — PLC ghi mỗi chu kỳ

# Các coil PLC THỰC SỰ ghi lên PC (không đụng M112/M114 — đó là PC ghi)
M_NAMES_PLC_WRITE = {
    M_AUTO: "AUTO", M_MANUAL: "MANUAL", M_ESTOP: "E_STOP",
    M_S1_ANGLE_SENSOR: "S1_ANGLE_SENSOR", M_S2_ANGLE_SENSOR: "S2_ANGLE_SENSOR",
    M_S3_ANGLE_SENSOR: "S3_ANGLE_SENSOR", M_LIFT_UP: "LIFT_UP", M_LIFT_DOWN: "LIFT_DOWN",
    M_WORK_DETECT: "WORK_DETECT", M_CYCLE_3P: "CYCLE_3P", M_HOME_DONE: "HOME_DONE",
    M_GRIP_DONE_NG: "GRIP_DONE_NG", M_AXIS_DEVIATION: "AXIS_DEVIATION_STOP",
    M_CONN_OK: "COMM_OK",
}

D_ROBOT_NAMES = [
    "servo1_pulse_b0_15", "servo1_pulse_b16_31", "servo1_pulse_b32_47", "servo1_pulse_b48_63",
    "servo2_pulse_b0_15", "servo2_pulse_b16_31", "servo2_pulse_b32_47", "servo2_pulse_b48_63",
    "servo3_pulse_b0_15", "servo3_pulse_b16_31", "servo3_pulse_b32_47", "servo3_pulse_b48_63",
    "servo1_speed_b0_15", "servo1_speed_b16_31", "servo1_speed_b32_47", "servo1_speed_b48_63",
    "servo2_speed_b0_15", "servo2_speed_b16_31", "servo2_speed_b32_47", "servo2_speed_b48_63",
    "servo3_speed_b0_15", "servo3_speed_b16_31", "servo3_speed_b32_47", "servo3_speed_b48_63",
]


# ──────────────────────────────────────────────────────────────────
# MODBUS TCP — MASTER (CLIENT) HELPERS
# ──────────────────────────────────────────────────────────────────
def _recv_exact(sock, n):
    data = b""
    while len(data) < n:
        chunk = sock.recv(n - len(data))
        if not chunk:
            raise ConnectionError("mat ket noi")
        data += chunk
    return data


def _request(sock, tid, unit, pdu):
    mbap = struct.pack(">HHHB", tid, 0, len(pdu) + 1, unit) + pdu
    sock.sendall(mbap)
    hdr = _recv_exact(sock, 7)
    r_tid, proto, length = struct.unpack(">HHH", hdr[:6])
    if proto != 0 or r_tid != tid:
        raise RuntimeError("phan hoi Modbus sai header")
    body = _recv_exact(sock, length - 1)
    if body and body[0] & 0x80:
        code = body[1] if len(body) > 1 else 0
        raise RuntimeError(f"Modbus exception fc={body[0]:#04x} code={code}")
    return body


def read_holding(sock, tid, unit, addr, count):
    """FC03 — đọc D register"""
    pdu = struct.pack(">BHH", 0x03, addr, count)
    body = _request(sock, tid, unit, pdu)
    bc = body[1]
    return list(struct.unpack(f">{bc // 2}H", body[2:2 + bc]))


def write_holding(sock, tid, unit, addr, vals):
    """FC16 — ghi nhiều D register (mỗi giá trị 1 word u16, big-endian trên wire)"""
    payload = b"".join(struct.pack(">H", v & 0xFFFF) for v in vals)
    pdu = struct.pack(">BHHB", 0x10, addr, len(vals), len(payload)) + payload
    _request(sock, tid, unit, pdu)


def write_register(sock, tid, unit, addr, val):
    """FC06 — ghi 1 D register (1 word, 16-bit) — đúng cách PLC Xinjie thật
    ghi từng word riêng lẻ (xem comment read_robot_value_safe trong main.rs:
    'PLC ghi từng word riêng lẻ qua FC06')."""
    pdu = struct.pack(">BHH", 0x06, addr, val & 0xFFFF)
    _request(sock, tid, unit, pdu)


def write_coil(sock, tid, unit, addr, value):
    """FC05 — ghi 1 M coil"""
    pdu = struct.pack(">BHH", 0x05, addr, 0xFF00 if value else 0x0000)
    _request(sock, tid, unit, pdu)


def read_coils(sock, tid, unit, addr, count):
    """FC01 — đọc M coil"""
    pdu = struct.pack(">BHH", 0x01, addr, count)
    body = _request(sock, tid, unit, pdu)
    bc = body[1]
    bits = []
    for byte in body[2:2 + bc]:
        for i in range(8):
            bits.append(bool((byte >> i) & 1))
    return bits[:count]


def i16(v):
    return v if v < 0x8000 else v - 0x10000


def shortest_angle_diff(prev, curr):
    """Lệch góc ngắn nhất theo vòng tròn 0-360°, y hệt delta_angle_signed()
    trong main.rs — dùng để so sánh 2 lần đọc angle360 (D8) cho đúng, tránh
    báo sai khi góc đi qua mốc 0°/360° (vd 359° -> 1° thực ra chỉ lệch 2°,
    không phải 358°)."""
    d = (curr - prev) % 360.0
    if d > 180.0:
        d -= 360.0
    return d


def i64_to_block_words(v: int):
    """
    Đóng gói 1 số nguyên 64-bit CÓ DẤU thành khối 4 word 16-bit, khớp đúng
    layout cố định D_ROBOT trong main.rs khi ROBOT_VALUE_MODE=3 (mỗi giá
    trị servo chiếm 4 word, CẢ 4 word đều là data thật, LSB ghi trước):
        word0 = bit 0-15   (LSB nhất)
        word1 = bit 16-31
        word2 = bit 32-47
        word3 = bit 48-63  (MSB nhất, mang bit dấu)

    v có thể âm hoặc dương — ép kiểu sang u64 (2's complement) trước khi
    tách word để word3 mang đúng bit dấu, đúng như Rust `raw as i64`
    trong d_words_to_robot_value() mode 3.
    """
    u64v = v & 0xFFFFFFFFFFFFFFFF      # 2's complement trong 64-bit
    w0 = u64v & 0xFFFF
    w1 = (u64v >> 16) & 0xFFFF
    w2 = (u64v >> 32) & 0xFFFF
    w3 = (u64v >> 48) & 0xFFFF
    return [w0, w1, w2, w3]


def i32_to_block_words(v: int):
    """[DỰ PHÒNG — KHÔNG DÙNG MẶC ĐỊNH]
    Đóng gói kiểu 32-bit cũ (2 word data + 2 word đệm = 0), khớp
    ROBOT_VALUE_MODE=1 trong main.rs. Giữ lại để dễ so sánh/thử lại
    nếu giả thuyết 64-bit không khớp PLC thật — xem hướng dẫn switch
    mode ở cuối file (phần __main__).
    """
    u32v = v & 0xFFFFFFFF
    w0 = u32v & 0xFFFF
    w1 = (u32v >> 16) & 0xFFFF
    return [w0, w1, 0, 0]


def write_robot_block_fc16(sock, next_tid, unit, base_addr, values64):
    """
    [MẶC ĐỊNH — đang dùng] Gửi CẢ KHỐI 24 word (6 giá trị x 4 word) trong
    CÙNG 1 LẦN qua FC16 (multi-register write). Khớp giả thuyết ưu tiên:
    PLC gửi 64-bit nguyên khối 1 lúc. values64 = list 6 số i64 có dấu
    theo thứ tự [pulse1, pulse2, pulse3, speed1, speed2, speed3].
    """
    words = []
    for v in values64:
        words.extend(i64_to_block_words(v))
    write_holding(sock, next_tid(), unit, base_addr, words)


def write_robot_words_fc06(sock, next_tid, unit, base_addr, values64):
    """
    [DỰ PHÒNG — KHÔNG GỌI MẶC ĐỊNH] Gửi TỪNG WORD riêng lẻ qua FC06,
    mô phỏng PLC quét chương trình theo từng lệnh DMOV/MOV rời rạc.
    Dùng hàm này thay cho write_robot_block_fc16() nếu muốn thử giả
    thuyết PLC ghi rời từng word (xem comment 'PLC ghi từng word riêng
    lẻ qua FC06' trong main.rs / read_robot_value_safe).
    """
    words = []
    for v in values64:
        words.extend(i64_to_block_words(v))
    for i, w in enumerate(words):
        write_register(sock, next_tid(), unit, base_addr + i, w)


# ──────────────────────────────────────────────────────────────────
# PLC SIMULATOR — chủ động đọc/ghi mỗi chu kỳ, giống PLC thật
# ──────────────────────────────────────────────────────────────────
def run(host="127.0.0.1", port=None):
    ports = [port] if port else [502, 5020]
    sock = None
    used_port = None
    for p in ports:
        try:
            sock = socket.create_connection((host, p), timeout=3.0)
            used_port = p
            break
        except OSError as e:
            print(f"[PLC] Chưa kết nối được {host}:{p}: {e}")
    if sock is None:
        print("[PLC] Không kết nối được PC Slave (main.rs). Hãy chạy main.rs trước.")
        return

    sock.settimeout(3.0)
    tid = 1

    def next_tid():
        nonlocal tid
        v = tid
        tid = (tid + 1) & 0xFFFF or 1
        return v

    print(f"[PLC] Đã kết nối {host}:{used_port}  Unit={UNIT_ID:#04x}")
    print("[PLC] Mô phỏng PLC chủ động: FC03 đọc D0..D10 | FC16 ghi D100..D123 (i64 có dấu, LSB-first, 1 lần/khối)")
    print("[PLC]   FC05 ghi M100..M111,M113 | FC05 ghi M2000 COMM_OK mỗi vòng")
    print("[PLC]   FC01 đọc M112 VISION_READY / M114 WORKPIECE_DETECT_PC (chỉ đọc, PC ghi)")
    print("[PLC] Pulse/speed dao động CẢ ÂM và DƯƠNG (servo đảo chiều).")
    print("[PLC] Ctrl+C để dừng.\n")

    t0 = time.time()
    cycle_count = 0
    auto_mode = True
    estop = False
    axis_deviation = False

    # ── Theo dõi "nhảy góc" (jitter) trên D0 (delta_angle) và D8 (angle360) ──
    # Mục đích: kiểm chứng bộ lọc deadband (ANGLE_DEADBAND_DEG=1.0° trong
    # main.rs) có hoạt động không. Với bộ lọc bật, 2 lần đọc liên tiếp CHỈ
    # được phép khác nhau: hoặc = 0 (giữ nguyên, vật đứng yên/nhiễu nhỏ bị
    # chặn), hoặc >= ~1.0° (thay đổi thật, robot/lót thật sự xoay).
    # Nếu thấy JITTER NHỎ (0 < |Δ| < 1.0°) xuất hiện nhiều lần nghĩa là bộ
    # lọc chưa hoạt động đúng — cần kiểm tra lại main.rs.
    JITTER_ALERT_DEG = 1.0
    prev_da = None
    prev_angle = None
    real_change_count = 0   # số lần đổi thật (>= ngưỡng) — mong đợi
    jitter_count = 0        # số lần "lọt lưới" nhảy nhỏ (< ngưỡng nhưng != 0) — mong đợi = 0
    hold_count = 0          # số lần giữ nguyên y hệt (0 thay đổi) — mong đợi chiếm đa số khi vật đứng yên

    try:
        while True:
            t = time.time() - t0

            # ── 1) PLC đọc D0..D10 (vision data từ PC) — FC03 ──────────
            regs = read_holding(sock, next_tid(), UNIT_ID, D_LOT_START, D_LOT_COUNT)
            da     = i16(regs[0]) / 10.0
            dx     = i16(regs[2]) / 10.0
            dy     = i16(regs[3]) / 10.0
            cx     = regs[4] / 10.0
            cy     = regs[5] / 10.0
            flags  = regs[6]
            angle  = regs[8] / 10.0
            ok     = regs[10]

            # ── Phân tích jitter góc (kiểm chứng bộ lọc deadband) ───────
            jitter_flag = ""
            if ok == 1:
                if prev_da is not None:
                    d_da = da - prev_da   # delta_angle không wrap 360 nên trừ thẳng
                    if d_da == 0.0:
                        hold_count += 1
                    elif abs(d_da) < JITTER_ALERT_DEG:
                        jitter_count += 1
                        jitter_flag += f" [!! JITTER da={d_da:+.2f}° <1°]"
                    else:
                        real_change_count += 1
                if prev_angle is not None:
                    d_ang = shortest_angle_diff(prev_angle, angle)
                    if d_ang != 0.0 and abs(d_ang) < JITTER_ALERT_DEG:
                        jitter_count += 1
                        jitter_flag += f" [!! JITTER angle={d_ang:+.2f}° <1°]"
                prev_da, prev_angle = da, angle
            else:
                prev_da, prev_angle = None, None   # hết lót -> reset mốc so sánh

            # ── 2) PLC ghi D100..D123 (xung + tốc độ servo, giả lập chạy) ──
            #     ĐANG ƯU TIÊN GIẢ THUYẾT: PLC gửi CẢ KHỐI 24 word (64-bit
            #     thật, 4 word/giá trị) trong 1 LẦN qua FC16 — KHÔNG rời rạc
            #     từng word qua FC06 như bản trước. Nếu PLC Xinje thật của
            #     bạn lại ghi rời, đổi lời gọi bên dưới sang
            #     write_robot_words_fc06(...) (đã viết sẵn ở trên).
            #     Không lấy abs(): pulse và speed dao động cả âm và dương,
            #     mô phỏng servo quay 2 chiều (thuận/nghịch).
            phase = t * 0.6
            pulse1 = int(5000 * math.sin(phase))
            pulse2 = int(5000 * math.sin(phase + 2.094))
            pulse3 = int(5000 * math.sin(phase + 4.189))
            speed1 = int(800 * math.cos(phase))
            speed2 = int(800 * math.cos(phase + 2.094))
            speed3 = int(800 * math.cos(phase + 4.189))

            write_robot_block_fc16(
                sock, next_tid, UNIT_ID, D_ROBOT_START,
                [pulse1, pulse2, pulse3, speed1, speed2, speed3],
            )

            # ── 3) PLC ghi từng M coil trạng thái I/O — FC05 (1 coil/lần) ──
            cycle_done   = (cycle_count % 30 == 0 and cycle_count > 0)   # giả lập M1024 nhấp 1 lần / 30 vòng
            home_done    = (int(t) % 10 < 2)                             # giả lập về home theo chu kỳ
            grip_done_ng = False

            coil_values = {
                M_AUTO:            auto_mode,
                M_MANUAL:          not auto_mode,
                M_ESTOP:           estop,
                M_S1_ANGLE_SENSOR: (int(t) % 4 == 0),
                M_S2_ANGLE_SENSOR: (int(t) % 4 == 1),
                M_S3_ANGLE_SENSOR: (int(t) % 4 == 2),
                M_LIFT_UP:         (int(t) % 6 < 3),
                M_LIFT_DOWN:       (int(t) % 6 >= 3),
                M_WORK_DETECT:     ok == 1,
                M_CYCLE_3P:        cycle_done,
                M_HOME_DONE:       home_done,
                M_GRIP_DONE_NG:    grip_done_ng,
                M_AXIS_DEVIATION:  axis_deviation,
            }
            for addr, val in coil_values.items():
                write_coil(sock, next_tid(), UNIT_ID, addr, val)

            # ── 4) PLC báo trạng thái kết nối OK — FC05 M2000 ───────────
            write_coil(sock, next_tid(), UNIT_ID, M_CONN_OK, True)

            # ── 5) PLC đọc M112 VISION_READY / M114 WORKPIECE_DETECT_PC ──
            #     (PC ghi, PLC chỉ đọc qua FC01 — không ghi đè)
            vr_bits = read_coils(sock, next_tid(), UNIT_ID, M_VISION_READY, 1)
            wp_bits = read_coils(sock, next_tid(), UNIT_ID, M_WORKPIECE_DETECT_PC, 1)
            vision_ready    = vr_bits[0] if vr_bits else False
            workpiece_detect = wp_bits[0] if wp_bits else False

            print(
                f"[PLC] t={t:6.1f}s | đọc D0..D10: ok={ok} da={da:+.1f}° dx={dx:+.1f} dy={dy:+.1f} "
                f"cx={cx:.1f} cy={cy:.1f} ang={angle:.1f} flags=0x{flags:02X} | "
                f"ghi pulse=({pulse1:+d},{pulse2:+d},{pulse3:+d}) speed=({speed1:+d},{speed2:+d},{speed3:+d}) | "
                f"vision_ready={vision_ready} workpiece_detect={workpiece_detect}"
                f"{jitter_flag}"
            )

            # Tổng kết định kỳ mỗi 50 chu kỳ (~5s) để dễ theo dõi khi chạy lâu
            if cycle_count > 0 and cycle_count % 50 == 0:
                print(f"[PLC][JITTER STATS] giữ nguyên={hold_count}  đổi thật(>=1°)={real_change_count}  "
                      f"NHẢY VẶT(<1° lọt lưới)={jitter_count}"
                      f"{'  <-- BỘ LỌC OK, KHÔNG CÓ NHẢY VẶT' if jitter_count == 0 else '  <-- CẢNH BÁO: BỘ LỌC CHƯA HOẠT ĐỘNG ĐÚNG'}")

            cycle_count += 1
            time.sleep(0.1)   # ~100ms, giống chu kỳ quét PLC thật

    except KeyboardInterrupt:
        print("\n[PLC] Dừng mô phỏng.")
    except (ConnectionError, RuntimeError, OSError) as e:
        print(f"[PLC] Lỗi kết nối: {e}")
    finally:
        total_reads = hold_count + real_change_count + jitter_count
        print("\n" + "=" * 60)
        print("[PLC] TỔNG KẾT KIỂM TRA BỘ LỌC GÓC (ANGLE_DEADBAND_DEG)")
        print("=" * 60)
        print(f"  Tổng lần so sánh 2 lần đọc liên tiếp: {total_reads}")
        print(f"  - Giữ nguyên (vật đứng yên / nhiễu bị chặn) : {hold_count}")
        print(f"  - Đổi thật (>= 1°, cập nhật ngay lập tức)    : {real_change_count}")
        print(f"  - NHẢY VẶT lọt lưới (0 < |Δ| < 1°)           : {jitter_count}")
        if jitter_count == 0:
            print("  => OK: không thấy nhảy số vặt nào. Bộ lọc deadband hoạt động đúng.")
        else:
            print("  => CẢNH BÁO: vẫn còn nhảy số vặt < 1°. Kiểm tra lại main.rs "
                  "(ANGLE_DEADBAND_DEG / SpineSmooth::update / TopAngleSmooth::update).")
        print("  Lưu ý: script này chỉ ĐỌC D0/D8 mà main.rs đã gửi ra — nếu vật lý")
        print("  thật đứng yên hoàn toàn trong lúc test thì 'đổi thật' phải = 0.")
        print("=" * 60)
        try:
            sock.close()
        except Exception:
            pass


if __name__ == "__main__":
    host = sys.argv[1] if len(sys.argv) > 1 else "127.0.0.1"
    port = int(sys.argv[2]) if len(sys.argv) > 2 else None
    run(host, port)