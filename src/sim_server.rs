// =============================================================================
//  sim_server.rs  —  PC Modbus TCP SLAVE  (PLC chủ động kết nối vào PC)
//
//  PC chỉ làm SERVER — PLC tự kết nối vào để đọc/ghi.
//  Không cần PC kết nối ra PLC.
//
//  CHẠY TRÊN WINDOWS — BẮT BUỘC Administrator:
//    Chuột phải sim_server.exe → "Run as Administrator"
//    Hoặc: PowerShell Admin → cargo run --bin sim_server --release
//
//  Lần đầu: Windows hỏi firewall → chọn CẢ HAI Private + Public
//
//  ► CHỈ SỬA PHẦN CẤU HÌNH BÊN DƯỚI ◄
// =============================================================================

use std::io::{Read, Write};
use std::net::{TcpListener, TcpStream};
use std::sync::{Arc, Mutex};
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

// =============================================================================
//  CẤU HÌNH
// =============================================================================

const SLAVE_PORT:   u16  = 502;    // ← 5020 nếu không có Admin
const UNIT_ID:      u8   = 0x01;   // ← phải khớp với PLC ("Station No")

// In raw bytes TX/RX ra console để debug
const DEBUG:        bool = true;

// Kích thước vùng nhớ PC slave
const D_SIZE: usize = 1000;   // D0..D999
const M_SIZE: usize = 1000;   // M0..M999

// =============================================================================
//  GIÁ TRỊ KHỞI TẠO — PLC sẽ đọc các giá trị này khi kết nối vào
//  Điền địa chỉ và giá trị mà PLC sẽ đọc từ PC
// =============================================================================
const INIT_D: &[(usize, u16)] = &[
    (1, 0),    // D102 = 99   (set_position)
    (0, 1234),  // thêm các D khác tại đây
];
const INIT_M: &[(usize, bool)] = &[
     (0, true),  // M0 = true
];

// =============================================================================
//  SHARED STATE
// =============================================================================
struct State {
    d: Vec<u16>,    // D register (PLC đọc/ghi)
    m: Vec<bool>,   // M coil     (PLC đọc/ghi)
}

impl State {
    fn new() -> Self {
        let mut s = Self {
            d: vec![0u16;  D_SIZE],
            m: vec![false; M_SIZE],
        };
        for &(addr, val) in INIT_D { s.d[addr] = val; }
        for &(addr, val) in INIT_M { s.m[addr] = val; }
        s
    }
}

// =============================================================================
//  MODBUS TCP FRAME BUILDER
//
//  Chuẩn Modbus TCP/IP Spec v1.1b3:
//  [TID_H][TID_L][0x00][0x00][LEN_H][LEN_L][UNIT][FC][DATA...]
//
//  LEN = số byte từ UNIT đến hết = 1(UNIT) + 1(FC) + len(DATA)
//
//  Hàm mbap() trả về 7 byte đã bao gồm UNIT.
//  QUY TẮC: pdu_len = số byte push SAU mbap() = FC(1) + DATA(n)
//  LEN field = pdu_len + 1
// =============================================================================

#[inline]
fn mbap(tid: u16, unit: u8, pdu_len: u16) -> Vec<u8> {
    let [t0, t1] = tid.to_be_bytes();
    let [l0, l1] = (pdu_len + 1).to_be_bytes();
    vec![t0, t1, 0x00, 0x00, l0, l1, unit]
}

// FC03 response: push FC(1)+BC(1)+DATA(bc) = 2+bc  → pdu_len=2+bc ✓
fn resp_fc03(tid: u16, unit: u8, regs: &[u16]) -> Vec<u8> {
    let bc = (regs.len() * 2) as u8;
    let mut f = mbap(tid, unit, 2 + bc as u16);
    f.push(0x03); f.push(bc);
    for &r in regs { f.extend_from_slice(&r.to_be_bytes()); }
    f
}

// FC06 response: push FC(1)+ADDR(2)+VAL(2) = 5  → pdu_len=5 ✓
fn resp_fc06(tid: u16, unit: u8, addr: u16, val: u16) -> Vec<u8> {
    let mut f = mbap(tid, unit, 5);
    f.push(0x06);
    f.extend_from_slice(&addr.to_be_bytes());
    f.extend_from_slice(&val.to_be_bytes());
    f
}

// FC16 response: push FC(1)+ADDR(2)+CNT(2) = 5  → pdu_len=5 ✓
fn resp_fc16(tid: u16, unit: u8, addr: u16, count: u16) -> Vec<u8> {
    let mut f = mbap(tid, unit, 5);
    f.push(0x10);
    f.extend_from_slice(&addr.to_be_bytes());
    f.extend_from_slice(&count.to_be_bytes());
    f
}

// FC01 response: push FC(1)+BC(1)+BYTES(bc) = 2+bc  → pdu_len=2+bc ✓
fn resp_fc01(tid: u16, unit: u8, coils: &[bool]) -> Vec<u8> {
    let bc = ((coils.len() + 7) / 8) as u8;
    let mut bytes = vec![0u8; bc as usize];
    for (i, &c) in coils.iter().enumerate() { if c { bytes[i/8] |= 1 << (i%8); } }
    let mut f = mbap(tid, unit, 2 + bc as u16);
    f.push(0x01); f.push(bc);
    f.extend_from_slice(&bytes);
    f
}

// FC05 response: mirrors request — push FC(1)+ADDR(2)+VAL(2) = 5  → pdu_len=5 ✓
fn resp_fc05(tid: u16, unit: u8, addr: u16, val: bool) -> Vec<u8> {
    let mut f = mbap(tid, unit, 5);
    f.push(0x05);
    f.extend_from_slice(&addr.to_be_bytes());
    f.extend_from_slice(if val { &[0xFF_u8, 0x00] } else { &[0x00_u8, 0x00] });
    f
}

// FC15 response: push FC(1)+ADDR(2)+CNT(2) = 5  → pdu_len=5 ✓
fn resp_fc15(tid: u16, unit: u8, addr: u16, count: u16) -> Vec<u8> {
    let mut f = mbap(tid, unit, 5);
    f.push(0x0F);
    f.extend_from_slice(&addr.to_be_bytes());
    f.extend_from_slice(&count.to_be_bytes());
    f
}

// Exception response: push ERROR_FC(1)+CODE(1) = 2  → pdu_len=2 ✓
fn resp_exc(tid: u16, unit: u8, fc: u8, code: u8) -> Vec<u8> {
    let mut f = mbap(tid, unit, 2);
    f.push(0x80 | fc); f.push(code);
    f
}

// =============================================================================
//  ĐỌC TCP FRAME ĐÚNG CÁCH — tránh partial read
//
//  Đọc MBAP header 6 byte trước (biết được length),
//  rồi đọc đúng số byte còn lại. Không dùng stream.read() 1 lần.
// =============================================================================

fn read_exact(stream: &mut TcpStream, buf: &mut [u8]) -> bool {
    let mut got = 0;
    let deadline = Instant::now() + Duration::from_secs(3);
    while got < buf.len() {
        if Instant::now() > deadline { return false; }
        match stream.read(&mut buf[got..]) {
            Ok(0)  => return false,
            Ok(n)  => got += n,
            Err(ref e) if e.kind() == std::io::ErrorKind::TimedOut
                       || e.kind() == std::io::ErrorKind::WouldBlock => continue,
            Err(_) => return false,
        }
    }
    true
}

// =============================================================================
//  XỬ LÝ MỖI CONNECTION TỪ PLC
// =============================================================================

fn handle_client(mut stream: TcpStream, state: Arc<Mutex<State>>,
                 running: Arc<AtomicBool>, peer: String) {
    stream.set_read_timeout(Some(Duration::from_millis(500))).ok();
    stream.set_nodelay(true).ok();

    println!("[SLAVE] ▶ PLC kết nối: {peer}");
    let mut req_count = 0u64;
    let mut hbuf = [0u8; 6]; // MBAP header buffer

    loop {
        if !running.load(Ordering::Relaxed) { break; }

        // ── Bước 1: Đọc MBAP header 6 byte ──────────────────────────────────
        // Dùng read_exact với timeout riêng cho header (chờ frame mới)
        {
            let mut got = 0usize;
            let ok = loop {
                match stream.read(&mut hbuf[got..]) {
                    Ok(0)  => break false,
                    Ok(n)  => {
                        got += n;
                        if got == 6 { break true; }
                    }
                    Err(ref e) if e.kind() == std::io::ErrorKind::TimedOut
                               || e.kind() == std::io::ErrorKind::WouldBlock => {
                        if got == 0 { continue; }  // chưa có frame → đợi tiếp
                        else        { continue; }  // đang nhận dở → đọc tiếp
                    }
                    Err(e) => { eprintln!("[SLAVE] {peer} lỗi đọc: {e}"); break false; }
                }
            };
            if !ok { println!("[SLAVE] ◀ {peer} ngắt. {req_count} req."); break; }
        }

        let tid      = u16::from_be_bytes([hbuf[0], hbuf[1]]);
        let proto    = u16::from_be_bytes([hbuf[2], hbuf[3]]);
        let data_len = u16::from_be_bytes([hbuf[4], hbuf[5]]) as usize;

        // Kiểm tra Protocol ID
        if proto != 0x0000 {
            eprintln!("[SLAVE] {peer} Protocol={proto:#06x} không phải 0x0000 → bỏ qua");
            continue;
        }
        if data_len < 2 || data_len > 260 {
            eprintln!("[SLAVE] {peer} Length={data_len} không hợp lệ → bỏ qua");
            continue;
        }

        // ── Bước 2: Đọc PDU = UNIT(1)+FC(1)+DATA ────────────────────────────
        let mut pdu = vec![0u8; data_len];
        if !read_exact(&mut stream, &mut pdu) {
            println!("[SLAVE] ◀ {peer} mất kết nối giữa frame.");
            break;
        }

        let unit = pdu[0];
        let fc   = pdu[1];
        let data = &pdu[2..]; // DATA sau UNIT+FC

        if DEBUG {
            let frame: Vec<String> = hbuf.iter().chain(pdu.iter())
                .map(|b| format!("{b:02X}")).collect();
            println!("[SLAVE] RX tid={tid} unit={unit:#04x} fc={fc:#04x}: {}",
                frame.join(" "));
        }

        // Kiểm tra Unit ID — log rõ nếu sai
        if unit != UNIT_ID {
            eprintln!(
                "[SLAVE] ⚠ UnitID={unit:#04x} ≠ mong đợi {UNIT_ID:#04x}\n\
                 [SLAVE]   → Kiểm tra 'Station No' trong cấu hình PLC\n\
                 [SLAVE]   → Đổi UNIT_ID = {unit} trong code nếu muốn chấp nhận"
            );
            let resp = resp_exc(tid, unit, fc, 0x0B);
            let _ = stream.write_all(&resp);
            req_count += 1;
            continue;
        }

        // ── Bước 3: Xử lý theo FC ───────────────────────────────────────────
        let resp: Vec<u8> = match fc {

            // FC03: PLC đọc D register từ PC
            0x03 => {
                if data.len() < 4 { resp_exc(tid, unit, fc, 0x03) } else {
                    let start = u16::from_be_bytes([data[0], data[1]]) as usize;
                    let count = u16::from_be_bytes([data[2], data[3]]) as usize;
                    if count == 0 || start.saturating_add(count) > D_SIZE {
                        eprintln!("[SLAVE] FC03 addr ngoài vùng start={start} count={count}");
                        resp_exc(tid, unit, fc, 0x02)
                    } else {
                        let regs = state.lock().unwrap().d[start..start+count].to_vec();
                        println!("[SLAVE] FC03 PLC đọc D{start}..D{} → {:?}",
                            start+count-1, &regs[..regs.len().min(5)]);
                        resp_fc03(tid, unit, &regs)
                    }
                }
            }

            // FC06: PLC ghi 1 D register lên PC (Write Single Register)
            0x06 => {
                if data.len() < 4 { resp_exc(tid, unit, fc, 0x03) } else {
                    let addr = u16::from_be_bytes([data[0], data[1]]) as usize;
                    let val  = u16::from_be_bytes([data[2], data[3]]);
                    if addr >= D_SIZE {
                        resp_exc(tid, unit, fc, 0x02)
                    } else {
                        state.lock().unwrap().d[addr] = val;
                        println!("[SLAVE] ★ FC06 PLC ghi D{addr} = {val}");
                        resp_fc06(tid, unit, addr as u16, val)
                    }
                }
            }

            // FC16: PLC ghi nhiều D register lên PC
            0x10 => {
                if data.len() < 5 { resp_exc(tid, unit, fc, 0x03) } else {
                    let start = u16::from_be_bytes([data[0], data[1]]) as usize;
                    let count = u16::from_be_bytes([data[2], data[3]]) as usize;
                    let bc    = data[4] as usize;
                    if count == 0 || start.saturating_add(count) > D_SIZE || data.len() < 5+bc {
                        resp_exc(tid, unit, fc, 0x02)
                    } else {
                        let mut st = state.lock().unwrap();
                        for i in 0..count {
                            st.d[start+i] = u16::from_be_bytes([data[5+i*2], data[6+i*2]]);
                        }
                        print!("[SLAVE] ★ FC16 PLC ghi");
                        for i in 0..count { print!(" D{}={}", start+i, st.d[start+i]); }
                        println!();
                        resp_fc16(tid, unit, start as u16, count as u16)
                    }
                }
            }

            // FC01: PLC đọc M coil từ PC
            0x01 => {
                if data.len() < 4 { resp_exc(tid, unit, fc, 0x03) } else {
                    let start = u16::from_be_bytes([data[0], data[1]]) as usize;
                    let count = u16::from_be_bytes([data[2], data[3]]) as usize;
                    if count == 0 || start.saturating_add(count) > M_SIZE {
                        resp_exc(tid, unit, fc, 0x02)
                    } else {
                        let coils = state.lock().unwrap().m[start..start+count].to_vec();
                        println!("[SLAVE] FC01 PLC đọc M{start}..M{} → {:?}",
                            start+count-1, &coils[..coils.len().min(8)]);
                        resp_fc01(tid, unit, &coils)
                    }
                }
            }

            // FC05: PLC ghi 1 M coil lên PC (Write Single Coil)
            0x05 => {
                if data.len() < 4 { resp_exc(tid, unit, fc, 0x03) } else {
                    let addr = u16::from_be_bytes([data[0], data[1]]) as usize;
                    let val  = u16::from_be_bytes([data[2], data[3]]) == 0xFF00;
                    if addr >= M_SIZE {
                        resp_exc(tid, unit, fc, 0x02)
                    } else {
                        state.lock().unwrap().m[addr] = val;
                        println!("[SLAVE] ★ FC05 PLC ghi M{addr} = {val}");
                        resp_fc05(tid, unit, addr as u16, val)
                    }
                }
            }

            // FC15: PLC ghi nhiều M coil lên PC
            0x0F => {
                if data.len() < 5 { resp_exc(tid, unit, fc, 0x03) } else {
                    let start = u16::from_be_bytes([data[0], data[1]]) as usize;
                    let count = u16::from_be_bytes([data[2], data[3]]) as usize;
                    let bc    = data[4] as usize;
                    if count == 0 || start.saturating_add(count) > M_SIZE || data.len() < 5+bc {
                        resp_exc(tid, unit, fc, 0x02)
                    } else {
                        let mut st = state.lock().unwrap();
                        for i in 0..count {
                            st.m[start+i] = (data[5+i/8] >> (i%8)) & 1 == 1;
                        }
                        println!("[SLAVE] ★ FC15 PLC ghi M{start}..M{}", start+count-1);
                        resp_fc15(tid, unit, start as u16, count as u16)
                    }
                }
            }

            _ => {
                eprintln!("[SLAVE] FC={fc:#04x} không hỗ trợ");
                resp_exc(tid, unit, fc, 0x01)
            }
        };

        if DEBUG {
            println!("[SLAVE] TX: {}", resp.iter().map(|b|format!("{b:02X}")).collect::<Vec<_>>().join(" "));
        }

        if stream.write_all(&resp).is_err() {
            println!("[SLAVE] ◀ {peer} lỗi ghi — ngắt.");
            break;
        }
        req_count += 1;
    }
}

// =============================================================================
//  PC SLAVE SERVER — lắng nghe PLC kết nối vào
// =============================================================================

fn run_slave(state: Arc<Mutex<State>>, running: Arc<AtomicBool>) {
    let bind = format!("0.0.0.0:{SLAVE_PORT}");

    let listener = match TcpListener::bind(&bind) {
        Ok(l) => {
            println!("[SLAVE] ✓ Lắng nghe tại {bind}  UnitID={UNIT_ID:#04x}");
            println!("[SLAVE]   Kênh PLC đọc từ PC (FC03): D{}={} ...",
                INIT_D.first().map(|x|x.0).unwrap_or(0),
                INIT_D.first().map(|x|x.1).unwrap_or(0));
            l
        }
        Err(e) => {
            eprintln!("\n\
╔═══════════════════════════════════════════════════════╗\n\
║  LỖI: Không bind được port {SLAVE_PORT}                       ║\n\
║  Chi tiết: {e:<43}║\n\
║                                                       ║\n\
║  FIX 1: Chạy lại với quyền Administrator             ║\n\
║         (chuột phải .exe → Run as Administrator)     ║\n\
║  FIX 2: Đổi SLAVE_PORT = 5020 trong code             ║\n\
║         rồi cấu hình PLC kết nối port 5020           ║\n\
╚═══════════════════════════════════════════════════════╝\n");
            std::process::exit(1);
        }
    };

    // Mở Windows Firewall tự động (chỉ khi Admin)
    #[cfg(windows)] {
        let _ = std::process::Command::new("netsh").args([
            "advfirewall","firewall","add","rule",
            &format!("name=Modbus_PC_Slave_{SLAVE_PORT}"),
            "dir=in","action=allow","protocol=TCP",
            &format!("localport={SLAVE_PORT}"), "profile=any",
        ]).output();
        println!("[SLAVE]   Windows Firewall: đã mở port {SLAVE_PORT} (profile=any)");
    }

    // In bảng giá trị khởi tạo
    println!("[SLAVE]   Giá trị khởi tạo:");
    for &(addr, val) in INIT_D { println!("[SLAVE]     D{addr} = {val}"); }
    for &(addr, val) in INIT_M { println!("[SLAVE]     M{addr} = {val}"); }
    println!("[SLAVE]   Chờ PLC kết nối...\n");

    listener.set_nonblocking(true).ok();
    let mut conn_id = 0u64;

    while running.load(Ordering::Relaxed) {
        match listener.accept() {
            Ok((stream, addr)) => {
                conn_id += 1;
                let id   = conn_id;
                let s    = Arc::clone(&state);
                let r    = Arc::clone(&running);
                let peer = format!("{addr} [#{id}]");
                std::thread::spawn(move || handle_client(stream, s, r, peer));
            }
            Err(ref e) if e.kind() == std::io::ErrorKind::WouldBlock => {
                std::thread::sleep(Duration::from_millis(10));
            }
            Err(e) => eprintln!("[SLAVE] Accept lỗi: {e}"),
        }
    }
}

// =============================================================================
//  STATUS LOG — in giá trị D/M mỗi 5 giây
// =============================================================================

fn status_thread(state: Arc<Mutex<State>>, running: Arc<AtomicBool>) {
    let t0 = Instant::now();
    loop {
        std::thread::sleep(Duration::from_secs(5));
        if !running.load(Ordering::Relaxed) { break; }
        let st = state.lock().unwrap();
        println!("\n[STATUS] {:.0}s uptime", t0.elapsed().as_secs_f64());
        println!("[STATUS] D register (PLC đọc/ghi):");
        for &(addr, _) in INIT_D {
            println!("[STATUS]   D{addr} = {}", st.d[addr]);
        }
        // Kiểm tra thêm các D thường dùng
        let extra = [0usize, 1, 2, 100, 101, 102, 103];
        let already: Vec<usize> = INIT_D.iter().map(|x|x.0).collect();
        for addr in extra {
            if !already.contains(&addr) && st.d[addr] != 0 {
                println!("[STATUS]   D{addr} = {} (có giá trị)", st.d[addr]);
            }
        }
        println!("[STATUS] M coil:");
        for i in 0..16usize {
            if st.m[i] { println!("[STATUS]   M{i} = true"); }
        }
    }
}

// =============================================================================
//  MAIN
// =============================================================================

fn main() {
    println!("
╔═══════════════════════════════════════════════════════╗
║      PC Modbus TCP SLAVE  —  sim_server               ║
║      PLC chủ động kết nối vào PC để đọc/ghi           ║
╠═══════════════════════════════════════════════════════╣
║  Port    : {SLAVE_PORT}                                       ║
║  UnitID  : {UNIT_ID:#04x}                                     ║
║  DEBUG   : {DEBUG:<43}║
╠═══════════════════════════════════════════════════════╣
║  Giá trị khởi tạo (PLC đọc FC03):                    ║");
    for &(addr, val) in INIT_D {
        println!("║    D{addr:<6} = {val:<43}║");
    }
    for &(addr, val) in INIT_M {
        println!("║    M{addr:<6} = {val:<43}║");
    }
    println!("╚═══════════════════════════════════════════════════════╝
");

    let state   = Arc::new(Mutex::new(State::new()));
    let running = Arc::new(AtomicBool::new(true));

    // Status thread
    { let s=Arc::clone(&state); let r=Arc::clone(&running);
      std::thread::spawn(move || status_thread(s, r)); }

    // Slave thread (blocking)
    run_slave(state, running);
}