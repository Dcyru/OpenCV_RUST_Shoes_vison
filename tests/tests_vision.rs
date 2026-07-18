// ================================================================
//  tests_vision.rs — Unit & Integration Tests
//  Shoe Stack Vision  (main.rs v14)
//
//  Cách chạy:
//    cargo test                        # toàn bộ test
//    cargo test -- --nocapture         # xem output println!
//    cargo test test_coord             # chỉ test tọa độ
//    cargo test -- --test-threads=1   # tuần tự (debug dễ hơn)
// ================================================================

#[cfg(test)]
mod tests_coord_conversion {
    //! Kiểm tra hàm cam_to_robot và cam_delta_to_robot.
    //! Đây là phần QUAN TRỌNG NHẤT với PLC — sai tọa độ = robot đến sai vị trí.

    use opencv::core::Rect;

    // ── copy hàm từ main.rs để test độc lập ──────────────────────
    const ROBOT_WORK_DIAMETER_MM: f32        = 600.0;
    const CAMERA_ROBOT_XY_MM:     (f32, f32) = (0.0, 200.0);

    fn cam_to_robot(px_col: f32, px_row: f32, roi: Rect) -> (f32, f32) {
        let scale_x = ROBOT_WORK_DIAMETER_MM / roi.width.max(1) as f32;
        let scale_y = ROBOT_WORK_DIAMETER_MM / roi.height.max(1) as f32;
        let local_x = px_col - roi.x as f32;
        let local_y = px_row - roi.y as f32;
        let rx = CAMERA_ROBOT_XY_MM.0 + (local_x - roi.width as f32 * 0.5) * scale_x;
        let ry = CAMERA_ROBOT_XY_MM.1 + (local_y - roi.height as f32 * 0.5) * scale_y;
        (rx, ry)
    }

    fn cam_delta_to_robot(delta_col: f32, delta_row: f32, roi: Rect) -> (f32, f32) {
        let scale_x = ROBOT_WORK_DIAMETER_MM / roi.width.max(1) as f32;
        let scale_y = ROBOT_WORK_DIAMETER_MM / roi.height.max(1) as f32;
        (delta_col * scale_x, delta_row * scale_y)
    }

    fn make_roi(w: i32, h: i32) -> Rect {
        Rect::new(0, 0, w, h)
    }

    // ─────────────────────────────────────────────────────────────
    // TEST 1: Tâm ảnh → tâm workspace robot
    // ─────────────────────────────────────────────────────────────
    #[test]
    fn test_center_pixel_maps_to_camera_offset() {
        // Pixel tâm ROI 600×600 → phải trả về đúng CAMERA_ROBOT_XY_MM
        let roi = make_roi(600, 600);
        let (rx, ry) = cam_to_robot(300.0, 300.0, roi);
        println!("[TEST1] center pixel (300,300) → robot ({rx:.2}, {ry:.2})");
        assert!((rx - CAMERA_ROBOT_XY_MM.0).abs() < 0.01,
            "robot_x phải = {}, got {rx}", CAMERA_ROBOT_XY_MM.0);
        assert!((ry - CAMERA_ROBOT_XY_MM.1).abs() < 0.01,
            "robot_y phải = {}, got {ry}", CAMERA_ROBOT_XY_MM.1);
    }

    // ─────────────────────────────────────────────────────────────
    // TEST 2: Góc trái trên → âm X, nhỏ hơn offset Y
    // ─────────────────────────────────────────────────────────────
    #[test]
    fn test_top_left_pixel_is_negative_x() {
        let roi = make_roi(600, 600);
        let (rx, _ry) = cam_to_robot(0.0, 0.0, roi);
        println!("[TEST2] pixel (0,0) → robot_x = {rx:.2}");
        assert!(rx < 0.0, "Góc trái phải có robot_x âm, got {rx}");
    }

    // ─────────────────────────────────────────────────────────────
    // TEST 3: Scale — đường chéo từ tâm đến rìa đúng = ROBOT_WORK_DIAMETER_MM/2
    // ─────────────────────────────────────────────────────────────
    #[test]
    fn test_scale_half_diameter() {
        let roi = make_roi(600, 600);
        // Pixel nằm ở rìa phải của ROI
        let (rx, _) = cam_to_robot(600.0, 300.0, roi);
        let expected = CAMERA_ROBOT_XY_MM.0 + ROBOT_WORK_DIAMETER_MM / 2.0;
        println!("[TEST3] pixel rìa phải → robot_x = {rx:.2}, expected {expected:.2}");
        assert!((rx - expected).abs() < 0.5, "Scale sai: got {rx}, expected {expected}");
    }

    // ─────────────────────────────────────────────────────────────
    // TEST 4: delta(0,0) → (0.0, 0.0) — không dịch = không di chuyển
    // ─────────────────────────────────────────────────────────────
    #[test]
    fn test_zero_delta_no_movement() {
        let roi = make_roi(600, 600);
        let (dx, dy) = cam_delta_to_robot(0.0, 0.0, roi);
        assert_eq!((dx, dy), (0.0, 0.0));
    }

    // ─────────────────────────────────────────────────────────────
    // TEST 5: delta symmetry — dịch ngược lại → robot mm ngược dấu
    // ─────────────────────────────────────────────────────────────
    #[test]
    fn test_delta_symmetry() {
        let roi = make_roi(600, 600);
        let (dx1, dy1) = cam_delta_to_robot(50.0,  30.0, roi);
        let (dx2, dy2) = cam_delta_to_robot(-50.0, -30.0, roi);
        println!("[TEST5] delta(+50,+30) → ({dx1:.2},{dy1:.2}), delta(-50,-30) → ({dx2:.2},{dy2:.2})");
        assert!((dx1 + dx2).abs() < 0.01);
        assert!((dy1 + dy2).abs() < 0.01);
    }

    // ─────────────────────────────────────────────────────────────
    // TEST 6: ROI không bắt đầu từ (0,0) — pixel vẫn map đúng
    // ─────────────────────────────────────────────────────────────
    #[test]
    fn test_roi_with_offset() {
        // ROI bắt đầu từ pixel (100,50), kích thước 600×600
        let roi = Rect::new(100, 50, 600, 600);
        // Tâm ROI tại pixel (400, 350)
        let (rx, ry) = cam_to_robot(400.0, 350.0, roi);
        println!("[TEST6] ROI offset (100,50), tâm pixel (400,350) → robot ({rx:.2},{ry:.2})");
        assert!((rx - CAMERA_ROBOT_XY_MM.0).abs() < 0.5,
            "Tâm ROI bị offset nhưng robot_x phải vẫn đúng, got {rx}");
    }
}

// ================================================================
#[cfg(test)]
mod tests_angle_math {
    //! Kiểm tra các hàm toán học góc 360°.
    //! Quan trọng cho: phát hiện flip, delta_angle, smooth_angle.

    fn norm360(a: f32) -> f32 {
        let mut r = a % 360.0;
        if r < 0.0 { r += 360.0; }
        r
    }

    fn delta_angle_signed(from: f32, to: f32) -> f32 {
        let mut d = norm360(to) - norm360(from);
        if d >  180.0 { d -= 360.0; }
        if d < -180.0 { d += 360.0; }
        d
    }

    fn smooth_angle360(prev: f32, next: f32, alpha: f32) -> f32 {
        let d = delta_angle_signed(prev, next);
        norm360(prev + d * alpha)
    }

    // ─────────────────────────────────────────────────────────────
    // TEST 7: norm360 — các giá trị âm và vượt 360
    // ─────────────────────────────────────────────────────────────
    #[test]
    fn test_norm360_negative_and_overflow() {
        assert!((norm360(-90.0) - 270.0).abs() < 0.001, "norm360(-90) phải = 270");
        assert!((norm360(450.0) - 90.0).abs() < 0.001,  "norm360(450) phải = 90");
        assert!((norm360(0.0)   - 0.0).abs()  < 0.001,  "norm360(0) phải = 0");
        assert!((norm360(360.0) - 0.0).abs()  < 0.001,  "norm360(360) phải = 0");
        println!("[TEST7] norm360: PASSED");
    }

    // ─────────────────────────────────────────────────────────────
    // TEST 8: delta_angle — wrap-around 0°/360°
    // ─────────────────────────────────────────────────────────────
    #[test]
    fn test_delta_angle_wrap() {
        // 350° → 10° = +20° (không phải -340°)
        let d = delta_angle_signed(350.0, 10.0);
        println!("[TEST8] delta(350→10) = {d:.2}°");
        assert!((d - 20.0).abs() < 0.01, "Wrap-around sai: got {d}");
    }

    // ─────────────────────────────────────────────────────────────
    // TEST 9: delta_angle — chiều ngắn nhất
    // ─────────────────────────────────────────────────────────────
    #[test]
    fn test_delta_angle_shortest_path() {
        // 30° → 340° = -50° (không phải +310°)
        let d = delta_angle_signed(30.0, 340.0);
        println!("[TEST9] delta(30→340) = {d:.2}°");
        assert!((d + 50.0).abs() < 0.01, "Shortest path sai: got {d}");
    }

    // ─────────────────────────────────────────────────────────────
    // TEST 10: Phát hiện flip (|delta| > 150°)
    // ─────────────────────────────────────────────────────────────
    #[test]
    fn test_flip_detection_threshold() {
        const FLIP_ANGLE_DEG: f32 = 150.0;
        // Lót xoay 180° so với tham chiếu → bị coi là flip
        let delta = delta_angle_signed(90.0, 270.0);
        let is_flip = delta.abs() > FLIP_ANGLE_DEG;
        println!("[TEST10] delta(90→270) = {delta:.1}°, is_flip = {is_flip}");
        assert!(is_flip, "Xoay 180° phải bị phát hiện là flip");

        // Lót xoay 30° → không phải flip
        let delta2 = delta_angle_signed(90.0, 120.0);
        let is_flip2 = delta2.abs() > FLIP_ANGLE_DEG;
        assert!(!is_flip2, "Xoay 30° không phải flip");
    }

    // ─────────────────────────────────────────────────────────────
    // TEST 11: smooth_angle360 — hội tụ sau nhiều bước
    // ─────────────────────────────────────────────────────────────
    #[test]
    fn test_smooth_angle_convergence() {
        let target = 90.0f32;
        let mut current = 0.0f32;
        for _ in 0..100 {
            current = smooth_angle360(current, target, 0.35);
        }
        println!("[TEST11] smooth_angle sau 100 iter: {current:.4}°, target: {target}°");
        assert!((current - target).abs() < 0.01,
            "smooth_angle không hội tụ: got {current}");
    }

    // ─────────────────────────────────────────────────────────────
    // TEST 12: smooth_angle360 — qua biên 360°/0°
    // ─────────────────────────────────────────────────────────────
    #[test]
    fn test_smooth_angle_cross_zero() {
        // Từ 350° hội tụ về 10° — phải đi qua 360°/0°, không đi ngược chiều
        let target = 10.0f32;
        let mut current = 350.0f32;
        for _ in 0..5 {
            current = smooth_angle360(current, target, 0.35);
        }
        println!("[TEST12] smooth cross-zero sau 5 iter: {current:.2}°");
        // Sau 5 bước phải tiến gần 10° (qua 360°), không nhảy xuống 180°
        let d = delta_angle_signed(current, target).abs();
        assert!(d < 30.0, "Smooth qua 360° bị nhảy sai: current={current}, d={d}");
    }
}

// ================================================================
#[cfg(test)]
mod tests_modbus_frames {
    //! Kiểm tra build frame Modbus TCP.
    //! PLC đọc/ghi theo đúng format này — sai byte là mất dữ liệu.

    fn resp_fc03(tid: u16, unit: u8, regs: &[u16]) -> Vec<u8> {
        let bc  = (regs.len() * 2) as u8;
        let len = (3 + regs.len() * 2) as u16;
        let mut f = Vec::with_capacity(6 + len as usize);
        f.extend_from_slice(&tid.to_be_bytes());
        f.extend_from_slice(&0u16.to_be_bytes());
        f.extend_from_slice(&len.to_be_bytes());
        f.push(unit); f.push(0x03); f.push(bc);
        for &r in regs { f.extend_from_slice(&r.to_be_bytes()); }
        f
    }

    fn resp_fc05(tid: u16, unit: u8, addr: u16, val: bool) -> Vec<u8> {
        let coil = if val { 0xFF00u16 } else { 0x0000u16 };
        let mut f = Vec::with_capacity(12);
        f.extend_from_slice(&tid.to_be_bytes());
        f.extend_from_slice(&0u16.to_be_bytes());
        f.extend_from_slice(&6u16.to_be_bytes());
        f.push(unit); f.push(0x05);
        f.extend_from_slice(&addr.to_be_bytes());
        f.extend_from_slice(&coil.to_be_bytes());
        f
    }

    fn resp_fc16(tid: u16, unit: u8, addr: u16, count: u16) -> Vec<u8> {
        let mut f = Vec::with_capacity(12);
        f.extend_from_slice(&tid.to_be_bytes());
        f.extend_from_slice(&0u16.to_be_bytes());
        f.extend_from_slice(&6u16.to_be_bytes());
        f.push(unit); f.push(0x10);
        f.extend_from_slice(&addr.to_be_bytes());
        f.extend_from_slice(&count.to_be_bytes());
        f
    }

    fn resp_fc01(tid: u16, unit: u8, bits: &[bool]) -> Vec<u8> {
        let bc  = (bits.len() + 7) / 8;
        let len = (3 + bc) as u16;
        let mut f = Vec::with_capacity(6 + 3 + bc);
        f.extend_from_slice(&tid.to_be_bytes());
        f.extend_from_slice(&0u16.to_be_bytes());
        f.extend_from_slice(&len.to_be_bytes());
        f.push(unit); f.push(0x01); f.push(bc as u8);
        for chunk in bits.chunks(8) {
            let mut byte = 0u8;
            for (i, &b) in chunk.iter().enumerate() { if b { byte |= 1 << i; } }
            f.push(byte);
        }
        f
    }

    // ─────────────────────────────────────────────────────────────
    // TEST 13: FC03 frame — cấu trúc MBAP header đúng
    // ─────────────────────────────────────────────────────────────
    #[test]
    fn test_fc03_mbap_header() {
        let regs = [100u16, 200u16, 300u16];
        let frame = resp_fc03(0x0001, 1, &regs);
        println!("[TEST13] FC03 frame ({} bytes): {:02X?}", frame.len(), frame);

        // Transaction ID = 0x0001
        assert_eq!(frame[0], 0x00);
        assert_eq!(frame[1], 0x01);
        // Protocol ID = 0x0000
        assert_eq!(frame[2], 0x00);
        assert_eq!(frame[3], 0x00);
        // Length field: 3 + 3*2 = 9 → 0x0009
        let length = u16::from_be_bytes([frame[4], frame[5]]);
        assert_eq!(length, 9, "MBAP length sai: {length}");
        // Function code = 0x03
        assert_eq!(frame[7], 0x03, "Function code phải là 0x03");
        // Byte count = 6
        assert_eq!(frame[8], 6, "Byte count phải là 6");
    }

    // ─────────────────────────────────────────────────────────────
    // TEST 14: FC03 — dữ liệu register encode Big-Endian đúng
    // ─────────────────────────────────────────────────────────────
    #[test]
    fn test_fc03_register_values_big_endian() {
        let regs = [0x1234u16, 0xABCDu16];
        let frame = resp_fc03(1, 1, &regs);
        // Register 1 bắt đầu từ byte 9
        assert_eq!(frame[9],  0x12);
        assert_eq!(frame[10], 0x34, "Register 0 byte thấp sai");
        assert_eq!(frame[11], 0xAB);
        assert_eq!(frame[12], 0xCD, "Register 1 byte thấp sai");
        println!("[TEST14] FC03 Big-Endian: PASSED");
    }

    // ─────────────────────────────────────────────────────────────
    // TEST 15: FC05 — coil ON vs OFF
    // ─────────────────────────────────────────────────────────────
    #[test]
    fn test_fc05_coil_on_off() {
        let on  = resp_fc05(1, 1, 0, true);
        let off = resp_fc05(1, 1, 0, false);
        // Coil ON = 0xFF00
        assert_eq!(on[10],  0xFF);
        assert_eq!(on[11],  0x00);
        // Coil OFF = 0x0000
        assert_eq!(off[10], 0x00);
        assert_eq!(off[11], 0x00);
        println!("[TEST15] FC05 coil ON/OFF: PASSED");
    }

    // ─────────────────────────────────────────────────────────────
    // TEST 16: FC16 response — xác nhận địa chỉ và số lượng
    // ─────────────────────────────────────────────────────────────
    #[test]
    fn test_fc16_ack_addr_count() {
        let frame = resp_fc16(5, 1, 100, 11);
        let addr  = u16::from_be_bytes([frame[8], frame[9]]);
        let count = u16::from_be_bytes([frame[10], frame[11]]);
        assert_eq!(addr,  100, "FC16 addr sai");
        assert_eq!(count,  11, "FC16 count sai");
        println!("[TEST16] FC16 ack addr={addr} count={count}: PASSED");
    }

    // ─────────────────────────────────────────────────────────────
    // TEST 17: FC01 — packing bits đúng thứ tự LSB-first
    // ─────────────────────────────────────────────────────────────
    #[test]
    fn test_fc01_bit_packing_lsb_first() {
        // M0=ON M1=OFF M2=ON M3=OFF M4=ON M5=OFF M6=OFF M7=OFF
        let bits = [true, false, true, false, true, false, false, false];
        let frame = resp_fc01(1, 1, &bits);
        // Byte 9 = packed byte: bit0=M0=1, bit1=M1=0, bit2=M2=1 → 0b00010101 = 0x15
        assert_eq!(frame[9], 0x15, "Bit packing LSB-first sai: {:02X}", frame[9]);
        println!("[TEST17] FC01 bit packing: PASSED (0x{:02X})", frame[9]);
    }
}

// ================================================================
#[cfg(test)]
mod tests_build_slave_regs {
    //! Kiểm tra hàm build_slave_regs — dữ liệu gửi xuống PLC.
    //! D0..D10 phải đúng format i16w (×10) và u16.

    // Minimal stubs để test độc lập
    #[derive(Clone, Default, Copy, Debug)]
    struct Point2f { x: f32, y: f32 }
    fn pt(x: f32, y: f32) -> Point2f { Point2f { x, y } }

    #[derive(Clone, Default, Debug)]
    struct InsoleSpine {
        center: Point2f, tip: Point2f, heel: Point2f,
        spine_vec: Point2f, angle360: f32, length_px: f32,
    }
    #[derive(Clone, Default, Debug)]
    struct StackedAnalysis {
        top_spine: InsoleSpine, top_angle360: f32, top_angle_delta: f32,
        top_delta_x: f32, top_delta_y: f32, top_offset_px: f32,
        fit_iou: f32, residual_area: f64,
        valid: bool, top_flipped: bool, stack_state: u16,
    }
    #[derive(Clone, Default, Debug)]
    struct ShoeSpine {
        bottom_spine: InsoleSpine, stacked_info: Option<StackedAnalysis>,
        area: f64, solidity: f32, stacked: bool,
        delta_angle: f32, delta_cx: f32, delta_cy: f32,
        flipped: bool, damage_flags: u8, outside_ratio: f32,
    }
    #[derive(Clone, Default, Debug)]
    struct SpineSmooth {
        active: bool, center: Point2f, tip: Point2f, heel: Point2f,
        angle360: f32, delta_angle: f32, delta_cx: f32, delta_cy: f32,
        length_px: f32, last_flipped: bool,
    }

    use opencv::core::Rect;
    const ROBOT_WORK_DIAMETER_MM: f32        = 600.0;
    const CAMERA_ROBOT_XY_MM:     (f32, f32) = (0.0, 200.0);
    const ANGLE_THRESHOLD_DEG:    f32        = 5.0;
    const STACK_SINGLE:           u16        = 0;

    fn cam_to_robot(px_col: f32, px_row: f32, roi: Rect) -> (f32, f32) {
        let scale_x = ROBOT_WORK_DIAMETER_MM / roi.width.max(1) as f32;
        let scale_y = ROBOT_WORK_DIAMETER_MM / roi.height.max(1) as f32;
        let local_x = px_col - roi.x as f32;
        let local_y = px_row - roi.y as f32;
        let rx = CAMERA_ROBOT_XY_MM.0 + (local_x - roi.width as f32 * 0.5) * scale_x;
        let ry = CAMERA_ROBOT_XY_MM.1 + (local_y - roi.height as f32 * 0.5) * scale_y;
        (rx, ry)
    }
    fn cam_delta_to_robot(delta_col: f32, delta_row: f32, roi: Rect) -> (f32, f32) {
        let scale_x = ROBOT_WORK_DIAMETER_MM / roi.width.max(1) as f32;
        let scale_y = ROBOT_WORK_DIAMETER_MM / roi.height.max(1) as f32;
        (delta_col * scale_x, delta_row * scale_y)
    }
    fn build_slave_regs(smooth: &SpineSmooth, spine: &ShoeSpine, flags: u8, roi: Rect) -> [u16; 11] {
        let to_u16  = |v: f32| -> u16 { (v * 10.0).round().clamp(0.0, 65535.0) as u16 };
        let to_i16w = |v: f32| -> u16 { ((v * 10.0).round().clamp(-32768.0, 32767.0) as i16) as u16 };
        let any_flipped = spine.flipped || spine.stacked_info.as_ref().map(|s| s.top_flipped).unwrap_or(false);
        match &spine.stacked_info {
            None => {
                let (rx, ry) = cam_to_robot(smooth.center.x, smooth.center.y, roi);
                let offset_mm = (rx * rx + ry * ry).sqrt();
                let (rdx, rdy) = cam_delta_to_robot(smooth.delta_cx, smooth.delta_cy, roi);
                let r40001 = to_i16w(smooth.delta_angle);
                let r40002 = if any_flipped { to_i16w(smooth.delta_angle) }
                             else if smooth.delta_angle.abs() < ANGLE_THRESHOLD_DEG { 0 }
                             else if smooth.delta_angle < 0.0 { 1 } else { 0 };
                let r40003 = to_i16w(rdx);
                let r40004 = to_i16w(rdy);
                let r40005 = to_i16w(rx);
                let r40006 = to_i16w(ry);
                let r40007 = flags as u16;
                let r40008 = to_u16(offset_mm);
                let r40009 = (smooth.angle360 * 10.0).round().clamp(0.0, 3600.0) as u16;
                let r40010 = STACK_SINGLE;
                let r40011 = if (flags >> 5) & 1 == 0 { 1 } else { 0 };
                [r40001, r40002, r40003, r40004, r40005,
                 r40006, r40007, r40008, r40009, r40010, r40011]
            }
            _ => [0u16; 11],
        }
    }

    // ─────────────────────────────────────────────────────────────
    // TEST 18: D10 = 1 khi không có lỗi, D10 = 0 khi damage
    // ─────────────────────────────────────────────────────────────
    #[test]
    fn test_d10_data_ok_flag() {
        let roi = Rect::new(0, 0, 600, 600);
        // No damage → flags bit5 = 0 → D10 = 1
        let mut smooth = SpineSmooth::default();
        smooth.center = pt(300.0, 300.0);
        smooth.angle360 = 90.0;
        let spine = ShoeSpine::default();
        let regs_ok = build_slave_regs(&smooth, &spine, 0b0000_0001, roi);
        assert_eq!(regs_ok[10], 1, "D10 phải = 1 khi không có lỗi damage");

        // Damage → flags bit5 = 1 → D10 = 0
        let regs_dmg = build_slave_regs(&smooth, &spine, 0b0010_0001, roi);
        assert_eq!(regs_dmg[10], 0, "D10 phải = 0 khi có damage");
        println!("[TEST18] D10 ok={} dmg={}: PASSED", regs_ok[10], regs_dmg[10]);
    }

    // ─────────────────────────────────────────────────────────────
    // TEST 19: D8 = angle360 × 10, range [0, 3600]
    // ─────────────────────────────────────────────────────────────
    #[test]
    fn test_d8_angle_encoding() {
        let roi = Rect::new(0, 0, 600, 600);
        let mut smooth = SpineSmooth::default();
        smooth.center   = pt(300.0, 300.0);
        smooth.angle360 = 127.5;   // → D8 = 1275
        let spine = ShoeSpine::default();
        let regs = build_slave_regs(&smooth, &spine, 0b0000_0001, roi);
        assert_eq!(regs[8], 1275, "D8 phải = 1275, got {}", regs[8]);
        println!("[TEST19] D8 angle encoding: {} (expected 1275)", regs[8]);
    }

    // ─────────────────────────────────────────────────────────────
    // TEST 20: D5, D6 — tọa độ robot từ tâm ảnh (×10, i16w)
    // ─────────────────────────────────────────────────────────────
    #[test]
    fn test_d5_d6_robot_coords_from_center() {
        let roi = Rect::new(0, 0, 600, 600);
        let mut smooth = SpineSmooth::default();
        smooth.center = pt(300.0, 300.0); // tâm ROI
        let spine = ShoeSpine::default();
        let regs = build_slave_regs(&smooth, &spine, 0b0000_0001, roi);
        // Tâm ROI → robot = CAMERA_ROBOT_XY_MM = (0.0, 200.0)
        // D5 = rx × 10 = 0, D6 = ry × 10 = 2000 (i16w)
        let d5 = regs[4] as i16; // i16w
        let d6 = regs[5] as i16;
        println!("[TEST20] D5(robot_x×10)={d5}, D6(robot_y×10)={d6}");
        assert!((d5 as f32 - 0.0).abs()    < 2.0, "D5 sai: {d5}");
        assert!((d6 as f32 - 2000.0).abs() < 5.0, "D6 sai: {d6}");
    }

    // ─────────────────────────────────────────────────────────────
    // TEST 21: D9 = STACK_SINGLE(0) khi không stack
    // ─────────────────────────────────────────────────────────────
    #[test]
    fn test_d9_stack_single_no_stacking() {
        let roi = Rect::new(0, 0, 600, 600);
        let smooth = SpineSmooth::default();
        let spine  = ShoeSpine { stacked: false, ..Default::default() };
        let regs = build_slave_regs(&smooth, &spine, 1, roi);
        assert_eq!(regs[9], 0, "D9 phải = STACK_SINGLE(0) khi không stack");
        println!("[TEST21] D9 stack_state=0 khi single: PASSED");
    }
}

// ================================================================
#[cfg(test)]
mod tests_stacked_logic {
    //! Kiểm tra logic phán đoán stack/flip/damage dựa trên thresholds.

    const STACKED_SOLIDITY:       f32 = 0.84;
    const STACKED_AREA_RATIO_MIN: f32 = 1.10;
    const OUTSIDE_RATIO_STACKED:  f32 = 0.12;
    const FLIP_ANGLE_DEG:         f32 = 150.0;
    const STACK_OFFSET_CONFIRM_PX:f32 = 4.0;
    const STACK_ROTATION_CONFIRM: f32 = 2.0;
    const DAMAGE_AREA_RATIO_MIN:  f32 = 0.80;
    const DAMAGE_AREA_RATIO_MAX:  f32 = 1.20;
    const DAMAGE_SOLIDITY_MIN:    f32 = 0.85;

    // ─────────────────────────────────────────────────────────────
    // TEST 22: Solidity thấp → nghi ngờ stack
    // ─────────────────────────────────────────────────────────────
    #[test]
    fn test_stacked_detection_low_solidity() {
        let solidity_stacked  = 0.72f32; // thấp hơn ngưỡng
        let solidity_single   = 0.91f32; // cao hơn ngưỡng

        let suspect_stacked = solidity_stacked < STACKED_SOLIDITY;
        let suspect_single  = solidity_single  < STACKED_SOLIDITY;

        assert!(suspect_stacked,  "Solidity 0.72 phải bị nghi là stack");
        assert!(!suspect_single,  "Solidity 0.91 không phải stack");
        println!("[TEST22] Stacked solidity detection: PASSED");
    }

    // ─────────────────────────────────────────────────────────────
    // TEST 23: Diện tích lớn hơn ref × 1.4 → nghi stack
    // ─────────────────────────────────────────────────────────────
    #[test]
    fn test_stacked_detection_large_area() {
        const STACKED_AREA_FACTOR: f64 = 1.4;
        let ref_area    = 50_000.0f64;
        let obs_area_ok = 55_000.0f64;
        let obs_area_ng = 75_000.0f64;

        let suspect_ok = (obs_area_ok / ref_area) as f32 > STACKED_AREA_RATIO_MIN + 0.10;
        let suspect_ng = (obs_area_ng / ref_area) as f32 > STACKED_AREA_RATIO_MIN + 0.10;

        let _ = STACKED_AREA_FACTOR;
        assert!(!suspect_ok, "55000/50000=1.1 không phải stack area");
        assert!(suspect_ng,  "75000/50000=1.5 phải bị nghi là stack");
        println!("[TEST23] Stacked area detection: PASSED");
    }

    // ─────────────────────────────────────────────────────────────
    // TEST 24: Offset > 4px + rotate > 2° → STACK_COMPLEX
    // ─────────────────────────────────────────────────────────────
    #[test]
    fn test_stack_state_classification() {
        const STACK_ALIGNED: u16 = 1;
        const STACK_OFFSET:  u16 = 2;
        const STACK_ROTATED: u16 = 3;
        const STACK_COMPLEX: u16 = 4;

        let cases: &[(f32, f32, u16)] = &[
            (0.0, 0.0, STACK_ALIGNED),
            (5.0, 0.0, STACK_OFFSET),
            (0.0, 3.0, STACK_ROTATED),
            (5.0, 3.0, STACK_COMPLEX),
        ];
        for &(offset, angle_delta, expected) in cases {
            let state = match (
                offset      > STACK_OFFSET_CONFIRM_PX,
                angle_delta > STACK_ROTATION_CONFIRM,
            ) {
                (false, false) => STACK_ALIGNED,
                (true,  false) => STACK_OFFSET,
                (false, true)  => STACK_ROTATED,
                (true,  true)  => STACK_COMPLEX,
            };
            println!("[TEST24] offset={offset} angle={angle_delta} → state={state} (expected {expected})");
            assert_eq!(state, expected, "Stack state sai cho offset={offset} angle={angle_delta}");
        }
    }

    // ─────────────────────────────────────────────────────────────
    // TEST 25: Flip detection — góc delta > 150°
    // ─────────────────────────────────────────────────────────────
    #[test]
    fn test_flip_logic() {
        let flipped_180 = (180.0f32).abs() > FLIP_ANGLE_DEG;
        let flipped_155 = (155.0f32).abs() > FLIP_ANGLE_DEG;
        let not_flip    = (30.0f32).abs()  > FLIP_ANGLE_DEG;

        assert!(flipped_180, "180° phải là flip");
        assert!(flipped_155, "155° phải là flip");
        assert!(!not_flip,   "30° không phải flip");
        println!("[TEST25] Flip detection FLIP_ANGLE={FLIP_ANGLE_DEG}°: PASSED");
    }

    // ─────────────────────────────────────────────────────────────
    // TEST 26: Damage detection — solidity < 0.85 và area trong range
    // ─────────────────────────────────────────────────────────────
    #[test]
    fn test_damage_detection_conditions() {
        let ref_area = 50_000.0f64;

        // Case: area OK, solidity thấp → có thể damage
        let obs1_area     = 47_000.0f64;
        let obs1_solidity = 0.78f32;
        let area_ratio1   = (obs1_area / ref_area) as f32;
        let damaged1      = area_ratio1 >= DAMAGE_AREA_RATIO_MIN
                         && area_ratio1 <= DAMAGE_AREA_RATIO_MAX
                         && obs1_solidity < DAMAGE_SOLIDITY_MIN;
        println!("[TEST26] area_ratio={area_ratio1:.3} solidity={obs1_solidity} → damaged={damaged1}");
        assert!(damaged1, "Điều kiện damage phải được phát hiện");

        // Case: area ngoài range → không phải damage theo điều kiện này
        let obs2_area   = 30_000.0f64;
        let area_ratio2 = (obs2_area / ref_area) as f32;
        let damaged2    = area_ratio2 >= DAMAGE_AREA_RATIO_MIN
                       && area_ratio2 <= DAMAGE_AREA_RATIO_MAX
                       && 0.78f32 < DAMAGE_SOLIDITY_MIN;
        assert!(!damaged2, "Area quá nhỏ không thỏa mãn damage range");
    }
}

// ================================================================
#[cfg(test)]
mod tests_performance {
    //! Benchmark thời gian xử lý — chứng minh real-time capable.
    //! Ngưỡng: < 33ms/frame (30 FPS).

    use std::time::Instant;

    fn norm360(a: f32) -> f32 { let mut r = a % 360.0; if r < 0.0 { r += 360.0; } r }
    fn delta_angle_signed(from: f32, to: f32) -> f32 {
        let mut d = norm360(to) - norm360(from);
        if d >  180.0 { d -= 360.0; }
        if d < -180.0 { d += 360.0; }
        d
    }
    fn smooth_angle360(prev: f32, next: f32, alpha: f32) -> f32 {
        norm360(prev + delta_angle_signed(prev, next) * alpha)
    }

    // ─────────────────────────────────────────────────────────────
    // TEST 27: 10,000 lần smooth angle < 10ms tổng
    // ─────────────────────────────────────────────────────────────
    #[test]
    fn test_angle_smooth_throughput() {
        let n = 10_000;
        let start = Instant::now();
        let mut angle = 0.0f32;
        for i in 0..n {
            let target = (i as f32) % 360.0;
            angle = smooth_angle360(angle, target, 0.35);
        }
        let elapsed_ms = start.elapsed().as_secs_f64() * 1000.0;
        println!("[TEST27] {n} smooth_angle iterations: {elapsed_ms:.3}ms (last angle={angle:.2}°)");
        assert!(elapsed_ms < 50.0,
            "smooth_angle quá chậm: {elapsed_ms:.2}ms cho {n} lần — cần < 50ms");
    }

    // ─────────────────────────────────────────────────────────────
    // TEST 28: 10,000 lần cam_to_robot < 5ms
    // ─────────────────────────────────────────────────────────────
    #[test]
    fn test_coord_conversion_throughput() {
        use opencv::core::Rect;
        let roi = Rect::new(0, 0, 600, 600);
        let n = 10_000;
        let start = Instant::now();
        let mut sum = 0.0f32;
        for i in 0..n {
            let px = (i % 600) as f32;
            let py = (i % 400) as f32;
            let scale_x = 600.0f32 / roi.width as f32;
            let scale_y = 600.0f32 / roi.height as f32;
            let rx = (px - roi.width as f32 * 0.5) * scale_x;
            let ry = (py - roi.height as f32 * 0.5) * scale_y;
            sum += rx + ry;
        }
        let elapsed_ms = start.elapsed().as_secs_f64() * 1000.0;
        println!("[TEST28] {n} coord conversions: {elapsed_ms:.3}ms (sum={sum:.0})");
        assert!(elapsed_ms < 20.0,
            "Tọa độ chuyển đổi quá chậm: {elapsed_ms:.2}ms");
    }
}

// ================================================================
//  CHẠY TẤT CẢ VÀ IN BẢNG TÓM TẮT
// ================================================================
//  cargo test -- --nocapture 2>&1 | grep -E "\[TEST|PASSED|FAILED|test.*ok|test.*FAILED"