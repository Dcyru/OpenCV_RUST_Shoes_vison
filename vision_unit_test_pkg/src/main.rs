// ================================================================
//  vision_unit_test.rs  — Unit test toàn bộ pipeline xử lý ảnh
//  Chạy: rustc vision_unit_test.rs $(pkg-config --libs --cflags opencv4)
//        ./vision_unit_test <path_to_shoe_image.png>
//  Hoặc dùng Cargo: xem Cargo.toml mẫu ở cuối file
//
//  Mỗi hàm chạy độc lập, lưu ảnh kết quả vào thư mục test_out/
//  Cuối cùng in bảng tổng kết PASS / FAIL cho từng unit.
// ================================================================

use opencv::{
    core::{
        self, Mat, Point, Point2f, Rect, Scalar, Size, BORDER_DEFAULT,
        AlgorithmHint,
    },
    highgui,
    imgcodecs,
    imgproc::{self, CHAIN_APPROX_SIMPLE, RETR_EXTERNAL},
    prelude::*,
    Result,
};
use std::path::Path;
use std::time::Instant;

// ── Copy hằng số từ main.rs ──────────────────────────────────────────
const MIN_SHOE_AREA:        f64 = 8_000.0;
const MAX_SHOE_AREA:        f64 = 250_000.0;
const STACKED_SOLIDITY:     f32 = 0.84;
const STACKED_AREA_FACTOR:  f64 = 1.4;
const STACKED_AREA_RATIO_MIN:f32 = 1.10;
const OUTSIDE_RATIO_STACKED: f32 = 0.12;
const TIP_SLICE_RATIO:      f32 = 0.15;
const ANGLE_THRESHOLD_DEG:  f32 = 5.0;
const FLIP_ANGLE_DEG:       f32 = 150.0;
const RESIDUAL_MIN_AREA:    f64 = 1_500.0;
const REG_DOWNSAMPLE:       i32 = 2;
const REG_SEED_RANGE_PX:    f32 = 60.0;
const REG_INIT_STEP_PX:     f32 = 10.0;
const REG_SEARCH_RANGE_DEG: f32 = 185.0;
const REG_INIT_STEP_DEG:    f32 = 6.0;
const REG_FINE_STEP_PX:     f32 = 1.0;
const REG_FINE_STEP_DEG:    f32 = 0.25;
const REG_HILL_ITERS:       i32 = 50;
const REG_MIN_IOU:          f32 = 0.42;
const STACK_OFFSET_CONFIRM_PX: f32 = 4.0;
const STACK_ROTATION_CONFIRM:  f32 = 2.0;
const DAMAGE_AREA_RATIO_MIN:   f32 = 0.80;
const DAMAGE_AREA_RATIO_MAX:   f32 = 1.20;
const DAMAGE_SOLIDITY_MIN:     f32 = 0.85;

// ── Structs (copy từ main.rs) ─────────────────────────────────────────
#[derive(Debug, Clone, Default, Copy)]
struct InsoleSpine {
    center:    Point2f,
    tip:       Point2f,
    heel:      Point2f,
    spine_vec: Point2f,
    angle360:  f32,
    length_px: f32,
}

#[derive(Debug, Clone, Default)]
struct RefSpine {
    angle360:  f32,
    center:    Point2f,
    tip:       Point2f,
    heel:      Point2f,
    length_px: f32,
    area:      f64,
    spine_vec: Point2f,
}

#[derive(Debug, Clone, Default)]
struct StackedAnalysis {
    top_spine:       InsoleSpine,
    top_angle360:    f32,
    top_angle_delta: f32,
    top_delta_x:     f32,
    top_delta_y:     f32,
    top_offset_px:   f32,
    fit_iou:         f32,
    residual_area:   f64,
    valid:           bool,
    top_flipped:     bool,
    stack_state:     u16,
}

#[derive(Debug, Clone, Default)]
struct ShoeSpine {
    bottom_spine:  InsoleSpine,
    stacked_info:  Option<StackedAnalysis>,
    area:          f64,
    solidity:      f32,
    stacked:       bool,
    delta_angle:   f32,
    delta_cx:      f32,
    delta_cy:      f32,
    flipped:       bool,
    damage_flags:  u8,
    outside_ratio: f32,
}

// ── Kết quả mỗi unit ─────────────────────────────────────────────────
struct UnitResult {
    name:     String,
    pass:     bool,
    elapsed:  u128,   // ms
    detail:   String,
    out_file: Option<String>,
}

// ================================================================
//  COPY HÀM TỪ MAIN.RS (không thay đổi logic)
// ================================================================

fn angle360_from_center_tip(center: Point2f, tip: Point2f) -> f32 {
    let dx = tip.x - center.x;
    let dy = tip.y - center.y;
    let mut a = dy.atan2(dx).to_degrees();
    if a < 0.0 { a += 360.0; }
    a
}

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

fn segment_shoe(img: &Mat, klg: &Mat, ksm: &Mat) -> Result<Mat> {
    let mut gray = Mat::default();
    imgproc::cvt_color(img, &mut gray, imgproc::COLOR_BGR2GRAY, 0,
        AlgorithmHint::ALGO_HINT_DEFAULT)?;
    let mut blur = Mat::default();
    imgproc::gaussian_blur(&gray, &mut blur, Size::new(7,7), 2.0, 2.0,
        BORDER_DEFAULT, AlgorithmHint::ALGO_HINT_DEFAULT)?;
    let mut thr = Mat::default();
    imgproc::threshold(&blur, &mut thr, 0.0, 255.0,
        imgproc::THRESH_BINARY | imgproc::THRESH_OTSU)?;
    let mut cl = Mat::default();
    imgproc::morphology_ex(&thr, &mut cl, imgproc::MORPH_CLOSE,
        klg, Point::new(-1,-1), 3, BORDER_DEFAULT, Scalar::default())?;
    let mut op = Mat::default();
    imgproc::morphology_ex(&cl, &mut op, imgproc::MORPH_OPEN,
        ksm, Point::new(-1,-1), 2, BORDER_DEFAULT, Scalar::default())?;
    let mut cf: core::Vector<core::Vector<Point>> = core::Vector::new();
    imgproc::find_contours(&op, &mut cf,
        RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point::new(0,0))?;
    let mut result = Mat::zeros(op.rows(), op.cols(), core::CV_8U)?.to_mat()?;
    imgproc::draw_contours(&mut result, &cf, -1, Scalar::all(255.0), -1,
        imgproc::LINE_8, &core::no_array(), i32::MAX, Point::new(0,0))?;
    Ok(result)
}

fn centroid(mask: &Mat) -> Result<Option<Point2f>> {
    let m = imgproc::moments(mask, true)?;
    if m.m00 < 1.0 { return Ok(None); }
    Ok(Some(Point2f::new((m.m10/m.m00) as f32, (m.m01/m.m00) as f32)))
}

fn pca_angle_raw(mask: &Mat) -> Result<Option<f32>> {
    let mut pts: Vec<f32> = Vec::new();
    for r in 0..mask.rows() {
        for c in 0..mask.cols() {
            if *mask.at_2d::<u8>(r,c)? > 128 {
                pts.push(c as f32); pts.push(r as f32);
            }
        }
    }
    let n = (pts.len()/2) as i32;
    if n < 10 { return Ok(None); }
    let data = unsafe { Mat::new_rows_cols_with_data_unsafe(
        n, 2, core::CV_32F, pts.as_ptr() as *mut _, core::Mat_AUTO_STEP)? };
    let mut pm = Mat::default(); let mut pe = Mat::default(); let mut pv = Mat::default();
    core::pca_compute2(&data, &mut pm, &mut pe, &mut pv, 2)?;
    let ex = *pe.at_2d::<f32>(0,0)?; let ey = *pe.at_2d::<f32>(0,1)?;
    Ok(Some(ey.atan2(ex).to_degrees()))
}

fn pca_spine_core(
    mask:    &Mat,
    contour: &core::Vector<Point>,
) -> Result<Option<(f32, f32, f32, f32, Point2f, Point2f, f32)>> {
    let mut pts: Vec<f32> = Vec::new();
    for r in 0..mask.rows() {
        for c in 0..mask.cols() {
            if *mask.at_2d::<u8>(r, c)? > 128 {
                pts.push(c as f32);
                pts.push(r as f32);
            }
        }
    }
    let n = (pts.len() / 2) as i32;
    if n < 10 { return Ok(None); }
    let data = unsafe { Mat::new_rows_cols_with_data_unsafe(
        n, 2, core::CV_32F, pts.as_ptr() as *mut _, core::Mat_AUTO_STEP)? };
    let mut pm = Mat::default();
    let mut pe = Mat::default();
    let mut pv = Mat::default();
    core::pca_compute2(&data, &mut pm, &mut pe, &mut pv, 2)?;
    let cx = *pm.at_2d::<f32>(0, 0)?;
    let cy = *pm.at_2d::<f32>(0, 1)?;
    let ex = *pe.at_2d::<f32>(0, 0)?;
    let ey = *pe.at_2d::<f32>(0, 1)?;
    let mut max_p = f32::NEG_INFINITY;
    let mut min_p = f32::INFINITY;
    let mut end_a = Point2f::new(cx, cy);
    let mut end_b = Point2f::new(cx, cy);
    for i in 0..contour.len() {
        let p = contour.get(i)?;
        let dx = p.x as f32 - cx;
        let dy = p.y as f32 - cy;
        let proj = dx * ex + dy * ey;
        if proj > max_p { max_p = proj; end_a = Point2f::new(p.x as f32, p.y as f32); }
        if proj < min_p { min_p = proj; end_b = Point2f::new(p.x as f32, p.y as f32); }
    }
    let spine_len = max_p - min_p;
    Ok(Some((cx, cy, ex, ey, end_a, end_b, spine_len)))
}

fn spine_from_mask(
    mask:          &Mat,
    ref_spine_vec: Point2f,
    roi_offset:    Point2f,
) -> Result<Option<InsoleSpine>> {
    let mut contours: core::Vector<core::Vector<Point>> = core::Vector::new();
    imgproc::find_contours(mask, &mut contours,
        RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point::new(0, 0))?;
    let mut best_c: Option<core::Vector<Point>> = None;
    let mut best_a = 0f64;
    for i in 0..contours.len() {
        let c = contours.get(i)?;
        let a = imgproc::contour_area(&c, false)?;
        if a > best_a { best_a = a; best_c = Some(c); }
    }
    let contour = match best_c {
        Some(c) if best_a > 300.0 => c,
        _ => return Ok(None),
    };
    let (cx, cy, ex, ey, end_a, end_b, _spine_len) =
        match pca_spine_core(mask, &contour)? {
            Some(v) => v,
            None    => return Ok(None),
        };
    let dot = ex * ref_spine_vec.x + ey * ref_spine_vec.y;
    let (tip_l, heel_l) = if dot >= 0.0 { (end_a, end_b) } else { (end_b, end_a) };
    let svx = tip_l.x - heel_l.x;
    let svy = tip_l.y - heel_l.y;
    let svl = (svx*svx + svy*svy).sqrt().max(1.0);
    let off = roi_offset;
    let center_global = Point2f::new(cx + off.x, cy + off.y);
    let tip_global    = Point2f::new(tip_l.x + off.x, tip_l.y + off.y);
    let heel_global   = Point2f::new(heel_l.x + off.x, heel_l.y + off.y);
    let a360 = angle360_from_center_tip(center_global, tip_global);
    Ok(Some(InsoleSpine {
        center:    center_global,
        tip:       tip_global,
        heel:      heel_global,
        spine_vec: Point2f::new(svx / svl, svy / svl),
        angle360:  a360,
        length_px: svl,
    }))
}

fn spine_from_mask_width_vote(
    mask:    &Mat,
    contour: &core::Vector<Point>,
    off:     Point2f,
) -> Result<Option<InsoleSpine>> {
    let (cx, cy, ex, ey, end_a, end_b, spine_len) =
        match pca_spine_core(mask, contour)? {
            Some(v) => v,
            None    => return Ok(None),
        };
    let px = -ey; let py = ex;
    let slice_w = (spine_len * TIP_SLICE_RATIO).max(15.0);
    let width_at = |end: &Point2f| -> f32 {
        let ep = (end.x - cx) * ex + (end.y - cy) * ey;
        let mut pmax = f32::NEG_INFINITY; let mut pmin = f32::INFINITY;
        for i in 0..contour.len() {
            if let Ok(p) = contour.get(i) {
                let dx = p.x as f32 - cx; let dy = p.y as f32 - cy;
                if ((dx * ex + dy * ey) - ep).abs() <= slice_w {
                    let perp = dx * px + dy * py;
                    if perp > pmax { pmax = perp; }
                    if perp < pmin { pmin = perp; }
                }
            }
        }
        if pmax > pmin { pmax - pmin } else { f32::INFINITY }
    };
    let wa = width_at(&end_a); let wb = width_at(&end_b);
    let (tip_l, heel_l) = if wa <= wb { (end_a, end_b) } else { (end_b, end_a) };
    let svx = tip_l.x - heel_l.x; let svy = tip_l.y - heel_l.y;
    let svl = (svx * svx + svy * svy).sqrt().max(1.0);
    let center_global = Point2f::new(cx + off.x, cy + off.y);
    let tip_global    = Point2f::new(tip_l.x + off.x, tip_l.y + off.y);
    let heel_global   = Point2f::new(heel_l.x + off.x, heel_l.y + off.y);
    let a360 = angle360_from_center_tip(center_global, tip_global);
    Ok(Some(InsoleSpine {
        center:    center_global,
        tip:       tip_global,
        heel:      heel_global,
        spine_vec: Point2f::new(svx / svl, svy / svl),
        angle360:  a360,
        length_px: svl,
    }))
}

fn warp_mask(mask: &Mat, dx: f32, dy: f32, ang: f32, cx: f32, cy: f32) -> Result<Mat> {
    let mut rot = imgproc::get_rotation_matrix_2d(
        core::Point2f::new(cx, cy), -ang as f64, 1.0)?;
    *rot.at_2d_mut::<f64>(0,2)? += dx as f64;
    *rot.at_2d_mut::<f64>(1,2)? += dy as f64;
    let mut dst = Mat::zeros(mask.rows(), mask.cols(), core::CV_8U)?.to_mat()?;
    imgproc::warp_affine(mask, &mut dst, &rot,
        Size::new(mask.cols(), mask.rows()),
        imgproc::INTER_NEAREST, core::BORDER_CONSTANT, Scalar::all(0.0))?;
    Ok(dst)
}

// ================================================================
//  HELPER: lưu ảnh kết quả vào thư mục test_out/
// ================================================================
fn save_result(out_dir: &str, name: &str, img: &Mat) -> String {
    let path = format!("{}/{}.png", out_dir, name);
    imgcodecs::imwrite(&path, img, &core::Vector::new()).ok();
    path
}

// Vẽ text nhiều dòng lên ảnh
fn put_lines(img: &mut Mat, lines: &[String], start_y: i32, color: Scalar) -> Result<()> {
    for (i, line) in lines.iter().enumerate() {
        imgproc::put_text(
            img, line,
            Point::new(10, start_y + i as i32 * 22),
            imgproc::FONT_HERSHEY_SIMPLEX, 0.55, color, 1, imgproc::LINE_AA, false,
        )?;
    }
    Ok(())
}

// ================================================================
//  UNIT 01: segment_shoe — Phân đoạn nhị phân (Otsu + morphology)
// ================================================================
fn test_segment(img: &Mat, out_dir: &str) -> Result<UnitResult> {
    let t = Instant::now();
    let klg = imgproc::get_structuring_element(
        imgproc::MORPH_ELLIPSE, Size::new(21,21), Point::new(-1,-1))?;
    let ksm = imgproc::get_structuring_element(
        imgproc::MORPH_ELLIPSE, Size::new(7,7),  Point::new(-1,-1))?;

    let mask = segment_shoe(img, &klg, &ksm)?;
    let area = core::count_non_zero(&mask)? as f64;
    let elapsed = t.elapsed().as_millis();

    // Hiển thị: BGR ảnh gốc + mask overlay xanh
    let mut vis = img.clone();
    let mut colored = Mat::default();
    imgproc::cvt_color(&mask, &mut colored, imgproc::COLOR_GRAY2BGR, 0,
        AlgorithmHint::ALGO_HINT_DEFAULT)?;
    // Tô vùng mask màu xanh bán trong suốt
    let green_mask = Scalar::new(0.0, 180.0, 0.0, 0.0);
    for r in 0..mask.rows() {
        for c in 0..mask.cols() {
            if *mask.at_2d::<u8>(r, c)? > 128 {
                let px = vis.at_2d_mut::<core::Vec3b>(r, c)?;
                px[1] = (px[1] as f32 * 0.5 + 90.0) as u8;
            }
        }
    }
    let pass = area >= MIN_SHOE_AREA && area <= MAX_SHOE_AREA * 2.0;
    let detail = format!("area={:.0}px  pass={}", area, if pass {"OK"} else {"FAIL (area ngoài ngưỡng)"});
    put_lines(&mut vis, &[
        format!("[U01] segment_shoe"),
        format!("Area: {:.0} px", area),
        format!("Range: [{:.0}, {:.0}]", MIN_SHOE_AREA, MAX_SHOE_AREA),
        format!("Status: {}", if pass {"PASS"} else {"FAIL"}),
    ], 25, if pass { Scalar::new(0.,220.,60.,0.) } else { Scalar::new(30.,30.,220.,0.) })?;
    let out = save_result(out_dir, "01_segment", &vis);
    // Lưu thêm mask thuần
    save_result(out_dir, "01_segment_mask", &mask);
    Ok(UnitResult { name: "segment_shoe".into(), pass, elapsed, detail, out_file: Some(out) })
}

// ================================================================
//  UNIT 02: centroid — Tính tọa độ trọng tâm mask
// ================================================================
fn test_centroid(img: &Mat, mask: &Mat, out_dir: &str) -> Result<UnitResult> {
    let t = Instant::now();
    let c = centroid(mask)?;
    let elapsed = t.elapsed().as_millis();
    let mut vis = img.clone();
    let pass = c.is_some();
    let detail = match &c {
        Some(p) => format!("centroid=({:.1},{:.1})", p.x, p.y),
        None    => "centroid=None".into(),
    };
    if let Some(p) = c {
        imgproc::circle(&mut vis, Point::new(p.x as i32, p.y as i32),
            12, Scalar::new(0.,200.,255.,0.), -1, imgproc::LINE_AA, 0)?;
        imgproc::circle(&mut vis, Point::new(p.x as i32, p.y as i32),
            12, Scalar::all(255.), 2, imgproc::LINE_AA, 0)?;
        imgproc::put_text(&mut vis, &format!("C({:.0},{:.0})", p.x, p.y),
            Point::new(p.x as i32 + 15, p.y as i32),
            imgproc::FONT_HERSHEY_SIMPLEX, 0.6, Scalar::new(0.,200.,255.,0.), 2, imgproc::LINE_AA, false)?;
    }
    put_lines(&mut vis, &[
        "[U02] centroid".into(),
        detail.clone(),
        format!("Status: {}", if pass {"PASS"} else {"FAIL"}),
    ], 25, if pass { Scalar::new(0.,220.,60.,0.) } else { Scalar::new(30.,30.,220.,0.) })?;
    let out = save_result(out_dir, "02_centroid", &vis);
    Ok(UnitResult { name: "centroid".into(), pass, elapsed, detail, out_file: Some(out) })
}

// ================================================================
//  UNIT 03: pca_angle_raw — PCA góc thô từ mask
// ================================================================
fn test_pca_angle_raw(img: &Mat, mask: &Mat, out_dir: &str) -> Result<UnitResult> {
    let t = Instant::now();
    let angle = pca_angle_raw(mask)?;
    let elapsed = t.elapsed().as_millis();
    let mut vis = img.clone();
    let pass = angle.is_some();
    let detail = match angle {
        Some(a) => format!("pca_angle_raw={:.2}°", a),
        None    => "pca_angle_raw=None".into(),
    };
    // Vẽ trục PCA
    if let (Some(a), Ok(Some(cen))) = (angle, centroid(mask)) {
        let rad = a.to_radians();
        let len = 80.0f32;
        let x1 = (cen.x - rad.cos()*len) as i32;
        let y1 = (cen.y - rad.sin()*len) as i32;
        let x2 = (cen.x + rad.cos()*len) as i32;
        let y2 = (cen.y + rad.sin()*len) as i32;
        imgproc::line(&mut vis, Point::new(x1,y1), Point::new(x2,y2),
            Scalar::new(255.,80.,0.,0.), 3, imgproc::LINE_AA, 0)?;
        imgproc::circle(&mut vis, Point::new(cen.x as i32, cen.y as i32),
            6, Scalar::new(0.,220.,60.,0.), -1, imgproc::LINE_AA, 0)?;
    }
    put_lines(&mut vis, &[
        "[U03] pca_angle_raw".into(),
        detail.clone(),
        format!("Status: {}", if pass {"PASS"} else {"FAIL"}),
    ], 25, if pass { Scalar::new(0.,220.,60.,0.) } else { Scalar::new(30.,30.,220.,0.) })?;
    let out = save_result(out_dir, "03_pca_angle_raw", &vis);
    Ok(UnitResult { name: "pca_angle_raw".into(), pass, elapsed, detail, out_file: Some(out) })
}

// ================================================================
//  UNIT 04: pca_spine_core — PCA trục chính + 2 đầu cuối
// ================================================================
fn test_pca_spine_core(img: &Mat, mask: &Mat, out_dir: &str)
    -> Result<(UnitResult, Option<(f32,f32,f32,f32,Point2f,Point2f,f32)>)>
{
    let t = Instant::now();
    // Lấy contour lớn nhất
    let mut contours: core::Vector<core::Vector<Point>> = core::Vector::new();
    imgproc::find_contours(mask, &mut contours,
        RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point::new(0,0))?;
    let mut best: Option<core::Vector<Point>> = None;
    let mut ba = 0f64;
    for i in 0..contours.len() {
        let c = contours.get(i)?;
        let a = imgproc::contour_area(&c, false)?;
        if a > ba { ba = a; best = Some(c); }
    }
    let result = match best {
        Some(ref c) => pca_spine_core(mask, c)?,
        None => None,
    };
    let elapsed = t.elapsed().as_millis();
    let mut vis = img.clone();
    let pass = result.is_some();
    let detail = match &result {
        Some((cx,cy,ex,ey,ea,eb,slen)) =>
            format!("cx={:.0} cy={:.0} ex={:.3} ey={:.3} len={:.1}px end_a=({:.0},{:.0}) end_b=({:.0},{:.0})",
                cx, cy, ex, ey, slen, ea.x, ea.y, eb.x, eb.y),
        None => "pca_spine_core=None".into(),
    };
    if let Some((cx,cy,ex,ey,ea,eb,slen)) = &result {
        imgproc::circle(&mut vis, Point::new(*cx as i32, *cy as i32),
            9, Scalar::new(0.,200.,255.,0.), -1, imgproc::LINE_AA, 0)?;
        let l = slen / 2.0;
        imgproc::line(&mut vis,
            Point::new((cx - ex*l) as i32, (cy - ey*l) as i32),
            Point::new((cx + ex*l) as i32, (cy + ey*l) as i32),
            Scalar::new(0.,215.,255.,0.), 2, imgproc::LINE_AA, 0)?;
        imgproc::circle(&mut vis, Point::new(ea.x as i32, ea.y as i32),
            8, Scalar::new(30.,30.,220.,0.), -1, imgproc::LINE_AA, 0)?;
        imgproc::put_text(&mut vis, "A", Point::new(ea.x as i32+10, ea.y as i32),
            imgproc::FONT_HERSHEY_SIMPLEX, 0.6, Scalar::new(30.,30.,220.,0.), 2, imgproc::LINE_AA, false)?;
        imgproc::circle(&mut vis, Point::new(eb.x as i32, eb.y as i32),
            8, Scalar::new(0.,140.,255.,0.), -1, imgproc::LINE_AA, 0)?;
        imgproc::put_text(&mut vis, "B", Point::new(eb.x as i32+10, eb.y as i32),
            imgproc::FONT_HERSHEY_SIMPLEX, 0.6, Scalar::new(0.,140.,255.,0.), 2, imgproc::LINE_AA, false)?;
    }
    put_lines(&mut vis, &[
        "[U04] pca_spine_core".into(),
        format!("Status: {}", if pass {"PASS"} else {"FAIL"}),
    ], 25, if pass { Scalar::new(0.,220.,60.,0.) } else { Scalar::new(30.,30.,220.,0.) })?;
    let out = save_result(out_dir, "04_pca_spine_core", &vis);
    Ok((UnitResult { name: "pca_spine_core".into(), pass, elapsed, detail, out_file: Some(out) }, result))
}

// ================================================================
//  UNIT 05: spine_from_mask_width_vote — Xác định Tip/Heel
// ================================================================
fn test_spine_width_vote(img: &Mat, mask: &Mat, out_dir: &str)
    -> Result<(UnitResult, Option<InsoleSpine>)>
{
    let t = Instant::now();
    let mut contours: core::Vector<core::Vector<Point>> = core::Vector::new();
    imgproc::find_contours(mask, &mut contours,
        RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point::new(0,0))?;
    let mut best: Option<core::Vector<Point>> = None;
    let mut ba = 0f64;
    for i in 0..contours.len() {
        let c = contours.get(i)?;
        let a = imgproc::contour_area(&c, false)?;
        if a >= MIN_SHOE_AREA && a > ba { ba = a; best = Some(c); }
    }
    let spine = match best {
        Some(ref c) => spine_from_mask_width_vote(mask, c, Point2f::new(0.,0.))?,
        None => None,
    };
    let elapsed = t.elapsed().as_millis();
    let mut vis = img.clone();

    // Vẽ contour
    imgproc::draw_contours(&mut vis, &contours, 0,
        Scalar::new(0.,255.,60.,0.), 2, imgproc::LINE_AA,
        &core::no_array(), i32::MAX, Point::new(0,0))?;

    let pass = spine.is_some();
    let detail = match &spine {
        Some(s) => format!(
            "tip=({:.0},{:.0}) heel=({:.0},{:.0}) angle={:.1}° len={:.1}px",
            s.tip.x, s.tip.y, s.heel.x, s.heel.y, s.angle360, s.length_px),
        None => "spine=None".into(),
    };
    if let Some(ref s) = spine {
        let yellow = Scalar::new(0.,215.,255.,0.);
        let red    = Scalar::new(30.,30.,230.,0.);
        let orange = Scalar::new(0.,140.,255.,0.);
        imgproc::line(&mut vis,
            Point::new(s.heel.x as i32, s.heel.y as i32),
            Point::new(s.tip.x  as i32, s.tip.y  as i32),
            yellow, 3, imgproc::LINE_AA, 0)?;
        imgproc::circle(&mut vis, Point::new(s.center.x as i32, s.center.y as i32),
            10, Scalar::new(0.,200.,255.,0.), -1, imgproc::LINE_AA, 0)?;
        imgproc::circle(&mut vis, Point::new(s.tip.x as i32, s.tip.y as i32),
            9, red, -1, imgproc::LINE_AA, 0)?;
        imgproc::put_text(&mut vis, "TIP",
            Point::new(s.tip.x as i32 + 12, s.tip.y as i32 - 6),
            imgproc::FONT_HERSHEY_SIMPLEX, 0.65, red, 2, imgproc::LINE_AA, false)?;
        imgproc::circle(&mut vis, Point::new(s.heel.x as i32, s.heel.y as i32),
            9, orange, -1, imgproc::LINE_AA, 0)?;
        imgproc::put_text(&mut vis, "HEEL",
            Point::new(s.heel.x as i32 + 12, s.heel.y as i32 - 6),
            imgproc::FONT_HERSHEY_SIMPLEX, 0.65, orange, 2, imgproc::LINE_AA, false)?;
        put_lines(&mut vis, &[
            "[U05] spine_from_mask_width_vote".into(),
            format!("Angle: {:.1}°  Len: {:.1}px", s.angle360, s.length_px),
            "Tip = đầu hẹp (Width Vote), Heel = đầu rộng".into(),
            "Status: PASS".into(),
        ], 25, Scalar::new(0.,220.,60.,0.))?;
    } else {
        put_lines(&mut vis, &[
            "[U05] spine_from_mask_width_vote".into(),
            "Status: FAIL — không tìm được spine".into(),
        ], 25, Scalar::new(30.,30.,220.,0.))?;
    }
    let out = save_result(out_dir, "05_spine_width_vote", &vis);
    Ok((UnitResult { name: "spine_from_mask_width_vote".into(), pass, elapsed, detail, out_file: Some(out) }, spine))
}

// ================================================================
//  UNIT 06: compute_shoe — Solidity, stacked flag, delta, damage
// ================================================================
fn test_compute_shoe(
    img:        &Mat,
    mask:       &Mat,
    ref_spine:  &RefSpine,
    ref_mask:   &Mat,
    out_dir:    &str,
) -> Result<(UnitResult, Option<ShoeSpine>)> {
    let t = Instant::now();
    let mut contours: core::Vector<core::Vector<Point>> = core::Vector::new();
    imgproc::find_contours(mask, &mut contours,
        RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point::new(0,0))?;
    let mut best: Option<core::Vector<Point>> = None;
    let mut ba = 0f64;
    for i in 0..contours.len() {
        let c = contours.get(i)?;
        let a = imgproc::contour_area(&c, false)?;
        if a >= MIN_SHOE_AREA && a > ba { ba = a; best = Some(c); }
    }
    let roi_offset = Point::new(0, 0);

    let shoe = match best {
        Some(ref contour) => {
            let area = ba;
            let mut hi: core::Vector<i32> = core::Vector::new();
            imgproc::convex_hull(contour, &mut hi, false, false)?;
            let mut hp: core::Vector<Point> = core::Vector::new();
            for i in 0..hi.len() { hp.push(contour.get(hi.get(i)? as usize)?); }
            let hull_area = imgproc::contour_area(&hp, false)?;
            let solidity  = if hull_area > 0.0 { (area/hull_area) as f32 } else { 1.0 };
            let area_ratio = (area / ref_spine.area.max(1.0)) as f32;

            let outside_ratio = if ref_mask.rows() == mask.rows() && ref_mask.cols() == mask.cols() {
                let mut not_ref = Mat::default();
                core::bitwise_not(ref_mask, &mut not_ref, &core::no_array())?;
                let mut outside = Mat::default();
                core::bitwise_and(mask, &not_ref, &mut outside, &core::no_array())?;
                let outside_px = core::count_non_zero(&outside)? as f32;
                if area > 0.0 { outside_px / area as f32 } else { 0.0 }
            } else { 0.0 };

            let stacked = solidity < STACKED_SOLIDITY
                || area > MAX_SHOE_AREA * STACKED_AREA_FACTOR
                || area_ratio > STACKED_AREA_RATIO_MIN
                || (outside_ratio > OUTSIDE_RATIO_STACKED && area_ratio > 1.08);

            let off2f = Point2f::new(0., 0.);
            let bottom_spine = if !stacked {
                spine_from_mask(mask, ref_spine.spine_vec, off2f)?
                    .unwrap_or(InsoleSpine {
                        center: ref_spine.center, tip: ref_spine.tip, heel: ref_spine.heel,
                        spine_vec: ref_spine.spine_vec, angle360: ref_spine.angle360,
                        length_px: ref_spine.length_px,
                    })
            } else {
                InsoleSpine {
                    center: ref_spine.center, tip: ref_spine.tip, heel: ref_spine.heel,
                    spine_vec: ref_spine.spine_vec, angle360: ref_spine.angle360,
                    length_px: ref_spine.length_px,
                }
            };

            let (blob_cx, blob_cy) = if !stacked {
                (bottom_spine.center.x, bottom_spine.center.y)
            } else {
                let m = imgproc::moments(mask, true)?;
                if m.m00 > 0.0 {
                    ((m.m10/m.m00) as f32 + off2f.x, (m.m01/m.m00) as f32 + off2f.y)
                } else { (ref_spine.center.x, ref_spine.center.y) }
            };
            let delta_cx = blob_cx - ref_spine.center.x;
            let delta_cy = blob_cy - ref_spine.center.y;

            let (delta_angle, flipped) = if !stacked {
                let da = delta_angle_signed(ref_spine.angle360, bottom_spine.angle360);
                let fl = da.abs() > FLIP_ANGLE_DEG;
                (da, fl)
            } else {
                (0.0f32, false)
            };

            let damage_flags: u8 = {
                let area_ok = area_ratio >= DAMAGE_AREA_RATIO_MIN && area_ratio <= DAMAGE_AREA_RATIO_MAX;
                let sol_ok  = solidity >= DAMAGE_SOLIDITY_MIN;
                let mut f = 0u8;
                if !area_ok { f |= 0b001; }
                if !sol_ok  { f |= 0b010; }
                f
            };

            Some(ShoeSpine {
                bottom_spine, stacked_info: None,
                area, solidity, stacked, delta_angle, delta_cx, delta_cy,
                flipped, damage_flags, outside_ratio,
            })
        }
        None => None,
    };

    let elapsed = t.elapsed().as_millis();
    let mut vis = img.clone();
    let pass = shoe.is_some();
    let detail = match &shoe {
        Some(s) => format!(
            "area={:.0} solidity={:.3} stacked={} flipped={} dAngle={:.1}° dX={:.1} dY={:.1} damage={:03b} outside={:.2}",
            s.area, s.solidity, s.stacked, s.flipped, s.delta_angle,
            s.delta_cx, s.delta_cy, s.damage_flags, s.outside_ratio),
        None => "shoe=None".into(),
    };

    if let Some(ref s) = shoe {
        let color_main = if s.stacked { Scalar::new(0.,100.,255.,0.) }
                         else if s.flipped { Scalar::new(200.,50.,150.,0.) }
                         else { Scalar::new(0.,220.,60.,0.) };
        imgproc::draw_contours(&mut vis, &contours, 0,
            color_main, 2, imgproc::LINE_AA, &core::no_array(), i32::MAX, Point::new(0,0))?;
        let b = &s.bottom_spine;
        imgproc::line(&mut vis,
            Point::new(b.heel.x as i32, b.heel.y as i32),
            Point::new(b.tip.x  as i32, b.tip.y  as i32),
            Scalar::new(0.,215.,255.,0.), 2, imgproc::LINE_AA, 0)?;
        imgproc::circle(&mut vis, Point::new(b.center.x as i32, b.center.y as i32),
            10, color_main, -1, imgproc::LINE_AA, 0)?;

        let status_label = if s.stacked { "STACKED" }
                           else if s.flipped { "FLIPPED" }
                           else { "NORMAL" };
        let dmg = if s.damage_flags != 0 { format!(" DMG={:03b}", s.damage_flags) } else { String::new() };
        put_lines(&mut vis, &[
            format!("[U06] compute_shoe — {}{}", status_label, dmg),
            format!("Solidity:{:.3} Area:{:.0} Outside:{:.2}", s.solidity, s.area, s.outside_ratio),
            format!("dAngle:{:.1}° dX:{:.1} dY:{:.1}", s.delta_angle, s.delta_cx, s.delta_cy),
            "Status: PASS".into(),
        ], 25, color_main)?;
    } else {
        put_lines(&mut vis, &["[U06] compute_shoe — FAIL".into()],
            25, Scalar::new(30.,30.,220.,0.))?;
    }
    let out = save_result(out_dir, "06_compute_shoe", &vis);
    Ok((UnitResult { name: "compute_shoe".into(), pass, elapsed, detail, out_file: Some(out) }, shoe))
}

// ================================================================
//  UNIT 07: analyze_stacked_v9 — Tìm lót trên (top) khi xếp chồng
//  (Chạy với chính ảnh input làm cả observed lẫn ref — demo pipeline)
// ================================================================
fn test_analyze_stacked(
    img:       &Mat,
    mask:      &Mat,
    ref_spine: &RefSpine,
    ref_mask:  &Mat,
    out_dir:   &str,
) -> Result<UnitResult> {
    let t = Instant::now();
    // Tạo "observed" mask = mask gốc (giả lập trường hợp xếp chồng)
    let roi_offset = Point::new(0, 0);
    let result = analyze_stacked_v9(mask, ref_mask, roi_offset, ref_spine);
    let elapsed = t.elapsed().as_millis();
    let mut vis = img.clone();
    let (pass, detail) = match &result {
        Ok(sa) => {
            let d = format!(
                "valid={} iou={:.3} offset={:.1}px dAngle={:.1}° flipped={} stack_state={} residual={:.0}px²",
                sa.valid, sa.fit_iou, sa.top_offset_px, sa.top_angle_delta,
                sa.top_flipped, sa.stack_state, sa.residual_area);
            if sa.valid {
                let ts = &sa.top_spine;
                let sky  = Scalar::new(255.,200.,50.,0.);
                let mint = Scalar::new(180.,255.,150.,0.);
                let purp = Scalar::new(200.,50.,150.,0.);
                imgproc::line(&mut vis,
                    Point::new(ts.heel.x as i32, ts.heel.y as i32),
                    Point::new(ts.tip.x  as i32, ts.tip.y  as i32),
                    sky, 3, imgproc::LINE_AA, 0)?;
                imgproc::circle(&mut vis, Point::new(ts.center.x as i32, ts.center.y as i32),
                    10, purp, -1, imgproc::LINE_AA, 0)?;
                imgproc::circle(&mut vis, Point::new(ts.tip.x as i32, ts.tip.y as i32),
                    8, sky, -1, imgproc::LINE_AA, 0)?;
                imgproc::put_text(&mut vis, "T2",
                    Point::new(ts.tip.x as i32+10, ts.tip.y as i32-6),
                    imgproc::FONT_HERSHEY_SIMPLEX, 0.6, sky, 2, imgproc::LINE_AA, false)?;
                imgproc::circle(&mut vis, Point::new(ts.heel.x as i32, ts.heel.y as i32),
                    8, mint, -1, imgproc::LINE_AA, 0)?;
                imgproc::put_text(&mut vis, "H2",
                    Point::new(ts.heel.x as i32+10, ts.heel.y as i32-6),
                    imgproc::FONT_HERSHEY_SIMPLEX, 0.6, mint, 2, imgproc::LINE_AA, false)?;
            }
            put_lines(&mut vis, &[
                "[U07] analyze_stacked_v9".into(),
                format!("valid={}  IoU={:.3}  offset={:.1}px", sa.valid, sa.fit_iou, sa.top_offset_px),
                format!("dAngle={:.1}°  state={}  flipped={}", sa.top_angle_delta, sa.stack_state, sa.top_flipped),
                format!("Status: {}", if sa.valid {"PASS (stacked detected)"} else {"PASS (không stacked)"}),
            ], 25, Scalar::new(0.,220.,60.,0.))?;
            (true, d)
        }
        Err(e) => {
            let d = format!("ERROR: {:?}", e);
            put_lines(&mut vis, &["[U07] analyze_stacked_v9 — ERROR".into(), d.clone()],
                25, Scalar::new(30.,30.,220.,0.))?;
            (false, d)
        }
    };
    let out = save_result(out_dir, "07_analyze_stacked", &vis);
    Ok(UnitResult { name: "analyze_stacked_v9".into(), pass, elapsed, detail, out_file: Some(out) })
}

// ================================================================
//  UNIT 08: warp_mask — Xoay/dịch mask (kiểm tra affine transform)
// ================================================================
fn test_warp_mask(img: &Mat, ref_mask: &Mat, out_dir: &str) -> Result<UnitResult> {
    let t = Instant::now();
    let cx = ref_mask.cols() as f32 / 2.0;
    let cy = ref_mask.rows() as f32 / 2.0;
    // Test 3 trường hợp: tịnh tiến, xoay, kết hợp
    let w1 = warp_mask(ref_mask, 30.0, 20.0, 0.0, cx, cy)?;
    let w2 = warp_mask(ref_mask,  0.0,  0.0, 25.0, cx, cy)?;
    let w3 = warp_mask(ref_mask, 15.0, -15.0, 15.0, cx, cy)?;
    let elapsed = t.elapsed().as_millis();

    let a1 = core::count_non_zero(&w1)? as f64;
    let a0 = core::count_non_zero(ref_mask)? as f64;
    let pass = a1 > 0.0 && (a1 / a0.max(1.0)) > 0.5;

    // Tạo ảnh 3 cột để so sánh
    let h = ref_mask.rows(); let w = ref_mask.cols();
    let mut canvas = Mat::zeros(h, w * 3 + 6, core::CV_8UC3)?.to_mat()?;
    let masks = [(&w1, "dx=30 dy=20"), (&w2, "rot=25°"), (&w3, "dx=15 dy=-15 rot=15°")];
    for (i, (m, label)) in masks.iter().enumerate() {
        let mut col = Mat::default();
        imgproc::cvt_color(m, &mut col, imgproc::COLOR_GRAY2BGR, 0, AlgorithmHint::ALGO_HINT_DEFAULT)?;
        let x_off = i as i32 * (w + 2);
        let roi = Rect::new(x_off, 0, w, h);
        let mut dst = Mat::roi_mut(&mut canvas, roi)?;
        col.copy_to(&mut dst)?;
        imgproc::put_text(&mut canvas, label,
            Point::new(x_off + 5, h - 10),
            imgproc::FONT_HERSHEY_SIMPLEX, 0.5, Scalar::new(0.,215.,255.,0.), 1, imgproc::LINE_AA, false)?;
    }
    imgproc::put_text(&mut canvas, "[U08] warp_mask",
        Point::new(5, 20), imgproc::FONT_HERSHEY_SIMPLEX, 0.6,
        if pass {Scalar::new(0.,220.,60.,0.)} else {Scalar::new(30.,30.,220.,0.)}, 2, imgproc::LINE_AA, false)?;

    let detail = format!("ref_area={:.0}px w1_area={:.0}px ratio={:.2}", a0, a1, a1/a0.max(1.0));
    let out = save_result(out_dir, "08_warp_mask", &canvas);
    Ok(UnitResult { name: "warp_mask".into(), pass, elapsed, detail, out_file: Some(out) })
}

// ================================================================
//  UNIT 09: compute_ref_spine — Tính RefSpine từ ảnh mẫu
// ================================================================
fn test_compute_ref_spine(img: &Mat, out_dir: &str) -> Result<(UnitResult, Option<(RefSpine, Mat)>)> {
    let t = Instant::now();
    let klg = imgproc::get_structuring_element(
        imgproc::MORPH_ELLIPSE, Size::new(21,21), Point::new(-1,-1))?;
    let ksm = imgproc::get_structuring_element(
        imgproc::MORPH_ELLIPSE, Size::new(7,7),  Point::new(-1,-1))?;
    let roi = Rect::new(0, 0, img.cols(), img.rows());
    let result = compute_ref_spine_inner(img, &roi, &klg, &ksm)?;
    let elapsed = t.elapsed().as_millis();
    let mut vis = img.clone();
    let pass = result.is_some();
    let detail = match &result {
        Some((rs, _)) => format!(
            "tip=({:.0},{:.0}) heel=({:.0},{:.0}) angle={:.1}° len={:.1}px area={:.0}",
            rs.tip.x, rs.tip.y, rs.heel.x, rs.heel.y, rs.angle360, rs.length_px, rs.area),
        None => "ref_spine=None".into(),
    };
    if let Some((rs, ref ref_mask)) = &result {
        imgproc::draw_contours(&mut vis,
            &{
                let mut contours: core::Vector<core::Vector<Point>> = core::Vector::new();
                imgproc::find_contours(ref_mask, &mut contours,
                    RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point::new(0,0))?;
                contours
            }, 0, Scalar::new(0.,255.,60.,0.), 2, imgproc::LINE_AA,
            &core::no_array(), i32::MAX, Point::new(0,0))?;
        imgproc::line(&mut vis,
            Point::new(rs.heel.x as i32, rs.heel.y as i32),
            Point::new(rs.tip.x  as i32, rs.tip.y  as i32),
            Scalar::new(0.,215.,255.,0.), 3, imgproc::LINE_AA, 0)?;
        imgproc::circle(&mut vis, Point::new(rs.center.x as i32, rs.center.y as i32),
            11, Scalar::new(0.,200.,255.,0.), -1, imgproc::LINE_AA, 0)?;
        imgproc::circle(&mut vis, Point::new(rs.tip.x as i32, rs.tip.y as i32),
            9, Scalar::new(30.,30.,220.,0.), -1, imgproc::LINE_AA, 0)?;
        imgproc::put_text(&mut vis, "TIP",
            Point::new(rs.tip.x as i32 + 12, rs.tip.y as i32 - 6),
            imgproc::FONT_HERSHEY_SIMPLEX, 0.65, Scalar::new(30.,30.,220.,0.), 2, imgproc::LINE_AA, false)?;
        imgproc::circle(&mut vis, Point::new(rs.heel.x as i32, rs.heel.y as i32),
            9, Scalar::new(0.,140.,255.,0.), -1, imgproc::LINE_AA, 0)?;
        imgproc::put_text(&mut vis, "HEEL",
            Point::new(rs.heel.x as i32 + 12, rs.heel.y as i32 - 6),
            imgproc::FONT_HERSHEY_SIMPLEX, 0.65, Scalar::new(0.,140.,255.,0.), 2, imgproc::LINE_AA, false)?;
        put_lines(&mut vis, &[
            "[U09] compute_ref_spine (width_vote)".into(),
            format!("angle={:.1}°  len={:.1}px  area={:.0}", rs.angle360, rs.length_px, rs.area),
            "Status: PASS".into(),
        ], 25, Scalar::new(0.,220.,60.,0.))?;
    } else {
        put_lines(&mut vis, &["[U09] compute_ref_spine — FAIL".into()],
            25, Scalar::new(30.,30.,220.,0.))?;
    }
    let out = save_result(out_dir, "09_compute_ref_spine", &vis);
    Ok((UnitResult { name: "compute_ref_spine".into(), pass, elapsed, detail, out_file: Some(out) }, result))
}

// Hàm nội bộ tương tự compute_ref_spine trong main.rs
fn compute_ref_spine_inner(ref_img: &Mat, roi: &Rect, klg: &Mat, ksm: &Mat)
    -> Result<Option<(RefSpine, Mat)>>
{
    let mask = segment_shoe(ref_img, klg, ksm)?;
    let mut contours: core::Vector<core::Vector<Point>> = core::Vector::new();
    imgproc::find_contours(&mask, &mut contours,
        RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point::new(0,0))?;
    let mut best: Option<core::Vector<Point>> = None;
    let mut ba = 0f64;
    for i in 0..contours.len() {
        let c = contours.get(i)?;
        let a = imgproc::contour_area(&c, false)?;
        if a > ba && a >= MIN_SHOE_AREA { ba = a; best = Some(c); }
    }
    let contour = match best { Some(c) => c, None => return Ok(None) };
    let off = Point2f::new(roi.x as f32, roi.y as f32);
    let spine_opt = spine_from_mask_width_vote(&mask, &contour, off)?;
    Ok(spine_opt.map(|s| {
        let rs = RefSpine {
            angle360: s.angle360, center: s.center, tip: s.tip, heel: s.heel,
            length_px: s.length_px, area: ba, spine_vec: s.spine_vec,
        };
        (rs, mask)
    }))
}

// ================================================================
//  UNIT 10: full pipeline — Chạy toàn bộ như main loop
// ================================================================
fn test_full_pipeline(img: &Mat, out_dir: &str) -> Result<UnitResult> {
    let t = Instant::now();
    let klg = imgproc::get_structuring_element(
        imgproc::MORPH_ELLIPSE, Size::new(21,21), Point::new(-1,-1))?;
    let ksm = imgproc::get_structuring_element(
        imgproc::MORPH_ELLIPSE, Size::new(7,7),  Point::new(-1,-1))?;
    let roi = Rect::new(0, 0, img.cols(), img.rows());

    // 1. Tính ref spine từ chính ảnh này (giả sử là ảnh mẫu)
    let (ref_spine, ref_mask_mat) = match compute_ref_spine_inner(img, &roi, &klg, &ksm)? {
        Some(v) => v,
        None => {
            return Ok(UnitResult {
                name: "full_pipeline".into(), pass: false,
                elapsed: t.elapsed().as_millis(),
                detail: "Không tính được ref_spine".into(), out_file: None,
            });
        }
    };

    // 2. Segment observed (cùng ảnh, mô phỏng tracking)
    let mask = segment_shoe(img, &klg, &ksm)?;
    let mut contours: core::Vector<core::Vector<Point>> = core::Vector::new();
    imgproc::find_contours(&mask, &mut contours,
        RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point::new(0,0))?;
    let mut best: Option<core::Vector<Point>> = None;
    let mut ba = 0f64;
    for i in 0..contours.len() {
        let c = contours.get(i)?;
        let a = imgproc::contour_area(&c, false)?;
        if a >= MIN_SHOE_AREA && a > ba { ba = a; best = Some(c); }
    }

    let elapsed = t.elapsed().as_millis();
    let mut vis = img.clone();

    // 3. Vẽ tất cả kết quả lên ảnh tổng
    let pass = best.is_some();
    let detail;
    if let Some(contour) = best {
        // Vẽ contour
        imgproc::draw_contours(&mut vis,
            &core::Vector::<core::Vector<Point>>::from_iter(std::iter::once(contour.clone())),
            0, Scalar::new(0.,255.,60.,0.), 2, imgproc::LINE_AA,
            &core::no_array(), i32::MAX, Point::new(0,0))?;

        // Spine
        let spine = spine_from_mask(&mask, ref_spine.spine_vec, Point2f::new(0.,0.))?;
        if let Some(ref s) = spine {
            imgproc::line(&mut vis,
                Point::new(s.heel.x as i32, s.heel.y as i32),
                Point::new(s.tip.x  as i32, s.tip.y  as i32),
                Scalar::new(0.,215.,255.,0.), 2, imgproc::LINE_AA, 0)?;
            imgproc::circle(&mut vis, Point::new(s.center.x as i32, s.center.y as i32),
                10, Scalar::new(0.,200.,60.,0.), -1, imgproc::LINE_AA, 0)?;
        }
        // Ref spine (xám)
        imgproc::line(&mut vis,
            Point::new(ref_spine.heel.x as i32, ref_spine.heel.y as i32),
            Point::new(ref_spine.tip.x  as i32, ref_spine.tip.y  as i32),
            Scalar::new(140.,140.,140.,0.), 1, imgproc::LINE_AA, 0)?;

        let da = spine.as_ref().map(|s| delta_angle_signed(ref_spine.angle360, s.angle360)).unwrap_or(0.0);
        let dcx = spine.as_ref().map(|s| s.center.x - ref_spine.center.x).unwrap_or(0.0);
        let dcy = spine.as_ref().map(|s| s.center.y - ref_spine.center.y).unwrap_or(0.0);

        detail = format!(
            "area={:.0} dAngle={:.1}° dCx={:.1} dCy={:.1}",
            ba, da, dcx, dcy);
        put_lines(&mut vis, &[
            "[U10] FULL PIPELINE".into(),
            format!("area={:.0} dAngle={:.1}° dX={:.1} dY={:.1}", ba, da, dcx, dcy),
            format!("ref_angle={:.1}° len={:.1}px", ref_spine.angle360, ref_spine.length_px),
            "Status: PASS".into(),
        ], 25, Scalar::new(0.,220.,60.,0.))?;
    } else {
        detail = "Không phát hiện lót".into();
        put_lines(&mut vis, &["[U10] FULL PIPELINE — FAIL".into(), detail.clone()],
            25, Scalar::new(30.,30.,220.,0.))?;
    }

    let out = save_result(out_dir, "10_full_pipeline", &vis);
    Ok(UnitResult { name: "full_pipeline".into(), pass, elapsed, detail, out_file: Some(out) })
}

// ================================================================
//  MAIN
// ================================================================
fn main() -> Result<()> {
    let args: Vec<String> = std::env::args().collect();
    if args.len() < 2 {
        eprintln!("Dùng: {} <path_to_shoe_image>", args[0]);
        eprintln!("  Ví dụ: {} lot_giay.jpg", args[0]);
        std::process::exit(1);
    }
    let img_path = &args[1];

    // Tạo thư mục output
    let out_dir = "test_out";
    std::fs::create_dir_all(out_dir).expect("Không tạo được test_out/");

    println!("╔═══════════════════════════════════════════════════════════╗");
    println!("║   Shoe Stack Vision — Unit Test Runner                   ║");
    println!("╠═══════════════════════════════════════════════════════════╣");
    println!("║  Ảnh đầu vào: {:44}║", img_path);
    println!("║  Kết quả lưu vào: test_out/                              ║");
    println!("╚═══════════════════════════════════════════════════════════╝\n");

    // Đọc ảnh
    let img = imgcodecs::imread(img_path, imgcodecs::IMREAD_COLOR)?;
    if img.empty() {
        eprintln!("Lỗi: Không đọc được ảnh '{}'", img_path);
        std::process::exit(1);
    }
    println!("Ảnh: {}×{} pixels\n", img.cols(), img.rows());

    // Chuẩn bị kernels một lần
    let klg = imgproc::get_structuring_element(
        imgproc::MORPH_ELLIPSE, Size::new(21,21), Point::new(-1,-1))?;
    let ksm = imgproc::get_structuring_element(
        imgproc::MORPH_ELLIPSE, Size::new(7,7),  Point::new(-1,-1))?;

    // Tính mask + ref_spine chung cho các unit cần
    let mask = segment_shoe(&img, &klg, &ksm)?;
    let roi  = Rect::new(0, 0, img.cols(), img.rows());
    let ref_result = compute_ref_spine_inner(&img, &roi, &klg, &ksm)?;
    let (ref_spine, ref_mask) = ref_result.unwrap_or_else(|| {
        println!("[WARN] Không tính được ref_spine — dùng mặc định trống");
        let dummy_rs = RefSpine {
            angle360: 0.0, center: Point2f::new(img.cols() as f32 / 2.0, img.rows() as f32 / 2.0),
            tip: Point2f::new(img.cols() as f32 * 0.7, img.rows() as f32 / 2.0),
            heel: Point2f::new(img.cols() as f32 * 0.3, img.rows() as f32 / 2.0),
            length_px: img.cols() as f32 * 0.4, area: 50_000.0,
            spine_vec: Point2f::new(1.0, 0.0),
        };
        (dummy_rs, mask.clone())
    });

    // ── Chạy từng unit ──────────────────────────────────────────────
    let mut results: Vec<UnitResult> = Vec::new();

    println!("Chạy unit tests...\n");

    results.push(test_segment(&img, out_dir)?);
    results.push(test_centroid(&img, &mask, out_dir)?);
    results.push(test_pca_angle_raw(&img, &mask, out_dir)?);
    let (r4, _) = test_pca_spine_core(&img, &mask, out_dir)?;
    results.push(r4);
    let (r5, _) = test_spine_width_vote(&img, &mask, out_dir)?;
    results.push(r5);
    let (r6, _) = test_compute_shoe(&img, &mask, &ref_spine, &ref_mask, out_dir)?;
    results.push(r6);
    results.push(test_analyze_stacked(&img, &mask, &ref_spine, &ref_mask, out_dir)?);
    results.push(test_warp_mask(&img, &ref_mask, out_dir)?);
    let (r9, _) = test_compute_ref_spine(&img, out_dir)?;
    results.push(r9);
    results.push(test_full_pipeline(&img, out_dir)?);

    // ── In bảng kết quả ─────────────────────────────────────────────
    println!("\n╔═══════════════════════════════════════════════════════════════════════════════╗");
    println!("║                         KẾT QUẢ UNIT TEST                                  ║");
    println!("╠════╦══════════════════════════════════════╦════════╦══════════╦═════════════╣");
    println!("║  # ║ Unit                                 ║ Kết quả║ Time(ms) ║ Output      ║");
    println!("╠════╬══════════════════════════════════════╬════════╬══════════╬═════════════╣");
    let mut pass_count = 0;
    for (i, r) in results.iter().enumerate() {
        let status = if r.pass { "✓ PASS" } else { "✗ FAIL" };
        let fname = r.out_file.as_deref()
            .map(|p| Path::new(p).file_name().unwrap_or_default().to_str().unwrap_or(""))
            .unwrap_or("");
        println!("║ {:2} ║ {:<36} ║ {:<6} ║ {:>8} ║ {:<11} ║",
            i+1, &r.name[..r.name.len().min(36)], status, r.elapsed, &fname[..fname.len().min(11)]);
        if r.pass { pass_count += 1; }
    }
    println!("╠════╩══════════════════════════════════════╩════════╩══════════╩═════════════╣");
    let all_pass = pass_count == results.len();
    println!("║  Tổng:  {} / {} PASS   {}                                            ║",
        pass_count, results.len(),
        if all_pass { "🎉 TẤT CẢ ĐẠT" } else { "⚠️  CÓ UNIT FAIL" });
    println!("╚═══════════════════════════════════════════════════════════════════════════════╝\n");

    println!("Chi tiết từng unit:");
    for (i, r) in results.iter().enumerate() {
        println!("  [{:02}] {}: {}", i+1, r.name, r.detail);
    }

    println!("\nẢnh kết quả đã lưu vào: {}/", out_dir);
    println!("  Mở bằng: eog test_out/  (Linux)");
    println!("           open test_out/  (macOS)");
    println!("           explorer test_out\\  (Windows)\n");

    Ok(())
}

// ================================================================
//  analyze_stacked_v9 — copy đầy đủ từ main.rs
// ================================================================
fn analyze_stacked_v9(
    mask_obs:   &Mat,
    ref_mask:   &Mat,
    roi_offset: Point,
    rs:         &RefSpine,
) -> Result<StackedAnalysis> {
    let off    = Point2f::new(roi_offset.x as f32, roi_offset.y as f32);
    let ref_cx = rs.center.x - off.x;
    let ref_cy = rs.center.y - off.y;
    let obs_area = core::count_non_zero(mask_obs)? as f32;
    if obs_area < 1.0 { return Ok(StackedAnalysis::default()); }

    let mut not_ref = Mat::default();
    core::bitwise_not(ref_mask, &mut not_ref, &core::no_array())?;
    let mut residual = Mat::default();
    core::bitwise_and(mask_obs, &not_ref, &mut residual, &core::no_array())?;
    let residual_area = core::count_non_zero(&residual)? as f64;
    let use_residual_cost = residual_area > RESIDUAL_MIN_AREA;

    let (seed_dx, seed_dy) = if use_residual_cost {
        match centroid(&residual)? {
            Some(rc) => (rc.x - ref_cx, rc.y - ref_cy),
            None => match centroid(mask_obs)? {
                Some(bc) => (bc.x - ref_cx, bc.y - ref_cy),
                None => (0.0f32, 0.0f32),
            },
        }
    } else {
        match centroid(mask_obs)? {
            Some(bc) => (bc.x - ref_cx, bc.y - ref_cy),
            None => (0.0f32, 0.0f32),
        }
    };

    let seed_ang = if residual_area > RESIDUAL_MIN_AREA * 3.0 {
        match pca_angle_raw(&residual)? {
            Some(ra) => delta_angle_signed(rs.angle360 - 90.0, ra),
            None => 0.0,
        }
    } else { 0.0 };

    let ds = REG_DOWNSAMPLE;
    let small_sz = Size::new((ref_mask.cols() / ds).max(1), (ref_mask.rows() / ds).max(1));
    let mut ref_small = Mat::default();
    imgproc::resize(ref_mask, &mut ref_small, small_sz, 0.0, 0.0, imgproc::INTER_NEAREST)?;
    let mut residual_small = Mat::default();
    imgproc::resize(&residual, &mut residual_small, small_sz, 0.0, 0.0, imgproc::INTER_NEAREST)?;
    let mut obs_small = Mat::default();
    imgproc::resize(mask_obs, &mut obs_small, small_sz, 0.0, 0.0, imgproc::INTER_NEAREST)?;

    let ref_cx_s = ref_cx / ds as f32;
    let ref_cy_s = ref_cy / ds as f32;

    let cost_small = |dx: f32, dy: f32, da: f32| -> Result<i32> {
        let top = warp_mask(&ref_small, dx/ds as f32, dy/ds as f32, da, ref_cx_s, ref_cy_s)?;
        if use_residual_cost {
            let mut inter = Mat::default();
            core::bitwise_and(&residual_small, &top, &mut inter, &core::no_array())?;
            let miss = core::count_non_zero(&residual_small)? - core::count_non_zero(&inter)?;
            let mut not_obs = Mat::default();
            core::bitwise_not(&obs_small, &mut not_obs, &core::no_array())?;
            let mut top_out = Mat::default();
            core::bitwise_and(&top, &not_obs, &mut top_out, &core::no_array())?;
            Ok(miss + core::count_non_zero(&top_out)? * 2)
        } else {
            let mut not_obs = Mat::default();
            core::bitwise_not(&obs_small, &mut not_obs, &core::no_array())?;
            let mut top_out = Mat::default();
            core::bitwise_and(&top, &not_obs, &mut top_out, &core::no_array())?;
            let mut inter = Mat::default();
            core::bitwise_and(&top, &obs_small, &mut inter, &core::no_array())?;
            let covered = core::count_non_zero(&inter)?;
            let top_px = core::count_non_zero(&top)? as i32;
            Ok((top_px - covered).max(0) + core::count_non_zero(&top_out)? * 3)
        }
    };

    let sxy = (REG_SEED_RANGE_PX / REG_INIT_STEP_PX).ceil() as i32;
    let sa  = (REG_SEARCH_RANGE_DEG / REG_INIT_STEP_DEG).ceil() as i32;
    let mut bdx = seed_dx; let mut bdy = seed_dy; let mut bda = seed_ang;
    let mut best_cost = cost_small(bdx, bdy, bda)?;

    for ix in -sxy..=sxy {
        for iy in -sxy..=sxy {
            for ia in -sa..=sa {
                let (dx,dy,da) = (seed_dx+ix as f32*REG_INIT_STEP_PX,
                                  seed_dy+iy as f32*REG_INIT_STEP_PX,
                                  seed_ang+ia as f32*REG_INIT_STEP_DEG);
                let cost = cost_small(dx,dy,da)?;
                if cost < best_cost { best_cost=cost; bdx=dx; bdy=dy; bda=da; }
            }
        }
    }

    let cost_full = |dx: f32, dy: f32, da: f32| -> Result<i32> {
        let top = warp_mask(ref_mask, dx, dy, da, ref_cx, ref_cy)?;
        if use_residual_cost {
            let mut inter = Mat::default();
            core::bitwise_and(&residual, &top, &mut inter, &core::no_array())?;
            let miss = core::count_non_zero(&residual)? - core::count_non_zero(&inter)?;
            let mut not_obs = Mat::default();
            core::bitwise_not(mask_obs, &mut not_obs, &core::no_array())?;
            let mut top_out = Mat::default();
            core::bitwise_and(&top, &not_obs, &mut top_out, &core::no_array())?;
            Ok(miss + core::count_non_zero(&top_out)? * 2)
        } else {
            let mut not_obs = Mat::default();
            core::bitwise_not(mask_obs, &mut not_obs, &core::no_array())?;
            let mut top_out = Mat::default();
            core::bitwise_and(&top, &not_obs, &mut top_out, &core::no_array())?;
            let mut inter = Mat::default();
            core::bitwise_and(&top, mask_obs, &mut inter, &core::no_array())?;
            let top_px = core::count_non_zero(&top)? as i32;
            Ok((top_px-core::count_non_zero(&inter)?).max(0)+core::count_non_zero(&top_out)?*3)
        }
    };

    let mut best_cost_full = cost_full(bdx,bdy,bda)?;
    let mut spx = REG_INIT_STEP_PX/2.0; let mut sdg = REG_INIT_STEP_DEG/2.0;
    for _ in 0..REG_HILL_ITERS {
        let mut imp = false;
        for &(ddx,ddy,dda) in &[(spx,0.,0.),(-spx,0.,0.),(0.,spx,0.),(0.,-spx,0.),(0.,0.,sdg),(0.,0.,-sdg)] {
            let c = cost_full(bdx+ddx,bdy+ddy,bda+dda)?;
            if c < best_cost_full { best_cost_full=c; bdx+=ddx; bdy+=ddy; bda+=dda; imp=true; }
        }
        if !imp {
            spx=(spx/2.0).max(REG_FINE_STEP_PX); sdg=(sdg/2.0).max(REG_FINE_STEP_DEG);
            if spx<=REG_FINE_STEP_PX && sdg<=REG_FINE_STEP_DEG { break; }
        }
    }

    let best_top = warp_mask(ref_mask, bdx, bdy, bda, ref_cx, ref_cy)?;
    let top_area = core::count_non_zero(&best_top)? as f32;
    let (fit_iou, iou_threshold) = if use_residual_cost {
        let mut inter = Mat::default();
        core::bitwise_and(&best_top, &residual, &mut inter, &core::no_array())?;
        let recall = core::count_non_zero(&inter)? as f32 / residual_area as f32;
        let mut not_obs = Mat::default();
        core::bitwise_not(mask_obs, &mut not_obs, &core::no_array())?;
        let mut top_out = Mat::default();
        core::bitwise_and(&best_top, &not_obs, &mut top_out, &core::no_array())?;
        let prec = 1.0 - (core::count_non_zero(&top_out)? as f32 / top_area.max(1.0)).min(1.0);
        let f1 = if prec+recall>0.0 { 2.0*prec*recall/(prec+recall) } else { 0.0 };
        (f1, REG_MIN_IOU)
    } else {
        let mut inter = Mat::default();
        core::bitwise_and(&best_top, mask_obs, &mut inter, &core::no_array())?;
        let cov = core::count_non_zero(&inter)? as f32 / top_area.max(1.0);
        (cov, 0.70)
    };

    let top_spine = spine_from_mask(&best_top, rs.spine_vec, off)?
        .unwrap_or_else(|| {
            let dr = bda.to_radians();
            let (c_r, s_r) = (dr.cos(), dr.sin());
            let svx = rs.spine_vec.x*c_r - rs.spine_vec.y*s_r;
            let svy = rs.spine_vec.x*s_r + rs.spine_vec.y*c_r;
            let svl = (svx*svx+svy*svy).sqrt().max(1e-6);
            let sv  = Point2f::new(svx/svl, svy/svl);
            let h   = rs.length_px / 2.0;
            let cg  = Point2f::new(ref_cx+bdx+off.x, ref_cy+bdy+off.y);
            let tg  = Point2f::new(cg.x+sv.x*h, cg.y+sv.y*h);
            InsoleSpine {
                center: cg, tip: tg,
                heel: Point2f::new(cg.x-sv.x*h, cg.y-sv.y*h),
                spine_vec: sv, angle360: angle360_from_center_tip(cg, tg),
                length_px: rs.length_px,
            }
        });

    let top_angle_delta = delta_angle_signed(rs.angle360, top_spine.angle360);
    let top_delta_x     = top_spine.center.x - rs.center.x;
    let top_delta_y     = top_spine.center.y - rs.center.y;
    let top_offset_px   = (top_delta_x*top_delta_x+top_delta_y*top_delta_y).sqrt();
    let valid           = fit_iou >= iou_threshold;
    let top_flipped     = valid && top_angle_delta.abs() > FLIP_ANGLE_DEG;

    let stack_state: u16 = match (top_offset_px > STACK_OFFSET_CONFIRM_PX,
                                  top_angle_delta.abs() > STACK_ROTATION_CONFIRM) {
        (false,false)=>1, (true,false)=>2, (false,true)=>3, (true,true)=>4,
    };

    Ok(StackedAnalysis {
        top_spine, top_angle360: top_spine.angle360,
        top_angle_delta, top_delta_x, top_delta_y, top_offset_px,
        fit_iou, residual_area, valid, top_flipped, stack_state,
    })
}

// ================================================================
//  Cargo.toml mẫu — thêm vào cuối comment:
//
//  [package]
//  name = "vision_unit_test"
//  version = "0.1.0"
//  edition = "2021"
//
//  [[bin]]
//  name = "vision_unit_test"
//  path = "vision_unit_test.rs"
//
//  [dependencies]
//  opencv = { version = "0.94", features = ["opencv-4"] }
// ================================================================
