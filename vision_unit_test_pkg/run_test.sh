#!/usr/bin/env bash
# ================================================================
#  run_test.sh — Chạy toàn bộ unit test xử lý ảnh 1 lệnh
#  Dùng:  ./run_test.sh <ảnh_lót_giày.jpg>
#  Hoặc:  ./run_test.sh          (mở hộp thoại chọn ảnh nếu có zenity)
# ================================================================
set -e

IMG="$1"

# ── Nếu không truyền ảnh → mở dialog chọn file ─────────────────
if [ -z "$IMG" ]; then
    if command -v zenity &>/dev/null; then
        IMG=$(zenity --file-selection \
            --title="Chọn ảnh lót giày để test" \
            --file-filter="Ảnh (*.png *.jpg *.jpeg *.bmp)|*.png *.jpg *.jpeg *.bmp" \
            2>/dev/null) || { echo "Không chọn ảnh."; exit 1; }
    elif command -v kdialog &>/dev/null; then
        IMG=$(kdialog --getopenfilename . "*.png *.jpg *.jpeg *.bmp" \
            --title "Chọn ảnh lót giày") || { echo "Không chọn ảnh."; exit 1; }
    else
        echo "Dùng: $0 <path_to_shoe_image>"
        echo "Ví dụ: $0 lot_giay.jpg"
        exit 1
    fi
fi

# ── Kiểm tra file tồn tại ───────────────────────────────────────
if [ ! -f "$IMG" ]; then
    echo "Lỗi: Không tìm thấy file '$IMG'"
    exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR"

echo "╔══════════════════════════════════════════════╗"
echo "║  Shoe Stack Vision — Unit Test Runner       ║"
echo "╠══════════════════════════════════════════════╣"
echo "║  Build & chạy vision_unit_test...           ║"
echo "╚══════════════════════════════════════════════╝"
echo ""

# ── Cài thư mục src nếu chưa có (Cargo layout) ──────────────────
mkdir -p src
if [ ! -f "src/main.rs" ]; then
    cp vision_unit_test.rs src/main.rs
fi

# ── Build (release để nhanh) ─────────────────────────────────────
echo "[1/2] Build Rust (cargo build --release)..."
cargo build --release 2>&1 | grep -E "^error|^warning|Compiling|Finished" || true

echo ""
echo "[2/2] Chạy unit test với ảnh: $IMG"
echo "──────────────────────────────────────────────"

./target/release/vision_unit_test "$IMG"

echo ""
echo "──────────────────────────────────────────────"
echo "Ảnh kết quả: test_out/"
echo ""

# ── Tự mở thư mục kết quả ───────────────────────────────────────
if command -v eog &>/dev/null; then
    echo "Mở ảnh bằng eog..."
    eog test_out/*.png &
elif command -v feh &>/dev/null; then
    feh test_out/ &
elif command -v xdg-open &>/dev/null; then
    xdg-open test_out/ &
fi
