# Shoe Stack Vision — Unit Test Runner

## Cấu trúc
```
vision_unit_test_pkg/
├── src/
│   └── main.rs          ← toàn bộ code unit test
├── Cargo.toml
├── run_test.sh           ← Linux/macOS: 1 lệnh chạy
├── run_test.bat          ← Windows:     kéo thả ảnh vào
└── README.md
```

## Cách dùng

### Linux / macOS
```bash
chmod +x run_test.sh
./run_test.sh lot_giay.jpg     # truyền ảnh trực tiếp
./run_test.sh                  # mở dialog chọn ảnh (cần zenity)
```

### Windows
```
run_test.bat lot_giay.jpg      # truyền ảnh
# hoặc kéo thả file ảnh vào run_test.bat
```

## Kết quả
Sau khi chạy, thư mục `test_out/` chứa **10 file ảnh PNG**, mỗi file
là kết quả 1 unit được vẽ annotation trực quan:

| File | Unit |
|------|------|
| `01_segment.png`          | segment_shoe — mask nhị phân Otsu |
| `01_segment_mask.png`     | mask thuần trắng/đen |
| `02_centroid.png`         | centroid — trọng tâm mask |
| `03_pca_angle_raw.png`    | pca_angle_raw — góc PCA thô |
| `04_pca_spine_core.png`   | pca_spine_core — trục chính + 2 đầu |
| `05_spine_width_vote.png` | spine_from_mask_width_vote — TIP/HEEL |
| `06_compute_shoe.png`     | compute_shoe — solidity, stacked, flipped |
| `07_analyze_stacked.png`  | analyze_stacked_v9 — phát hiện lót xếp chồng |
| `08_warp_mask.png`        | warp_mask — affine transform 3 trường hợp |
| `09_compute_ref_spine.png`| compute_ref_spine — RefSpine từ ảnh mẫu |
| `10_full_pipeline.png`    | Toàn bộ pipeline từ đầu đến cuối |

## Yêu cầu
- Rust + Cargo
- OpenCV 4.x đã cài (`pkg-config --libs opencv4` chạy được)
- Linux: `sudo apt install libopencv-dev`
- macOS: `brew install opencv`
- Windows: cài vcpkg + OpenCV, xem https://github.com/twistedfall/opencv-rust
