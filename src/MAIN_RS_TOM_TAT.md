# Tóm tắt cách hoạt động `main.rs`

Tài liệu này mô tả nhanh luồng hoạt động của chương trình `shoe_vision` trong `main.rs`.

## 1. Mục đích chính

`main.rs` là chương trình vision chạy bằng Rust + OpenCV để:

- Đọc hình từ camera.
- Xác định lót giày trong ROI.
- So sánh với mẫu chuẩn `reference.png` và `ref_mask.png`.
- Tính góc, tâm, đầu mũi/gót, độ lệch so với mẫu.
- Phát hiện các trạng thái: lót đơn, lót chồng, lệch tâm, xoay, lật, lỗi/hư, vật lạ, tay gắp lọt ROI.
- Chuyển tọa độ camera sang tọa độ robot.
- Gửi dữ liệu cho PLC qua Modbus TCP slave.
- Gửi dữ liệu + ảnh JPEG cho GUI qua IPC TCP local.

## 2. Các file dữ liệu dùng kèm

- `reference.png`: ảnh mẫu chuẩn của lót.
- `ref_mask.png`: mask nhị phân của lót mẫu.
- `roi.txt`: vùng xử lý ảnh.

Nếu chưa có mẫu, chương trình ở trạng thái `NoReference`. Khi bấm `S`, chương trình chụp nhiều frame rồi tạo mẫu mới.

## 3. Luồng khởi động

Khi chạy `main()`:

1. Tạo `AppShared` để chia sẻ dữ liệu giữa các thread.
2. Chạy thread Modbus TCP slave:
   - Port chính: `0.0.0.0:502`.
   - Fallback: `0.0.0.0:5020`.
3. Chạy thread IPC server cho GUI:
   - `127.0.0.1:5556`.
4. Mở camera `CAM_ID`.
5. Cấu hình camera giảm trễ:
   - Buffer size = `1`.
   - FPS = `30`.
   - Mỗi vòng dùng `grab()` bỏ frame cũ rồi `retrieve()` lấy frame mới.
6. Load ROI, ảnh mẫu và mask mẫu nếu có.
7. Vào vòng lặp xử lý ảnh liên tục.

## 4. Các trạng thái chính của ứng dụng

`AppState` có 3 trạng thái:

- `NoReference`: chưa có mẫu hoặc mẫu không hợp lệ.
- `Capturing`: đang chụp ảnh mẫu mới.
- `Tracking`: đã có mẫu, đang xử lý lót theo thời gian thực.

Các phím chính:

- `D`: đặt ROI toàn frame và lưu vào `roi.txt`.
- `S`: chụp ảnh mẫu mới.
- `R`: load lại mẫu từ file.
- `Q`: thoát chương trình.

## 5. Xử lý ảnh cơ bản

Hàm chính: `segment_shoe()`.

Các bước:

1. Chuyển ảnh ROI sang grayscale.
2. Làm mờ Gaussian để giảm nhiễu.
3. Dùng Otsu threshold để tạo ảnh trắng/đen.
4. Morph close để nối vùng bị hở.
5. Morph open để loại nhiễu nhỏ.
6. Tìm contour ngoài cùng.
7. Vẽ lại các contour thành mask nhị phân sạch.

Mask này là đầu vào cho phần phân tích lót.

## 6. Tìm trục lót bằng PCA

Các hàm liên quan:

- `pca_spine_core()`
- `spine_from_mask()`
- `spine_from_mask_width_vote()`

Ý tưởng:

- Lấy toàn bộ pixel trắng trong mask.
- Chạy PCA để tìm trục dài nhất của lót.
- Tìm 2 đầu xa nhất trên trục đó.
- Chọn đâu là mũi/gót:
  - Với ảnh tracking thường: so với vector của mẫu.
  - Với mẫu hoặc stacked: dùng width vote, đầu hẹp hơn là mũi.

Kết quả được lưu trong `InsoleSpine`:

- `center`: tâm lót.
- `tip`: mũi lót.
- `heel`: gót lót.
- `angle360`: góc 0-360 độ.
- `length_px`: chiều dài theo trục chính.

## 7. Phát hiện lót chồng

Hàm chính: `compute_shoe()` và `analyze_stacked_v9()`.

`compute_shoe()` kiểm tra:

- Diện tích contour.
- Solidity: độ đặc của hình.
- Tỉ lệ diện tích so với mẫu.
- Tỉ lệ vùng nằm ngoài mask mẫu.

Nếu các dấu hiệu cho thấy có thể là lót chồng, chương trình gọi `analyze_stacked_v9()`.

`analyze_stacked_v9()` làm việc như sau:

1. Lấy phần dư: `mask_obs AND NOT ref_mask`.
2. Dùng phần dư để đoán vị trí lót trên.
3. Warp mask mẫu theo nhiều vị trí/góc để tìm khớp tốt nhất.
4. Tính độ khớp `fit_iou`.
5. Nếu hợp lệ, trả về thông tin lót trên:
   - Góc lót trên.
   - Độ lệch tâm.
   - Lệch xoay.
   - Trạng thái stack.

Các trạng thái stack:

- `STACK_SINGLE`: lót đơn.
- `STACK_ALIGNED`: chồng nhưng gần thẳng.
- `STACK_OFFSET`: chồng lệch tâm.
- `STACK_ROTATED`: chồng xoay.
- `STACK_COMPLEX`: vừa lệch vừa xoay.

## 8. Chặn nhầm do tay gắp lọt ROI

Hàm mới: `detect_robot_intrusion()`.

Lý do cần hàm này:

- Tay gắp/khung kim loại đi vào ROI có thể bị mask dính vào lót.
- Phần dư ngoài mẫu ref làm thuật toán tưởng có lót thứ hai.

Cách chặn:

1. Lấy vùng dư ngoài `ref_mask`.
2. Giãn nhẹ `ref_mask` để bỏ qua sai lệch nhỏ quanh mép lót.
3. Nếu vùng dư đủ lớn và có nhiều pixel tối/xám kiểu kim loại, xem là tay gắp lọt ROI.
4. Khi phát hiện, chương trình freeze frame đó:
   - Không phân tích stacked.
   - Không cập nhật dữ liệu sai cho PLC.
   - Hiển thị nhãn `[TAY GAP]`.

## 9. Chặn vật lạ và frame không ổn định

Chương trình có `Stabilizer` để xử lý vật lạ:

- Nếu tổng diện tích contour lạ vượt `FOREIGN_TOTAL_AREA`, tăng `frozen_count`.
- Khi vượt `FROZEN_CONFIRM_FRAMES`, đóng băng kết quả.
- PLC nhận D lot = 0.
- GUI/frame hiển thị trạng thái lỗi.

Tay gắp lọt ROI cũng dùng cơ chế freeze, nhưng được nhận diện riêng bằng `robot_intrusion`.

## 10. Làm mượt kết quả

`SpineSmooth` làm mượt:

- Tâm.
- Mũi/gót.
- Góc 360.
- Độ lệch góc.
- Độ lệch tâm.

Hệ số làm mượt: `SMOOTH_ALPHA`.

Mục đích là giảm rung số liệu khi camera hoặc mask dao động nhẹ.

## 11. Chuyển tọa độ camera sang robot

Các hàm:

- `cam_to_robot()`
- `cam_delta_to_robot()`
- `robot_debug_for_point()`

Mô hình:

- ROI được xem như vùng làm việc robot.
- `ROBOT_WORK_DIAMETER_MM` là kích thước thật tương ứng vùng ROI.
- Tâm camera/robot dùng `CAMERA_ROBOT_XY_MM`.
- Pixel trong ảnh được đổi sang mm.

Dữ liệu robot dùng để:

- Gửi PLC.
- Hiển thị debug trên GUI.
- Biết điểm đang nằm trong vùng camera hay không.

## 12. Dữ liệu gửi PLC

Hàm chính: `build_slave_regs()`.

PC chạy như Modbus TCP slave. PLC chủ động đọc/ghi.

Vùng D chính:

- `D0..D10`: dữ liệu vision cho PLC đọc.
- `D100..D109`: robot status do PLC ghi về PC.

Ý nghĩa `D0..D10`:

- `D0`: delta angle x10.
- `D1`: direction flag.
- `D2`: delta_x robot mm x10.
- `D3`: delta_y robot mm x10.
- `D4`: robot_x tuyệt đối x10.
- `D5`: robot_y tuyệt đối x10.
- `D6`: flags trạng thái.
- `D7`: offset từ tâm x10.
- `D8`: angle360 x10.
- `D9`: stack_state.
- `D10`: data_ok.

Khi có lót trên hợp lệ, chương trình ưu tiên gửi tọa độ/góc của lót trên. Nếu không, gửi dữ liệu lót đơn.

## 13. Dữ liệu gửi GUI

Thread `ipc_server_thread()` gửi packet TCP local cho GUI.

Mỗi packet gồm:

1. Header: `DATA:<json_len>:<jpeg_len>\n`
2. JSON trạng thái vision.
3. JPEG frame đang hiển thị.

Tần suất gửi GUI hiện là `IPC_FRAME_MS = 66`, khoảng 15fps, để giảm tải và giảm trễ backlog.

JPEG dùng `JPEG_QUALITY = 58` để nhẹ hơn.

## 14. Luồng xử lý trong mỗi frame

Trong trạng thái `Tracking`, mỗi frame đi qua luồng:

1. Lấy frame mới nhất từ camera.
2. Cắt ROI.
3. Tạo mask bằng `segment_shoe()`.
4. Kiểm tra có đủ pixel để xem là có lót không.
5. Tìm contour ứng viên lớn nhất.
6. Resize/load `ref_mask` cho cùng kích thước ROI.
7. Kiểm tra tay gắp lọt ROI bằng `detect_robot_intrusion()`.
8. Kiểm tra vật lạ/freeze.
9. Nếu ổn, gọi `compute_shoe()`.
10. Nếu nghi stacked, gọi `analyze_stacked_v9()`.
11. Làm mượt kết quả.
12. Vẽ overlay.
13. Build D register gửi PLC.
14. Cập nhật `VisionFrame` gửi GUI.
15. Lưu `last_good` để dùng khi frame sau bị freeze.

## 15. Các ngưỡng thường cần chỉnh

Nhóm camera/GUI:

- `CAM_ID`
- `JPEG_QUALITY`
- `IPC_FRAME_MS`

Nhóm robot/map tọa độ:

- `ROBOT_WORK_DIAMETER_MM`
- `CAMERA_ROBOT_XY_MM`
- `CAMERA_HEIGHT_MM`

Nhóm nhận diện lót:

- `MIN_SHOE_AREA`
- `MAX_SHOE_AREA`
- `STACKED_SOLIDITY`
- `STACKED_AREA_RATIO_MIN`
- `OUTSIDE_RATIO_STACKED`

Nhóm phát hiện tay gắp:

- `ROBOT_INTRUSION_MIN_AREA`
- `ROBOT_INTRUSION_OUTSIDE_RATIO`
- `ROBOT_INTRUSION_DARK_GRAY`
- `ROBOT_INTRUSION_DARK_RATIO`

Nếu `[TAY GAP]` xuất hiện quá nhạy, giảm độ nhạy bằng cách tăng `ROBOT_INTRUSION_MIN_AREA` hoặc `ROBOT_INTRUSION_DARK_RATIO`.

Nếu tay gắp vẫn bị nhận nhầm là lót trên, tăng độ nhạy bằng cách giảm `ROBOT_INTRUSION_MIN_AREA` hoặc `ROBOT_INTRUSION_DARK_RATIO`.

## 16. Điểm cần chú ý khi vận hành

- Khi chụp mẫu bằng `S`, phải đảm bảo chỉ có một lót chuẩn trong ROI, không có tay gắp.
- ROI càng sạch, thuật toán càng ít nhầm stacked.
- Nếu robot/tay gắp đi qua vùng nhìn camera, nên cho vision freeze hoặc chỉ đọc dữ liệu khi robot ở trạng thái an toàn.
- Nếu PLC có trạng thái robot rõ ràng, có thể nâng cấp tiếp: khi `robot_state` là MOVE/GRIP thì bỏ qua vision hoặc freeze chủ động.

