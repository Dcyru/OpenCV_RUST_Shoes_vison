@echo off
REM =====================================================================
REM  build.bat v12 — Quản lý dự án Shoe Stack Vision
REM  Đặt file này tại thư mục gốc (cùng Cargo.toml)
REM =====================================================================
if "%1"==""        goto help
if "%1"=="help"    goto help
if "%1"=="check"   goto check
if "%1"=="build"   goto build
if "%1"=="release" goto release
if "%1"=="run"     goto run
if "%1"=="sim"     goto sim
if "%1"=="gui"     goto gui
if "%1"=="all"     goto all
if "%1"=="clean"   goto clean
goto help

:check
echo.
echo [CHECK] Kiểm tra môi trường...
rustc   --version 2>nul && echo   OK  Rust    || echo   !!  Rust THIẾU — tai: https://rustup.rs
cargo   --version 2>nul && echo   OK  Cargo   || echo   !!  Cargo THIẾU
python  --version 2>nul && echo   OK  Python  || python3 --version 2>nul || echo   !!  Python THIẾU
pip show pillow  2>nul && echo   OK  Pillow  || echo   !!  Pillow THIẾU — chay: pip install pillow
echo.
echo [CHECK] Địa chỉ D hiện tại (xem src\main.rs và src\sim_server.rs):
findstr /C:"PC_SLAVE_START" /C:"PLC_WRITE_D_START" /C:"PLC_READ_D_REGIONS" /C:"PLC_IP" src\main.rs 2>nul
goto end

:build
echo.
echo [BUILD] cargo build...
cargo build
if errorlevel 1 ( echo [!!] Build thất bại & goto end )
echo [OK] Build xong — target\debug\shoe_vision.exe
goto end

:release
echo.
echo [RELEASE] cargo build --release (lto=true, mất 2-3 phút)...
cargo build --release
if errorlevel 1 ( echo [!!] Release build thất bại & goto end )
echo [OK] Release: target\release\shoe_vision.exe
goto end

:run
echo.
echo [RUN] Shoe Stack Vision v12...
echo   Modbus TCP Slave  :502 (PLC đọc vào)
echo   IPC GUI server    :5556
echo   Phím: [D] ROI  [S] mẫu  [R] load  [P] TCP  [Q] Thoát
echo.
cargo run --bin shoe_vision
goto end

:sim
echo.
echo [SIM] Sim Server v3 (không cần camera, không cần OpenCV)...
echo   Modbus TCP Slave  :502/:5020
echo   IPC GUI server    :5556
echo   Ctrl+C để dừng
echo.
echo Lưu ý: Sửa SIM_MODE=false và PLC_IP trong src\sim_server.rs để test PLC thật
echo.
cargo run --bin sim_server --release
goto end

:gui
echo.
echo [GUI] Khởi động Dashboard Python...
echo   Kết nối IPC localhost:5556
echo   Yêu cầu: main.rs HOẶC sim_server đang chạy
echo.
python gui\gui.py
if errorlevel 1 python3 gui\gui.py
goto end

:all
echo.
echo [ALL] Build + Chạy Vision + GUI...
cargo build --release
if errorlevel 1 ( echo [!!] Build thất bại & goto end )
echo Khởi động Vision (cửa sổ mới)...
start "Shoe Vision v12" target\release\shoe_vision.exe
echo Chờ 3 giây...
timeout /t 3 /nobreak >nul
echo Khởi động GUI...
python gui\gui.py
if errorlevel 1 python3 gui\gui.py
goto end

:clean
cargo clean
echo [OK] Đã xóa target\
goto end

:help
echo.
echo  Shoe Stack Vision v12 — Build Script
echo  =====================================
echo.
echo  build.bat check    Kiểm tra Rust, Python, Pillow
echo  build.bat build    Build debug
echo  build.bat release  Build release (tối ưu)
echo  build.bat run      Chạy vision + Modbus TCP + IPC
echo  build.bat sim      Chạy sim server (test không cần camera)
echo  build.bat gui      Chạy GUI Python dashboard
echo  build.bat all      Build release + chạy vision + GUI
echo  build.bat clean    Xóa build artifacts
echo.
echo  ── Workflow chuẩn ───────────────────────────────────────────
echo  1. build.bat check          Kiểm tra môi trường
echo  2. build.bat sim            Test GUI + PLC không cần camera
echo     (song song) build.bat gui
echo  3. build.bat all            Chạy hệ thống thật
echo.
echo  ── Sửa địa chỉ D ───────────────────────────────────────────
echo  Mở src\main.rs, tìm và sửa các hằng số đầu file:
echo    PC_SLAVE_START    = D bao nhiêu PLC sẽ đọc từ PC
echo    PLC_WRITE_D_START = D bao nhiêu PC sẽ ghi xuống PLC
echo    PLC_READ_D_REGIONS= Mảng (tên, D_start, count) cần đọc
echo    PLC_READ_M_REGIONS= Mảng (tên, M_start, count) cần đọc
echo    PLC_IP            = IP PLC thật
echo.
echo  Sau khi sửa: build.bat release để build lại
echo.

:end
