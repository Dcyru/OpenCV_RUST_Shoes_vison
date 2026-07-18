@echo off
:: ================================================================
::  run_test.bat — Chạy toàn bộ unit test xử lý ảnh 1 lệnh
::  Dùng:  run_test.bat <ảnh_lót_giày.jpg>
::  Hoặc:  kéo thả file ảnh vào run_test.bat
:: ================================================================
setlocal

set "IMG=%~1"

:: ── Nếu ĐÃ truyền ảnh vào thì nhảy qua bước mở hộp thoại ───────
if not "%IMG%"=="" goto skip_dialog

echo Dang mo hop thoai chon anh...
for /f "delims=" %%I in ('powershell -noprofile -command "Add-Type -AssemblyName System.Windows.Forms; $f=New-Object System.Windows.Forms.OpenFileDialog; $f.Filter='Anh (*.png;*.jpg;*.jpeg;*.bmp)|*.png;*.jpg;*.jpeg;*.bmp'; $f.Title='Chon anh lot giay de test'; if($f.ShowDialog() -eq 'OK'){$f.FileName}"') do set "IMG=%%I"

:skip_dialog

if "%IMG%"=="" (
    echo Khong chon anh. Thoat.
    pause
    exit /b 1
)

if not exist "%IMG%" (
    echo Loi: Khong tim thay file '%IMG%'
    pause
    exit /b 1
)

echo =============================================
echo   Shoe Stack Vision -- Unit Test Runner
echo =============================================
echo.

cd /d "%~dp0"

:: Cài thư mục src nếu chưa có
if not exist "src" mkdir src
if not exist "src\main.rs" copy vision_unit_test.rs src\main.rs >nul

echo [1/2] Build Rust (cargo build --release)...
cargo build --release 2>&1 | findstr /C:"error" /C:"Compiling" /C:"Finished"

echo.
echo [2/2] Chay unit test voi anh: %IMG%
echo ---------------------------------------------

if exist "target\release\vision_unit_test.exe" (
    target\release\vision_unit_test.exe "%IMG%"
) else if exist "vision_unit_test.exe" (
    vision_unit_test.exe "%IMG%"
) else (
    echo Loi: Khong tim thay file thuc thi sau khi build!
)

echo.
echo ---------------------------------------------
echo Anh ket qua duoc luu trong: test_out/
echo.
pause