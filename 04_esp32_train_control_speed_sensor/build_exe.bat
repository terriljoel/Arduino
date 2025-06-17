@echo off
echo ==========================================
echo   LEGO Train Controller - EXE Builder
echo ==========================================
echo.

REM Check if Python is installed
python --version >nul 2>&1
if errorlevel 1 (
    echo ❌ Python is not installed or not in PATH
    echo Please install Python and try again
    pause
    exit /b 1
)

echo ✅ Python found
echo.

REM Check if PyInstaller is installed
pip show pyinstaller >nul 2>&1
if errorlevel 1 (
    echo 📦 Installing PyInstaller...
    pip install pyinstaller
    if errorlevel 1 (
        echo ❌ Failed to install PyInstaller
        pause
        exit /b 1
    )
    echo ✅ PyInstaller installed
) else (
    echo ✅ PyInstaller already installed
)

echo.
echo 🔨 Building EXE...
echo.

REM Clean previous builds
if exist "dist" rmdir /s /q "dist"
if exist "build" rmdir /s /q "build"
if exist "*.spec" del /q "*.spec"

REM Build the EXE
pyinstaller ^
    --onefile ^
    --windowed ^
    --name "LEGO-Train-Controller-v2.0" ^
    --hidden-import=bleak ^
    --hidden-import=bleak.backends.winrt ^
    --hidden-import=bleak.backends.winrt.scanner ^
    --hidden-import=bleak.backends.winrt.client ^
    --hidden-import=asyncio ^
    --hidden-import=threading ^
    --distpath=release ^
    --clean ^
    advanced_controller.py

if errorlevel 1 (
    echo.
    echo ❌ Build failed! Check the error messages above.
    echo.
    echo 💡 Common solutions:
    echo   - Make sure advanced_train_controller.py exists
    echo   - Check that all Python packages are installed: pip install bleak
    echo   - Try running the Python script first to ensure it works
    echo.
    pause
    exit /b 1
)

echo.
echo ✅ BUILD SUCCESSFUL!
echo.
echo 📁 Your EXE file is located at:
echo    release\LEGO-Train-Controller-v2.0.exe
echo.
echo 📋 File size:
for %%I in ("release\LEGO-Train-Controller-v2.0.exe") do echo    %%~zI bytes (%%~zI bytes = ~%%~zI MB)
echo.
echo 🚀 You can now distribute this EXE file to other Windows computers!
echo.
echo 💡 Note: The first run might be slow as Windows extracts the files.
echo    Subsequent runs will be faster.
echo.

REM Open the release folder
if exist "release\LEGO-Train-Controller-v2.0.exe" (
    echo 📂 Opening release folder...
    explorer release
)

echo.
echo ✨ Build complete! Press any key to exit.
pause >nul