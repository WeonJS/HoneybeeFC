@echo off
idf.py build
if %errorlevel% neq 0 (
    echo Build failed
    exit /b %errorlevel%
)

idf.py -p COM3 -b 115200 flash
if %errorlevel% neq 0 (
    echo Flash failed
    exit /b %errorlevel%
)

idf.py monitor
if %errorlevel% neq 0 (
    echo Monitor failed
    exit /b %errorlevel%
)