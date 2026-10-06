@echo off
echo === Building HumanCallRobot ===
if not exist build mkdir build
cmake -G "MinGW Makefiles" -B build
cmake --build build
if %ERRORLEVEL% EQU 0 (
    echo === Build Successful! ===
) else (
    echo === Build Failed! ===
)
