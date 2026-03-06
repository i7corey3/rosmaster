cd %~p0
cd ..
call C:\opt\ros\galactic\x64\setup.bat
@echo off

if "%2"=="test" (goto :TESTMSG)
if "%3"=="test" (goto :TESTMSG)

if "%1"=="python" (goto :PYTHON)
if "%1"=="c++" (goto :CPP)
if "%1"=="main" (goto :MAIN)


:PYTHON
call ros2 pkg create --build-type ament_python %2
move %2 src/
call python %~p0buildNode.py %2 python %3
exit 1


:CPP
call ros2 pkg create --build-type ament_cmake %2
move %2 src/
call python %~p0buildNode.py %2 c++ %3
exit 1

:MAIN
call ros2 pkg create --build-type ament_cmake %2
move %2 src/
call python %~p0buildNode.py %2 c++ pass
exit 1

:TESTMSG
echo:
echo The node name test is reserved and not allowed
echo:
exit 1




