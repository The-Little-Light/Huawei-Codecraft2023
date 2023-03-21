$ROOT='C:/Huawei-Developer-Contest/WindowsRelease'
rm -r -fo $ROOT/SDK/c++/build/*
rm -r -fo $ROOT/replay/*
rm -fo $ROOT/SDK/c++/log.txt
cd $ROOT/SDK/c++/build
cmake .. -G "MinGW Makefiles"
mingw32-make -j
cd $ROOT/SDK/c++
&"$ROOT/Robot.exe" -f -m $ROOT/maps/1.txt -r $ROOT/replay/1.rep $ROOT/SDK/c++/build/main.exe    
&"$ROOT/Robot.exe" -f -m $ROOT/maps/2.txt -r $ROOT/replay/2.rep $ROOT/SDK/c++/build/main.exe    
&"$ROOT/Robot.exe" -f -m $ROOT/maps/3.txt -r $ROOT/replay/3.rep $ROOT/SDK/c++/build/main.exe    
&"$ROOT/Robot.exe" -f -m $ROOT/maps/4.txt -r $ROOT/replay/4.rep $ROOT/SDK/c++/build/main.exe    
# Read-Host "Press Enter any key to exit"