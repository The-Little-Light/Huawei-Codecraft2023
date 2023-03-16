$ROOT='C:/Huawei-Developer-Contest/WindowsRelease'
cd $ROOT/SDK/c++/build
rm -r -fo *
cmake .. -G "MinGW Makefiles"
mingw32-make -j
rm -r -fo $ROOT/replay/*
&"$ROOT/Robot.exe" -f -m $ROOT/maps/1.txt -r $ROOT/replay/1.rep $ROOT/SDK/c++/build/main.exe    
&"$ROOT/Robot.exe" -f -m $ROOT/maps/2.txt -r $ROOT/replay/2.rep $ROOT/SDK/c++/build/main.exe    
&"$ROOT/Robot.exe" -f -m $ROOT/maps/3.txt -r $ROOT/replay/3.rep $ROOT/SDK/c++/build/main.exe    
&"$ROOT/Robot.exe" -f -m $ROOT/maps/4.txt -r $ROOT/replay/4.rep $ROOT/SDK/c++/build/main.exe    
Read-Host "Press Enter any key to exit"