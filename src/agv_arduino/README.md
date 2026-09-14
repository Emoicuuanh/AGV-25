# README

## Git copy file with history to another repository

```bash
mkdir agv_arduino/patch
cd to/origin/repo
git format-patch -o ~/catkin_ws/src/agv_arduino/patch --root ./agv_arduino/upload_arduino.py
cd to/new/repo
git am --3way patch/*.patch && rm patch/*.patch
```
