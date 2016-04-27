@ECHO OFF
"C:\Program Files\Atmel\AVR Tools\AvrAssembler2\avrasm2.exe" -S "C:\mbUSB\CISAVR\labels.tmp" -fI -W+ie -C V2E -o "C:\mbUSB\CISAVR\CISAVR.hex" -d "C:\mbUSB\CISAVR\CISAVR.obj" -e "C:\mbUSB\CISAVR\CISAVR.eep" -m "C:\mbUSB\CISAVR\CISAVR.map" "C:\mbUSB\CISAVR\CISAVR.asm"
