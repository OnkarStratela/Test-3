#!/bin/bash

echo "[GC Build] Compiling rfid_lowest..."

if [ ! -d "SRC" ]; then
    echo "[GC Build] ERROR: SRC directory not found!"
    exit 1
fi

if [ ! -f "rfid_lowest.c" ]; then
    echo "[GC Build] ERROR: rfid_lowest.c not found!"
    exit 1
fi

gcc \
  rfid_lowest.c \
  SRC/host.c SRC/CAENRFIDLib_Light.c SRC/IO_Light.c \
  -ISRC \
  -o rfid_lowest \
  -lpthread -lm \
  -Wall

if [ $? -eq 0 ]; then
    chmod +x rfid_lowest
    echo "[GC Build] Success! Run with: ./rfid_lowest"
else
    echo "[GC Build] Compilation failed!"
    exit 1
fi
