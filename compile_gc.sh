#!/bin/bash

echo "[GC Build] Compiling rfid_gc_live..."

if [ ! -d "SRC" ]; then
    echo "[GC Build] ERROR: SRC directory not found!"
    exit 1
fi

if [ ! -f "rfid_gc_live.c" ]; then
    echo "[GC Build] ERROR: rfid_gc_live.c not found!"
    exit 1
fi

gcc \
  rfid_gc_live.c \
  SRC/host.c SRC/CAENRFIDLib_Light.c SRC/IO_Light.c \
  -ISRC \
  -o rfid_gc_live \
  -lpthread -lm \
  -Wall

if [ $? -eq 0 ]; then
    chmod +x rfid_gc_live
    echo "[GC Build] Success! Run with: ./rfid_gc_live"
else
    echo "[GC Build] Compilation failed!"
    exit 1
fi
