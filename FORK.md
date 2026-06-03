# QMSD ESP-IDF 6 Fork

This branch reconciles the ESP-IDF 6 fork with upstream
`smartpanle/QMSD-ESP32-BSP` master while keeping BSP patch types separate.

Reconciled upstream commit: `d452ad7` (`fixed: rgb driver bug`).

- `Apply local display, GUI, and BSP cleanup patches` preserves local BSP
  changes for configurable RGB panel timing, RGB transfer stability,
  GUI task/touch handling, optional UI performance tracing, draw-bitmap error
  reporting, and removal of unused vendor examples/audio assets.
- `Port BSP for ESP-IDF 6` contains the generic ESP-IDF 6 compatibility port.

Keep changes BSP-scoped so the fork remains usable outside any single firmware
application.
