# Product

<!-- impeccable:product-schema 1 -->

## Platform

web

## Users
The owner of the car, using it alone: tuning or diagnosing in the garage, or glancing at live telemetry while driving. Used on a phone and on a PC. Daylight and night both occur. Other audiences (friends, shops, customers) are not confirmed.

## Product Purpose
An ESP32-S3 CAN/OBD-II sniffer that plugs into the car and serves its own web UI over Wi-Fi. It reads PIDs, DTCs and the VIN, records trips to an SD card, and shows live telemetry. Success: the owner reads their car's data with nothing but a browser.

## Positioning
All-in-one device: no companion app, no cloud, no external network. The UI is served by the device itself.

## Operating Context
Device sits in the car unattended and is reached over its own Wi-Fi. The UI ships as a single gzipped HTML file in an 832K flash partition, so page weight matters. Tabs: Dashboard (customizable card grid), Recorder, Diagnostics, Telemetry, CAN Logging, Settings.

## Capabilities and Constraints
- Frontend is Svelte 5 with Pico CSS, built with bun into one `index.html.gz`.
- No external network assets: fonts, icons and scripts are bundled.
- Must work on mobile and desktop.
- Talks to firmware only via REST and WebSocket (`docs/api.md`).
- Page size is a budget: baseline `index.html.gz` was 293,951 bytes before the glass work.

## Brand Commitments
None confirmed. Current UI uses Pico orange and Atkinson Hyperlegible; whether they stay is a visual decision, not recorded here.

## Evidence on Hand
Working firmware and frontend in this repo. No testimonials, customers or benchmarks exist; none should be invented.

## Product Principles
- Self-contained: nothing may depend on a network beyond the device.
- Glanceable while driving; detailed when parked.
- Small: every kilobyte ships over Wi-Fi from flash.
- Same experience on phone and PC.

## Accessibility & Inclusion
No formal standard confirmed. Daylight legibility in a car is a real need, so text contrast over any translucent surface matters.
