# GP26 injector remap and GP16 onboard RGB LED (latest revision)

**Wiring changes:** INJ1=GP21, INJ2=GP20, INJ3=GP19, INJ4=GP18, INJ5=GP17, **INJ6=GP26**. Coils 1–6 remain GP10–GP15. GP16 is now dedicated to the onboard **WS2812/NeoPixel-type single-wire RGB LED** (assumed GRB order, 800 kHz). All DAC and cam pins remain unchanged; GP22 remains the 720° marker.

The capture PIO on PIO1 SM0 now reads **17 bits from GP10 through GP26** at 125 kSamples/s, ignoring GP16, GP23, GP24 and GP25; GP22 is the marker (sample bit 12), and GP26 is INJ6 (sample bit 16). Crucially, capture initialization **does not change GP16's GPIO function**, so the LED driver on PIO0 SM2 owns GP16. The logical channel order is unchanged: INJ1..INJ6 bits 0..5, COIL1..COIL6 bits 6..11. `POL 0xFC0` remains the default with active-low coil bits set. The polarity XOR/interpretation is performed **after** the raw physical GPIO bits are remapped.

**Onboard status LED (GP16):** blue solid when firmware is idle (RPM=0) with USB CDC open; slow blue pulse when USB CDC is not open; flashing yellow during cranking RPM 100–400; solid green while running >400 RPM; a short purple (~65 ms) flash when an injector or coil changes state, overlaying the current state; flashing red (250 ms cadence) when a sample overrun or serial event-queue overflow is detected, latched until reboot. LED brightness is kept low; red error has highest priority. The WS2812 is fed by a dedicated PIO state machine, without blocking the crank, cam or capture engines. `stdio_usb_connected()` approximates PC-app connection via the CDC/DTR state, so other serial terminal software can also count as connected.

**Hardware assumption:** GP16 is a single-wire WS2812-compatible onboard RGB LED. If the board instead has a discrete RGB LED with multiple pins or a different color byte order, the driver must be adapted. This firmware is not SDK-compiled or on-device validated; verify before attaching the ECU.

## PIO/DMA waveform engine (current revision)

The MCP4922 crank signal is **now DMA-fed and PIO-clocked**, not software-timed SPI. On PIO0:

- **SM0** shifts two 16-bit MCP4922 frames per sample, asserts /CS separately for each channel, and pulses LDAC only after both frames. GP4=LDAC, GP5=/CS, GP6=SCK, GP7=MOSI; SPI mode 0, MSB first. This uses SPI-compatible physical pins **but the SPI controller is not used**: PIO owns the serial clock, CS and LDAC.
- **SM1** consumes one 32-bit control word per sample and drives GP2/G40, GP3/G163 and GP22/720° marker. The two programs have precisely **103 PIO instruction cycles per sample** and are enabled with synchronized clock dividers.
- Two DMA channels stream one 720° cycle each from precomputed arrays. Completion interrupts refill their read pointers once per 720° revolution pair, rather than servicing each DAC sample in C. The 125 ksample/s PIO1 capture DMA remains independent.
- Each nominal crank tooth slot contains eight differential sine-wave samples; missing tooth slots are 0 V differential. RPM and wheel/cam changes stop/rebuild/restart the same two PIO streams together. RPM zero writes both DAC legs to their midpoint and drives cams/marker low using a one-off software GPIO transaction.

**Limits:** The DMA buffer restart IRQ must refill both streams before their FIFO headroom is exhausted. This design uses DMA IRQ re-arming per 720° cycle, not endlessly chained DMA descriptors. It is not yet oscilloscope-validated, compiled with the Pico SDK, or proven safe against all USB/interrupt and power-change transients. At 12,000 RPM/60 teeth the nominal DAC update frequency is 96 kSamples/s (103 PIO cycles/sample at ~9.89 MHz), and the SPI clock is about 3.30 MHz. Verify DAC sample timing, cam phase, LDAC timing, high RPM operation, and output overrun with a logic analyzer before ECU connection.

**Pin mappings:** GP2 G40, GP3 G163, GP4 LDAC, GP5 /CS, GP6 SCK, GP7 MOSI, GP21–16 injector 1–6, GP10–15 coil 1–6, GP22 internal reference. GP22 is **not** an ECU connection. `POL 0xFC0` remains the assumed monitor polarity.

**Safety:** The MCP4922 output is not an electrically isolated G28 sensor. Use the previously discussed TLV9062 interface, appropriate protective elements and test impedance/common-mode voltage before connecting the ECU. The G40/G163 offsets are still placeholders.

# Audi APB ME7.1 — RP2040 + MCP4922 DAC bench simulator

**Experimental firmware — not yet Pico-SDK compiled or hardware validated. For an isolated bench ECU only.**

## Exact GPIO mapping

| Function | RP2040 pin | Notes |
|---|---|---|
| G40 cam CPS | **GP2** | 3.3 V control to external 2N7002 open drain; ECU waveform inverted by MOSFET |
| G163 cam CPS | **GP3** | Same as G40 |
| MCP4922 LDAC | **GP4** | simultaneous DAC update, active low pulse |
| MCP4922 CS | **GP5** | software-controlled SPI chip select |
| MCP4922 SCK | **GP6** | hardware SPI0 SCK |
| MCP4922 MOSI/SDI | **GP7** | hardware SPI0 TX |
| Injector 1–6 | **GP21–GP16** | inputs in reverse GPIO order, conditioned 3.3 V only |
| Coil 1 | **GP10** | conditioned 3.3 V input |
| Coil 2 | **GP11** | conditioned 3.3 V input |
| Coil 3 | **GP12** | conditioned 3.3 V input |
| Coil 4 | **GP13** | conditioned 3.3 V input |
| Coil 5 | **GP14** | conditioned 3.3 V input |
| Coil 6 | **GP15** | conditioned 3.3 V input |
| Internal 720° marker | **GP22** | driven by SIO and read back by PIO, not wired to ECU |

**Default `POL 0xFC0`:** conditioned injector inputs treated as active HIGH and coil inputs treated as active LOW, per the previously discussed MOSFET interface assumptions. Verify signal polarity on the bench; this is NOT an assertion of ECU physical trigger polarity.

## DAC analog interface

The MCP4922 operates at 3.3 V with a **2.048 V external reference**, wired to VREFA and VREFB. The firmware uses 1x gain, channel-A SPI word `0x3000 | code`, channel-B `0xB000 | code`, and pulses LDAC after updating both. `/SHDN` should be HIGH. Bypass VDD and the reference properly. Connect VOUTA/B to the previously discussed dual rail-to-rail buffer and output protection, **not directly to the ECU**.

Firmware produces two complementary sampled sine waves centered at **1.024 V**; nominal unloaded differential peak is approximately **±1.8 V**. There are 8 samples per nominal tooth position, and missing positions drive the DAC outputs to equal midpoint voltage. The `WHEEL N M I` command changes slot count, missing teeth, and differential polarity. At RPM 0 both outputs return to midpoint and the cams/reference go low. Adjustable signal amplitude has **not** been implemented yet.

This is a voltage-driven, ground-referenced differential approximation to G28, not a galvanically isolated or true sensor equivalent. The external 820 Ω shunt and 100 Ω series resistors, if fitted, reduce the actual terminal differential voltage; measure it with an oscilloscope. Do not connect the ECU until common-mode bias, output current, and transient protection are verified. The APB cam phase defaults are provisional and not validated factory angles.

**Performance limitations:** DAC sampling is coordinated with the two cam GPIOs and GP22 reference in a busy-wait loop on core 1 using hardware SPI0 at 10 MHz. It is subject to microsecond jitter and overload, especially for high RPM or high slot counts. It is not yet synchronized by DMA/PIO hardware triggers. Angle capture uses PIO1 and DMA at 125 kSamples/s with 8 µs sampling. Validate both waveform jitter and marker alignment on a scope at each desired RPM before interpreting measured timing as exact.

## Capture mapping

PIO1 samples 13 adjacent pins GP10–GP22. GP10–GP15 map directly to coil 1–6 in ascending order. The six injector inputs arrive in **reverse** order, GP21..GP16; firmware reverses them into logical injector channels 0–5. Logical channels 6–11 remain coil 1–6. GP22 is the phase marker (bit 12 of the capture word), not a signal you need to wire externally. The existing `ANG`, `STAT`, `WHEEL`, `CAM`, `TDC`, `POL`, and `STREAM` protocol remains compatible with `controller.py`.

## Firmware build (Pico SDK v2.x)

Set up `PICO_SDK_PATH` and an ARM embedded compiler, then:

```sh
cmake -S . -B build -DPICO_BOARD=pico
cmake --build build -j4
```

A small `pico_sdk_import.cmake` wrapper is included and expects `PICO_SDK_PATH` to point to your pico-sdk checkout. Copy `build/me71_bench_sim.uf2` onto the Pico in BOOTSEL mode. USB CDC serial appears as COMx on Windows or `/dev/ttyACM*` on Linux.

## Desktop GUI

Python 3 with Tkinter installed (Linux: `python3-tk`) and pyserial:

```sh
python -m pip install -r requirements.txt
python controller.py --port COM5             # Windows example
python controller.py --port /dev/ttyACM0     # Linux example
```

Click **Set RPM** after connecting. For an initial safe bench diagnostic, set 200–300 RPM, verify that the ECU reports crank synchronization using your diagnostic instrument, and only then increase RPM.

## Configurable crank wheel (PC GUI)

The **Crank trigger wheel** panel controls the wheel geometry live; no firmware reflash is required:

- **Total teeth**: 12–120 nominal equally spaced tooth positions per revolution (default 60).
- **Missing teeth**: 1–16 absent tooth positions, always at the end of a revolution (default 2).
- **Invert crank signal**: switch high-going pulses to low-going pulses (default off).
- **Apply wheel** sends `WHEEL 60 2 0`; **Read wheel** sends `WHEEL?` to synchronize the GUI with Pico settings. On connection, the GUI also reads the configured wheel.

Changing the wheel immediately restarts all crank/cam/reference signals from slot zero; it does not change the requested RPM. Cam windows remain defined in **crank degrees** in the 720° cycle. With wheels other than 60 teeth, cam transitions are quantized to the new wheel-slot spacing (360/N degrees). The marker angle corresponds to the first slot after the missing-tooth gap. TDC offset remains a separately calibrated engine-specific setting. Measurements remain angle-referenced to that marker.

Settings are **volatile** and revert to defaults after a Pico reset. Only evenly spaced missing-tooth wheel patterns (one gap per revolution) are supported; this is not an arbitrary multi-gap or asymmetric encoder simulator. The current PIO waveform is fixed at 50% tooth duty. High RPM/large wheel combinations can exceed 125 kHz capture sampling resolution (the capture sampler is intended for ECU output channels, not high-speed validation of the simulated tooth shape). Verify electrical edges with an oscilloscope and condition the VR input appropriately.

## Serial commands

- `WHEEL 60 2 0` — configure nominal teeth, missing teeth and crank inversion; resets phase.
- `WHEEL?` — query current wheel configuration.
- `RPM 800` or `RPM 0` — changes the DAC engine RPM / stops signal.
- `STREAM 1` / `STREAM 0` — enable or disable individual input-edge messages.
- `POL 0xFC0` — 12-bit active-low polarity mask, bit 0 = INJ1, bit 6 = SPK1.
- `CAM G40 120 60 0` — start angle, pulse width (both crank degrees), inversion 0/1. Resets output phase on changes.
- `CAM G163 480 60 0` — same for second cam input. These are **calibration placeholders**.
- `CAM?` — read configured cam windows and waveform cycle counter.
- `TDC 0` — cylinder-1 firing TDC position within the 720° phase-marker cycle. **0 is only a placeholder**, not confirmed APB geometry; use your measured G28-to-TDC reference.
- `TDC?` — read the TDC offset and engine profile.
- `STAT` — emit counts, latest pulse widths and loss counters.
- `HELP` — help.

`STAT,rpm,samples,sample_drops,event_drops,<on,off,last_width_us repeated 12 times>` is emitted every 250 ms. `ANG,timestamp_us,I|S,cylinder,ON|OFF,pulse_width_us,cycle_angle_deg,relative_to_cylinder_TDC_deg,valid,cycle_number` is emitted when streaming. The angle is rounded to the nearest whole degree. Negative relative angles mean before the corresponding cylinder firing TDC, positive mean after; this is a nearest-TDC signed angle (range -360° to +359°), not a 0–720° cylinder phase. `-1` for cycle angle and `-999` for relative angle indicate invalid/unsynchronized readings. Angle precision is constrained by 8 µs sampling, especially at high RPM. **ON represents drive/trigger assertion, not the actual secondary spark moment.** Timing and the GP5 reference are both captured in the same PIO stream so they share a timebase (8 µs quantization). The timing is relative to the **simulated** crank and a **configurable, not yet factory-calibrated** TDC offset; it is not an independent mechanical crank-angle measurement. Pulse durations are recorded on the deassertion transition. GUI maintains cumulative counts until Pico restart. Sample drop and event queue drop indicators flag measurements that may be incomplete.

## Implementation notes

- SPI0 hardware shifts each 16-bit command to the MCP4922; `/CS` pulses separately for channels A and B, then `/LDAC` pulses once to update them simultaneously.
- Core 1 executes the DAC waveform loop and cam/marker outputs. Core 0 handles the PIO1 capture DMA and USB serial protocol.
- Input edges and the GP22 marker are acquired through the same PIO/DMA stream, allowing angles to be computed relative to that marker. Both are still subject to hardware/scheduling jitter and 8 µs quantization.
- A 32 KiB DMA ring covers roughly 65.5 ms. `STAT` reports sample/event drops; prolonged sessions require handling of 32-bit DMA transfer-count rollover.
- The firmware has **not been compiled with the Pico SDK or tested against an ECU** in this environment. Verify compiler output and physical signals before connection.

## Engine profile / adaptation

Edit `engine_profile.h` to adapt the simulator for a different engine. The default APB profile has firing order **1-4-3-6-2-5** (120° between firing TDC events), a 60-2 crank wheel, and two Hall cam outputs. The primary customization point is:

```c
static inline uint8_t engine_cam_bits(const engine_shape_t *shape, uint16_t slot);
```

Return bit 0 for G40 and bit 1 for G163 at each 6° slot in a 720° cycle. For a different cam trigger wheel, implement whatever pulse windows it requires here. The `engine_slot_word()` function handles multiplexing those cam bits with the 60-2 crank waveform. For different crank wheels or angular resolution, `engine_slot_word`, the PIO slot timing, and the `set_rpm` divider assumptions must also be updated.

**Important calibration warning:** The APB 2.7T service information verifies the firing order, two cam Hall sensors, and a brief sensor blink once per two revolutions, but does not verify the exact rising-edge position with respect to the 60-2 gap. The 120°/480° start positions and 60° widths shipped here are *examples only*, not OEM timing claims. Measure a real 2.7T G28/G40/G163 waveform, then set both start angles and widths accordingly. The engine firing order is metadata at this stage, not a separately synthesized injector/spark waveform, and the simulator does not infer true TDC alignment from those cam defaults.

The resolution is 6° crank; choose more granular PIO slots or a transition-scheduled output architecture if finer cam phasing is necessary. Verify VR polarity, Hall output voltage, input buffering, and signal phasing before connecting to the ECU.

## Angle measurements / calibration

The GUI shows the latest **ON** event phase (0–719°) and signed delta to the cylinder firing TDC. The event log includes ON and OFF edges and pulse widths. For injector events, ON means injector-driver assertion and OFF means driver deassertion; pulse width is electrical drive time, **not a directly measured fuel-delivery interval**. For spark events, timing refers to the configured ECU ignition trigger polarity. Actual coil discharge can be associated with the opposite edge depending on ignition-module behavior; use a scope to identify it.

The 720° phase convention is **GP5 rising = 0°**, at the same instant the generated crank slot 0 begins, immediately after the simulated missing-tooth slots. The first tooth is slot 0; this is not an asserted OEM TDC reference. To calibrate, determine the crank angle from that tooth to **cylinder 1 firing TDC** using a known good scope capture, then enter it as `TDC <0..719>` (e.g. `TDC 90` only if the real engine measurements establish 90°). Firing order in `engine_profile.h` defines cylinder-specific TDC targets 120° apart. For other engines, change `ENGINE_CYLINDERS`, `engine_firing_order`, and the 120° step in `relative_angle()` to `720/ENGINE_CYLINDERS` (requires an even-fire engine). Uneven-fire engines need an explicit per-cylinder TDC-angle table.

**Boundary limitations:** If 720° phase is not established, output angle is invalid; the first cycle after start uses a nominal-RPM period; later cycles use measured marker-to-marker sample counts. Sudden RPM changes temporarily invalidate/recalibrate this estimate. Very short events (8 µs or less) can be missed. This prototype does not compensate input conditioning delays or the output waveform phase delay caused by the VR interface. The GUI angle fields display ON/assertion angle, not an assumed actual spark time.

## 720° firing waterfall (PC GUI)

The **720° firing waterfall** tab uses streamed `ANG` records from the same capture reference as the numerical table. The vertical axis spans 0° (top) to 720° (bottom), referenced to the GP5 phase marker; the X axis has twelve columns (INJ1..INJ6, SPK1..SPK6). Every synchronized ON/assert edge is plotted as a point. With **Show pulse duration** enabled, OFF/deassert edges are drawn as short ticks and paired ON→OFF edges are connected vertically (split at 720° on wrap). Injector events are green and spark triggers are orange. Newer cycles are brighter, older cycles dimmer. The **Show latest cycles** control (1..200) selects the retained overlay window. **Freeze display** stops adding new points without stopping capture or the event log; **Clear waterfall** removes accumulated plot points.

The graph is a *cycle-overlaid timing raster*, not a graph of ECU power or spark voltage. Its X position denotes **channel**, not elapsed time, while brightness denotes age. A missing mark can indicate no firing or a lost input/serial event; check drop counters. ON edge denotes trigger assertion and may not correspond to ignition discharge. The plot uses whole-degree angles carried by the existing serial protocol and requires `STREAM 1`, running RPM and a valid phase reference. The plot is implemented with standard Tkinter Canvas and adds no Python dependencies.

### Persistence plot controls (PC GUI)

The firing plot uses six cylinder columns. Each column contains green injector
and orange ignition events at slightly offset X coordinates. The plot is fixed
vertically; events fade as new **engine cycles** arrive rather than scrolling.

- **Persistence cycles** (1–200; default 8): maximum amount of cycle history shown.
- **Fade speed** (1–10; default 5): how quickly older events dim within that history.
  The fade advances with simulated engine cycles, not wall-clock time; when RPM
  is stopped it stays in its most recent state.
- **Y-axis**: `Absolute (0–720°)` or
  `Relative to cylinder TDC (−360° to +360°)`.
  In relative mode 0° is the nominal firing TDC for the displayed cylinder;
  negative angles are before TDC. The relative mode depends on the simulator's
  configurable cylinder-1 TDC reference and engine firing-order profile.

A pulse that crosses the ±360° wrap in relative mode is shown as separate edge
marks rather than an incorrect long vertical line. No firmware changes are
necessary: the existing `ANG` serial packets already carry both angles.
