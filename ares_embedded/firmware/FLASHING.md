# Flashing Firmware to NUCLEO-H723ZG

## Prerequisites

1. **Connect the board:**
   - Connect NUCLEO-H723ZG to your computer via USB (ST-Link USB port)
   - The board should appear as a mass storage device

2. **Install flashing tools (choose one):**

### Option 1: OpenOCD (Recommended)
```bash
# macOS
brew install openocd

# Linux
sudo apt-get install openocd
```

### Option 2: STM32CubeProgrammer
Download from: https://www.st.com/en/development-tools/stm32cubeprog.html

### Option 3: st-flash (stlink tools)
```bash
# macOS
brew install stlink

# Linux
sudo apt-get install stlink-tools
```

---

## Flashing Methods

### Method 1: OpenOCD (Recommended)

**Location:** Run from `ares_embedded/` directory

```bash
# Build the firmware first
cmake -S . -B build -DCMAKE_TOOLCHAIN_FILE=cmake/toolchains/arm-none-eabi.cmake
cmake --build build

# Flash using OpenOCD
openocd -f interface/stlink.cfg \
        -f target/stm32h7x.cfg \
        -c "program build/firmware/firmware.elf verify reset exit"
```

**What it does:**
- Connects to ST-Link on the NUCLEO board
- Erases flash
- Programs `firmware.elf`
- Verifies the write
- Resets the board to run the new firmware

---

### Method 2: STM32CubeProgrammer (GUI)

1. Open STM32CubeProgrammer
2. Click "Connect" (ST-Link should auto-detect)
3. Click "Open File" → Select `build/firmware/firmware.elf`
4. Click "Download" (or press F8)
5. Click "Disconnect"

**Location:** Run STM32CubeProgrammer from anywhere, navigate to `build/firmware/firmware.elf`

---

### Method 3: st-flash (Command Line)

**Location:** Run from `ares_embedded/` directory

```bash
# Build the firmware first
cmake -S . -B build -DCMAKE_TOOLCHAIN_FILE=cmake/toolchains/arm-none-eabi.cmake
cmake --build build

# Convert ELF to binary (optional, st-flash can use ELF)
arm-none-eabi-objcopy -O binary build/firmware/firmware.elf build/firmware/firmware.bin

# Flash the binary
st-flash write build/firmware/firmware.bin 0x08000000
```

---

### Method 4: Drag and Drop (Easiest for Testing)

**Location:** Run from `ares_embedded/` directory

```bash
# Build the firmware
cmake -S . -B build -DCMAKE_TOOLCHAIN_FILE=cmake/toolchains/arm-none-eabi.cmake
cmake --build build

# Convert to binary
arm-none-eabi-objcopy -O binary -S build/firmware/firmware.elf build/firmware/firmware.bin

# The NUCLEO board appears as a USB drive when connected
# Simply drag firmware.bin to the NUCLEO drive
# The board will automatically flash and reset
```

**Note:** The NUCLEO board must be in "bootloader mode" or appear as a mass storage device.

---

## Verifying the Flash

After flashing, you should see:
- **Yellow LED (LD2, PE1)** blinking on for 2.5 seconds
- **Red LED (LD3, PB14)** blinking on for 2.5 seconds
- Pattern repeats every 5 seconds

---

## Troubleshooting

**Board not detected:**
- Check USB cable connection
- Try a different USB port
- Install ST-Link drivers if needed

**Flash fails:**
- Make sure no other program is using the ST-Link (close STM32CubeIDE, etc.)
- Try disconnecting and reconnecting the board
- Check that `firmware.elf` exists in `build/firmware/`

**LEDs don't blink:**
- Verify the firmware was flashed successfully
- Check that the board is powered (LEDs should be visible)
- Verify the pin definitions match your board revision

---

## Quick Test Command

**Location:** Run from `ares_embedded/` directory

```bash
# Build and flash in one go (using OpenOCD)
cmake -S . -B build -DCMAKE_TOOLCHAIN_FILE=cmake/toolchains/arm-none-eabi.cmake && \
cmake --build build && \
openocd -f interface/stlink.cfg -f target/stm32h7x.cfg \
        -c "program build/firmware/firmware.elf verify reset exit"
```
