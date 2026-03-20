#!/usr/bin/env python3
"""
BHR Battery Diagnostics Tool - Release Preparation Script
This script copies all necessary firmware files to a distribution folder
Supports both ESP32 and AVR (Arduino Uno) platforms
"""

import argparse
import os
import shutil
import sys
from pathlib import Path

def select_build_directory(project_root, build_selector=None):
    """Let the user select a PlatformIO build directory."""
    build_root = project_root / ".pio" / "build"

    if not build_root.exists() or not build_root.is_dir():
        print("[ERROR] Build directory not found")
        print(f"   Expected: {build_root}")
        return None

    all_build_dirs = sorted([path for path in build_root.iterdir() if path.is_dir()])
    build_dirs = [path for path in all_build_dirs if (path / "firmware.bin").exists()]

    if not all_build_dirs:
        print("[ERROR] No build directories found")
        print(f"   Location: {build_root}")
        return None

    if not build_dirs:
        print("[ERROR] No releasable build directories found")
        print("   A releasable build must contain firmware.bin")
        print(f"   Location: {build_root}")
        return None

    if build_selector:
        selector = str(build_selector).strip()
        selected_by_name = next((path for path in build_dirs if path.name == selector), None)
        if selected_by_name is not None:
            print(f"[*] Selected build (argument): {selected_by_name.name}")
            return selected_by_name

        if selector.isdigit():
            selected_index = int(selector)
            if 1 <= selected_index <= len(build_dirs):
                selected_by_index = build_dirs[selected_index - 1]
                print(f"[*] Selected build (argument): {selected_by_index.name}")
                return selected_by_index

        print(f"[ERROR] Invalid --build value: {build_selector}")
        print("   Use an exact build name or list index")
        return None

    if len(build_dirs) == 1:
        selected = build_dirs[0]
        print(f"[*] Using build: {selected.name}")
        return selected

    if not sys.stdin.isatty():
        print("[ERROR] Interactive selection is not available in this terminal")
        print("   Re-run with --build <name|index>")
        print("   Example: python release.py --build heltec_wifi_kit_32")
        return None

    print("[*] Available builds:")
    for index, build_dir in enumerate(build_dirs, start=1):
        print(f"   {index}. {build_dir.name}")

    while True:
        choice = input("Select build number or name to release: ").strip()

        if not choice:
            print("[WARN] Please enter a selection")
            continue

        selected = next((path for path in build_dirs if path.name == choice), None)
        if selected is not None:
            print(f"[*] Selected build: {selected.name}")
            return selected

        if not choice.isdigit():
            print("[WARN] Enter a valid number or exact build name")
            continue

        selected_index = int(choice)
        if 1 <= selected_index <= len(build_dirs):
            selected = build_dirs[selected_index - 1]
            print(f"[*] Selected build: {selected.name}")
            return selected

        print(f"[WARN] Please choose a number between 1 and {len(build_dirs)}")


def prepare_esp32_release(project_root, release_dir, build_dir):
    """Prepare ESP32 firmware release"""
    print("\n[*] Preparing ESP32 firmware...")
    print(f"   Build source: {build_dir.name}")

    firmware_dir = release_dir / "firmware" / build_dir.name
    
    # Check if build directory exists
    if not build_dir.exists():
        print("[SKIP] ESP32 build directory not found")
        print(f"   Expected: {build_dir}")
        return False
    
    firmware_dir.mkdir(parents=True, exist_ok=True)
    
    # Try to find boot_app0.bin in platformio packages
    pio_packages = Path.home() / ".platformio" / "packages"
    boot_app0_search_paths = [
        build_dir / "boot_app0.bin",
        pio_packages / "framework-arduinoespressif32" / "tools" / "partitions" / "boot_app0.bin",
        pio_packages / "framework-espidf" / "components" / "partition_table" / "boot_app0.bin",
    ]
    
    boot_app0_source = None
    for path in boot_app0_search_paths:
        if path.exists():
            boot_app0_source = path
            break
    
    # Files to copy from build directory
    firmware_files = {
        "bootloader.bin": build_dir / "bootloader.bin",
        "partitions.bin": build_dir / "partitions.bin",
        "firmware.bin": build_dir / "firmware.bin",
    }
    
    # Add boot_app0.bin if found
    if boot_app0_source:
        firmware_files["boot_app0.bin"] = boot_app0_source
    
    # Copy firmware files
    missing_files = []
    for dest_name, src_path in firmware_files.items():
        dest_path = firmware_dir / dest_name
        if src_path.exists():
            shutil.copy2(src_path, dest_path)
            size_kb = dest_path.stat().st_size / 1024
            print(f"   [OK] {dest_name} ({size_kb:.1f} KB)")
        else:
            print(f"   [FAIL] {dest_name} - NOT FOUND")
            missing_files.append(str(src_path))
    
    # Create boot_app0.bin if not found
    if not (firmware_dir / "boot_app0.bin").exists():
        print("   [INFO] Creating boot_app0.bin (standard ESP32 boot selector)")
        boot_app0_data = bytearray([0xFF] * 4096)
        boot_app0_data[0:4] = b'\xaa\x50\x01\x00'
        with open(firmware_dir / "boot_app0.bin", "wb") as f:
            f.write(boot_app0_data)
        print(f"   [OK] boot_app0.bin (4.0 KB) - created")
    
    if missing_files:
        print("\n[ERROR] Some ESP32 firmware files are missing:")
        for f in missing_files:
            print(f"   - {f}")
        return False
    
    return True

def main():
    print("=" * 60)
    print("Rework TC Release Script")
    print("=" * 60)
    print()
    
    # Define paths
    project_root = Path(__file__).parent
    release_dir = project_root / "release"
    parser = argparse.ArgumentParser(description="Prepare release files from PlatformIO build output")
    parser.add_argument(
        "--build",
        help="Build directory name or index from .pio/build to release (optional)",
    )
    args = parser.parse_args()

    selected_build_dir = select_build_directory(project_root, args.build)
    if selected_build_dir is None:
        sys.exit(1)
    
    # Prepare selected build
    esp32_success = prepare_esp32_release(project_root, release_dir, selected_build_dir)

    
    if not esp32_success:
        print("\n[ERROR] No ESP32 firmware build found!")
        print("Please build the ESP32 platform first:")
        print("   For ESP32: pio run -e wemos_d1_mini32")
        sys.exit(1)
    
    
    # Print summary
    print("\n" + "=" * 60)
    print("SUCCESS! Release package created")
    print("=" * 60)
    print(f"\nRelease directory: {release_dir}")
    print(f"Variant directory: {release_dir / 'firmware' / selected_build_dir.name}")
    print("\nContents:")
    print(f"  [DIR] firmware/{selected_build_dir.name}/")
    print("     - bootloader.bin")
    print("     - partitions.bin")
    print("     - boot_app0.bin")
    print("     - firmware.bin")
    print("\nReady to flash!")
    print()

if __name__ == "__main__":
    main()
