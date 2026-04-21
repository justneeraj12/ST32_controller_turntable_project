#!/usr/bin/env python3

import argparse
import json
import shutil
import subprocess
import sys
from pathlib import Path


VALID_CORES = {
    "Cortex-M0",
    "Cortex-M0plus",
    "Cortex-M0+",
    "Cortex-M3",
    "Cortex-M4",
    "Cortex-M7",
    "Cortex-M23",
    "Cortex-M33",
    "Cortex-M55",
    "Cortex-M85",
}


def infer_core(device_name: str) -> str:
    name = (device_name or "").upper()
    if "G0" in name:
        return "Cortex-M0+"
    if "F0" in name or "L0" in name:
        return "Cortex-M0"
    if "F1" in name or "L1" in name:
        return "Cortex-M3"
    if "F3" in name or "F4" in name or "G4" in name or "L4" in name:
        return "Cortex-M4"
    return ""


def resolve_workspace_path(path_value: str, workspace_root: Path) -> Path:
    return Path(path_value.replace("${workspaceFolder}", str(workspace_root))).expanduser()


def check_stlink_presence(warnings):
    if shutil.which("st-info") is None:
        warnings.append("st-info not found in PATH; cannot verify connected ST-Link device automatically.")
        return

    try:
        result = subprocess.run(
            ["st-info", "--probe"],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            check=False,
            timeout=6,
        )
    except Exception as exc:
        warnings.append(f"st-info probe failed to execute: {exc}")
        return

    if result.returncode != 0:
        warnings.append("st-info --probe failed; check ST-Link USB connection and udev permissions.")
        return

    print("[OK] ST-Link probe succeeded")
    for line in result.stdout.splitlines():
        if line.strip():
            print(f"      {line.strip()}")


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Verify and optionally auto-fix STM32 launch configuration for stlinkgdbtarget."
    )
    parser.add_argument("--fix", action="store_true", help="Auto-fix missing launch fields when possible.")
    parser.add_argument("--strict", action="store_true", help="Treat warnings as failures.")
    args = parser.parse_args()

    workspace_root = Path(__file__).resolve().parents[1]
    launch_path = workspace_root / ".vscode" / "launch.json"

    if not launch_path.exists():
        print(f"[FAIL] Missing launch file: {launch_path}")
        return 2

    try:
        data = json.loads(launch_path.read_text(encoding="utf-8"))
    except Exception as exc:
        print(f"[FAIL] Invalid JSON in {launch_path}: {exc}")
        return 2

    configs = data.get("configurations")
    if not isinstance(configs, list):
        print("[FAIL] launch.json is missing a valid 'configurations' array.")
        return 2

    changed = False
    errors = []
    warnings = []
    stlink_found = False

    for idx, cfg in enumerate(configs):
        if not isinstance(cfg, dict) or cfg.get("type") != "stlinkgdbtarget":
            continue

        stlink_found = True
        cfg_name = cfg.get("name", f"stlink-config-{idx}")

        if cfg.get("request") not in {"launch", "attach"}:
            if args.fix:
                cfg["request"] = "launch"
                changed = True
                print(f"[FIX] {cfg_name}: request -> launch")
            else:
                errors.append(f"{cfg_name}: request must be 'launch' or 'attach'.")

        device_name = cfg.get("deviceName") or cfg.get("deviceNumber")
        if not device_name:
            if args.fix:
                cfg["deviceName"] = "STM32G0B1RE"
                device_name = cfg["deviceName"]
                changed = True
                print(f"[FIX] {cfg_name}: deviceName -> STM32G0B1RE")
            else:
                errors.append(f"{cfg_name}: missing deviceName/deviceNumber.")

        device_core = cfg.get("deviceCore", "")
        if not device_core:
            inferred = infer_core(str(device_name))
            if args.fix and inferred:
                cfg["deviceCore"] = inferred
                device_core = inferred
                changed = True
                print(f"[FIX] {cfg_name}: deviceCore -> {inferred}")
            else:
                errors.append(f"{cfg_name}: missing deviceCore.")
        elif device_core not in VALID_CORES:
            warnings.append(
                f"{cfg_name}: deviceCore '{device_core}' is unusual. Expected one of: {sorted(VALID_CORES)}"
            )

        if not cfg.get("cwd"):
            if args.fix:
                cfg["cwd"] = "${workspaceFolder}"
                changed = True
                print(f"[FIX] {cfg_name}: cwd -> ${workspaceFolder}")
            else:
                warnings.append(f"{cfg_name}: missing cwd.")

        image_file = ""
        symbols = cfg.get("imagesAndSymbols")
        if isinstance(symbols, list) and symbols and isinstance(symbols[0], dict):
            image_file = symbols[0].get("imageFileName", "")

        if not image_file:
            image_file = cfg.get("program", "") or cfg.get("executableFile", "")

        if not image_file:
            if args.fix:
                cfg["imagesAndSymbols"] = [
                    {"imageFileName": "${workspaceFolder}/firmware/Debug/firmware.elf"}
                ]
                changed = True
                image_file = "${workspaceFolder}/firmware/Debug/firmware.elf"
                print(f"[FIX] {cfg_name}: imagesAndSymbols[0].imageFileName set to default ELF path")
            else:
                errors.append(f"{cfg_name}: missing program image path.")

        if image_file:
            resolved = resolve_workspace_path(str(image_file), workspace_root)
            if not resolved.exists():
                warnings.append(
                    f"{cfg_name}: ELF not found at {resolved}. Build once or update imageFileName."
                )

    if not stlink_found:
        errors.append("No configuration with type 'stlinkgdbtarget' was found.")

    check_stlink_presence(warnings)

    if changed:
        launch_path.write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")

    if errors:
        print("[FAIL] STM32 launch verification failed:")
        for msg in errors:
            print(f"  - {msg}")
        if warnings:
            print("[WARN] Additional warnings:")
            for msg in warnings:
                print(f"  - {msg}")
        return 2

    if warnings:
        print("[WARN] STM32 launch verification completed with warnings:")
        for msg in warnings:
            print(f"  - {msg}")
        return 3 if args.strict else 0

    print("[OK] STM32 launch configuration verified.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
