"""Check the F051 port and prove unsupported firmware fails with its diagnostic."""

from pathlib import Path
import subprocess
import sys


def main():
    repo = Path(__file__).resolve().parent.parent
    command = [
        "cargo", "build", "--manifest-path=rm32_stm32/Cargo.toml", "--release",
        "--no-default-features", "--features=stm32f051", "--target=thumbv6m-none-eabi",
    ]
    subprocess.run([*command, "--lib"], cwd=repo, check=True)
    result = subprocess.run(
        [*command, "--bin=rm32_firmware"], cwd=repo,
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
    )
    diagnostic = "STM32F051 firmware is unavailable: the image exceeds the 27 KiB application flash partition."
    if result.returncode == 0 or diagnostic not in result.stdout:
        print(result.stdout)
        sys.exit("F051 guard failed: expected the explicit unsupported-firmware diagnostic")
    print("F051 library builds; firmware correctly rejected by the flash-fit guard.")


if __name__ == "__main__":
    main()
