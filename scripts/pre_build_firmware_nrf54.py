#!/usr/bin/env python3
"""
Prepare the Morse HaLow Zephyr module for the nRF54 PlatformIO build.
"""
Import("env")
import os
import re
import shutil
import shlex
from pathlib import Path

PROJECT_DIR = Path(env.subst("$PROJECT_DIR")).resolve()
env.Replace(PROJECT_SRC_DIR=str(PROJECT_DIR / "nrf54" / "src"))

MORSE_CHIP = env.GetProjectOption("custom_morse_chip", "mm6108").lower()
if MORSE_CHIP not in ("mm6108", "mm8108"):
    print(f"ERROR: Unsupported custom_morse_chip: {MORSE_CHIP}")
    env.Exit(1)
MORSE_FIRMWARE = "mm8108b2-rl.mbin" if MORSE_CHIP == "mm8108" else "mm6108.mbin"

module_root = (PROJECT_DIR / "platforms" / "mm-iot-zephyr").resolve()
fallback_root = Path(os.environ.get("MMIOT_ZEPHYR_ROOT", PROJECT_DIR / ".." / "mm-iot-zephyr")).expanduser().resolve()


def is_complete_morse_module(path):
    return (
        path.exists()
        and (path / "CMakeLists.txt").exists()
        and (path / "components" / "morsemicro" / "mbedtls" / "morse_mbedtls_config.h").exists()
    )


def patch_zephyr_440_platformio_builder():
    """Bridge Zephyr 4.4 CMake targets that Seeed's SCons builder omits."""
    platform = env.PioPlatform()
    framework_root = platform.get_package_dir("framework-zephyr-nrf54l15")
    if not framework_root:
        return

    builder = Path(framework_root) / "scripts" / "platformio" / "platformio-build.py"
    if not builder.is_file():
        return

    old = 'env.Append(LIBS=lib_dep["lib_path"])'
    new = 'env.Append(LIBS=[lib_dep["lib_path"]])'
    contents = builder.read_text(encoding="utf-8")
    changed = False
    if old in contents:
        contents = contents.replace(old, new)
        changed = True

    anchor = "target_configs = load_target_configurations(codemodel)\n"
    morse_build = '''\n# PlatformIO consumes CMake's code model but does not execute custom targets.\n# Build Morse's merged/mangled archive before SCons resolves its dependencies.\nif os.environ.get("MORSE_SM_USE_APP_BINARIES"):\n    ninja = os.path.join(platform.get_package_dir("tool-ninja"), "ninja")\n    subprocess.check_call([ninja, "-C", BUILD_DIR, "libmorse_build"])\n'''
    previous_morse_build = morse_build
    # Ninja may regenerate CMake after a module source list changes. Refresh
    # the model before SCons chooses objects, otherwise new RF sources are
    # silently omitted until a second build invocation.
    morse_build += '    codemodel = get_cmake_code_model(west_manifest)\n    target_configs = load_target_configurations(codemodel)\n'
    if previous_morse_build in contents and morse_build not in contents:
        contents = contents.replace(previous_morse_build, morse_build)
        changed = True
    if morse_build not in contents:
        if anchor not in contents:
            print("ERROR: Unsupported Zephyr 4.4 PlatformIO builder layout")
            env.Exit(1)
        contents = contents.replace(anchor, anchor + morse_build)
        changed = True

    if changed:
        builder.write_text(contents, encoding="utf-8")
        print("Patched Zephyr 4.4.0 PlatformIO Morse build integration")


patch_zephyr_440_platformio_builder()


mm_root = module_root if is_complete_morse_module(module_root) else fallback_root
if not is_complete_morse_module(mm_root):
    print(f"ERROR: Complete mm-iot-zephyr source checkout not found at {mm_root}")
    env.Exit(1)

def patch_sensor_sleep_irq_pull(module):
    """Keep the optional sensor sleep-pin fix reproducible in vendor sources."""
    path = module / "drivers/wifi/morsemicro/morsemicro_spi.c"
    source = path.read_text(encoding="utf-8")
    marker = "CONFIG_OFFGRID_NRF54_SENSOR_SPI_IRQ_SLEEP_NOPULL"
    if marker in source:
        return
    enable = '\tif (enabled) {\n\t\t/* The transiver'
    disable = ('\t} else {\n'
               '\t\tgpio_pin_interrupt_configure_dt(&bus_config.spi_irq, GPIO_INT_DISABLE);\n'
               '\t}\n}\n\nstatic int morsemicro_bus_init')
    if source.count(enable) != 1 or source.count(disable) != 1:
        raise RuntimeError("Unsupported Morse SPI IRQ layout for sensor sleep pull fix")
    source = source.replace(enable, '\tif (enabled) {\n'
        '#if defined(' + marker + ')\n'
        '\t\t/* Restore open-drain bias before reading/unmasking IRQ. */\n'
        '\t\tgpio_pin_configure_dt(&bus_config.spi_irq, GPIO_INPUT | GPIO_PULL_UP);\n'
        '#endif\n\t\t/* The transiver')
    source = source.replace(disable, '\t} else {\n'
        '\t\tgpio_pin_interrupt_configure_dt(&bus_config.spi_irq, GPIO_INT_DISABLE);\n'
        '#if defined(' + marker + ')\n'
        '\t\t/* IRQ is masked before retained sleep; BUSY remains the wake source. */\n'
        '\t\tgpio_pin_configure_dt(&bus_config.spi_irq, GPIO_INPUT);\n'
        '#endif\n\t}\n}\n\nstatic int morsemicro_bus_init')
    path.write_text(source, encoding="utf-8")
    print("Prepared optional sensor sleep SPI IRQ pull fix")


patch_sensor_sleep_irq_pull(mm_root)

def patch_sensor_spi_sleep_pins(module):
    """Reapply the opt-in SPI PM experiment without changing SDK PS policy."""
    marker = "CONFIG_OFFGRID_NRF54_SENSOR_SPI_SLEEP_PINS"
    path = module / "drivers/wifi/morsemicro/morsemicro_spi.c"
    source = path.read_text(encoding="utf-8")
    fragment = (PROJECT_DIR / "patches/morsemicro_spi_sleep_pins.inc").read_text(encoding="utf-8")
    if marker not in source:
        replacements = {
            "static uint8_t training_sequence[]": fragment + "static uint8_t training_sequence[]",
            "void mmhal_wlan_set_spi_irq_enabled(bool enabled)\n{\n\tif (enabled) {\n":
                "void mmhal_wlan_set_spi_irq_enabled(bool enabled)\n{\n\tif (enabled) {\n"
                "#if defined(" + marker + ")\n"
                "\t\tif (morsemicro_sensor_spi_resume() != 0) return;\n#endif\n",
            "\t\tgpio_pin_configure_dt(&bus_config.spi_irq, GPIO_INPUT);\n#endif\n\t}\n}":
                "\t\tgpio_pin_configure_dt(&bus_config.spi_irq, GPIO_INPUT);\n#endif\n"
                "#if defined(" + marker + ")\n"
                "\t\tmorsemicro_sensor_spi_suspend();\n#endif\n\t}\n}",
        }
        for anchor, replacement in replacements.items():
            if source.count(anchor) != 1:
                raise RuntimeError("Unsupported Morse SPI layout for sleep-pin experiment: " + anchor[:60])
            source = source.replace(anchor, replacement, 1)
        # Resume before every SPI operation as well as before IRQ re-enable.
        for signature, failure in [
            ("uint8_t mmhal_wlan_spi_rw(uint8_t data)", "return 0xff;"),
            ("void mmhal_wlan_spi_read_buf(uint8_t *buf, unsigned len)", "memset(buf, 0xff, len); return;"),
            ("void mmhal_wlan_spi_write_buf(const uint8_t *buf, unsigned len)", "return;"),
            ("void mmhal_wlan_send_training_seq(void)", "return;"),
        ]:
            anchor = signature + "\n{\n"
            if source.count(anchor) != 1:
                raise RuntimeError("Unsupported Morse SPI function: " + signature)
            source = source.replace(anchor, anchor + "#if defined(" + marker + ")\n"
                "\tif (morsemicro_sensor_spi_resume() != 0) { " + failure + " }\n#endif\n", 1)
        path.write_text(source, encoding="utf-8")
    else:
        anchor = "#if defined(" + marker + ")\n#include <zephyr/pm/device.h>"
        start = source.index(anchor)
        end = source.index("static uint8_t training_sequence[]", start)
        updated = source[:start] + fragment + source[end:]
        if updated != source:
            path.write_text(updated, encoding="utf-8")
    path = module / "drivers/wifi/morsemicro/shims/mmhal_wlan.c"
    source = path.read_text(encoding="utf-8")
    if marker not in source:
        declaration = "extern const struct device *morsemicro_dev;\n"
        wake = "void mmhal_wlan_wake_assert(void)\n{\n"
        if source.count(declaration) != 1 or source.count(wake) != 2:
            raise RuntimeError("Unsupported Morse wake shim for SPI restore")
        source = source.replace(declaration, declaration + "#if defined(" + marker + ")\n"
            "extern int morsemicro_sensor_spi_resume(void);\n#endif\n", 1)
        source = source.replace(wake, wake + "#if defined(" + marker + ")\n"
            "\tif (morsemicro_sensor_spi_resume() != 0) return;\n#endif\n", 1)
        path.write_text(source, encoding="utf-8")


patch_sensor_spi_sleep_pins(mm_root)

pio_env = env.get("PIOENV", "")
edgez_source_option = ""  # Public example always uses protected SDK prebuilts.
edgez_source_root = None
if edgez_source_option:
    edgez_source_root = Path(edgez_source_option).expanduser().resolve()
    if not (
        (edgez_source_root / "ports" / "zephyr" / "component.cmake").is_file()
        and (edgez_source_root / "include" / "edgez_diagnostics.h").is_file()
    ):
        print(f"ERROR: EdgeZ HaLow mesh source checkout not found at {edgez_source_root}")
        env.Exit(1)


def first_existing(paths):
    for path in paths:
        if path.exists():
            return path
    return None


def read_kconfig_string(path, option, required=True):
    pattern = re.compile(rf'^\s*{re.escape(option)}\s*=\s*"([^"]+)"\s*(?:#.*)?$')

    if not path.exists():
        print(f"ERROR: Kconfig file not found: {path}")
        env.Exit(1)

    for line in path.read_text(encoding="utf-8").splitlines():
        match = pattern.match(line)
        if match:
            return match.group(1)

    if required:
        print(f"ERROR: {option} is not configured in {path}")
        env.Exit(1)
    return None

env.setdefault("ENV", {})
env["ENV"]["EDGE_DEVICE_ENABLE_HALOW"] = "1"
env["ENV"]["MORSE_ZEPHYR_MODULE"] = str(mm_root)
os.environ["EDGE_DEVICE_ENABLE_HALOW"] = "1"
os.environ["MORSE_ZEPHYR_MODULE"] = str(mm_root)
if edgez_source_root is not None:
    env["ENV"]["EDGEZ_HALOW_MESH_SOURCE_ROOT"] = str(edgez_source_root)
    os.environ["EDGEZ_HALOW_MESH_SOURCE_ROOT"] = str(edgez_source_root)
    env.AppendUnique(CPPPATH=[str(edgez_source_root / "include")])
else:
    env["ENV"].pop("EDGEZ_HALOW_MESH_SOURCE_ROOT", None)
    os.environ.pop("EDGEZ_HALOW_MESH_SOURCE_ROOT", None)
    edgez_prebuilt_include = mm_root / "submodules" / "edgez-halow-mesh-prebuilt" / "include"
    if not (edgez_prebuilt_include / "edgez_mesh_zephyr.h").is_file():
        print(f"ERROR: EdgeZ HaLow mesh public API headers not found at {edgez_prebuilt_include}")
        env.Exit(1)
    # PlatformIO recompiles the application sources outside Zephyr's CMake
    # target graph, so explicitly carry the public API include directory into
    # that second compilation phase.
    env.AppendUnique(CPPPATH=[str(edgez_prebuilt_include)])
    # PlatformIO recompiles application sources outside Zephyr's CMake target
    # graph, so mirror the release-archive definition into its SCons flags.
    env.AppendUnique(CPPDEFINES=[("EDGEZ_PLATFORM_ARCHIVE_BUILD", 1)])

if MORSE_CHIP == "mm8108":
    bcf_filename = "bcf_mf15457.mbin"
elif "hc01" in pio_env:
    bcf_filename = "bcf_HC01_V2_3V3.mbin"
else:
    bcf_filename = "bcf_fgh100mhaamd.mbin"
# An environment-specific fragment selects both the Zephyr override and the
# matching BCF blob staged by this script.
extra_conf = env.GetProjectOption("custom_zephyr_extra_conf", "")
if extra_conf:
    extra_conf_paths = []
    for fragment in extra_conf.split(","):
        fragment = fragment.strip()
        if not fragment:
            continue
        path = (PROJECT_DIR / fragment).resolve()
        if not path.is_file():
            print(f"ERROR: Zephyr config fragment not found: {path}")
            env.Exit(1)
        extra_conf_paths.append(str(path))
    if not extra_conf_paths:
        print("ERROR: custom_zephyr_extra_conf did not name a config fragment")
        env.Exit(1)
    board = env.BoardConfig()
    cmake_args = board.get("build.zephyr.cmake_extra_args", "")
    board.update(
        "build.zephyr.cmake_extra_args",
        cmake_args + " " + shlex.quote("-DEXTRA_CONF_FILE=" + ";".join(extra_conf_paths)),
    )

if Path(bcf_filename).name != bcf_filename:
    print(f"ERROR: Morse BCF must contain a filename, got: {bcf_filename}")
    env.Exit(1)
source_roots = [mm_root]
if fallback_root != mm_root:
    source_roots.append(fallback_root)

blob_dir = mm_root / "zephyr" / "blobs"
staged_files = [
    (
        first_existing(
            [root / "zephyr" / "blobs" / "lib" / "mm6108" / "arm-cortex-m33f" / "libmorse.a" for root in source_roots]
            + [root / "submodules" / "mm-iot-sdk" / "framework" / "morselib" / "lib" / "arm-cortex-m33f" / "libmorse.a" for root in source_roots]
        ),
        blob_dir / "lib" / "mm6108" / "arm-cortex-m33f" / "libmorse.a",
        "libmorse.a",
    ),
    (
        first_existing(
            [root / "zephyr" / "blobs" / "firmware" / MORSE_FIRMWARE for root in source_roots]
            + [root / "submodules" / "mm-iot-sdk" / "framework" / "morsefirmware" / MORSE_FIRMWARE for root in source_roots]
        ),
        blob_dir / "firmware" / MORSE_FIRMWARE,
        MORSE_FIRMWARE,
    ),
    (
        first_existing(
            [root / "submodules" / "mm-iot-sdk" / "framework" / "morsefirmware" / MORSE_CHIP / "bcfs" / bcf_filename for root in source_roots]
        ),
        blob_dir / "firmware" / bcf_filename,
        bcf_filename,
    ),
]

for src, dst, label in staged_files:
    if src is None:
        print(f"ERROR: {label} not found in project or Morse module checkouts")
        env.Exit(1)
    dst.parent.mkdir(parents=True, exist_ok=True)
    if not dst.exists() or src.read_bytes() != dst.read_bytes():
        shutil.copyfile(src, dst)
        print(f"Staged {label}: {dst}")

env["ENV"]["MORSE_SM_USE_APP_BINARIES"] = "1"
os.environ["MORSE_SM_USE_APP_BINARIES"] = "1"

print(f"Using Morse Zephyr source module: {mm_root} ({MORSE_CHIP}, BCF {bcf_filename})")
if edgez_source_root is not None:
    print(f"Using EdgeZ HaLow mesh source with diagnostics: {edgez_source_root}")
else:
    print("Using EdgeZ HaLow mesh release prebuilt")
