"""Replace Seeed's unchecked RRAM load with verified OpenOCD upload."""
Import('env')

from pathlib import Path

# Build-only runs do not necessarily install a debug/upload tool package.
openocd_root = env.PioPlatform().get_package_dir('tool-openocd')
command = ('"$PYTHONEXE" "$PROJECT_DIR/scripts/nrf54_openocd.py" '
           'flash "$SOURCE"')
if openocd_root:
    openocd = Path(openocd_root) / 'bin' / 'openocd'
    command += ' --openocd "' + str(openocd) + '"'

if env.subst('$UPLOAD_PORT').strip() not in ('', '$UPLOAD_PORT'):
    command += ' --serial "$UPLOAD_PORT"'
env.Replace(UPLOADCMD=command)
