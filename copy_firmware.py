# PlatformIO post build script: copies the firmware .bin to the firmware/ folder with the 
# firmware name and version in the file name, for example firmware/DAQcvbias_v1.2.bin
#
# The name and version are taken from the Version string in src/DMSDMSMB.cpp, for example:
#   const char Version[] PROGMEM = "DAQcvbias version 1.2, June 23, 2024";
# so update the version there, in one place. There is one Version string for each firmware 
# variant, the one whose name matches the PlatformIO environment (cvbias or waveforms) is used.
# Building again without changing the version replaces the file of the same name.
import os
import re
import shutil
Import("env")

def copy_firmware(source, target, env):
    projdir = env.subst("$PROJECT_DIR")
    variant = env["PIOENV"]
    with open(os.path.join(projdir, "src", "DMSDMSMB.cpp")) as f:
        text = f.read()
    # Find the Version string for this variant, name followed by "version" and the number
    m = re.search(r'Version\[\][^"]*"(DAQ' + re.escape(variant) + r')\s+version\s+([0-9][0-9.]*[0-9])', text, re.I)
    if not m:
        print("copy_firmware: can't find the version string for '%s' in src/DMSDMSMB.cpp" % variant)
        env.Exit(1)
    name = "%s_v%s.bin" % (m.group(1), m.group(2))
    outdir = os.path.join(projdir, "firmware")
    os.makedirs(outdir, exist_ok=True)
    src = str(target[0])
    dst = os.path.join(outdir, name)
    shutil.copyfile(src, dst)
    print("Firmware copied to firmware/%s" % name)

env.AddPostAction("$BUILD_DIR/${PROGNAME}.bin", copy_firmware)
