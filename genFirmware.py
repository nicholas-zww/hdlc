Import("env")
import shutil
import os

def after_build(source, target, env):
    # Path to the generated firmware
    firmware_path = target[0].get_abspath()

    # Path where you want to copy the firmware
    new_location = os.path.join(env.get("PROJECT_DIR"), "firmware.bin")

    print(f"Copying {firmware_path} to {new_location}")
    shutil.copy(firmware_path, new_location)

env.AddPostAction("$BUILD_DIR/${PROGNAME}.bin", after_build)
