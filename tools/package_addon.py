"""Packages the AVBD Physics add-on into an installable zip.

Usage:
    python tools/package_addon.py

Produces dist/avbd_physics-<version>.zip containing the add-on package and the
native libraries found under addon/avbd_physics/lib/. Build the `avbd` shared
library for every platform you want to support and drop the binaries into the
matching lib/<platform>/ folder before packaging.
"""

import os
import re
import zipfile

_REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_PKG = os.path.join(_REPO, "addon", "avbd_physics")
_DIST = os.path.join(_REPO, "dist")


def _version():
    init = os.path.join(_PKG, "__init__.py")
    with open(init, "r", encoding="utf-8") as fh:
        text = fh.read()
    match = re.search(r'"version":\s*\((\d+),\s*(\d+),\s*(\d+)\)', text)
    return ".".join(match.groups()) if match else "0.0.0"


def main():
    version = _version()
    os.makedirs(_DIST, exist_ok=True)
    out_path = os.path.join(_DIST, "avbd_physics-{}.zip".format(version))

    found_libs = []
    with zipfile.ZipFile(out_path, "w", zipfile.ZIP_DEFLATED) as zf:
        for root, dirs, files in os.walk(_PKG):
            dirs[:] = [d for d in dirs if d != "__pycache__"]
            for name in files:
                if name.endswith((".pyc", ".pyo")):
                    continue
                abs_path = os.path.join(root, name)
                # Store paths as avbd_physics/<...> so Blender installs cleanly.
                rel = os.path.relpath(abs_path, os.path.dirname(_PKG))
                zf.write(abs_path, rel)
                if name.startswith(("avbd", "libavbd")) and name.endswith(
                        (".dll", ".so", ".dylib")):
                    found_libs.append(rel)

    print("Wrote", out_path)
    if found_libs:
        print("Bundled native libraries:")
        for lib in found_libs:
            print("  ", lib)
    else:
        print("WARNING: no native libraries bundled - build the 'avbd' shared "
              "library and place it under addon/avbd_physics/lib/<platform>/.")


if __name__ == "__main__":
    main()
