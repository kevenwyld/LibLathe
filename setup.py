import os
import re

from setuptools import find_packages, setup

here = os.path.dirname(os.path.abspath(__file__))

with open(os.path.join(here, "liblathe", "version.py"), encoding="utf-8") as f:
    match = re.search(r"^__version__ = ['\"]([^'\"]*)['\"]", f.read(), re.M)
if not match:
    raise RuntimeError("Could not find __version__ in liblathe/version.py")

with open(os.path.join(here, "README.md"), encoding="utf-8") as f:
    long_description = f.read()

setup(
    name="liblathe",
    version=match.group(1),
    description="Python library for generating lathe paths and G-code",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/kevenwyld/LibLathe",
    author="Keven Wyld",
    license="GPL-2.0-or-later",
    keywords="lathe turning CAD CAM CNC",
    packages=find_packages(exclude=["tests*"]),
    python_requires=">=3.8",
    install_requires=["numpy", "pillow"],
    classifiers=[
        "Development Status :: 3 - Alpha",
        "License :: OSI Approved :: GNU General Public License v2 or later (GPLv2+)",
        "Intended Audience :: Manufacturing",
        "Operating System :: OS Independent",
        "Programming Language :: Python :: 3",
    ],
)
