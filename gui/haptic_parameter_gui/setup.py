from setuptools import setup


package_name = "haptic_parameter_gui"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    install_requires=["PySide6>=6.7"],
    zip_safe=True,
    maintainer="HX Lab",
    maintainer_email="",
    description="Runtime parameter GUI for haptic teleoperation controllers.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "haptic-parameter-gui = haptic_parameter_gui.main:main",
        ],
    },
)
