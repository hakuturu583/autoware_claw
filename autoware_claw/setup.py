from setuptools import setup

package_name = "autoware_claw"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    install_requires=["setuptools", "httpx", "websockets"],
    zip_safe=True,
    entry_points={
        "console_scripts": [
            "autoware_mcp_server = autoware_claw.autoware_mcp_server:main",
        ],
    },
)
