"""
Utility functions for the FACTR teleoperation system.
"""

import os


def find_ttyusb(port_name: str) -> str:
    """
    Locate the underlying ttyUSB device from a symbolic link.

    Args:
        port_name: Name of the port in /dev/serial/by-id/

    Returns:
        The ttyUSB device name (e.g., "ttyUSB0")

    Raises:
        Exception: If the port doesn't exist or doesn't correspond to a ttyUSB device
    """
    base_path = "/dev/serial/by-id/"
    full_path = os.path.join(base_path, port_name)

    if not os.path.exists(full_path):
        raise Exception(f"Port '{port_name}' does not exist in {base_path}.")

    try:
        resolved_path = os.readlink(full_path)
        actual_device = os.path.basename(resolved_path)

        if actual_device.startswith("ttyUSB"):
            return actual_device
        else:
            raise Exception(
                f"The port '{port_name}' does not correspond to a ttyUSB device. "
                f"It links to {resolved_path}."
            )
    except OSError as e:
        raise Exception(
            f"Unable to resolve the symbolic link for '{port_name}'. {e}"
        ) from e
