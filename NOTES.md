# Notes

This document serves as a temporary "scratch paper" for me to write notes on.

## Checking USB Information

- Use lsusb to see basic device information:

    ```bash
    lsusb -v
    ```

- For more specific USB port and device details (000M for USB 3.0, 10000M for USB 3.1, etc.):

    ```bash
    lsusb -t
    ```

- Check system logs for USB connection details:

    ```bash
    dmesg | grep -i usb
    ```

- Use upower to check power-related information:

    ```bash
    upower -d | grep -A 10 usb
    ```

- For detailed power delivery information (if your USB supports USB-PD):

    ```bash
    sudo apt install usbutils
    sudo lsusb -v | grep -i "power"
    ```
