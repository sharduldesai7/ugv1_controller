# Arducam IMX477 Motorized Focus Setup

Required for software-controlled focus on Ubuntu 24.04 with the Arducam IMX477 motorized focus camera.
The standard `dtoverlay=imx477,vcm` does not work on Ubuntu 24.04 — a custom Arducam overlay is needed.

## Steps

### 1. Run the install script

```bash
cd ~/ros2_ws/src/ugv1_controller/scripts/
./arducam_imx477_focus_install.sh
```

### 2. Configure /boot/firmware/config.txt
camera_auto_detect=0
dtoverlay=imx477,vcm

### 3. Reboot

```bash
sudo reboot
```

### 4. Verify VCM loaded

```bash
sudo dmesg | grep -i "dw9807\|vcm"
```

### 5. Test focus control

```bash
rpicam-still --autofocus-mode manual --lens-position 5 \
  --tuning-file /usr/share/libcamera/ipa/rpi/vc4/imx477_af.json \
  -t 2000 -o /tmp/focus_test.jpg
```

## Notes

- lens_position range: 0.0 (infinity/far) to 10.0 (close/macro)
- For warehouse VIO use case, 0.5-2.0 is the useful range (0.5m-3m distance)
- The imx477_af.json tuning file is installed by the Arducam build script
- This overlay is separate from the standard Raspberry Pi OS overlay
