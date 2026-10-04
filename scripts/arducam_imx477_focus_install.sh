#!/bin/bash

source /etc/os-release

support_systems=("bookworm" "bullseye" "trixie")
version_codename=$(echo $VERSION_CODENAME)
found=0

for system in "${support_systems[@]}"; do
    if [[ "$version_codename" == "$system" ]]; then
        found=1
        break
    fi
done

if [ $found -eq 0 ]; then
    echo -e "\033[31m Unsupported systems! \033[0m"
    exit
fi

echo "The current system is: "$version_codename
dtc -@ -I dts -O dtb -o imx477.dtbo ./$version_codename/imx477-overlay.dts
sudo cp /boot/overlays/imx477.dtbo /boot/overlays/imx477.dtbo.bak
sudo cp imx477.dtbo /boot/overlays/imx477.dtbo

echo "Complete!"