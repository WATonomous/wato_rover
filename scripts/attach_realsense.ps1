# Attaches the RealSense camera using usbipd so it is available in WSL (can see the device after running lsusb)
$busid = (usbipd list | Select-String "RealSense" | ForEach-Object { ($_ -split '\s+')[0] })
usbipd attach --wsl --busid $busid
