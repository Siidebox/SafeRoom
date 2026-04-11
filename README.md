# SafeRoom - Indoor Presence Monitoring with mmWave Radar
> [!IMPORTANT]  
Not finished
Privacy-oriented indoor presence and movement monitoring system using a millimeter-wave radar sensor.

## Hardware

- **Radar**: Texas Instruments IWR6843AOPEVM Rev G (Out-of-Box Demo, SDK 3.6)
- **Processing**: Raspberry Pi 5 (8 GB RAM)
- **Connection**: USB (CP2105 dual UART bridge)

## Project Structure
```
mmwave-project/
├── config/             # Radar configuration files (.cfg)
├── scripts/            # Utility and test scripts
│   └── basic_read.py   # Basic communication test
├── src/                # Main source code
│   └── radar/          # Radar communication module
├── data/               # Captured data (not tracked in git)
│   └── captures/
├── docs/               # Project documentation
├── tests/              # Unit tests
├── requirements.txt    # Python dependencies
└── README.md
```

## Quick Start

### Prerequisites

- Raspberry Pi OS (64-bit, Bookworm)
- Python 3 with virtual environment
- IWR6843AOPEVM flashed with Out-of-Box Demo (SDK 3.6)

### Setup
```bash
git clone https://github.com/YOUR_USERNAME/mmwave-project.git
cd mmwave-project
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

### Udev Rules (fixed port names)
```bash
sudo bash -c 'cat > /etc/udev/rules.d/99-mmwave.rules << UDEV
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea70", ENV{ID_USB_INTERFACE_NUM}=="00", SYMLINK+="mmwave_cli"
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea70", ENV{ID_USB_INTERFACE_NUM}=="01", SYMLINK+="mmwave_data"
UDEV'
sudo udevadm control --reload-rules
sudo udevadm trigger
```

### User Permissions
```bash
sudo usermod -aG dialout $USER
# Log out and back in
```

### Run Basic Test
```bash
source venv/bin/activate
python scripts/basic_read.py
```

## Serial Port Mapping

| Port | Symlink | Function | Baud Rate |
|------|---------|----------|-----------|
| /dev/ttyUSB0 | /dev/mmwave_cli | CLI / CFG (commands) | 115200 |
| /dev/ttyUSB1 | /dev/mmwave_data | Data (point cloud) | 921600 |

## License

TBD
