# Introduction

```
Repository to move image to gstreamer
```

# Dependencies
```
sudo apt install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev
```

## launch variables
```
ros2 launch image_to_gstreamer image_to_gstreamer.launch.py host:=127.0.0.1 port:=5001
```
## terminal launch for gstreamer
```
gst-launch-1.0 udpsrc port=5001 caps="application/x-rtp,media=video,encoding-name=H265,payload=96" ! rtph265depay ! avdec_h265 ! videoconvert ! fpsdisplaysink
```

## Using pre-commit

This project uses [pre-commit](https://github.com/pre-commit/pre-commit) to manage automated checks.

### Install pre-commit
```bash
pip install pre-commit
```

### Run all hooks manually
```bash
pre-commit run --all-files
```

### Install the git hook
This will make the checks run automatically every time you `git commit`:
```bash
pre-commit install
```

### Update pre-commit hooks
```bash
pre-commit autoupdate
```

## GitHub Actions CI
- This project uses GitHub Actions for CI.
- Workflows are located in [`.github/workflows`](.github/workflows/).
- For more information, see [vortex-ci](https://github.com/vortexntnu/vortex-ci).
