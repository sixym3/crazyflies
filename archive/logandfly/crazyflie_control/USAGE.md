# Quick Usage Guide

## ✅ Import Issues Fixed!

The modular Crazyflie control system is now fully functional with all import issues resolved.

## 🚀 How to Run

### From the command line you used before:

```bash
# Navigate to the modular system
cd /home/eric/crazyflies/logandfly/crazyflie_control

# Launch with 3D visualization (equivalent to original)
python3 scripts/launch_3d.py

# Or launch headless (no GUI conflicts)
python3 scripts/launch_headless.py

# Or use the configurable launcher
python3 scripts/drone_bringup.py --help
```

## 🎯 Key Advantages

1. **No More Import Errors**: All relative import issues fixed
2. **No GUI Conflicts**: Headless mode eliminates PyQt5/terminal conflicts
3. **Original Functionality**: Same controls and autonomous navigation
4. **Modular Design**: Clean, separated components
5. **Multiple Options**: Choose your preferred visualization

## 🎮 Controls (Same as Original)

- **Arrow Keys**: Move left/right/forward/back
- **W/S**: Height up/down
- **A/D**: Yaw left/right
- **M**: Toggle autonomous mode
- **Q**: Quit (in headless mode)

## 🧪 Test First

```bash
# Verify everything works
python3 test_imports.py
```

## 🔧 Troubleshooting

If you still get import errors:
1. Make sure you're in the `crazyflie_control` directory
2. Use `python3` not `python`
3. Check that all dependencies are installed in your venv

## 📁 Original Code Untouched

The original `multiranger_pointcloud.py` file remains completely unchanged and can still be used normally.

---

**Ready to fly!** The modular system provides the same autonomous navigation experience with much better architecture and no GUI conflicts.