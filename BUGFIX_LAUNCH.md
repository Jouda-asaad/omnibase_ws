# Launch File Fix - 2025-12-27

## Issue
Launch files were using `PathJoinSubstitution` incorrectly, causing this error:
```
[ERROR] [launch]: Caught exception in launch (see debug for traceback): 
join() argument must be str, bytes, or os.PathLike object, not 'PathJoinSubstitution'
```

## Root Cause
`PythonLaunchDescriptionSource` and `xacro.process_file()` expect string paths at parse time, but `PathJoinSubstitution` is a launch substitution that's evaluated at runtime. Mixing these caused the error.

## Files Fixed

### 1. `/src/omnibase_description/launch/description.launch.py`
**Changed:**
- Replaced `PathJoinSubstitution` with `get_package_share_directory()` + `os.path.join()`
- Added proper imports

### 2. `/src/omnibase_gazebo/launch/simulation.launch.py`
**Changed:**
- Fixed robot description launch file inclusion
- Uses string-based path construction

### 3. `/src/omnibase_bringup/launch/full_simulation.launch.py`
**Changed:**
- Replaced all `PathJoinSubstitution` usage with `get_package_share_directory()`
- Proper string path construction for both simulation and vision launch files

## Testing
Rebuild and test:
```bash
cd ~/omnibase_ws
source install/setup.bash
ros2 launch omnibase_bringup full_simulation.launch.py
```

Should now launch without errors!
