# Passbox Server - Quick Reference

## Files Created

### Scripts (src/passbox/scripts/)
- ✅ `passbox_server.py` - Main server (2613 lines, based on elevator_server.py)
- ✅ `agf_mc_protocol.py` - PLC communication protocol

### Configuration (src/passbox/cfg/)
- ✅ `hub.json` - Main configuration parameters
- ✅ `waiting_path.json` - Waiting position path
- ✅ `docking_path.json` - Docking path  
- ✅ `undocking_path.json` - Undocking path

## Key Changes from Elevator

| Aspect | Elevator | Passbox |
|--------|----------|---------|
| Class name | `ElevatorAction` | `PassboxAction` |
| Node name | `elevator_server` | `passbox_server` |
| Config path | `matehan/cfg` | `passbox/cfg` |
| Status topic | `/status_elevator_io` | `/status_passbox_io` |
| Thread name | `read_data_elevator` | `read_data_passbox` |

## Quick Start

```bash
# Run passbox server
rosrun passbox passbox_server.py

# With custom config
rosrun passbox passbox_server.py -c /path/to/config

# Debug mode
rosrun passbox passbox_server.py -d
```

## ROS Parameters

```bash
# Set PLC connection
rosparam set /passbox_server/plc_address "192.168.20.XX"
rosparam set /passbox_server/plc_port 2002
```

## Important Notes

- ✅ `elevator_server.py` **NOT modified** - remains unchanged
- ✅ Both elevator and passbox can run simultaneously
- ✅ Uses same PLC protocol (agf_mc_protocol)
- ⚠️ Verify PLC register addresses before production use
- ⚠️ Update config files with actual positions/parameters

## Next Steps

1. Configure PLC addresses in `hub.json` or via ROS params
2. Update path files with actual waypoint positions
3. Test with actual PLC hardware
4. Customize state machine if needed for passbox-specific operations

See [walkthrough.md](file:///C:/Users/Admin/.gemini/antigravity/brain/5235bae7-96c5-436c-bf65-69dcbb48de68/walkthrough.md) for detailed documentation.
