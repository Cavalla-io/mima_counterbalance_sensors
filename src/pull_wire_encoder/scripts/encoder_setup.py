import subprocess
import time

def setup_pull_wire_encoder(interface="can1", node_id=5):
    # CANopen Base ID for SDO is 0x600 (1536 decimal)
    sdo_id = hex(0x600 + node_id)[2:]  # e.g., 5 -> "605"
    
    # NMT Command needs the node ID in hex, padded to 2 digits (e.g., "05")
    hex_node = f"{node_id:02x}"

    commands = [
        f"cansend {interface} 000#80{hex_node}",             # Enter Pre-Operational
        f"cansend {interface} {sdo_id}#2F001802FE000000",     # Set Transmission Type
        f"cansend {interface} {sdo_id}#2B00180564000000",     # Set Event Timer
        f"cansend {interface} {sdo_id}#2310100173617665",     # Save to memory
        f"cansend {interface} 000#01{hex_node}"              # Start Node
    ]

    print(f"Configuring Node ID {node_id} on {interface}...")
    for cmd in commands:
        try:
            subprocess.run(cmd.split(), check=True)
            time.sleep(0.05)
        except subprocess.CalledProcessError as e:
            print(f"Failed: {cmd}")

if __name__ == "__main__":
    # You can now easily change this if a new unit comes in as ID 1
    setup_pull_wire_encoder(node_id=5)
