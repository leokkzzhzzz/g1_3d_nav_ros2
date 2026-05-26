#!/bin/bash
set -e

# 从 robot_config.yaml 读取网口名
IFACE=$(python3 -c "
import yaml
with open('/botbrain_ws/robot_config.yaml') as f:
    c = yaml.safe_load(f) or {}
print(c.get('robot_configuration',{}).get('network_interface','eth0'))
")

# 生成与 unitree_sdk2py ChannelConfigHasInterface 完全一致的 DDS XML
cat > /tmp/manipulation_cyclonedds.xml <<XMLEOF
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS>
    <Domain Id="any">
        <General>
            <Interfaces>
                <NetworkInterface name="${IFACE}" priority="default" multicast="default"/>
            </Interfaces>
        </General>
        <Tracing>
            <Verbosity>config</Verbosity>
            <OutputFile>/tmp/cdds.LOG</OutputFile>
        </Tracing>
    </Domain>
</CycloneDDS>
XMLEOF

export CYCLONEDDS_URI=file:///tmp/manipulation_cyclonedds.xml
echo "[manipulation] DDS interface: ${IFACE}, config: /tmp/manipulation_cyclonedds.xml"

source /botbrain_ws/install/setup.bash
exec ros2 launch g1_manipulation_pkg manipulation_launcher.launch.py
