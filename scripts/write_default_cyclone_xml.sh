#!/usr/bin/env bash
# Write a plain (no-SHM) CycloneDDS config to ~/.config/cyclonedds/cyclonedds.xml,
# bound to the wired ethernet interface. Used by `make teardown_cyclone` to leave
# a working default after the Iceoryx shared-memory transport is removed.
#
# Interface: pass one as $1 to override, else auto-detect the first PHYSICAL wired
# NIC (ARPHRD type 1, has a /device, not wireless) — e.g. eno1/enp*/eth0, never
# wlan*/docker0/lo. Falls back to eth0 if none found.
set -euo pipefail

XML="${XDG_CONFIG_HOME:-$HOME/.config}/cyclonedds/cyclonedds.xml"

IFACE="${1:-}"
if [ -z "$IFACE" ]; then
    for d in /sys/class/net/*; do
        n=$(basename "$d")
        [ "$(cat "$d/type" 2>/dev/null)" = "1" ] && [ -e "$d/device" ] && [ ! -d "$d/wireless" ] && { IFACE="$n"; break; }
    done
fi
[ -z "$IFACE" ] && IFACE=eth0

mkdir -p "$(dirname "$XML")"
cat > "$XML" <<EOF
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain Id="any">
    <General>
      <Interfaces>
        <NetworkInterface name="$IFACE" priority="default" multicast="default"/>
      </Interfaces>
      <!-- For very large samples (point clouds, images): -->
      <MaxMessageSize>65500B</MaxMessageSize>
      <!-- <=MTU(1500) so each DDS fragment is one IP packet, no IP reassembly. -->
      <FragmentSize>1400B</FragmentSize>
    </General>
    <Internal>
      <SocketReceiveBufferSize min="10MB" max="default"/>
      <SocketSendBufferSize min="10MB" max="default"/>
    </Internal>
    <Sizing>
      <ReceiveBufferSize>120MiB</ReceiveBufferSize>
    </Sizing>
    <Discovery>
      <MaxAutoParticipantIndex>120</MaxAutoParticipantIndex>
    </Discovery>
  </Domain>
</CycloneDDS>
EOF
echo "wrote $XML (NetworkInterface=$IFACE)"
