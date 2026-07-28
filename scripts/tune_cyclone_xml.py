#!/usr/bin/env python3
"""Patch ~/.config/cyclonedds/cyclonedds.xml in place for zero-copy transport.

Parses the existing CycloneDDS config (if any) and ensures only the keys the
large camera/LiDAR payloads need — SharedMemory enabled, big socket buffers,
watermarks, small fragment size — WITHOUT discarding any other custom settings.
If no file exists, a minimal correct one is created.

Usage: tune_cyclone_xml.py [xml_path] [socket_buffer]   (e.g. "120 MB")
"""
import os
import sys
import xml.etree.ElementTree as ET

XML = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser(
    "~/.config/cyclonedds/cyclonedds.xml")
SOCK_BUF = sys.argv[2] if len(sys.argv) > 2 else "120 MB"
WHC_HIGH = "100 MB"
NS = "https://cdds.io/config"

changes = []


def q(tag):
    return f"{{{NS}}}{tag}" if NS else tag


def child(parent, tag):
    """Find or create a child element by tag."""
    el = parent.find(q(tag))
    if el is None:
        el = ET.SubElement(parent, q(tag))
        changes.append(f"added <{tag}>")
    return el


def set_text(parent, tag, val):
    el = child(parent, tag)
    if (el.text or "").strip() != val:
        el.text = val
        changes.append(f"set <{tag}>={val}")


def set_attr(parent, tag, attr, val):
    el = child(parent, tag)
    if el.get(attr) != val:
        el.set(attr, val)
        changes.append(f"set <{tag} {attr}={val}>")


# ── Load or create ────────────────────────────────────────────────────────
if os.path.isfile(XML) and os.path.getsize(XML) > 0:
    global_ns = NS
    tree = ET.parse(XML)
    root = tree.getroot()
    # Detect the actual namespace of the existing file (may be empty).
    if root.tag.startswith("{"):
        NS = root.tag[1:].split("}")[0]
    else:
        NS = ""
    print(f"[tune] parsed existing {XML}")
else:
    root = ET.Element(q("CycloneDDS"))
    tree = ET.ElementTree(root)
    changes.append("created new config")
    print(f"[tune] {XML} missing — creating")

if NS:
    ET.register_namespace("", NS)

# ── Ensure Domain ───────────────────────────────────────────────────────────
domain = root.find(q("Domain"))
if domain is None:
    domain = ET.SubElement(root, q("Domain"))
    domain.set("id", "any")
    changes.append("added <Domain id=any>")

# ── General: multicast + small fragment ──────────────────────────────────────
general = child(domain, "General")
set_text(general, "AllowMulticast", "true")
set_text(general, "FragmentSize", "1400 B")  # <=MTU(1500): 1 IP packet/frag, no reassembly

# ── Internal: large socket buffers + watermark ───────────────────────────────
internal = child(domain, "Internal")
set_attr(internal, "SocketReceiveBufferSize", "min", SOCK_BUF)
set_attr(internal, "SocketSendBufferSize", "min", SOCK_BUF)
watermarks = child(internal, "Watermarks")
set_text(watermarks, "WhcHigh", WHC_HIGH)

# ── SharedMemory: enable Iceoryx zero-copy ───────────────────────────────────
shm = child(domain, "SharedMemory")
set_text(shm, "Enable", "true")
if shm.find(q("LogLevel")) is None:
    set_text(shm, "LogLevel", "info")

# ── Write back ────────────────────────────────────────────────────────────────
os.makedirs(os.path.dirname(XML), exist_ok=True)
if hasattr(ET, "indent"):
    ET.indent(tree, space="    ")
tree.write(XML, encoding="utf-8", xml_declaration=True)

if changes:
    print("[tune] adjusted: " + "; ".join(changes))
else:
    print("[tune] already correct — no change")
print(f"[tune] wrote {XML}")
