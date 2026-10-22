#!/usr/bin/env python3
"""Wrap the installed vendor URDF in a prefixable macro without copying its data.

Only URDF identifier declarations and references change. Geometry, inertial,
joint, and extension elements are deep-copied in their original order.
"""
import argparse
from copy import deepcopy
import xml.etree.ElementTree as ET


def generate(source, destination):
    upstream = ET.parse(source).getroot()
    namespace = "http://www.ros.org/wiki/xacro"
    ET.register_namespace("xacro", namespace)
    output = ET.Element("robot")
    macro = ET.SubElement(output, f"{{{namespace}}}macro", {
        "name": "firefly_y6_geometry", "params": "prefix:=''",
    })
    links = {element.attrib["name"] for element in upstream.findall("link")}
    joints = {element.attrib["name"] for element in upstream.findall("joint")}
    for original in upstream:
        element = deepcopy(original)
        for child in element.iter():
            if child.tag in ("link", "joint") and child.get("name") in links | joints:
                child.set("name", "${prefix}" + child.attrib["name"])
            for key in ("link", "reference"):
                if child.get(key) in links | joints:
                    child.set(key, "${prefix}" + child.attrib[key])
        macro.append(element)
    ET.indent(output, space="  ")
    ET.ElementTree(output).write(destination, encoding="utf-8", xml_declaration=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source")
    parser.add_argument("destination")
    args = parser.parse_args()
    generate(args.source, args.destination)
