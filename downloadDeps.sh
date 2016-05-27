#!/bin/bash
#
# This script is meant to go fetch the most recent versions of various libraries that
#   ManuvrOS has been written against. None of this is strictly required for a basic build,
#   but most real-world applications will want at least one of them.

# Make the lib directory...
mkdir lib

# CoAP, if desired.
rm -rf lib/wakaama
git clone https://github.com/eclipse/wakaama.git lib/wakaama

# MQTT, if desired.
# Note that we do special-handling here to make the build-process smoother...
rm -rf lib/paho.mqtt.embedded-c
git clone https://github.com/eclipse/paho.mqtt.embedded-c.git lib/paho.mqtt.embedded-c
cp lib/paho.mqtt.embedded-c/MQTTPacket/src/* lib/paho.mqtt.embedded-c/

# Telehash
rm -rf lib/telehash-c
git clone https://github.com/telehash/telehash-c.git lib/telehash-c

# Return...
cd ..
