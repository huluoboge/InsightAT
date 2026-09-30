#!/bin/bash
set -e

# Provide the same command-line entry as the standalone viewer package.
ln -sf /opt/insightat/insightat-sfm-gui /usr/bin/insightat-sfm-gui

# Electron's Chromium sandbox requires a root-owned setuid helper.
for helper in \
  /opt/insightat/chrome-sandbox \
  /opt/insightat/sfm-viewer-app/chrome-sandbox
do
  if [ -f "$helper" ]; then
    chown root:root "$helper"
    chmod 4755 "$helper"
  fi
done

exit 0
