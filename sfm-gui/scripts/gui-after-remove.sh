#!/bin/bash
set -e

target=/usr/bin/insightat-sfm-gui
if [ -L "${target}" ] && [ "$(readlink -f "${target}")" = "/opt/insightat/insightat-sfm-gui" ]; then
  rm -f "${target}"
fi

exit 0
