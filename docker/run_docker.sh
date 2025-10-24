#!/bin/bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Allow local docker access to X server
xhost +local:docker

# Ensure access is revoked on script exit
trap 'xhost -local:docker' EXIT

# Run script in the directory of the script
cd "$SCRIPT_DIR"

docker compose -f compose.yaml up -d

docker exec -it liloc /bin/bash