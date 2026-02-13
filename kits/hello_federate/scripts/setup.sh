#!/bin/bash

red_port=3023
blue_port=3024
yellow_port=3025
painter_port=3026

PROD=${PROD:-0}
if [ "$PROD" -eq "1" ]; then
    kind=prod
    insecure=false
else
    kind=dev
    insecure=true
fi

echo "Deploying nodes"
para docker deploy node --config nodes/local.paranet.node.yaml --set id=red --set port=$red_port --set kind=$kind --set insecure=$insecure $@
para docker deploy node --config nodes/local.paranet.node.yaml --set id=blue --set port=$blue_port --set kind=$kind --set insecure=$insecure $@
para docker deploy node --config nodes/local.paranet.node.yaml --set id=yellow --set port=$yellow_port --set kind=$kind --set insecure=$insecure $@
para docker deploy node --config nodes/local.paranet.node.yaml --set id=painter --set port=$painter_port --set kind=$kind --set insecure=$insecure $@


echo "Federating"
para docker federate --from painter --to red -bidirectional $@
para docker federate --from painter --to blue -bidirectional $@
para docker federate --from painter --to yellow -bidirectional $@

echo "Installing packages"
source ./scripts/install-packages.sh $@

echo "Done"
echo "--------------------------------"
para docker ls

echo "RED: http://localhost:$red_port"
echo "BLUE: http://localhost:$blue_port"
echo "YELLOW: http://localhost:$yellow_port"
echo "PAINTER: http://localhost:$painter_port"
