#!/bin/bash
VM=$1
GITHUB_TOKEN=$(gh auth token)
ssh -i  ~/.ssh/UW-Capstone.pem otonoma@$VM "echo $GITHUB_TOKEN | docker login ghcr.io -u otonoma --password-stdin"