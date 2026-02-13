#!/bin/bash

rm paranet.lock
COLOR=red para docker deploy package -C color --node red $@
COLOR=blue para docker deploy package -C color --node blue $@
COLOR=yellow para docker deploy package -C color --node yellow $@
para docker deploy package -C painter --node painter $@
