#!/bin/bash

bin="$PWD/$1"

cd ../probe-run
cargo run -- "$bin" --chip nrf52840