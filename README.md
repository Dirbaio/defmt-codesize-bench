# `defmt` code size testbench

Small repo to measure defmt code size improvements. Contains 2 benchmarks

- `spam`: prints lots of random things. based on [this](https://github.com/knurling-rs/defmt/blob/main/firmware/qemu/src/bin/log.rs).
- `smoltcp`: runs the smoltcp loopback example.

How to use:

- Clone `defmt` and `probe-run` next to this repo
- Patch `probe-run` to get `defmt-decoder` from `../defmt/decoder`
- `cargo run --release --bin spam` (or smoltcp)
- `cargo size --release --bin spam` 

To test [PR 258](https://github.com/knurling-rs/defmt/pull/258) edit `rtt.rs` and uncomment `rtt_2_nowrite`