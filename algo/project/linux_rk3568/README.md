# Configure rk3399 environment
export PATH=/usr/local/gcc-arm-10.3-2021.07-x86_64-aarch64-none-linux-gnu/bin:$PATH
aarch64-none-linux-gnu-gcc


# loc-engine
Pay attention to the case requirements of letters in the compiled version.

generate libloc_core.so:
cmd: make loc_core or make

generate loc_app.bin:
cmd: make loc_app

generate loc_pb.bin:
cmd: make loc_pb

generate all output:
cmd: make all
