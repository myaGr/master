# Configure oecore environment
source /usr/local/oecore-x86_64/environment-setup-armv7a-vfp-neon-oe-linux-gnueabi
caimagic@caimagic-ubuntu:/usr/local/oecore-x86_64$ echo $CC
arm-oe-linux-gnueabi-gcc -march=armv7-a -mfloat-abi=softfp -mfpu=neon --sysroot=/usr/local/oecore-x86_64/sysroots/armv7a-vfp 


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
