set -e
SCALASRC=rocket-chip/src/main/scala
rm -rf rocket-chip/vsim/generated-src/*
pushd $SCALASRC
rm -f litex
ln -s ../../../../litex litex
popd
for BITS in 32 64
do
	for MEM in 64 128 256
	do
		for CFG in LitexConfig LitexLinuxConfig LitexFullConfig
		do
			CONFIG=${CFG}${BITS}Mem${MEM}
			if [ $CFG != LitexFullConfig ] || [ $BITS != 32 ]
			then
				make -C rocket-chip/vsim verilog \
					CONFIG=$CONFIG MODEL=LitexRocketSystem
			fi
		done
	done
done
#rm -f $SCALASRC/litex


