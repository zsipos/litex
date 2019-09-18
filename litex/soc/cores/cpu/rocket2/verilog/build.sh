SCALASRC=rocket-chip/src/main/scala
rm -rf rocket-chip/vsim/generated-src/*
pushd $SCALASRC
rm -f litex
ln -s ../../../../litex litex
popd
make -C bootrom
for CFG in LitexConfig LitexLinuxConfig LitexFullConfig
do
  make -C rocket-chip/vsim verilog CONFIG=$CFG MODEL=LitexRocketSystem bootrom_img=`pwd`/bootrom/bootrom.img
done
#rm -f $SCALASRC/litex


