SCALASRC=rocket-chip/src/main/scala
rm -rf rocket-chip/vsim/generated-src/*
pushd $SCALASRC
rm -f litex
ln -s ../../../../litex litex
popd
for CFG in LitexConfig LitexLinuxConfig LitexFullConfig
do
  make -C rocket-chip/vsim verilog CONFIG=$CFG MODEL=LitexRocketSystem
done
#rm -f $SCALASRC/litex


