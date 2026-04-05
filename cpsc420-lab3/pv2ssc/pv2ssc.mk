#=========================================================================
# pv2ssc Subpackage
#=========================================================================

pv2ssc_deps = \
  vc \
  imuldiv \
  pcache \

pv2ssc_srcs = \
  pv2ssc-CoreDpath.v \
  pv2ssc-CoreDpathRegfile.v \
  pv2ssc-CoreDpathAlu.v \
  pv2ssc-CoreDpathPipeMulDiv.v \
  pv2ssc-CoreCtrl.v \
  pv2ssc-Core.v \
  pv2ssc-InstMsg.v \

pv2ssc_test_srcs = \
  pv2ssc-InstMsg.t.v \
  pv2ssc-CoreDpathPipeMulDiv.t.v \

pv2ssc_prog_srcs = \
  pv2ssc-sim.v \
  pv2ssc-randdelay-sim.v \

