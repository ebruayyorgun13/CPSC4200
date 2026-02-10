#=========================================================================
# pv2long Subpackage
#=========================================================================

pv2long_deps = \
  vc \
  imuldiv \

pv2long_srcs = \
  pv2long-CoreDpath.v \
  pv2long-CoreDpathRegfile.v \
  pv2long-CoreDpathAlu.v \
  pv2long-CoreCtrl.v \
  pv2long-Core.v \
  pv2long-InstMsg.v \
  pv2long-CoreDpathPipeMulDiv.v \

pv2long_test_srcs = \
  pv2long-InstMsg.t.v \
  pv2long-CoreDpathPipeMulDiv.t.v \

pv2long_prog_srcs = \
  pv2long-sim.v \
  pv2long-randdelay-sim.v \

