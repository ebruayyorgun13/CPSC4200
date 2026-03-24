#=========================================================================
# pv2dualfetch Subpackage
#=========================================================================

pv2dualfetch_deps = \
  vc \
  imuldiv \
  pcache \

pv2dualfetch_srcs = \
  pv2dualfetch-CoreDpath.v \
  pv2dualfetch-CoreDpathRegfile.v \
  pv2dualfetch-CoreDpathAlu.v \
  pv2dualfetch-CoreDpathPipeMulDiv.v \
  pv2dualfetch-CoreCtrl.v \
  pv2dualfetch-Core.v \
  pv2dualfetch-InstMsg.v \

pv2dualfetch_test_srcs = \
  pv2dualfetch-InstMsg.t.v \
  pv2dualfetch-CoreDpathPipeMulDiv.t.v \

pv2dualfetch_prog_srcs = \
  pv2dualfetch-sim.v \
  pv2dualfetch-randdelay-sim.v \

