#=========================================================================
# pv2stall Subpackage
#=========================================================================

pv2stall_deps = \
  vc \
  imuldiv \

pv2stall_srcs = \
  pv2stall-CoreDpath.v \
  pv2stall-CoreDpathRegfile.v \
  pv2stall-CoreDpathAlu.v \
  pv2stall-CoreCtrl.v \
  pv2stall-Core.v \
  pv2stall-InstMsg.v \
  pv2stall-CoreDpathPipeMulDiv.v \

pv2stall_test_srcs = \
  pv2stall-InstMsg.t.v \
  pv2stall-CoreDpathPipeMulDiv.t.v \

pv2stall_prog_srcs = \
  pv2stall-sim.v \
  pv2stall-randdelay-sim.v \

