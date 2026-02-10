#=========================================================================
# pv2byp Subpackage
#=========================================================================

pv2byp_deps = \
  vc \
  imuldiv \

pv2byp_srcs = \
  pv2byp-CoreDpath.v \
  pv2byp-CoreDpathRegfile.v \
  pv2byp-CoreDpathAlu.v \
  pv2byp-CoreCtrl.v \
  pv2byp-Core.v \
  pv2byp-InstMsg.v \
  pv2byp-CoreDpathPipeMulDiv.v \

pv2byp_test_srcs = \
  pv2byp-InstMsg.t.v \
  pv2byp-CoreDpathPipeMulDiv.t.v \

pv2byp_prog_srcs = \
  pv2byp-sim.v \
  pv2byp-randdelay-sim.v \

