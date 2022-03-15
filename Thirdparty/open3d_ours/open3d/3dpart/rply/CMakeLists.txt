# rply is only used in IO. It is thus included in IO.
# If you want to use rply as an independent library, uncomment the following
# lines.
#
#project(rply)
add_library(rply_lib SHARED rply/rply.c rply/rply.h)
