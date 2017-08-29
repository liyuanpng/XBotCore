
set(VN100_lib_srcs      ./src/vn100.c
                        ./src/vndevice.c
                        ./src/vn200.c
                        ./src/arch/linux/vncp_services.c)

set(VN100_lib_hdrs     ./include/vn_kinematics.h
                        ./include/vncp_services.h
                        ./include/vn_math.h
                        ./include/vndevice.h
                        ./include/vn_common.h
                        ./include/vn200.h
                        ./include/vnint.h
                        ./include/vectornav.h
                        ./include/vn_linearAlgebra.h
                        ./include/vn_errorCodes.h
                        ./include/vn100.h)
                        
include_directories(include)
add_library(VN100_lib SHARED  ${VN100_lib_srcs} ${VN100_lib_hdrs} )

yarp_install(TARGETS VN100_lib
             COMPONENT runtime
             LIBRARY DESTINATION ${XBOTYARP_DYNAMIC_PLUGINS_INSTALL_DIR})
