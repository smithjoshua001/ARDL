prefix=@CMAKE_INSTALL_PREFIX@
libdir=@CMAKE_INSTALL_FULL_LIBDIR@
includedir=@CMAKE_INSTALL_FULL_INCLUDEDIR@

Name: ARDL-SP
Description: ARDL-SP
URL: http://nothingyet/
Version: @ARDL-SP_VERSION@
Requires: eigen3 urdfdom_headers urdfdom assimp ccd fcl json11 ARDL-NP
Conflicts:
Libs: -L${libdir} -wl
Libs.private:
Cflags: -I${includedir}
