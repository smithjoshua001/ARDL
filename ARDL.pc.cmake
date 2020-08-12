prefix=@CMAKE_INSTALL_PREFIX@
libdir=@CMAKE_INSTALL_FULL_LIBDIR@
includedir=@CMAKE_INSTALL_FULL_INCLUDEDIR@

Name: ARDL
Description: ARDL
URL: http://nothingyet/
Version: @ARDL_VERSION@
Requires: eigen3 urdfdom_headers urdfdom assimp ccd fcl json11
Conflicts:
Libs: -L${libdir} -Wl,-rpath ${libdir}
Libs.private:
Cflags: -I${includedir}
