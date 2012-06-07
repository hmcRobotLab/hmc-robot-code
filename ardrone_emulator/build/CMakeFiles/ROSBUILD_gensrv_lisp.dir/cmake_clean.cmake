FILE(REMOVE_RECURSE
  "../src/ardrone_emulator/msg"
  "../src/ardrone_emulator/srv"
  "../msg_gen"
  "../srv_gen"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_lisp"
  "../srv_gen/lisp/DroneControl.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_DroneControl.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
