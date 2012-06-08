FILE(REMOVE_RECURSE
  "../src/ardrone_emulator/msg"
  "../src/ardrone_emulator/srv"
  "../msg_gen"
  "../srv_gen"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_py"
  "../src/ardrone_emulator/srv/__init__.py"
  "../src/ardrone_emulator/srv/_Control.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
