Import("env")

# See https://docs.platformio.org/en/latest/scripting/examples/split_build_flags.html

# General options that are passed to the C and C++ compilers
# env.Append(CCFLAGS=["flag1", "flag2"])

# General options that are passed to the C compiler (C only; not C++).
# env.Append(CFLAGS=["flag1", "flag2"])

# General options that are passed to the C++ compiler
env.Append(CXXFLAGS=["-Wnrvo", "-Wpessimizing-move", "-Wno-overloaded-virtual", "-Wno-interference-size"])

