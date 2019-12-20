# THIS REPO IS ARCHIVED

Please use https://github.com/Lyceum/MuJoCo.jl for all your MuJoCo needs in Julia. It is nice.


# MuJoCo.jl
Julia wrapper for the MuJoCo Physics Engine. This wrapper tries to both keep full access to all MuJoCo functionality through C pointers while also allow faster development through higher level interface.

MuJoCo functions and structs retain their C-library names (e.g. mj_step, mju_copy), while enums are scoped to the module name of mj (e.g. mj.NGROUP, etc). MuJoCo C-library functions can be passed a pointer to the mjModel or mjData structs, or can be passed a convenience wrapper struct of jlModel, jlData.

```
using MuJoCo

# mj_activate(ENV["MUJOCO_KEY_PATH"]) # this shouldn't be needed as, on load, the module attempts to activate

modelfile = "humanoid.xml"
pm = mj_loadXML(modelfile)  # Raw C pointer to mjModel
pd = mj_makeData(pm)            # Raw C pointer to mjData

mj_step(pm, pd) # At this point you can pass the pointers to mujoco functions

# Access mjModel and mjData structs; this makes a local copy
m = unsafe_load(pm)
println(m.nq, " position elements")
d = unsafe_load(pd)
println(d.qpos, " is a raw pointer still")
```

Higher level interfacing tries to make it easier to manipulate data in the mjModel and mjData structures through Julia types. We do this by wrapping mjModel & mjData with jlModel and jlData types that expose ```Vector{Float64}``` instead of MuJoCo's raw arrays.

```
m, d = mj.mapmujoco(pm, pd) # wrap with our jlModel, jlData types

# we can manipulate data in the raw C structs now
time = d.d[].time
d.d[].time = 1.0
@assert d.d[].time == 1.0

d.qpos[:] = rand(m.m[].nq) # d.qpos is a jlData Vector; free to access and maps to raw pointer

# functions work on the jlModel and jlData types
mj_step(m, d)
mj_resetData(m, d)

mj_step(m.m, d.d) # our wrapped functions can take in the convenience struct or the Ref pointers

# MuJoCo convenience functions are understood to use 0-based indexing, for now...
id = mj_name2id(m, mj.OBJ_GEOM, "floor") # for humanoid.xml
@assert id == 0
```

# Installation

```
julia> Pkg.clone("git://www.github.com/klowrey/MuJoCo.jl.git")
julia> Pkg.build("MuJoCo")
```
MuJoCo v2.00 should be installed automatically through Julia Pkg. You will need a mjkey.txt license file, and your system should set the environment variable "MUJOCO_KEY_PATH" to be the path to your mjkey.txt file.

Currently, this package is less on Windows and OSX.

# Examples
Temporary examples can be found in test suite.

# Visualization
A version of `simulate.cpp` has been ported to julia here: https://github.com/klowrey/MujocoSim.jl
This allows you to build up the visualization features you need for your experiments, but nominally does passive dynamics.

# Todo
Test using @cfunctions to pass Julia code to MuJoCo callback functions
Test / contemplate making sure all conversions from 0-based to 1-based indexing is correct?
make convert functions that convert enums to Ints or Int32s
