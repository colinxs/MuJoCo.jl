#__precompile__()

module MuJoCo

using Reexport
using Libdl

const depsfile = joinpath(dirname(@__FILE__), "..", "deps", "deps.jl")
if isfile(depsfile)
   include(depsfile)
else
   error("MuJoCo was not installed correctly.")
end

include("./mj.jl")
@reexport using .mj

include("./export_all.jl") # a list of all function and struct names

function teardown()
   ACTIVATED[] = false
   mj_deactivate()
end

const ACTIVATED = Base.RefValue(false)
isactivated() = ACTIVATED[]
function __init__()
   if !isactivated()
      if Sys.islinux()
         mjglew = Libdl.dlopen_e(libglew, Libdl.RTLD_LAZY | Libdl.RTLD_DEEPBIND | Libdl.RTLD_GLOBAL)
      end

      key = ""
      try
         key = ENV["MUJOCO_KEY_PATH"]

         mjlib = Libdl.dlopen_e(libmujoco, Libdl.RTLD_LAZY | Libdl.RTLD_DEEPBIND | Libdl.RTLD_GLOBAL)
         if mjlib == C_NULL
            @error("Please build MuJoCo.jl")
         end
         ACTIVATED[] = Bool( ccall(Libdl.dlsym(mjlib, :mj_activate),Cint,(Cstring,),key) )
         if !isactivated()
            println("MuJoCo not activated; call mj_activate with path to license key.")
         end
      catch e
         println("Set MUJOCO_KEY_PATH environment variable, please.")
      end

      atexit(teardown)
   end
end

end
