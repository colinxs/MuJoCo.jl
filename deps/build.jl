# version of library to download
const version = v"2.00"

using BinDeps
using Libdl
@BinDeps.setup

function compatible_version(lib, handle)
   major, minor, rev = Ref{Cint}(), Ref{Cint}(), Ref{Cint}()
   f = Libdl.dlsym(handle, :mj_version)
   f == C_NULL && return false
   v = ccall(f, Int32, ())
   @info(v)
   return v >= 200
end

baseurl = "https://www.roboti.us/download/mujoco200_"
basedir = dirname(@__FILE__)
unpack = joinpath(basedir, "mujoco200")
libpath = unpack*"/bin"

function download_extract(url, unpack, target)
   basedir = dirname(@__FILE__)
   if isdir(unpack) == false
      @info("Downloading: ", url, " to ", unpack)
      file = Base.download(url)
      run(`unzip -o $file -d $basedir`)
      if isdir(unpack*target)
         run(`mv $(unpack*target) $(unpack)`)
      end
   end
end

# library source code
if Sys.islinux()
   push!(BinDeps.defaults, Binaries)

   # First find three library dependents
   mujoco_glfw= library_dependency("libglfw", aliases=["libglfw.so.3"])
   mujoco_glew= library_dependency("libglew", aliases=["libglew"])
   mujoco_GL  = library_dependency("libGL") # looks in system
   mujoco_nix = library_dependency("libmujoco", aliases=["libmujoco200.so", "libmujoco200"])

   url = baseurl*"linux.zip"
   download_extract(url, unpack, "_linux/")

   preloads = string("Libdl.dlopen(\"$(libpath)/libglew.so\", Libdl.RTLD_LAZY | Libdl.RTLD_DEEPBIND | Libdl.RTLD_GLOBAL)")

   provides(Binaries, URI(url), mujoco_glfw, unpacked_dir=unpack, installed_libpath=libpath)
   provides(Binaries, URI(url), mujoco_glew, unpacked_dir=unpack, installed_libpath=libpath)
   eval(Meta.parse(preloads))
   provides(Binaries, URI(url), mujoco_nix, unpacked_dir=unpack, installed_libpath=libpath, preload=preloads)

   @BinDeps.install Dict([(:libglfw, :libglfw),
                          (:libglew, :libglew),
                          (:libGL, :libGL),
                          (:libmujoco, :libmujoco)])
elseif Sys.isapple()
   mujoco_osx = library_dependency("libmujoco", aliases=["libmujoco200", "libmujoco200.dylib"])
   url = baseurl*"macos.zip"

   download_extract(url, unpack, "_macos/")

   provides(Binaries, URI(url), mujoco_osx, unpacked_dir=unpack, installed_libpath=libpath)
   @BinDeps.install Dict([(:libmujoco, :libmujoco)])
elseif Sys.iswindows()
   mujoco_win = library_dependency("libmujoco", aliases=["mujoco200"])
   url = baseurl*"win64.zip"
   @info("Downloading: ", url, " to ", unpack)
   provides(Binaries, URI(url), mujoco_win, unpacked_dir=unpack, installed_libpath=libpath)
   @BinDeps.install Dict(:libmujoco=>:libmujoco)
end

Sys.islinux() && pop!(BinDeps.defaults)

