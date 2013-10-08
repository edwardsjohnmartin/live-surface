# DO NOT EDIT
# This makefile makes sure all linkable targets are
# up-to-date with anything they link to
default:
	echo "Do not invoke directly"

# For each target create a dummy rule so the target does not have to exist
/opt/local/lib/libjpeg.dylib:
/opt/local/lib/libpng.dylib:
/usr/lib/libz.dylib:
/opt/X11/lib/libX11.dylib:


# Rules to remove targets that are older than anything to which they
# link.  This forces Xcode to relink the targets from scratch.  It
# does not seem to check these dependencies itself.
PostBuild.surface.Debug:
/Users/jedwards/projects/live-surface/xcode/Debug/surface:\
	/opt/local/lib/libjpeg.dylib\
	/opt/local/lib/libpng.dylib\
	/usr/lib/libz.dylib\
	/opt/X11/lib/libX11.dylib
	/bin/rm -f /Users/jedwards/projects/live-surface/xcode/Debug/surface


PostBuild.surface.Release:
/Users/jedwards/projects/live-surface/xcode/Release/surface:\
	/opt/local/lib/libjpeg.dylib\
	/opt/local/lib/libpng.dylib\
	/usr/lib/libz.dylib\
	/opt/X11/lib/libX11.dylib
	/bin/rm -f /Users/jedwards/projects/live-surface/xcode/Release/surface


PostBuild.surface.MinSizeRel:
/Users/jedwards/projects/live-surface/xcode/MinSizeRel/surface:\
	/opt/local/lib/libjpeg.dylib\
	/opt/local/lib/libpng.dylib\
	/usr/lib/libz.dylib\
	/opt/X11/lib/libX11.dylib
	/bin/rm -f /Users/jedwards/projects/live-surface/xcode/MinSizeRel/surface


PostBuild.surface.RelWithDebInfo:
/Users/jedwards/projects/live-surface/xcode/RelWithDebInfo/surface:\
	/opt/local/lib/libjpeg.dylib\
	/opt/local/lib/libpng.dylib\
	/usr/lib/libz.dylib\
	/opt/X11/lib/libX11.dylib
	/bin/rm -f /Users/jedwards/projects/live-surface/xcode/RelWithDebInfo/surface


