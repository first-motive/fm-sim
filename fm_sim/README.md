# fm_sim (metapackage)

`ament_cmake` metapackage. Exec-depends on the simulation sub-packages so the whole
group installs as one unit and stays split-ready.

It lives as a sibling of the children (not their parent directory) because colcon prunes
its crawl at any directory that is itself a package — nesting the children under the
metapackage would hide them from the build.

See `../README.md` for the two sim paths and the simulation layer overview.
