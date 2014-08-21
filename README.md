ode
===

This is a Go binding for the Open Dynamics Engine 3D physics library.  It
sticks fairly closely to the development version of the ODE C API, with a few
stylistic and idiomatic changes thrown in here and there where it seemed
useful.

ODE must be compiled as a shared library with double precision support.  The
following will configure ODE with these options:

`> cd /path/to/ode-src; ./configure --enable-double-precision --enabled-shared`

Todo
* Heightfields
* Triangle Meshes
* Probably other things I missed
* Documentation and tests
