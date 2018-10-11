This is basically a copy(just changed where it reads the URDF from) of RBDL's sample code for reading URDFs into a model and doing forward dynamics on it. The original can be found under /examples/urdfreader in the source code.

* note: In the README for the URDF addon, it says that this feature hasn't properly been tested yet. Use with caution.
* the base link's name should be `base_link` to be read by the URDF reader.
* the links must have some mass and inertia for calculations to work.
