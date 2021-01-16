# Python examples
In its current implementation, you must set the `$PYTHONPATH` environment variable to point to the directory containing the library binaries in order to run. (probably `[CMakeLists.txt directory]/lib`)

```bash
## run this everytime you open a new terminal to run a python script using this library
PYTHONPATH=$PYTHONPATH:/path/to/lib
## Alternatively, append the line to ~/.bashrc if you don't want to run it every time.
```