# Extlibs (External Libraries)
The extlibs folder contains code that was imported from other sources which do not have their own Bazel support. 
We aim to limit the integration with these extlibs to a small interface, so we may modify the imported code to make that happen. 
To make integration easier, we also may convert the code to use the Bazel build system. The following build files are 
referenced in the root `MODULE` file and serve as the central build file for their respective repos.  
