# Select Point Pair Adapter

A simple tool for wrap4d, transfer xyz coordinate position to barycentric coordinate.

## How to build

```
git submodule init
git submodule upgrade
mkdir build
cd build
cmake ..
cmake --build .
```

Warning: This project uses vcpkg as default package manager. If you have any problem in building, please fix the CMakeLists.txt or your own parameter yourself. I have test the build procedure in both Windows and Linux platforms and the result is good. If you are using windows platform, please don't use Ninja with gcc and use MSVC as default generator. By the way if you hoping build faster you can try Ninja with MSVC on Windows.

## How to use

### Convert points to target json

Usage:
```
SelectPointPairAdapter transfer {OPTIONS} [inputMesh] [inputPoints]                                                    
    [outputfile]                                                                                                         
                                                                                                                         
    transfer file itself.                                                                                                
                                                                                                                         
  OPTIONS:                                                                                                               
                                                                                                                         
      If you use -g, you can view the                                                                                    
      points in imgui:                                                                                                   
        -g, --gui                         The GUI flag.                                                                  
        -e, --engine                      The inserted engine flag                                                       
      inputMesh                         The input mesh file position                                                     
      inputPoints                       The input points file position                                                   
      outputfile                        The out points file position                                                     
      "--" can be used to terminate flag options and force all following                                                 
      arguments to be treated as positional options
```

Example:
```
./<name> transfer -e ./inputMesh.obj ./inputPoints.off ./outputFile.json
```

### Generate project with selected pair node

Usage:
```
SelectPointPairAdapter generate {OPTIONS} [pair_name] [input_wrap]                                                     
    [output_wrap] [left_points] [right_points]                                                                           
                                                                                                                         
    Generate specific wrap4d project file                                                                                
                                                                                                                         
  OPTIONS:                                                                                                               
                                                                                                                         
      is Using b algo?                                                                                                   
        -l, --left                        The left flag                                                                  
        -r, --right                       The right flag                                                                 
      pair_name                         Select pair node name.                                                           
      input_wrap                        Input wrap file dir                                                              
      output_wrap                       Output wrap file dir                                                             
      left_points                       Output wrap file dir                                                             
      right_points                      Output wrap file dir                                                             
      "--" can be used to terminate flag options and force all following                                                 
      arguments to be treated as positional options
```

Example:
```
./<name> generate SelectPointPairs ./test.wrap ./outputTest.wrap leftPoints.txt rightPoints.json
```