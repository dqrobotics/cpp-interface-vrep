# cpp-interface-vrep [![CPP Build](https://github.com/dqrobotics/cpp-interface-vrep/actions/workflows/cpp_build.yml/badge.svg?branch=master)](https://github.com/dqrobotics/cpp-interface-vrep/actions/workflows/cpp_build.yml)
Vrep interface for the dqrobotics in C++

Refer to the [docs](https://dqroboticsgithubio.readthedocs.io/en/latest/installation/cpp.html).


### Workflow cpp-interface-vrep

- Clone the repository 
  ```shell
  git clone https://github.com/dqrobotics/cpp-interface-vrep.git --recursive && cd cpp-interface-vrep
  ```
- Create a new branch using the following format name: *master-v`{`x.y.z`}`-rev`{`w`}`*, where *{`x.y.z`}`-rev`{`w`}`* correspond to the version of CoppeliaSim. For example, if the version of CoppeliaSim is 4.5.1-rev4, 

  ```shell
  git checkout -b master-v4.5.1-rev4
  ```
- Checkout the submodules (coppeliarobotics/include/ and coppeliarobotics/remoteApi) to match the version of CoppeliaSim. 
  ```shell
  cd coppeliarobotics/include/
  git checkout coppeliasim-v4.5.1-rev4
  ```

    ```shell
  cd ..
  cd remoteApi/
  git checkout coppeliasim-v4.5.1-rev4 
  ```
- Implement the required modifications to ensure that `cpp-interface-vrep` compiles and passes the tests.
- Update the `debian` folder `{rules, control, changelog}`. You can find an example [here](https://github.com/dqrobotics/cpp-interface-vrep/tree/master-v4.5.1-rev4/debian).
- Open a PR.

