# cpp-interface-vrep [![CPP Build](https://github.com/dqrobotics/cpp-interface-vrep/actions/workflows/cpp_build.yml/badge.svg?branch=master)](https://github.com/dqrobotics/cpp-interface-vrep/actions/workflows/cpp_build.yml)
Vrep interface for the dqrobotics in C++

Refer to the [docs](https://dqroboticsgithubio.readthedocs.io/en/latest/installation/cpp.html).


### Workflow cpp-interface-vrep

The following instructions describe the workflow to create a `cpp-interface-vrep` branch with a specific version of CoppeliaSim. 

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
- Update the `debian` folder `{rules, control, changelog}`. Check this [example](https://github.com/dqrobotics/cpp-interface-vrep/tree/master-v4.5.1-rev4/debian), and this [PR](https://github.com/dqrobotics/cpp-interface-vrep/pull/14/files).

- Check that `cpp-interface-vrep` compiles and installs using the developer workflow. For instance:

  ```shell
  chmod +x debian/rules
  fakeroot debian/rules clean
  fakeroot debian/rules build
  fakeroot debian/rules binary
  cd ..
  sudo dpkg -i libdqrobotics-interface-vrep-4.5.1-4_19.10.0_amd64.deb
  ```
- Open a PR.

