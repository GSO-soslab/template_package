# Template Package for ROS1 development

[Original source from Croman and the Barbarians](https://bitbucket.org/croman_and_the_barbarians/template_package/src/master/)

## Package usage

###  \#0 Start. 

Use this package as a template to a new package or create
a new package using `catkin_create_pkg`. Either way use this
package as a design guide. If you'll crate your own package
using `catkin_create_pkg`, you may stop at this step.

### \#1 Rename everything.

Many of the files starts with `template`. Final package should
not contain any artifacts from this template.

1. Change the author and maintainer name and e-mail in the
`package.xml` file
2. Change Package name both in `CMakeLists.txt` file and
`package.xml` file.
3. Rename directories to match the package name.
4. Rename class, variable, and method names.
> ROS package names uses `snake_case`.


### \#2 Placement of the files

`CMakeLists.txt` file will automatically recognize your source
code as long as the naming of them complies with the convention.

- All the header files, should reside in `include` directory. 
  `include/*.h`or `include/<package_name>/*.h`
- All the library source code **must** reside in 
`src/<package_name>` directory.
  `src/<package_name>/*.cpp`
- Sources for nodes **must** reside in `src` directory 
and **must** have `node` in the file name.
  `src/*node*.cpp`


After refactoring, package structure should look somewhat
similar to below directory tree.
```bash
├── CMakeLists.txt
├── config
│   └── <package_name>_config.yaml
├── include
│   └── <package_name> 
│       └── <trivial>.h
├── launch
│   └── <trivial>.launch
├── package.xml
├── README.md
└── src
    ├── <trivial>_node.cpp
    └── <package_name> 
        └── <trivial>.cpp
```

### \#3 Miscellaneous

- Configuration files should be in [YAML](https://yaml.org/) format.
- All configuration file along with the examples should be in `config`
directory.
- All launch files should be in `launch` directory.

### \#4 Validating the refactor

For validating that there is no file left has `template` word in its name.
```bash
find . -name "*template*"
```

For validating that there is no content with the word "template" inside
the code
```bash
grep -rin "template"
```

### \#5 Version control

- Use `git` for version control.
- Use [Conventional Commits](https://www.conventionalcommits.org/en/v1.0.0/)
- Have a proper `.gitignore` file

> A good resource on GIT https://learngitbranching.js.org

> There is a good `.gitignore` file generator https://www.toptal.com/developers/gitignore

## C++ Style Guide

Styling guide would work as follows.
- [template_class.h](./include/template_package/template_class.h) file holds
style guide in the doxygen comments.
- For the things that aren't described in [template_class.h](./include/template_package/template_class.h) 
file author should fallback to [ROS C++ Style guide](http://wiki.ros.org/CppStyleGuide).
- Fot the things that aren't described in [ROS C++ Style guide](http://wiki.ros.org/CppStyleGuide)
author should fallback to [Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html)