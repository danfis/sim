# sim

sim is a robotic simulator originaly created for Symbrion/Replicator european
project (http://www.symbrion.eu).
The goal of this project is to create a full library/framework for
simulation of robots, sensors and environment.

sim is written mostly in C++ and is currently based on OpenSceneGraph,
ODE and Bullet libraries.


## License
sim is licensed under LGPLv3 license. Text of license can be found
at http://www.gnu.org/copyleft/lesser.html or in COPYING.LESSER file
distributed along with sources.

The exception is OpenSURF implementation (src/alg/surf) that is licensed
under GPL license. See http://www.chrisevansdev.com/ for more info.


## Documentation
Documentation is automaticaly generated from source code using Doxygen.

You can generate documentation from doc/ directory:
```sh
    $ make doc
```

It will generate HTML documentation into doxy/html directory.


## Compilation Under Mac OS
[Here](https://docs.google.com/document/d/18_r9FMkcgAUOjAuA5FlkqK92e064M0vD8S3KZabErzU/edit)
is available full installation manual for sim and all required
libraries.
Credit goes to Honza Dvorksy who was kind to make the manual public.

## Acknowledgement
This work was supported by SYMBRION and REPLICATOR projects.

The SYMBRION project is funded by European Commission within the work "Future and Emergent Technologies Proactive" under grant agreement no. 216342.

The REPLICATOR project is funded within the work programme "Cognitive Systems, Interaction, Robotics" under grant agreement no. 216240.

http://www.symbrion.eu

http://www.replicators.eu/
