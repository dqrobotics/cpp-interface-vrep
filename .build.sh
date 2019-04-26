#!/bin/bash

chmod +x debian/rules
fakeroot debian/rules clean
fakeroot debian/rules build
